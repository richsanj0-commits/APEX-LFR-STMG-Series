#include <Arduino.h>
#include <U8g2lib.h>

// ============================================================
//  8-Sensor Array Calibration & Binary (0/1) Verification Bench
// ============================================================

// 8 Fresh Analog Pins on Port A
const int SENSOR_PINS[8] = { PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7 };

// Control Buttons (Active LOW with internal pull-up)
#define BTN_CALIBRATE PC13   // Press to calibrate 10 seconds
#define BTN_BINARY    PB5    // Press to toggle 0/1 binary display

// OLED Display (SSD1306 via software I2C on PB6/PB7)
#define OLED_SCL PB6
#define OLED_SDA PB7
U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2(U8G2_R0, OLED_SCL, OLED_SDA, U8X8_PIN_NONE);

// Sensor readings and calibration variables
uint16_t sensor_val[8];
uint16_t sensor_min[8];
uint16_t sensor_max[8];
uint16_t sensor_threshold[8];
bool is_calibrated = false;

// System states
enum State {
  STATE_IDLE,
  STATE_CALIBRATING,
  STATE_READY,
  STATE_SHOW_BINARY
};
State current_state = STATE_IDLE;

// Non-blocking button edge detection
bool check_button_pressed(int pin) {
  static unsigned long last_check_time = 0;
  if (digitalRead(pin) == LOW) {
    delay(25); // Debounce
    if (digitalRead(pin) == LOW) {
      while (digitalRead(pin) == LOW) {
        delay(10); // Wait for release
      }
      return true;
    }
  }
  return false;
}

void start_calibration() {
  current_state = STATE_CALIBRATING;
  Serial.println();
  Serial.println("==================================================");
  Serial.println(">>> STARTING 10-SECOND CALIBRATION <<<");
  Serial.println(">>> SWEEP THE ROBOT SENSORS OVER WHITE & BLACK <<<");
  Serial.println("==================================================");

  // Initialize min/max
  for (int i = 0; i < 8; i++) {
    uint16_t first_read = analogRead(SENSOR_PINS[i]);
    sensor_min[i] = first_read;
    sensor_max[i] = first_read;
  }

  unsigned long start_ms = millis();
  const unsigned long duration_ms = 10000; // 10 seconds
  int last_sec_remaining = -1;

  while (millis() - start_ms < duration_ms) {
    // Read all sensors and update bounds
    for (int i = 0; i < 8; i++) {
      uint16_t val = analogRead(SENSOR_PINS[i]);
      if (val < sensor_min[i]) sensor_min[i] = val;
      if (val > sensor_max[i]) sensor_max[i] = val;
    }

    int sec_remaining = (int)((duration_ms - (millis() - start_ms) + 999) / 1000);
    if (sec_remaining != last_sec_remaining) {
      last_sec_remaining = sec_remaining;
      
      // Print countdown to Serial
      Serial.print("Calibrating... ");
      Serial.print(sec_remaining);
      Serial.println("s remaining");

      // Update OLED
      u8g2.clearBuffer();
      u8g2.drawStr(0, 10, "CALIBRATING SENSORS");
      char buf[24];
      snprintf(buf, sizeof(buf), "Time: %d sec left", sec_remaining);
      u8g2.drawStr(0, 22, buf);
      int progress_w = map(10 - sec_remaining, 0, 10, 0, 128);
      u8g2.drawBox(0, 26, progress_w, 6);
      u8g2.sendBuffer();
    }
    delay(10);
  }

  // Calculate thresholds: (min + max) / 2
  for (int i = 0; i < 8; i++) {
    sensor_threshold[i] = (sensor_min[i] + sensor_max[i]) / 2;
  }
  is_calibrated = true;
  current_state = STATE_READY;

  // Print summary to Serial
  Serial.println("--------------------------------------------------");
  Serial.println("Calibration Bounds & Midpoints:");
  char line[120];
  snprintf(line, sizeof(line), "MIN | %4u | %4u | %4u | %4u | %4u | %4u | %4u | %4u |",
    sensor_min[0], sensor_min[1], sensor_min[2], sensor_min[3],
    sensor_min[4], sensor_min[5], sensor_min[6], sensor_min[7]);
  Serial.println(line);

  snprintf(line, sizeof(line), "MAX | %4u | %4u | %4u | %4u | %4u | %4u | %4u | %4u |",
    sensor_max[0], sensor_max[1], sensor_max[2], sensor_max[3],
    sensor_max[4], sensor_max[5], sensor_max[6], sensor_max[7]);
  Serial.println(line);

  snprintf(line, sizeof(line), "MID | %4u | %4u | %4u | %4u | %4u | %4u | %4u | %4u |",
    sensor_threshold[0], sensor_threshold[1], sensor_threshold[2], sensor_threshold[3],
    sensor_threshold[4], sensor_threshold[5], sensor_threshold[6], sensor_threshold[7]);
  Serial.println(line);
  Serial.println("--------------------------------------------------");
  Serial.println("==================================================");
  Serial.println(">>> READY! <<<");
  Serial.println(">>> PRESS PB5 BUTTON TO SHOW 0 AND 1 <<<");
  Serial.println("==================================================\n");

  // OLED Ready Screen
  u8g2.clearBuffer();
  u8g2.drawStr(0, 10, "CALIBRATION DONE!");
  u8g2.drawStr(0, 22, "STATUS: READY");
  u8g2.drawStr(0, 32, "Press PB5 for 0/1");
  u8g2.sendBuffer();
}

void setup() {
  Serial.begin(115200);

  // 12-bit ADC (0 - 4095)
  analogReadResolution(12);

  // Setup sensor pins
  for (int i = 0; i < 8; i++) {
    pinMode(SENSOR_PINS[i], INPUT_ANALOG);
    sensor_threshold[i] = 2500; // Default midpoint fallback
  }

  // Setup buttons with internal pullups
  pinMode(BTN_CALIBRATE, INPUT_PULLUP);
  pinMode(BTN_BINARY,    INPUT_PULLUP);

  // Initialize OLED
  u8g2.begin();
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.clearBuffer();
  u8g2.drawStr(0, 10, "8-SENSOR 0/1 TEST");
  u8g2.drawStr(0, 22, "PC13: Calibrate 10s");
  u8g2.drawStr(0, 32, "PB5:  Show 0 / 1");
  u8g2.sendBuffer();

  Serial.println();
  Serial.println("==================================================");
  Serial.println("    STM32 8-Sensor Binary (0/1) Verification      ");
  Serial.println("==================================================");
  Serial.println("Instructions:");
  Serial.println("  1. Press PC13 button (or send 'c') to calibrate 10s");
  Serial.println("     Sweep sensors across black line and white floor.");
  Serial.println("  2. 'READY' will be displayed when calibration ends.");
  Serial.println("  3. Press PB5 button (or send 'b') to see 0 & 1 values.");
  Serial.println("     0 = White floor | 1 = Black line");
  Serial.println("==================================================\n");
}

void loop() {
  // Check Serial commands as alternatives to buttons
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'c' || c == 'C') {
      start_calibration();
    } else if (c == 'b' || c == 'B') {
      if (current_state == STATE_SHOW_BINARY) {
        current_state = STATE_READY;
        Serial.println("\n[Binary display PAUSED. Press PB5 to resume]\n");
      } else {
        current_state = STATE_SHOW_BINARY;
        Serial.println("\n[Binary display STARTED: 0=White, 1=Black]\n");
      }
    }
  }

  // Check PC13 Button: Calibrate 10 seconds
  if (check_button_pressed(BTN_CALIBRATE)) {
    start_calibration();
  }

  // Check PB5 Button: Toggle 0/1 binary display
  if (check_button_pressed(BTN_BINARY)) {
    if (current_state == STATE_SHOW_BINARY) {
      current_state = STATE_READY;
      Serial.println("\n[Binary display PAUSED. Press PB5 to resume]\n");
      u8g2.clearBuffer();
      u8g2.drawStr(0, 12, "PAUSED");
      u8g2.drawStr(0, 24, "Press PB5 to resume");
      u8g2.sendBuffer();
    } else {
      current_state = STATE_SHOW_BINARY;
      Serial.println("\n[Binary display STARTED: 0=White, 1=Black]\n");
    }
  }

  // Live Binary Display Mode
  if (current_state == STATE_SHOW_BINARY) {
    int bin_vals[8];
    char visual[32] = "[ ";

    for (int i = 0; i < 8; i++) {
      sensor_val[i] = analogRead(SENSOR_PINS[i]);
      
      // Binary threshold: 
      // Black line = Lower ADC value (< threshold) -> 1
      // White floor = Higher ADC value (>= threshold) -> 0
      if (sensor_val[i] < sensor_threshold[i]) {
        bin_vals[i] = 1;
        strcat(visual, "1 ");
      } else {
        bin_vals[i] = 0;
        strcat(visual, "0 ");
      }
    }
    strcat(visual, "]");

    // Print to Serial Monitor
    char line[120];
    snprintf(line, sizeof(line),
      "BIN | S0:%d | S1:%d | S2:%d | S3:%d | S4:%d | S5:%d | S6:%d | S7:%d |  %s",
      bin_vals[0], bin_vals[1], bin_vals[2], bin_vals[3],
      bin_vals[4], bin_vals[5], bin_vals[6], bin_vals[7],
      visual
    );
    Serial.println(line);

    // Update OLED Display
    u8g2.clearBuffer();
    u8g2.drawStr(0, 8, "0:WHITE | 1:BLACK");
    
    char bin_str[32];
    snprintf(bin_str, sizeof(bin_str), "D: %d %d %d %d %d %d %d %d",
      bin_vals[0], bin_vals[1], bin_vals[2], bin_vals[3],
      bin_vals[4], bin_vals[5], bin_vals[6], bin_vals[7]);
    u8g2.drawStr(0, 19, bin_str);

    // Draw visual boxes for each sensor (solid block = 1, empty outline = 0)
    for (int i = 0; i < 8; i++) {
      int x = i * 16 + 2;
      int y = 22;
      if (bin_vals[i] == 1) {
        u8g2.drawBox(x, y, 12, 10);    // Solid filled block for Black (1)
      } else {
        u8g2.drawFrame(x, y, 12, 10);  // Hollow outline for White (0)
      }
    }
    u8g2.sendBuffer();

    delay(120);
  }
}
