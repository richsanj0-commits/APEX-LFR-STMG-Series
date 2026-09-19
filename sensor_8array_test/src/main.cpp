#include <Arduino.h>
#include <U8g2lib.h>

// ============================================================
//  8-Sensor Array Calibration, Binary (0/1) & Wheel Drive Bench
// ============================================================

// 8 Fresh Analog Pins on Port A
const int SENSOR_PINS[8] = { PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7 };

// Pushbuttons (Active LOW with internal pull-up)
#define BTN_CALIBRATE PC13   // Press to calibrate 10 seconds
#define BTN_FWD       PB5    // Press/Hold to drive both wheels forward
#define BTN_REV       PB4    // Press/Hold to drive both wheels backward
#define BTN_TOGGLE    PB3    // Optional auxiliary button

// TB6612FNG Dual H-Bridge Motor Driver
#define AIN1  PB0     // Right motor direction 1
#define AIN2  PB1     // Right motor direction 2
#define BIN1  PB10    // Left motor direction 1
#define BIN2  PB11    // Left motor direction 2
#define PWMA  PA8     // Right motor PWM
#define PWMB  PA9     // Left motor PWM
#define STBY  PA10    // Motor driver standby (HIGH = enabled)

// Motor test speed (0 - 255)
#define MOTOR_SPEED 150

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

// Motor states
enum MotorState {
  MOTOR_STOPPED,
  MOTOR_FORWARD,
  MOTOR_BACKWARD
};
MotorState current_motor_state = MOTOR_STOPPED;

// Display refresh timing
unsigned long last_display_ms = 0;
const unsigned long DISPLAY_INTERVAL = 80; // 80ms refresh (~12 FPS)

// ------------------------------------------------------------
// Motor Control Functions
// ------------------------------------------------------------
void motor_left(int speed) {
  // Channel B = Left Motor
  // BIN1=LOW, BIN2=HIGH drives Forward; BIN1=HIGH, BIN2=LOW drives Reverse
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, speed);
  } else if (speed < 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, -speed);
  } else {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, 0);
  }
}

void motor_right(int speed) {
  // Channel A = Right Motor
  // AIN1=HIGH, AIN2=LOW drives Forward; AIN1=LOW, AIN2=HIGH drives Reverse
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, speed);
  } else if (speed < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -speed);
  } else {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, 0);
  }
}

void motor_stop() {
  motor_left(0);
  motor_right(0);
  current_motor_state = MOTOR_STOPPED;
}

void motor_forward(int speed = MOTOR_SPEED) {
  motor_left(speed);
  motor_right(speed);
  current_motor_state = MOTOR_FORWARD;
}

void motor_backward(int speed = MOTOR_SPEED) {
  motor_left(-speed);
  motor_right(-speed);
  current_motor_state = MOTOR_BACKWARD;
}

void motors_init() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Enable the TB6612 driver
  digitalWrite(STBY, HIGH);
  motor_stop();
}

// ------------------------------------------------------------
// Button Handling
// ------------------------------------------------------------
bool check_button_pressed(int pin) {
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

// ------------------------------------------------------------
// Calibration Routine (PC13)
// ------------------------------------------------------------
void start_calibration() {
  motor_stop();
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
  Serial.println(">>> PB5: Both Wheels Forward  |  PB4: Both Wheels Backward <<<");
  Serial.println("==================================================");
  Serial.println();

  // OLED Ready Screen
  u8g2.clearBuffer();
  u8g2.drawStr(0, 10, "CALIBRATION DONE!");
  u8g2.drawStr(0, 22, "PB5: FWD | PB4: REV");
  u8g2.drawStr(0, 32, "STATUS: READY LIVE");
  u8g2.sendBuffer();
  delay(1200);
}

// ------------------------------------------------------------
// Sensor Read and Live Binary Display Routine
// ------------------------------------------------------------
void update_sensors_and_display() {
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

  const char* m_str = (current_motor_state == MOTOR_FORWARD)  ? "FWD " :
                      (current_motor_state == MOTOR_BACKWARD) ? "REV " : "STOP";

  // Print to Serial Monitor
  char line[140];
  snprintf(line, sizeof(line),
    "BIN | S0:%d | S1:%d | S2:%d | S3:%d | S4:%d | S5:%d | S6:%d | S7:%d |  %s  | MOT:%s",
    bin_vals[0], bin_vals[1], bin_vals[2], bin_vals[3],
    bin_vals[4], bin_vals[5], bin_vals[6], bin_vals[7],
    visual, m_str
  );
  Serial.println(line);

  // Update OLED Display
  u8g2.clearBuffer();

  // Header line with motor state indicator
  char header[28];
  if (current_motor_state == MOTOR_FORWARD) {
    snprintf(header, sizeof(header), "0:W 1:B  [>>>FWD>>>]");
  } else if (current_motor_state == MOTOR_BACKWARD) {
    snprintf(header, sizeof(header), "0:W 1:B  [<<<REV<<<]");
  } else {
    snprintf(header, sizeof(header), is_calibrated ? "0:W 1:B   [CAL READY]" : "0:W 1:B   [DEFAULT]");
  }
  u8g2.drawStr(0, 8, header);

  // Line 2: numerical string
  char bin_str[32];
  snprintf(bin_str, sizeof(bin_str), "D: %d %d %d %d %d %d %d %d",
    bin_vals[0], bin_vals[1], bin_vals[2], bin_vals[3],
    bin_vals[4], bin_vals[5], bin_vals[6], bin_vals[7]);
  u8g2.drawStr(0, 19, bin_str);

  // Line 3: visual 8-box bar (solid block = 1, empty outline = 0)
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
}

// ------------------------------------------------------------
// Arduino Setup
// ------------------------------------------------------------
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
  pinMode(BTN_FWD,       INPUT_PULLUP);
  pinMode(BTN_REV,       INPUT_PULLUP);
  pinMode(BTN_TOGGLE,    INPUT_PULLUP);

  // Initialize TB6612FNG Motor Driver
  motors_init();

  // Initialize OLED
  u8g2.begin();
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.clearBuffer();
  u8g2.drawStr(0, 10, "8-SENSOR + MOTOR BENCH");
  u8g2.drawStr(0, 20, "PB5: FWD | PB4: REV");
  u8g2.drawStr(0, 30, "PC13: Calibrate 10s");
  u8g2.sendBuffer();

  Serial.println();
  Serial.println("==================================================");
  Serial.println("    STM32 8-Sensor + Wheel Direction Bench        ");
  Serial.println("==================================================");
  Serial.println("Controls:");
  Serial.println("  - PB5 Button (Hold): Both wheels FORWARD");
  Serial.println("  - PB4 Button (Hold): Both wheels BACKWARD");
  Serial.println("  - Release PB5 / PB4: Wheels STOP");
  Serial.println("  - PC13 Button:       Calibrate 10s over line/floor");
  Serial.println("Serial Commands:");
  Serial.println("  - 'f' / 'w': Jog Forward  |  'b' / 's': Jog Backward");
  Serial.println("  - 'x' / ' ': Stop Motors  |  'c': Calibrate");
  Serial.println("==================================================");
  Serial.println();
}

// ------------------------------------------------------------
// Arduino Loop
// ------------------------------------------------------------
void loop() {
  // 1. Check PC13 calibration button
  if (check_button_pressed(BTN_CALIBRATE)) {
    motor_stop();
    start_calibration();
  }

  // 2. Check Serial commands
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'c' || c == 'C') {
      motor_stop();
      start_calibration();
    } else if (c == 'f' || c == 'F' || c == 'w' || c == 'W') {
      motor_forward(MOTOR_SPEED);
      Serial.println("\n>>> MOTORS: FORWARD (via Serial) <<<");
    } else if (c == 'b' || c == 'B' || c == 's' || c == 'S') {
      motor_backward(MOTOR_SPEED);
      Serial.println("\n>>> MOTORS: BACKWARD (via Serial) <<<");
    } else if (c == 'x' || c == 'X' || c == ' ') {
      motor_stop();
      Serial.println("\n>>> MOTORS: STOPPED (via Serial) <<<");
    }
  }

  // 3. Physical Buttons: PB5 (Forward) and PB4 (Backward)
  // Active LOW: LOW = pressed, HIGH = released
  bool fwd_pressed = (digitalRead(BTN_FWD) == LOW);
  bool rev_pressed = (digitalRead(BTN_REV) == LOW);

  if (fwd_pressed && !rev_pressed) {
    if (current_motor_state != MOTOR_FORWARD) {
      motor_forward(MOTOR_SPEED);
      Serial.println("\n>>> MOTORS: FORWARD (PB5 Held) <<<");
    }
  } else if (rev_pressed && !fwd_pressed) {
    if (current_motor_state != MOTOR_BACKWARD) {
      motor_backward(MOTOR_SPEED);
      Serial.println("\n>>> MOTORS: BACKWARD (PB4 Held) <<<");
    }
  } else {
    // If neither is pressed, stop the motors
    if (current_motor_state != MOTOR_STOPPED && !fwd_pressed && !rev_pressed) {
      motor_stop();
      Serial.println("\n>>> MOTORS: STOPPED (Released) <<<");
    }
  }

  // 4. Periodic sensor reading and OLED / Serial display update
  if (millis() - last_display_ms >= DISPLAY_INTERVAL) {
    last_display_ms = millis();
    update_sensors_and_display();
  }
}
