#include <Arduino.h>
#include <U8g2lib.h>

// ============================================================
//  8-Sensor Autonomous PID Line Follower Robot (LFR)
//  Hardware: STM32G431CB (Arm Cortex-M4 @ 170MHz)
//  Sensors:  8x Direct Analog IR (PA0 - PA7)
//  Driver:   TB6612FNG Dual H-Bridge (Ch B=Left, Ch A=Right)
// ============================================================

// --- Pin Definitions ---
// 8 Direct Analog Inputs on Port A
const int SENSOR_PINS[8] = { PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7 };

// Pushbuttons (Active LOW with internal pull-up)
#define BTN_CALIBRATE PC13   // Press to calibrate 10s
#define BTN_START     PB5    // Press to START line following
#define BTN_JOG_REV   PB4    // Bench test: Jog backward
#define BTN_STOP      PB3    // Press to STOP line follower

// TB6612FNG Dual H-Bridge Motor Driver
#define AIN1  PB0     // Right motor direction 1
#define AIN2  PB1     // Right motor direction 2
#define BIN1  PB10    // Left motor direction 1
#define BIN2  PB11    // Left motor direction 2
#define PWMA  PA8     // Right motor PWM
#define PWMB  PA9     // Left motor PWM
#define STBY  PA10    // Motor driver standby (HIGH = enabled)

// OLED Display (SSD1306 via software I2C on PB6/PB7)
#define OLED_SCL PB6
#define OLED_SDA PB7
U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2(U8G2_R0, OLED_SCL, OLED_SDA, U8X8_PIN_NONE);

// --- Sensor Position Weights ---
// S0 (Far Left) to S7 (Far Right). Center is between S3 and S4.
const float SENSOR_WEIGHTS[8] = { -70.0f, -45.0f, -25.0f, -8.0f, +8.0f, +25.0f, +45.0f, +70.0f };

// Calibration & Sensor Variables
uint16_t sensor_val[8];
uint16_t sensor_min[8];
uint16_t sensor_max[8];
uint16_t sensor_threshold[8];
bool is_calibrated = false;

// --- PID Tuning Parameters ---
float Kp = 1.60f;        // Proportional gain
float Ki = 0.00f;        // Integral gain (kept 0 for responsive line tracking)
float Kd = 6.50f;        // Derivative damping gain
int base_speed = 110;    // Nominal cruise speed (0 - 255)
int max_speed  = 210;    // Maximum speed clamp

// PID State variables
float follower_integral = 0.0f;
float follower_last_error = 0.0f;
float filtered_derivative = 0.0f;

// Operating States
enum RobotState {
  STATE_IDLE,          // Waiting for user command / bench viewing
  STATE_CALIBRATING,   // 10-second sweep calibration
  STATE_RUNNING        // Active PID line following
};
RobotState current_state = STATE_IDLE;

// Timing
unsigned long last_display_ms = 0;
const unsigned long DISPLAY_INTERVAL = 80; // 80ms refresh (~12 FPS)

// ------------------------------------------------------------
// Motor Control Functions (TB6612FNG)
// ------------------------------------------------------------
void motor_left(int speed) {
  // Channel B = Left Motor
  // BIN1=LOW, BIN2=HIGH for Forward; BIN1=HIGH, BIN2=LOW for Reverse
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
  // Channel A = Right Motor (Polarity aligned for forward motion)
  // AIN1=LOW, AIN2=HIGH for Forward; AIN1=HIGH, AIN2=LOW for Reverse
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, speed);
  } else if (speed < 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
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
}

void motor_forward(int speed = 150) {
  motor_left(speed);
  motor_right(speed);
}

void motor_backward(int speed = 150) {
  motor_left(-speed);
  motor_right(-speed);
}

void motors_init() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);
  motor_stop();
}

// ------------------------------------------------------------
// Button Handling
// ------------------------------------------------------------
bool check_button_pressed(int pin) {
  if (digitalRead(pin) == LOW) {
    delay(20); // Debounce
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
// Sensor Centroid & Error Calculation
// ------------------------------------------------------------
float get_line_error(int bin_vals[8]) {
  float sum_weighted = 0.0f;
  float sum_active = 0.0f;
  int active_sensors = 0;

  for (int i = 0; i < 8; i++) {
    sensor_val[i] = analogRead(SENSOR_PINS[i]);

    // Black line = Lower ADC value (< threshold) -> 1
    // White floor = Higher ADC value (>= threshold) -> 0
    if (sensor_val[i] < sensor_threshold[i]) {
      bin_vals[i] = 1;
      sum_weighted += SENSOR_WEIGHTS[i];
      sum_active += 1.0f;
      active_sensors++;
    } else {
      bin_vals[i] = 0;
    }
  }

  // If no sensors detect black line (line lost)
  if (active_sensors == 0) {
    // Steer in the direction where the line was last seen to recover
    if (follower_last_error > 0.0f) return 75.0f;   // Turn hard right
    if (follower_last_error < 0.0f) return -75.0f;  // Turn hard left
    return 0.0f;
  }

  float error = sum_weighted / sum_active;
  follower_last_error = error;
  return error;
}

// ------------------------------------------------------------
// Display & Serial Telemetry
// ------------------------------------------------------------
void update_display_and_serial(float error, int current_base, int left_spd, int right_spd, int bin_vals[8]) {
  char visual[32] = "[ ";
  for (int i = 0; i < 8; i++) {
    strcat(visual, bin_vals[i] ? "1 " : "0 ");
  }
  strcat(visual, "]");

  const char* st_str = (current_state == STATE_RUNNING) ? "RUN" : "IDLE";

  // Serial Monitor Log
  char line[150];
  snprintf(line, sizeof(line),
    "%s | Err:%+5.1f | Spd:%3d | L:%+4d R:%+4d | %s",
    st_str, error, current_base, left_spd, right_spd, visual
  );
  Serial.println(line);

  // OLED Display Update
  u8g2.clearBuffer();

  // Line 1: Header / Status & Error
  char header[28];
  if (current_state == STATE_RUNNING) {
    snprintf(header, sizeof(header), "RUN  Err:%+3d  Spd:%d", (int)error, current_base);
  } else {
    snprintf(header, sizeof(header), is_calibrated ? "READY | PB5:START" : "IDLE  | PC13:CAL");
  }
  u8g2.drawStr(0, 8, header);

  // Line 2: Numerical binary string
  char bin_str[32];
  snprintf(bin_str, sizeof(bin_str), "D: %d %d %d %d %d %d %d %d",
    bin_vals[0], bin_vals[1], bin_vals[2], bin_vals[3],
    bin_vals[4], bin_vals[5], bin_vals[6], bin_vals[7]);
  u8g2.drawStr(0, 19, bin_str);

  // Line 3: Visual 8-box bar (solid block = 1 on line, hollow = 0 on floor)
  for (int i = 0; i < 8; i++) {
    int x = i * 16 + 2;
    int y = 22;
    if (bin_vals[i] == 1) {
      u8g2.drawBox(x, y, 12, 10);
    } else {
      u8g2.drawFrame(x, y, 12, 10);
    }
  }
  u8g2.sendBuffer();
}

// ------------------------------------------------------------
// Calibration Routine (PC13)
// ------------------------------------------------------------
void start_calibration() {
  motor_stop();
  current_state = STATE_CALIBRATING;

  Serial.println();
  Serial.println("==================================================");
  Serial.println(">>> STARTING 10-SECOND CALIBRATION <<<");
  Serial.println(">>> SWEEP THE ROBOT SENSORS OVER WHITE & BLACK <<<");
  Serial.println("==================================================");

  for (int i = 0; i < 8; i++) {
    uint16_t first_read = analogRead(SENSOR_PINS[i]);
    sensor_min[i] = first_read;
    sensor_max[i] = first_read;
  }

  unsigned long start_ms = millis();
  const unsigned long duration_ms = 10000;
  int last_sec_remaining = -1;

  while (millis() - start_ms < duration_ms) {
    for (int i = 0; i < 8; i++) {
      uint16_t val = analogRead(SENSOR_PINS[i]);
      if (val < sensor_min[i]) sensor_min[i] = val;
      if (val > sensor_max[i]) sensor_max[i] = val;
    }

    int sec_remaining = (int)((duration_ms - (millis() - start_ms) + 999) / 1000);
    if (sec_remaining != last_sec_remaining) {
      last_sec_remaining = sec_remaining;

      Serial.print("Calibrating... ");
      Serial.print(sec_remaining);
      Serial.println("s remaining");

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

  for (int i = 0; i < 8; i++) {
    sensor_threshold[i] = (sensor_min[i] + sensor_max[i]) / 2;
  }
  is_calibrated = true;
  current_state = STATE_IDLE;

  Serial.println("--------------------------------------------------");
  Serial.println("Calibration Midpoints:");
  char line[120];
  snprintf(line, sizeof(line), "MID | %4u | %4u | %4u | %4u | %4u | %4u | %4u | %4u |",
    sensor_threshold[0], sensor_threshold[1], sensor_threshold[2], sensor_threshold[3],
    sensor_threshold[4], sensor_threshold[5], sensor_threshold[6], sensor_threshold[7]);
  Serial.println(line);
  Serial.println("==================================================");
  Serial.println(">>> READY! Press PB5 to START Line Following <<<");
  Serial.println("==================================================\n");

  u8g2.clearBuffer();
  u8g2.drawStr(0, 10, "CALIBRATION DONE!");
  u8g2.drawStr(0, 22, "Press PB5 to START");
  u8g2.drawStr(0, 32, "Press PB3 to STOP");
  u8g2.sendBuffer();
  delay(1200);
}

// ------------------------------------------------------------
// Line Following Execution Step (Called continuously in RUN)
// ------------------------------------------------------------
void line_follower_step() {
  int bin_vals[8];
  float error = get_line_error(bin_vals);

  // 1. Proportional term
  float P = Kp * error;

  // 2. Integral term with anti-windup
  follower_integral += error;
  follower_integral = constrain(follower_integral, -150.0f, 150.0f);
  float I = Ki * follower_integral;

  // 3. Filtered derivative term (low-pass filter removes high-frequency jitter)
  float raw_deriv = error - follower_last_error;
  filtered_derivative = (0.70f * filtered_derivative) + (0.30f * raw_deriv);
  float D = Kd * filtered_derivative;

  // Total PID steering correction
  float pid_val = P + I + D;

  // 4. Dynamic Corner Auto-Braking:
  // When error is large (approaching sharp corner), dynamically reduce forward momentum
  int current_base = base_speed;
  float abs_err = abs(error);
  if (abs_err > 15.0f) {
    float speed_factor = 1.0f - ((abs_err - 15.0f) / 65.0f);
    if (speed_factor < 0.35f) speed_factor = 0.35f;
    current_base = (int)(base_speed * speed_factor);
  }

  // 5. Differential Motor Speeds
  int left_speed  = current_base + (int)pid_val;
  int right_speed = current_base - (int)pid_val;

  left_speed  = constrain(left_speed,  -max_speed, max_speed);
  right_speed = constrain(right_speed, -max_speed, max_speed);

  motor_left(left_speed);
  motor_right(right_speed);

  // Telemetry update
  if (millis() - last_display_ms >= DISPLAY_INTERVAL) {
    last_display_ms = millis();
    update_display_and_serial(error, current_base, left_speed, right_speed, bin_vals);
  }
}

// ------------------------------------------------------------
// Serial Command Handler
// ------------------------------------------------------------
void handle_serial() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'c' || c == 'C') {
      start_calibration();
    } else if (c == 's' || c == 'S') {
      current_state = STATE_RUNNING;
      follower_integral = 0.0f;
      follower_last_error = 0.0f;
      filtered_derivative = 0.0f;
      Serial.println("\n>>> LINE FOLLOWER STARTED (via Serial) <<<\n");
    } else if (c == 'x' || c == 'X' || c == ' ') {
      motor_stop();
      current_state = STATE_IDLE;
      Serial.println("\n>>> LINE FOLLOWER STOPPED (via Serial) <<<\n");
    } else if (c == '+') {
      base_speed = constrain(base_speed + 10, 50, 240);
      Serial.print("Base Speed: "); Serial.println(base_speed);
    } else if (c == '-') {
      base_speed = constrain(base_speed - 10, 50, 240);
      Serial.print("Base Speed: "); Serial.println(base_speed);
    } else if (c == 'p') {
      Kp += 0.1f;
      Serial.print("Kp: "); Serial.println(Kp, 2);
    } else if (c == 'P') {
      Kp = max(0.1f, Kp - 0.1f);
      Serial.print("Kp: "); Serial.println(Kp, 2);
    } else if (c == 'd') {
      Kd += 0.5f;
      Serial.print("Kd: "); Serial.println(Kd, 2);
    } else if (c == 'D') {
      Kd = max(0.0f, Kd - 0.5f);
      Serial.print("Kd: "); Serial.println(Kd, 2);
    }
  }
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
  pinMode(BTN_START,     INPUT_PULLUP);
  pinMode(BTN_JOG_REV,   INPUT_PULLUP);
  pinMode(BTN_STOP,      INPUT_PULLUP);

  // Initialize TB6612FNG Motor Driver
  motors_init();

  // Initialize OLED
  u8g2.begin();
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.clearBuffer();
  u8g2.drawStr(0, 10, "APEX 8-IR FOLLOWER");
  u8g2.drawStr(0, 20, "PC13: Calibrate 10s");
  u8g2.drawStr(0, 30, "PB5: START | PB3:STOP");
  u8g2.sendBuffer();

  Serial.println();
  Serial.println("==================================================");
  Serial.println("   STM32 8-IR Autonomous PID Line Follower Robot  ");
  Serial.println("==================================================");
  Serial.println("Controls:");
  Serial.println("  - PC13: Calibrate 10 seconds over line & floor");
  Serial.println("  - PB5:  START Autonomous Line Following");
  Serial.println("  - PB3 (or PC13): EMERGENCY STOP");
  Serial.println("  - PB4:  Jog backward bench test (when idle)");
  Serial.println("Serial Commands:");
  Serial.println("  - 's': Start  |  'x': Stop  |  'c': Calibrate");
  Serial.println("  - '+' / '-': Adjust base speed (current: " + String(base_speed) + ")");
  Serial.println("  - 'p' / 'P': Adjust Kp (current: " + String(Kp, 2) + ")");
  Serial.println("  - 'd' / 'D': Adjust Kd (current: " + String(Kd, 2) + ")");
  Serial.println("==================================================\n");
}

// ------------------------------------------------------------
// Arduino Loop
// ------------------------------------------------------------
void loop() {
  // 1. Check PC13 Calibration button
  if (check_button_pressed(BTN_CALIBRATE)) {
    motor_stop();
    current_state = STATE_IDLE;
    start_calibration();
  }

  // 2. State Machine Handling
  if (current_state == STATE_RUNNING) {
    // Emergency STOP check: PB3 or PC13
    if (digitalRead(BTN_STOP) == LOW || digitalRead(BTN_CALIBRATE) == LOW) {
      motor_stop();
      current_state = STATE_IDLE;
      Serial.println("\n>>> LINE FOLLOWER STOPPED! <<<");
      delay(300);
      return;
    }

    // Run high-speed PID line following loop
    line_follower_step();
  }
  else {
    // In IDLE state:
    // PB5 press starts Line Following!
    if (check_button_pressed(BTN_START)) {
      current_state = STATE_RUNNING;
      follower_integral = 0.0f;
      follower_last_error = 0.0f;
      filtered_derivative = 0.0f;
      Serial.println("\n>>> LINE FOLLOWER STARTED! <<<");
      Serial.println("Press PB3 (or PC13) to STOP.\n");
      return;
    }

    // PB4 press: Jog backward bench test
    if (digitalRead(BTN_JOG_REV) == LOW) {
      motor_backward(150);
    } else {
      motor_stop();
    }

    // Idle telemetry refresh
    if (millis() - last_display_ms >= DISPLAY_INTERVAL) {
      last_display_ms = millis();
      int bin_vals[8];
      float error = get_line_error(bin_vals);
      update_display_and_serial(error, 0, 0, 0, bin_vals);
    }
  }

  // 3. Serial command handling
  handle_serial();
}
