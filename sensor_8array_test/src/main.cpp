#include <Arduino.h>
#include <U8g2lib.h>

/*
  ============================================================
  Simple Autonomous 8-Channel Line Follower Robot (LF-2 Style)
  Target MCU: STM32G431CB (170MHz)
  Controls:
    - PC13: Calibrate (8 seconds)
    - PB5:  Start line following
    - PB4:  Jog backward bench test (when idle)
    - PB3:  Reset / Emergency Stop
  ============================================================
*/

// -------- Pin definitions for TB6612FNG Motor Driver --------
#define AIN1  PB0     // Right motor direction 1
#define AIN2  PB1     // Right motor direction 2
#define BIN1  PB10    // Left motor direction 1
#define BIN2  PB11    // Left motor direction 2
#define PWMA  PA8     // Right motor PWM
#define PWMB  PA9     // Left motor PWM
#define STBY  PA10    // Motor driver standby (HIGH = enabled)
// ------------------------------------------------------------

// -------- Pushbuttons (Active LOW with internal pull-up) -----
#define BTN_CALIBRATE PC13   // PC13 for calibration
#define BTN_START     PB5    // PB5 for start
#define BTN_JOG_REV   PB4    // PB4 for backward jog
#define BTN_RESET     PB3    // PB3 for reset / stop
// ------------------------------------------------------------

// -------- 8-Channel Analog IR Sensors (Port A) --------------
// Left-to-right physical alignment: S0=PA7 ... S7=PA0
const int SENSOR_PINS[8] = { PA7, PA6, PA5, PA4, PA3, PA2, PA1, PA0 };
// ------------------------------------------------------------

// -------- OLED Display (SSD1306 via SW I2C on PB6/PB7) ------
#define OLED_SCL PB6
#define OLED_SDA PB7
U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2(U8G2_R0, OLED_SCL, OLED_SDA, U8X8_PIN_NONE);
// ------------------------------------------------------------

// -------- Line Details --------------------------------------
bool isBlackLine = 1;
unsigned int numSensors = 8;
// ------------------------------------------------------------

// -------- Speed & PID Settings (High Performance) -----------
int lfSpeed = 110;            // Target cruise speed
int currentSpeed = 40;        // Starting acceleration speed
// S0 (Left: +8) ... S7 (Right: -8)
int sensorWeight[8] = { 8, 4, 2, 1, -1, -2, -4, -8 };

float Kp = 0.080f;            // Proportional gain for responsive turns
float Ki = 0.000f;
float Kd = 0.350f;            // Derivative gain to prevent overshoot

int P, D, I, previousError, PIDvalue;
double error;
int lsp, rsp;
int activeSensors;
int onLine = 1;
int minValues[8], maxValues[8], threshold[8], sensorValue[8], sensorArray[8];
bool isCalibrated = false;

// Function prototypes
void motor1run(int motorSpeed);
void motor2run(int motorSpeed);
void calibrate();
void readLine();
void linefollow();
void updateOLED(const char* status);

void setup() {
  Serial.begin(115200);

  // Set ADC to 12-bit resolution (0 - 4095)
  analogReadResolution(12);

  // Sensor pin configuration & default threshold baselines
  for (int i = 0; i < 8; i++) {
    pinMode(SENSOR_PINS[i], INPUT_ANALOG);
    minValues[i] = 1000;
    maxValues[i] = 3800;
    threshold[i] = 2400;
  }

  // Motor driver pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH); // Enable driver

  // Buttons with internal pullups
  pinMode(BTN_CALIBRATE, INPUT_PULLUP);
  pinMode(BTN_START,     INPUT_PULLUP);
  pinMode(BTN_JOG_REV,   INPUT_PULLUP);
  pinMode(BTN_RESET,     INPUT_PULLUP);

  // Motors initially stopped
  motor1run(0);
  motor2run(0);

  // OLED display init
  u8g2.begin();
  u8g2.setFont(u8g2_font_5x7_tr);
  updateOLED("PC13:CAL | PB5:START");

  Serial.println("\n==================================================");
  Serial.println("  8-Sensor PID Line Follower (LF-2 Style)         ");
  Serial.println("==================================================");
  Serial.println("Controls:");
  Serial.println("  - PC13: Calibrate sensors (8s sweep over line & floor)");
  Serial.println("  - PB5:  START line following");
  Serial.println("  - PB4:  Jog backward (while idle)");
  Serial.println("  - PB3:  RESET / EMERGENCY STOP");
  Serial.println("Layout: S0=LEFT-MOST [PA7] ... S7=RIGHT-MOST [PA0]");
  Serial.println("Target Speed: " + String(lfSpeed) + " | Kp: " + String(Kp, 3) + " | Kd: " + String(Kd, 3));
  Serial.println("==================================================\n");
}

void loop() {
  motor1run(0);
  motor2run(0);
  currentSpeed = 30;

  // Wait / Idle loop: live sensor view, calibration, or start
  while (1) {
    // 1. Check PC13 Calibration
    if (digitalRead(BTN_CALIBRATE) == LOW) {
      delay(200);
      while (digitalRead(BTN_CALIBRATE) == LOW) delay(10);
      calibrate();
    }

    // 2. Check PB5 Start
    if (digitalRead(BTN_START) == LOW) {
      delay(200);
      while (digitalRead(BTN_START) == LOW) delay(10);
      break; // Start line following!
    }

    // 3. PB4 Jog backward bench test
    if (digitalRead(BTN_JOG_REV) == LOW) {
      motor1run(-100);
      motor2run(-100);
    } else {
      motor1run(0);
      motor2run(0);
    }

    // 4. Live sensor reading and telemetry during idle
    readLine();

    static unsigned long lastIdleDisp = 0;
    if (millis() - lastIdleDisp >= 120) {
      lastIdleDisp = millis();
      updateOLED(isCalibrated ? "READY | PB5:START" : "PC13:CAL | PB5:START");
      char logBuf[100];
      snprintf(logBuf, sizeof(logBuf), "IDLE | BIN:[%d%d%d%d%d%d%d%d]",
        sensorArray[0], sensorArray[1], sensorArray[2], sensorArray[3],
        sensorArray[4], sensorArray[5], sensorArray[6], sensorArray[7]);
      Serial.println(logBuf);
    }

    delay(10);
  }

  Serial.println(">>> LINE FOLLOWING STARTED! <<<");
  Serial.println("Press PB3 anytime to STOP / RESET.");
  updateOLED("FOLLOWING LINE");

  P = 0; I = 0; D = 0;
  previousError = 0;

  // Active Line Following Loop
  while (1) {
    // Check PB3 for RESET / STOP
    if (digitalRead(BTN_RESET) == LOW) {
      motor1run(0);
      motor2run(0);
      Serial.println("\n>>> STOPPED / RESET VIA PB3 <<<\n");
      updateOLED("STOPPED / RESET");
      delay(600);
      while (digitalRead(BTN_RESET) == LOW) delay(10);
      break; // Return to idle state
    }

    readLine();

    // Smooth speed ramp up to cruise speed
    if (currentSpeed < lfSpeed) {
      currentSpeed++;
    }

    if (onLine == 1) {
      // PID Line Follow
      linefollow();
    } else {
      // Off line recovery: steer towards last known direction
      if (previousError < 0) {
        // Line lost to right -> turn right
        motor1run(-25);
        motor2run(70);
      } else if (previousError > 0) {
        // Line lost to left -> turn left
        motor1run(70);
        motor2run(-25);
      }
    }

    // Periodic telemetry
    static unsigned long lastDisp = 0;
    if (millis() - lastDisp >= 100) {
      lastDisp = millis();
      updateOLED("FOLLOWING LINE");
      char logBuf[100];
      snprintf(logBuf, sizeof(logBuf), "FOLLOW | BIN:[%d%d%d%d%d%d%d%d] Err:%+d Spd:%d L:%d R:%d",
        sensorArray[0], sensorArray[1], sensorArray[2], sensorArray[3],
        sensorArray[4], sensorArray[5], sensorArray[6], sensorArray[7],
        (int)error, currentSpeed, lsp, rsp);
      Serial.println(logBuf);
    }

    delay(1);
  }
}

// -------- PID Line Following Function -----------------------
void linefollow() {
  error = 0;
  activeSensors = 0;

  for (int i = 0; i < 8; i++) {
    error += (double)sensorWeight[i] * sensorArray[i] * sensorValue[i];
    activeSensors += sensorArray[i];
  }

  if (activeSensors > 0) {
    error = error / activeSensors;
  } else {
    error = previousError;
  }

  P = error;
  I = I + error;
  D = error - previousError;

  // Anti-windup
  if (I > 1000) I = 1000;
  if (I < -1000) I = -1000;

  PIDvalue = (int)((Kp * P) + (Ki * I) + (Kd * D));
  previousError = error;

  lsp = currentSpeed - PIDvalue;
  rsp = currentSpeed + PIDvalue;

  // Constrain speeds with safe limits
  lsp = constrain(lsp, -30, 255);
  rsp = constrain(rsp, -30, 255);

  motor1run(rsp); // Right Motor
  motor2run(lsp); // Left Motor
}

// -------- Calibration Routine (PC13) ------------------------
void calibrate() {
  updateOLED("CALIBRATING...");
  Serial.println("\n>>> CALIBRATING SENSORS (8 Seconds - 2x) <<<");
  Serial.println("Sweep robot across white floor and black line...");

  for (int i = 0; i < 8; i++) {
    uint16_t val = analogRead(SENSOR_PINS[i]);
    minValues[i] = val;
    maxValues[i] = val;
  }

  unsigned long calStart = millis();
  while (millis() - calStart < 8000) {
    // Slowly rotate to scan line automatically
    motor1run(50);
    motor2run(-50);

    for (int i = 0; i < 8; i++) {
      uint16_t v = analogRead(SENSOR_PINS[i]);
      if (v < minValues[i]) minValues[i] = v;
      if (v > maxValues[i]) maxValues[i] = v;
    }
    delay(2);
  }

  motor1run(0);
  motor2run(0);

  Serial.println("Calibration Midpoint Thresholds:");
  for (int i = 0; i < 8; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print("S"); Serial.print(i); Serial.print(":");
    Serial.print(threshold[i]); Serial.print("  ");
  }
  Serial.println("\n>>> CALIBRATION COMPLETE! <<<\n");

  isCalibrated = true;
  delay(500);
}

// -------- Sensor Read & Normalization -----------------------
void readLine() {
  onLine = 0;

  for (int i = 0; i < 8; i++) {
    uint16_t raw = analogRead(SENSOR_PINS[i]);

    // Inversed mapping as requested (map raw from minValues..maxValues to 0..1000)
    if (isBlackLine) {
      sensorValue[i] = map(raw, minValues[i], maxValues[i], 0, 1000);
    } else {
      sensorValue[i] = map(raw, minValues[i], maxValues[i], 1000, 0);
    }

    sensorValue[i] = constrain(sensorValue[i], 0, 1000);
    sensorArray[i] = (sensorValue[i] > 500) ? 1 : 0;

    if (sensorArray[i]) {
      onLine = 1;
    }
  }
}

// -------- Motor 1 (Right Motor) -----------------------------
void motor1run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    // Forward (Polarity aligned with physical chassis)
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, motorSpeed);
  } else if (motorSpeed < 0) {
    // Backward
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, abs(motorSpeed));
  } else {
    // Short brake / Stop
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, 0);
  }
}

// -------- Motor 2 (Left Motor) ------------------------------
void motor2run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    // Forward
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, motorSpeed);
  } else if (motorSpeed < 0) {
    // Backward
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, abs(motorSpeed));
  } else {
    // Short brake / Stop
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, 0);
  }
}

// -------- Helper to draw status & visual bar on OLED --------
void updateOLED(const char* status) {
  u8g2.clearBuffer();
  u8g2.drawStr(0, 8, status);

  char buf[32];
  snprintf(buf, sizeof(buf), "D:%d%d%d%d%d%d%d%d Spd:%d",
    sensorArray[0], sensorArray[1], sensorArray[2], sensorArray[3],
    sensorArray[4], sensorArray[5], sensorArray[6], sensorArray[7],
    currentSpeed);
  u8g2.drawStr(0, 19, buf);

  for (int i = 0; i < 8; i++) {
    int x = i * 16 + 2;
    int y = 22;
    if (sensorArray[i]) {
      u8g2.drawBox(x, y, 12, 10);    // Solid filled block
    } else {
      u8g2.drawFrame(x, y, 12, 10);  // Hollow outline
    }
  }
  u8g2.sendBuffer();
}
