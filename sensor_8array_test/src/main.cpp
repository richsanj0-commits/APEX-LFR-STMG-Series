#include <Arduino.h>
#include <U8g2lib.h>

/*
  ============================================================
  Simple Autonomous 8-Channel Line Follower Robot (LF-2 Style)
  Target MCU: STM32G431CB (170MHz)
  Controls:
    - PC13: Calibrate
    - PB5:  Start line following
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
#define BTN_RESET     PB3    // PB3 for reset / stop
// ------------------------------------------------------------

// -------- 8-Channel Analog IR Sensors (Port A) --------------
const int SENSOR_PINS[8] = { PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7 };
// ------------------------------------------------------------

// -------- OLED Display (SSD1306 via SW I2C on PB6/PB7) ------
#define OLED_SCL PB6
#define OLED_SDA PB7
U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2(U8G2_R0, OLED_SCL, OLED_SDA, U8X8_PIN_NONE);
// ------------------------------------------------------------

// -------- Line Details --------------------------------------
bool isBlackLine = 1;         // 1 = Black line, 0 = White line
unsigned int numSensors = 8;
// ------------------------------------------------------------

// -------- Speed & PID Settings (Reduced Safe Speed) ---------
int lfSpeed = 70;             // Target cruise speed (reduced for smooth following)
int currentSpeed = 30;        // Starting acceleration speed
int sensorWeight[8] = { 8, 4, 2, 1, -1, -2, -4, -8 };

float Kp = 0.045f;
float Ki = 0.000f;
float Kd = 0.280f;

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

  // Sensor pin configuration
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
  pinMode(BTN_RESET,     INPUT_PULLUP);

  // Motors initially stopped
  motor1run(0);
  motor2run(0);

  // OLED display init
  u8g2.begin();
  u8g2.setFont(u8g2_font_5x7_tr);
  updateOLED("PC13:CAL | PB5:START");

  Serial.println("\n==================================================");
  Serial.println("  8-Sensor PID Line Follower (LF-2 Simplified)    ");
  Serial.println("==================================================");
  Serial.println("Controls:");
  Serial.println("  - PC13: Calibrate sensors (sweep over black/white)");
  Serial.println("  - PB5:  START line following");
  Serial.println("  - PB3:  RESET / STOP");
  Serial.println("Target Cruise Speed: " + String(lfSpeed));
  Serial.println("==================================================\n");
}

void loop() {
  motor1run(0);
  motor2run(0);
  currentSpeed = 30;

  updateOLED(isCalibrated ? "READY | PB5:START" : "PC13:CAL | PB5:START");

  // Wait for user action: PC13 to calibrate or PB5 to start
  while (1) {
    if (digitalRead(BTN_CALIBRATE) == LOW) {
      delay(200);
      while (digitalRead(BTN_CALIBRATE) == LOW) delay(10);
      calibrate();
      updateOLED("READY | PB5:START");
    }

    if (digitalRead(BTN_START) == LOW) {
      delay(200);
      while (digitalRead(BTN_START) == LOW) delay(10);
      break; // Exit wait loop and start running!
    }
    delay(20);
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
      break; // Returns to main setup/wait state
    }

    readLine();

    // Smooth speed ramp up to safe cruise speed
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

    // Telemetry display periodically
    static unsigned long lastDisp = 0;
    if (millis() - lastDisp >= 120) {
      lastDisp = millis();
      updateOLED("FOLLOWING LINE");
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
  Serial.println("\n>>> CALIBRATING SENSORS (4 Seconds) <<<");
  Serial.println("Sweep robot across white floor and black line...");

  for (int i = 0; i < 8; i++) {
    uint16_t val = analogRead(SENSOR_PINS[i]);
    minValues[i] = val;
    maxValues[i] = val;
  }

  unsigned long calStart = millis();
  while (millis() - calStart < 4000) {
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

    // On this sensor array:
    // Black Line = Low ADC reading (near minValues)
    // White Floor = High ADC reading (near maxValues)
    if (isBlackLine) {
      sensorValue[i] = map(raw, minValues[i], maxValues[i], 1000, 0);
    } else {
      sensorValue[i] = map(raw, minValues[i], maxValues[i], 0, 1000);
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
  snprintf(buf, sizeof(buf), "Spd:%d  Err:%+d", currentSpeed, (int)error);
  u8g2.drawStr(0, 19, buf);

  for (int i = 0; i < 8; i++) {
    int x = i * 16 + 2;
    int y = 22;
    if (sensorArray[i]) {
      u8g2.drawBox(x, y, 12, 10);    // Solid block for Black line
    } else {
      u8g2.drawFrame(x, y, 12, 10);  // Hollow outline for White
    }
  }
  u8g2.sendBuffer();
}
