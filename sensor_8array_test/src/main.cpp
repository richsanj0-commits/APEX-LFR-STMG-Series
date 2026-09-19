#include <Arduino.h>
#include <U8g2lib.h>

/*
  ============================================================
  Interactive Menu & PID Line Follower Robot (LF-2 Style)
  Target MCU: STM32G431CB (170MHz)
  
  4-Button Controls:
    - PC13: ENTER / SELECT (Enter submenu / Confirm edit / Run)
    - PB5:  BACK / EXIT    (Exit submenu / Emergency Stop)
    - PB3:  UP   / (+)     (Navigate Up / Increase value)
    - PB4:  DOWN / (-)     (Navigate Down / Decrease value)
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
#define BTN_ENTER PC13   // PC13: Enter / Select
#define BTN_BACK  PB5    // PB5:  Back / Exit / Stop
#define BTN_UP    PB3    // PB3:  Up / Increase (+)
#define BTN_DOWN  PB4    // PB4:  Down / Decrease (-)
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

// -------- Speed & PID Settings ------------------------------
int lfSpeed = 100;            // Target cruise speed (adjustable +/- 5)
int currentSpeed = 30;        // Starting acceleration speed
// S0 (Left: +8) ... S7 (Right: -8)
int sensorWeight[8] = { 8, 4, 2, 1, -1, -2, -4, -8 };

float Kp = 0.030f;            // Proportional gain (adjustable +/- 0.01)
float Ki = 0.000f;
float Kd = 0.200f;            // Derivative gain (adjustable +/- 0.1)

int P, D, I, previousError, PIDvalue;
double error;
int lsp, rsp;
int activeSensors;
int onLine = 1;
int minValues[8], maxValues[8], threshold[8], sensorValue[8], sensorArray[8];
bool isCalibrated = false;

// -------- Menu States & Navigation --------------------------
enum AppState {
  STATE_MENU,
  STATE_EDIT_PARAM,
  STATE_RUNNING,
  STATE_CALIBRATING,
  STATE_SENSOR_TEST
};
AppState appState = STATE_MENU;

enum MenuItem {
  MENU_RUN,
  MENU_CALIBRATE,
  MENU_SPEED,
  MENU_KP,
  MENU_KD,
  MENU_SENSORS,
  MENU_ITEM_COUNT
};
int currentMenuItem = MENU_RUN;
int editParamIndex = MENU_SPEED;

// Button timing variables
unsigned long upHoldTimer = 0, upRepeatTimer = 0;
unsigned long downHoldTimer = 0, downRepeatTimer = 0;
unsigned long enterHoldTimer = 0;
unsigned long backHoldTimer = 0;

// Function prototypes
void motor1run(int motorSpeed);
void motor2run(int motorSpeed);
void calibrate();
void readLine();
void linefollow();
void drawMenu();
void drawEditParam();
void drawRunning();
void drawSensorTest();
bool checkButtonSingle(int pin, unsigned long &holdTimer);
bool checkButtonRepeat(int pin, unsigned long &holdTimer, unsigned long &repeatTimer);
void handleSerial();

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

  // 4 Pushbuttons with internal pullups
  pinMode(BTN_ENTER, INPUT_PULLUP);
  pinMode(BTN_BACK,  INPUT_PULLUP);
  pinMode(BTN_UP,    INPUT_PULLUP);
  pinMode(BTN_DOWN,  INPUT_PULLUP);

  // Motors initially stopped
  motor1run(0);
  motor2run(0);

  // OLED display init
  u8g2.begin();
  drawMenu();

  Serial.println("\n==================================================");
  Serial.println("   APEX LFR - Interactive Menu & Line Follower    ");
  Serial.println("==================================================");
  Serial.println("Controls:");
  Serial.println("  - PC13: ENTER / SELECT (Confirm / Run / Calibrate)");
  Serial.println("  - PB5:  BACK / EXIT    (Back to menu / Stop motors)");
  Serial.println("  - PB3:  UP   / (+)     (Navigate Up / Speed+5 / PID+)");
  Serial.println("  - PB4:  DOWN / (-)     (Navigate Down / Speed-5 / PID-)");
  Serial.println("Defaults: Speed=" + String(lfSpeed) + " | Kp=" + String(Kp, 3) + " | Kd=" + String(Kd, 3));
  Serial.println("==================================================\n");
}

void loop() {
  // Check for any serial commands
  handleSerial();

  // State Machine Handling
  switch (appState) {

    // --------------------------------------------------------
    // 1. MENU NAVIGATION STATE
    // --------------------------------------------------------
    case STATE_MENU: {
      motor1run(0);
      motor2run(0);

      // PB3: Navigate UP
      if (checkButtonRepeat(BTN_UP, upHoldTimer, upRepeatTimer)) {
        currentMenuItem = (currentMenuItem - 1 + MENU_ITEM_COUNT) % MENU_ITEM_COUNT;
        drawMenu();
      }

      // PB4: Navigate DOWN
      if (checkButtonRepeat(BTN_DOWN, downHoldTimer, downRepeatTimer)) {
        currentMenuItem = (currentMenuItem + 1) % MENU_ITEM_COUNT;
        drawMenu();
      }

      // PC13: ENTER / SELECT
      if (checkButtonSingle(BTN_ENTER, enterHoldTimer)) {
        if (currentMenuItem == MENU_RUN) {
          Serial.println("\n>>> [MENU] STARTING LINE FOLLOWER <<<");
          appState = STATE_RUNNING;
          currentSpeed = 30;
          P = 0; I = 0; D = 0;
          previousError = 0;
        } else if (currentMenuItem == MENU_CALIBRATE) {
          appState = STATE_CALIBRATING;
          calibrate();
          appState = STATE_MENU;
          drawMenu();
        } else if (currentMenuItem == MENU_SPEED || currentMenuItem == MENU_KP || currentMenuItem == MENU_KD) {
          editParamIndex = currentMenuItem;
          appState = STATE_EDIT_PARAM;
          drawEditParam();
        } else if (currentMenuItem == MENU_SENSORS) {
          appState = STATE_SENSOR_TEST;
        }
      }

      // Refresh display periodically
      static unsigned long lastMenuRefresh = 0;
      if (millis() - lastMenuRefresh >= 150) {
        lastMenuRefresh = millis();
        drawMenu();
      }
      break;
    }

    // --------------------------------------------------------
    // 2. PARAMETER EDIT STATE (Speed, Kp, Kd)
    // --------------------------------------------------------
    case STATE_EDIT_PARAM: {
      motor1run(0);
      motor2run(0);

      // PB3: Increase value
      if (checkButtonRepeat(BTN_UP, upHoldTimer, upRepeatTimer)) {
        if (editParamIndex == MENU_SPEED) {
          lfSpeed = constrain(lfSpeed + 5, 30, 220);
        } else if (editParamIndex == MENU_KP) {
          Kp = constrain(Kp + 0.010f, 0.000f, 0.500f);
          Kp = roundf(Kp * 1000.0f) / 1000.0f;
        } else if (editParamIndex == MENU_KD) {
          Kd = constrain(Kd + 0.100f, 0.000f, 2.000f);
          Kd = roundf(Kd * 1000.0f) / 1000.0f;
        }
        drawEditParam();
      }

      // PB4: Decrease value
      if (checkButtonRepeat(BTN_DOWN, downHoldTimer, downRepeatTimer)) {
        if (editParamIndex == MENU_SPEED) {
          lfSpeed = constrain(lfSpeed - 5, 30, 220);
        } else if (editParamIndex == MENU_KP) {
          Kp = constrain(Kp - 0.010f, 0.000f, 0.500f);
          Kp = roundf(Kp * 1000.0f) / 1000.0f;
        } else if (editParamIndex == MENU_KD) {
          Kd = constrain(Kd - 0.100f, 0.000f, 2.000f);
          Kd = roundf(Kd * 1000.0f) / 1000.0f;
        }
        drawEditParam();
      }

      // PC13 (ENTER) or PB5 (BACK): Confirm and return to Menu
      if (checkButtonSingle(BTN_ENTER, enterHoldTimer) || checkButtonSingle(BTN_BACK, backHoldTimer)) {
        Serial.println("[TUNE] Saved: Speed=" + String(lfSpeed) + " | Kp=" + String(Kp, 3) + " | Kd=" + String(Kd, 3));
        appState = STATE_MENU;
        drawMenu();
      }

      static unsigned long lastEditRefresh = 0;
      if (millis() - lastEditRefresh >= 150) {
        lastEditRefresh = millis();
        drawEditParam();
      }
      break;
    }

    // --------------------------------------------------------
    // 3. RUNNING STATE (Active PID Line Following)
    // --------------------------------------------------------
    case STATE_RUNNING: {
      // Emergency STOP / Return to Menu via PB5 (BACK) or PC13
      if (checkButtonSingle(BTN_BACK, backHoldTimer) || checkButtonSingle(BTN_ENTER, enterHoldTimer)) {
        motor1run(0);
        motor2run(0);
        Serial.println("\n>>> [STOPPED] RETURNED TO MENU <<<\n");
        appState = STATE_MENU;
        drawMenu();
        delay(200);
        break;
      }

      readLine();

      // Smooth acceleration ramp up
      if (currentSpeed < lfSpeed) {
        currentSpeed++;
      }

      if (onLine == 1) {
        linefollow();
      } else {
        // Off-line recovery in last known direction
        if (previousError < 0) {
          motor1run(-25);
          motor2run(70);
        } else if (previousError > 0) {
          motor1run(70);
          motor2run(-25);
        }
      }

      // Periodic live telemetry
      static unsigned long lastRunDisp = 0;
      if (millis() - lastRunDisp >= 100) {
        lastRunDisp = millis();
        drawRunning();
        char logBuf[100];
        snprintf(logBuf, sizeof(logBuf), "FOLLOW | BIN:[%d%d%d%d%d%d%d%d] Err:%+d Spd:%d L:%d R:%d",
          sensorArray[0], sensorArray[1], sensorArray[2], sensorArray[3],
          sensorArray[4], sensorArray[5], sensorArray[6], sensorArray[7],
          (int)error, currentSpeed, lsp, rsp);
        Serial.println(logBuf);
      }

      delay(1);
      break;
    }

    // --------------------------------------------------------
    // 4. SENSOR TEST VIEW STATE
    // --------------------------------------------------------
    case STATE_SENSOR_TEST: {
      // Exit test mode via PB5 (BACK) or PC13
      if (checkButtonSingle(BTN_BACK, backHoldTimer) || checkButtonSingle(BTN_ENTER, enterHoldTimer)) {
        motor1run(0);
        motor2run(0);
        appState = STATE_MENU;
        drawMenu();
        delay(200);
        break;
      }

      // Optional jog backward bench test with PB4
      if (digitalRead(BTN_DOWN) == LOW) {
        motor1run(-100);
        motor2run(-100);
      } else {
        motor1run(0);
        motor2run(0);
      }

      readLine();

      static unsigned long lastTestDisp = 0;
      if (millis() - lastTestDisp >= 120) {
        lastTestDisp = millis();
        drawSensorTest();
        char logBuf[100];
        snprintf(logBuf, sizeof(logBuf), "TEST | BIN:[%d%d%d%d%d%d%d%d]",
          sensorArray[0], sensorArray[1], sensorArray[2], sensorArray[3],
          sensorArray[4], sensorArray[5], sensorArray[6], sensorArray[7]);
        Serial.println(logBuf);
      }

      delay(10);
      break;
    }

    case STATE_CALIBRATING:
      break;
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

// -------- Calibration Routine -------------------------------
void calibrate() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 10, "CALIBRATING (8s)...");
  u8g2.drawStr(0, 24, "Rotating on track");
  u8g2.sendBuffer();

  Serial.println("\n>>> CALIBRATING SENSORS (8 Seconds) <<<");
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

  u8g2.clearBuffer();
  u8g2.drawStr(0, 15, "CALIBRATION DONE!");
  u8g2.drawStr(0, 28, "Returning to menu...");
  u8g2.sendBuffer();
  delay(800);
}

// -------- Sensor Read & Normalization -----------------------
void readLine() {
  onLine = 0;

  for (int i = 0; i < 8; i++) {
    uint16_t raw = analogRead(SENSOR_PINS[i]);

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

// -------- Motor Control Functions ---------------------------
void motor1run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, abs(motorSpeed));
  } else {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, 0);
  }
}

void motor2run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, abs(motorSpeed));
  } else {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, 0);
  }
}

// -------- OLED UI Functions ---------------------------------
void drawMenu() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tr);

  // Top Title Bar
  u8g2.drawStr(0, 9, "MENU | C13:OK B5:ESC");

  // Show 2 items centered on current selection
  int firstVisible = currentMenuItem;
  if (firstVisible > MENU_ITEM_COUNT - 2) {
    firstVisible = MENU_ITEM_COUNT - 2;
  }
  if (firstVisible < 0) firstVisible = 0;

  for (int i = 0; i < 2; i++) {
    int idx = firstVisible + i;
    if (idx >= MENU_ITEM_COUNT) break;

    char lineBuf[32];
    char prefix = (idx == currentMenuItem) ? '>' : ' ';

    if (idx == MENU_RUN) {
      snprintf(lineBuf, sizeof(lineBuf), "%c 1.RUN FOLLOWER", prefix);
    } else if (idx == MENU_CALIBRATE) {
      snprintf(lineBuf, sizeof(lineBuf), "%c 2.CALIBRATE (8s)", prefix);
    } else if (idx == MENU_SPEED) {
      snprintf(lineBuf, sizeof(lineBuf), "%c 3.SPEED: %d", prefix, lfSpeed);
    } else if (idx == MENU_KP) {
      snprintf(lineBuf, sizeof(lineBuf), "%c 4.Kp: %s", prefix, String(Kp, 3).c_str());
    } else if (idx == MENU_KD) {
      snprintf(lineBuf, sizeof(lineBuf), "%c 5.Kd: %s", prefix, String(Kd, 3).c_str());
    } else if (idx == MENU_SENSORS) {
      snprintf(lineBuf, sizeof(lineBuf), "%c 6.SENSOR TEST", prefix);
    }

    u8g2.drawStr(0, 20 + (i * 11), lineBuf);
  }

  u8g2.sendBuffer();
}

void drawEditParam() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tr);

  char titleBuf[32];
  char valBuf[32];

  if (editParamIndex == MENU_SPEED) {
    snprintf(titleBuf, sizeof(titleBuf), "TUNE SPEED (+/-5)");
    snprintf(valBuf, sizeof(valBuf), "<  %d  >", lfSpeed);
  } else if (editParamIndex == MENU_KP) {
    snprintf(titleBuf, sizeof(titleBuf), "TUNE Kp (+/-0.01)");
    snprintf(valBuf, sizeof(valBuf), "<  %s  >", String(Kp, 3).c_str());
  } else if (editParamIndex == MENU_KD) {
    snprintf(titleBuf, sizeof(titleBuf), "TUNE Kd (+/-0.1)");
    snprintf(valBuf, sizeof(valBuf), "<  %s  >", String(Kd, 3).c_str());
  }

  u8g2.drawStr(0, 9, titleBuf);
  u8g2.drawStr(24, 20, valBuf);
  u8g2.drawStr(0, 31, "B3:+ B4:- C13:OK");

  u8g2.sendBuffer();
}

void drawRunning() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x7_tr);

  char line1[32];
  snprintf(line1, sizeof(line1), "RUNNING | Spd:%d", currentSpeed);
  u8g2.drawStr(0, 8, line1);

  char line2[32];
  snprintf(line2, sizeof(line2), "Err:%+d L:%d R:%d", (int)error, lsp, rsp);
  u8g2.drawStr(0, 19, line2);

  for (int i = 0; i < 8; i++) {
    int x = i * 16 + 2;
    int y = 22;
    if (sensorArray[i]) {
      u8g2.drawBox(x, y, 12, 10);
    } else {
      u8g2.drawFrame(x, y, 12, 10);
    }
  }
  u8g2.sendBuffer();
}

void drawSensorTest() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x7_tr);

  u8g2.drawStr(0, 8, "TEST | B5:EXIT B4:JOG");

  char binStr[32];
  snprintf(binStr, sizeof(binStr), "D: %d %d %d %d %d %d %d %d",
    sensorArray[0], sensorArray[1], sensorArray[2], sensorArray[3],
    sensorArray[4], sensorArray[5], sensorArray[6], sensorArray[7]);
  u8g2.drawStr(0, 19, binStr);

  for (int i = 0; i < 8; i++) {
    int x = i * 16 + 2;
    int y = 22;
    if (sensorArray[i]) {
      u8g2.drawBox(x, y, 12, 10);
    } else {
      u8g2.drawFrame(x, y, 12, 10);
    }
  }
  u8g2.sendBuffer();
}

// -------- Button Handling Helpers ---------------------------
bool checkButtonSingle(int pin, unsigned long &holdTimer) {
  if (digitalRead(pin) == LOW) {
    if (holdTimer == 0) {
      holdTimer = millis();
      return true; // Click on leading edge
    }
  } else {
    holdTimer = 0;
  }
  return false;
}

bool checkButtonRepeat(int pin, unsigned long &holdTimer, unsigned long &repeatTimer) {
  if (digitalRead(pin) == LOW) {
    if (holdTimer == 0) {
      holdTimer = millis();
      repeatTimer = millis();
      return true; // Click on leading edge
    }
    // Auto-repeat when held > 350ms
    if (millis() - holdTimer > 350) {
      if (millis() - repeatTimer > 100) {
        repeatTimer = millis();
        return true;
      }
    }
  } else {
    holdTimer = 0;
  }
  return false;
}

// -------- Optional Serial Command Handler -------------------
void handleSerial() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'r' || c == 's' || c == 'R' || c == 'S') {
      appState = STATE_RUNNING;
      currentSpeed = 30;
      P = 0; I = 0; D = 0;
      previousError = 0;
      Serial.println("\n>>> RUN STARTED (via Serial) <<<");
    } else if (c == 'x' || c == 'X' || c == ' ') {
      motor1run(0);
      motor2run(0);
      appState = STATE_MENU;
      drawMenu();
      Serial.println("\n>>> STOPPED / MENU (via Serial) <<<");
    } else if (c == 'c' || c == 'C') {
      calibrate();
      appState = STATE_MENU;
      drawMenu();
    } else if (c == '+') {
      lfSpeed = constrain(lfSpeed + 5, 30, 220);
      Serial.println("Speed: " + String(lfSpeed));
      drawMenu();
    } else if (c == '-') {
      lfSpeed = constrain(lfSpeed - 5, 30, 220);
      Serial.println("Speed: " + String(lfSpeed));
      drawMenu();
    } else if (c == 'p') {
      Kp = constrain(Kp + 0.010f, 0.000f, 0.500f);
      Kp = roundf(Kp * 1000.0f) / 1000.0f;
      Serial.println("Kp: " + String(Kp, 3));
      drawMenu();
    } else if (c == 'P') {
      Kp = constrain(Kp - 0.010f, 0.000f, 0.500f);
      Kp = roundf(Kp * 1000.0f) / 1000.0f;
      Serial.println("Kp: " + String(Kp, 3));
      drawMenu();
    } else if (c == 'd') {
      Kd = constrain(Kd + 0.100f, 0.000f, 2.000f);
      Kd = roundf(Kd * 1000.0f) / 1000.0f;
      Serial.println("Kd: " + String(Kd, 3));
      drawMenu();
    } else if (c == 'D') {
      Kd = constrain(Kd - 0.100f, 0.000f, 2.000f);
      Kd = roundf(Kd * 1000.0f) / 1000.0f;
      Serial.println("Kd: " + String(Kd, 3));
      drawMenu();
    }
  }
}
