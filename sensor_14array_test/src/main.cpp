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

// -------- 14-Channel MUX Setup ------------------------------
#define MUX_S0   PA7
#define MUX_S1   PA6
#define MUX_S2   PA5
#define MUX_S3   PA4
#define MUX_SIG  PA2   
#define SENSOR_COUNT 14

void setMuxChannel(int logicalChannel) {
  const int muxMap[14] = {0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14}; // Skips I7
  int physicalChannel = muxMap[logicalChannel];
  digitalWrite(MUX_S0, bitRead(physicalChannel, 0));
  digitalWrite(MUX_S1, bitRead(physicalChannel, 1));
  digitalWrite(MUX_S2, bitRead(physicalChannel, 2));
  digitalWrite(MUX_S3, bitRead(physicalChannel, 3));
}

// -------- Fast ADC (PA2 = ADC1_IN3, configured once) --------
// analogRead() re-initialises the ADC on every call (~158 us per read).
// Never call analogRead() on MUX_SIG after this, it would de-init the ADC.
ADC_HandleTypeDef hadcMux;

bool fastAdcInit() {
  hadcMux.Instance = ADC1;
  hadcMux.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadcMux.Init.Resolution = ADC_RESOLUTION_12B;
  hadcMux.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadcMux.Init.GainCompensation = 0;
  hadcMux.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadcMux.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadcMux.Init.LowPowerAutoWait = DISABLE;
  hadcMux.Init.ContinuousConvMode = DISABLE;
  hadcMux.Init.NbrOfConversion = 1;
  hadcMux.Init.DiscontinuousConvMode = DISABLE;
  hadcMux.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadcMux.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadcMux.Init.DMAContinuousRequests = DISABLE;
  hadcMux.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadcMux.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadcMux) != HAL_OK) return false;
  if (HAL_ADCEx_Calibration_Start(&hadcMux, ADC_SINGLE_ENDED) != HAL_OK) return false;

  ADC_ChannelConfTypeDef ch = {};
  ch.Channel = ADC_CHANNEL_3;
  ch.Rank = ADC_REGULAR_RANK_1;
  ch.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  ch.SingleDiff = ADC_SINGLE_ENDED;
  ch.OffsetNumber = ADC_OFFSET_NONE;
  ch.Offset = 0;
  return HAL_ADC_ConfigChannel(&hadcMux, &ch) == HAL_OK;
}

uint16_t fastRead() {
  HAL_ADC_Start(&hadcMux);
  HAL_ADC_PollForConversion(&hadcMux, 1);
  return HAL_ADC_GetValue(&hadcMux);
}
// ------------------------------------------------------------

// -------- OLED Display (SSD1306 via SW I2C on PB6/PB7) ------
#define OLED_SCL PB6
#define OLED_SDA PB7
U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2(U8G2_R0, OLED_SCL, OLED_SDA, U8X8_PIN_NONE);
// ------------------------------------------------------------

// -------- Line Details --------------------------------------
bool isBlackLine = 1;
unsigned int numSensors = 14;
// ------------------------------------------------------------

// -------- Speed & PID Settings ------------------------------
int lfSpeed = 100;            // Target cruise speed (adjustable +/- 5)
int currentSpeed = 30;        // Starting acceleration speed
// Lateral sensor position in mm from the PCB (D1..D14), left = positive.
// Error is therefore the line position in mm: + = line left of centre.
const float sensorX[14] = {
  60.24f, 50.24f, 40.24f, 30.24f, 22.62f, 12.62f, 5.0f,
  -5.0f, -12.62f, -22.62f, -30.24f, -40.24f, -50.24f, -60.24f
};
// D1 (S0) and D9 (S8) read ~0 on black: suspected open pull-up RD1 / RD9.
// Set back to false once repaired.
const bool sensorDead[14] = { false, false, false, false, false, false, false,
                              false, false, false, false, false, false, false };
const int   NOISE_FLOOR = 250;    // per-mille of white..black ignored (outer sensors sit 10-25% above white)
const int   ON_LINE_LEVEL = 400;  // a sensor above this (after floor) means the line is seen
const float LOST_POS = 60.0f;     // error held while the line is lost (mm)
const unsigned long LOOP_US = 1000;  // fixed 1 kHz control loop

float Kp = 6.5f;              // per mm of error (old error was ~217x larger: 0.030 x 217)
float Ki = 0.000f;
float Kd = 43.0f;             // per mm change over 10 ms (0.200 x 217)

float P, D, I, previousError, Dfilt;
int PIDvalue;
float error;
float linePos = 0;            // mm, + = line to the left
int lsp, rsp;
int onLine = 1;
int minValues[14], maxValues[14], threshold[14], sensorValue[14], sensorArray[14];
bool isCalibrated = false;
bool runScreenDrawn = false;

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
void calibrate(bool spin = true);
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

  // Mux Init
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  pinMode(MUX_SIG, INPUT_ANALOG);
  if (!fastAdcInit()) Serial.println("ADC_INIT_FAIL");

  // Default threshold baselines
  for (int i = 0; i < 14; i++) {
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
  Serial.println("Defaults: Speed=" + String(lfSpeed) + " | Kp=" + String(Kp, 1) + " | Kd=" + String(Kd, 0));
  Serial.println("==================================================\n");
}

// -------- Sensor Diagnostics (serial) -----------------------
bool rawStream = false;
bool posStream = false;

// Firmware line position + normalised values: POS,<ms>,<mm|lost>,n0..n13
void streamPos() {
  static unsigned long lastPos = 0;
  if (millis() - lastPos < 20) return;
  lastPos = millis();
  readLine();
  char buf[160];
  int n = onLine ? snprintf(buf, sizeof(buf), "POS,%lu,%.2f", lastPos, linePos)
                 : snprintf(buf, sizeof(buf), "POS,%lu,lost", lastPos);
  for (int i = 0; i < 14; i++) {
    n += snprintf(buf + n, sizeof(buf) - n, ",%d", sensorValue[i]);
  }
  Serial.println(buf);
}

uint16_t readRaw(int ch, unsigned int settleUs) {
  setMuxChannel(ch);
  delayMicroseconds(settleUs);
  return fastRead();
}

// One unfiltered sample per channel as CSV: RAW,<ms>,s0..s13
void streamRaw() {
  static unsigned long lastRaw = 0;
  if (millis() - lastRaw < 20) return;
  lastRaw = millis();
  char buf[128];
  int n = snprintf(buf, sizeof(buf), "RAW,%lu", lastRaw);
  for (int i = 0; i < 14; i++) {
    n += snprintf(buf + n, sizeof(buf) - n, ",%u", readRaw(i, 50));
  }
  Serial.println(buf);
}

// Per-channel mean / std / peak-to-peak for several MUX settle times.
// Robot must stay still; put some sensors on the line to expose crosstalk.
void noiseTest() {
  const unsigned int settles[] = {0, 1, 2, 5, 10, 20, 50};
  const int FRAMES = 200;

  unsigned long t0 = micros();
  for (int k = 0; k < 100; k++) fastRead();
  Serial.print("ADC_US,"); Serial.println((micros() - t0) / 100.0f, 2);

  for (unsigned int s : settles) {
    float sum[14] = {0}, sumSq[14] = {0};
    uint16_t lo[14], hi[14];
    for (int i = 0; i < 14; i++) { lo[i] = 4095; hi[i] = 0; }

    unsigned long f0 = micros();
    for (int f = 0; f < FRAMES; f++) {
      for (int i = 0; i < 14; i++) {
        uint16_t v = readRaw(i, s);
        sum[i] += v;
        sumSq[i] += (float)v * v;
        if (v < lo[i]) lo[i] = v;
        if (v > hi[i]) hi[i] = v;
      }
    }
    float frameUs = (micros() - f0) / (float)FRAMES;

    Serial.print("FRAME_US,"); Serial.print(s); Serial.print(","); Serial.println(frameUs, 1);
    for (int i = 0; i < 14; i++) {
      float mean = sum[i] / FRAMES;
      float var = sumSq[i] / FRAMES - mean * mean;
      char buf[80];
      snprintf(buf, sizeof(buf), "NOISE,%u,%d,%.1f,%.2f,%u", s, i, mean, sqrtf(var > 0 ? var : 0), hi[i] - lo[i]);
      Serial.println(buf);
    }
  }
  Serial.println("NOISE_DONE");
}

void printCalibration() {
  for (int i = 0; i < 14; i++) {
    char buf[64];
    snprintf(buf, sizeof(buf), "CAL,%d,%d,%d,%d,%d", i, minValues[i], maxValues[i], maxValues[i] - minValues[i], threshold[i]);
    Serial.println(buf);
  }
  Serial.println("CAL_DONE");
}

void loop() {
  // Check for any serial commands
  handleSerial();
  if (rawStream) streamRaw();
  if (posStream) streamPos();

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
          P = 0; I = 0; D = 0; Dfilt = 0; runScreenDrawn = false;
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
          Kp = constrain(Kp + 0.1f, 0.0f, 50.0f);
          Kp = roundf(Kp * 10.0f) / 10.0f;
        } else if (editParamIndex == MENU_KD) {
          Kd = constrain(Kd + 1.0f, 0.0f, 300.0f);
          Kd = roundf(Kd);
        }
        drawEditParam();
      }

      // PB4: Decrease value
      if (checkButtonRepeat(BTN_DOWN, downHoldTimer, downRepeatTimer)) {
        if (editParamIndex == MENU_SPEED) {
          lfSpeed = constrain(lfSpeed - 5, 30, 220);
        } else if (editParamIndex == MENU_KP) {
          Kp = constrain(Kp - 0.1f, 0.0f, 50.0f);
          Kp = roundf(Kp * 10.0f) / 10.0f;
        } else if (editParamIndex == MENU_KD) {
          Kd = constrain(Kd - 1.0f, 0.0f, 300.0f);
          Kd = roundf(Kd);
        }
        drawEditParam();
      }

      // PC13 (ENTER) or PB5 (BACK): Confirm and return to Menu
      if (checkButtonSingle(BTN_ENTER, enterHoldTimer) || checkButtonSingle(BTN_BACK, backHoldTimer)) {
        Serial.println("[TUNE] Saved: Speed=" + String(lfSpeed) + " | Kp=" + String(Kp, 1) + " | Kd=" + String(Kd, 0));
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

      // The SW-I2C OLED takes tens of ms per frame, so draw it only once per run.
      if (!runScreenDrawn) {
        drawRunning();
        runScreenDrawn = true;
      }

      // Fixed-rate control loop so the D term is consistent
      static unsigned long lastTick = 0;
      if (micros() - lastTick < LOOP_US) break;
      lastTick = micros();

      readLine();

      // Smooth acceleration ramp up (+1 every 4 ms)
      static uint8_t rampTicks = 0;
      if (currentSpeed < lfSpeed && ++rampTicks >= 4) {
        rampTicks = 0;
        currentSpeed++;
      }

      if (onLine == 1) {
        linefollow();
      } else {
        // Off-line recovery in last known direction
        error = linePos;
        if (previousError < 0) {
          lsp = 70; rsp = -25;
        } else if (previousError > 0) {
          lsp = -25; rsp = 70;
        }
        motor1run(rsp);
        motor2run(lsp);
      }

      // Periodic live telemetry
      static unsigned long lastRunDisp = 0;
      if (millis() - lastRunDisp >= 100) {
        lastRunDisp = millis();
        char logBuf[120];
        snprintf(logBuf, sizeof(logBuf), "FOLLOW | BIN:[%d%d%d%d%d%d%d%d%d%d%d%d%d%d] Err:%+.1f Spd:%d L:%d R:%d",
          sensorArray[0], sensorArray[1], sensorArray[2], sensorArray[3],
          sensorArray[4], sensorArray[5], sensorArray[6], sensorArray[7],
          sensorArray[8], sensorArray[9], sensorArray[10], sensorArray[11],
          sensorArray[12], sensorArray[13], error, currentSpeed, lsp, rsp);
        Serial.println(logBuf);
      }
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
        char logBuf[120];
        snprintf(logBuf, sizeof(logBuf), "TEST | BIN:[%d%d%d%d%d%d%d%d%d%d%d%d%d%d]",
          sensorArray[0], sensorArray[1], sensorArray[2], sensorArray[3],
          sensorArray[4], sensorArray[5], sensorArray[6], sensorArray[7],
          sensorArray[8], sensorArray[9], sensorArray[10], sensorArray[11],
          sensorArray[12], sensorArray[13]);
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
  error = linePos;

  P = error;
  I = I + error;
  // Change in mm per 10 ms, independent of LOOP_US, lightly low-passed
  D = (error - previousError) * (10000.0f / LOOP_US);
  Dfilt = 0.7f * Dfilt + 0.3f * D;

  // Anti-windup
  if (I > 1000) I = 1000;
  if (I < -1000) I = -1000;

  PIDvalue = (int)((Kp * P) + (Ki * I) + (Kd * Dfilt));
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
// spin = false: motors stay off, slide the robot across the line by hand.
void calibrate(bool spin) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 10, "CALIBRATING (8s)...");
  u8g2.drawStr(0, 24, spin ? "Rotating on track" : "Slide across line");
  u8g2.sendBuffer();

  Serial.println("\n>>> CALIBRATING SENSORS (8 Seconds) <<<");
  Serial.println("Sweep robot across white floor and black line...");

  for (int i = 0; i < 14; i++) {
    setMuxChannel(i);
    delayMicroseconds(50);
    uint16_t val = fastRead();
    minValues[i] = val;
    maxValues[i] = val;
  }

  unsigned long calStart = millis();
  while (millis() - calStart < 8000) {
    // Slowly rotate to scan line automatically
    if (spin) {
      motor1run(50);
      motor2run(-50);
    }

    for (int i = 0; i < 14; i++) {
      setMuxChannel(i);
      delayMicroseconds(50);
      uint16_t v = fastRead();
      if (v < minValues[i]) minValues[i] = v;
      if (v > maxValues[i]) maxValues[i] = v;
    }
    delay(2);
  }

  motor1run(0);
  motor2run(0);

  Serial.println("Calibration Midpoint Thresholds:");
  for (int i = 0; i < 14; i++) {
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
// Reads all sensors (~0.4 ms), normalises each to 0..1000 with the noise floor
// removed, and sets linePos (mm, weighted average) and onLine.
void readLine() {
  onLine = 0;
  float sumW = 0, sumWX = 0;

  for (int i = 0; i < 14; i++) {
    setMuxChannel(i);
    delayMicroseconds(10);  // settles to ~1-3 counts (measured)
    uint16_t raw = (fastRead() + fastRead()) / 2;

    if (sensorDead[i]) {
      sensorValue[i] = 0;
      sensorArray[i] = 0;
      continue;
    }

    int v = isBlackLine ? map(raw, minValues[i], maxValues[i], 0, 1000)
                        : map(raw, minValues[i], maxValues[i], 1000, 0);
    v = constrain(v, 0, 1000);
    v = constrain((v - NOISE_FLOOR) * 1000 / (1000 - NOISE_FLOOR), 0, 1000);

    sensorValue[i] = v;
    sensorArray[i] = (v > ON_LINE_LEVEL) ? 1 : 0;
    if (sensorArray[i]) onLine = 1;

    sumW += v;
    sumWX += v * sensorX[i];
  }

  if (onLine) {
    linePos = sumWX / sumW;
  } else {
    // Hold full deflection on the side the line was last seen
    linePos = (previousError >= 0) ? LOST_POS : -LOST_POS;
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
      snprintf(lineBuf, sizeof(lineBuf), "%c 4.Kp: %s", prefix, String(Kp, 1).c_str());
    } else if (idx == MENU_KD) {
      snprintf(lineBuf, sizeof(lineBuf), "%c 5.Kd: %s", prefix, String(Kd, 0).c_str());
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
    snprintf(titleBuf, sizeof(titleBuf), "TUNE Kp (+/-0.1)");
    snprintf(valBuf, sizeof(valBuf), "<  %s  >", String(Kp, 1).c_str());
  } else if (editParamIndex == MENU_KD) {
    snprintf(titleBuf, sizeof(titleBuf), "TUNE Kd (+/-1)");
    snprintf(valBuf, sizeof(valBuf), "<  %s  >", String(Kd, 0).c_str());
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

  for (int i = 0; i < 14; i++) {
    int x = i * 9 + 1;
    int y = 22;
    if (sensorArray[i]) {
      u8g2.drawBox(x, y, 7, 10);
    } else {
      u8g2.drawFrame(x, y, 7, 10);
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

  for (int i = 0; i < 14; i++) {
    int x = i * 9 + 1;
    int y = 22;
    if (sensorArray[i]) {
      u8g2.drawBox(x, y, 7, 10);
    } else {
      u8g2.drawFrame(x, y, 7, 10);
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
      P = 0; I = 0; D = 0; Dfilt = 0; runScreenDrawn = false;
      previousError = 0;
      Serial.println("\n>>> RUN STARTED (via Serial) <<<");
    } else if (c == 'x' || c == 'X' || c == ' ') {
      motor1run(0);
      motor2run(0);
      appState = STATE_MENU;
      drawMenu();
      Serial.println("\n>>> STOPPED / MENU (via Serial) <<<");
    } else if (c == 'a') {
      rawStream = !rawStream;
    } else if (c == 'l') {
      posStream = !posStream;
    } else if (c == 'm') {
      calibrate(false);
      appState = STATE_MENU;
      drawMenu();
    } else if (c == 'n') {
      noiseTest();
    } else if (c == 'k') {
      printCalibration();
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
      Kp = constrain(Kp + 0.1f, 0.0f, 50.0f);
      Kp = roundf(Kp * 10.0f) / 10.0f;
      Serial.println("Kp: " + String(Kp, 1));
      drawMenu();
    } else if (c == 'P') {
      Kp = constrain(Kp - 0.1f, 0.0f, 50.0f);
      Kp = roundf(Kp * 10.0f) / 10.0f;
      Serial.println("Kp: " + String(Kp, 1));
      drawMenu();
    } else if (c == 'd') {
      Kd = constrain(Kd + 1.0f, 0.0f, 300.0f);
      Kd = roundf(Kd);
      Serial.println("Kd: " + String(Kd, 0));
      drawMenu();
    } else if (c == 'D') {
      Kd = constrain(Kd - 1.0f, 0.0f, 300.0f);
      Kd = roundf(Kd);
      Serial.println("Kd: " + String(Kd, 0));
      drawMenu();
    }
  }
}
