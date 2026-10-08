#include <Arduino.h>
#include <U8g2lib.h>

/*
  ============================================================
  APEX LFR - 14-Sensor Analog PID Line Follower
  Target MCU: STM32G431CB (170MHz)

  Line position is measured in mm (weighted average of the
  calibrated analog sensor values), updated at a fixed 1 kHz.
  + error = line is LEFT of centre.

  4-Button Controls:
    - PC13: ENTER / SELECT (Enter submenu / Confirm edit / Run)
    - PB5:  BACK / EXIT    (Exit submenu / Emergency Stop)
    - PB3:  UP   / (+)     (Navigate Up / Increase value)
    - PB4:  DOWN / (-)     (Navigate Down / Decrease value)

  Serial (115200): r=run x=stop c=calibrate(spin) m=calibrate(by hand)
                   +/-=speed p/P=Kp d/D=Kd k=print calibration
                   l=toggle position stream
                   b=cycle branch priority (LEFT/RIGHT/NEAREST)
                   w=cycle line colour (AUTO/BLACK/WHITE)
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

// -------- Pushbuttons (Active LOW with internal pull-up) -----
#define BTN_ENTER PC13
#define BTN_BACK  PB5
#define BTN_UP    PB3
#define BTN_DOWN  PB4

// -------- 14-Channel MUX ------------------------------------
#define MUX_S0   PA7
#define MUX_S1   PA6
#define MUX_S2   PA5
#define MUX_S3   PA4
#define MUX_SIG  PA2   // ADC1_IN3
#define NUM_SENSORS 14

// -------- OLED Display (SSD1306 128x32, SW I2C) -------------
#define OLED_SCL PB6
#define OLED_SDA PB7
U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2(U8G2_R0, OLED_SCL, OLED_SDA, U8X8_PIN_NONE);

// -------- Sensor array (from PCB layout + bench tests) ------
// Sensor index 0 = D1 (left-most). MUX input I7 is not connected.
const uint8_t MUX_MAP[NUM_SENSORS] = {0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14};
// Lateral position of each sensor in mm, left = positive.
const float SENSOR_X[NUM_SENSORS] = {
  60.24f, 50.24f, 40.24f, 30.24f, 22.62f, 12.62f, 5.0f,
  -5.0f, -12.62f, -22.62f, -30.24f, -40.24f, -50.24f, -60.24f
};
// Set a sensor to true to exclude it (e.g. a faulty channel).
// D1 (S0) and D9 (S8) were faulty until RD1 / RD9 were re-soldered (2026-10-08).
const bool SENSOR_DEAD[NUM_SENSORS] = {
  false, false, false, false, false, false, false,
  false, false, false, false, false, false, false
};

const unsigned int MUX_SETTLE_US = 10; // measured: settles to 1-3 counts
const int NOISE_FLOOR = 250;           // ignore bottom 25% (outer sensors sit above white)
const int ON_LINE_LEVEL = 400;         // any sensor above this = line seen
const int SEGMENT_LEVEL = 100;         // neighbouring sensors above this form one line segment
const float LOST_POS = 60.0f;          // mm reported while line is lost
const unsigned long CAL_MS = 8000;

// -------- Control loop --------------------------------------
const unsigned long LOOP_US = 1000;    // fixed 1 kHz
const int MOTOR_MIN = -30;
const int MOTOR_MAX = 255;
const int START_SPEED = 30;
const int RECOVER_FWD = 70;            // lost line: outer wheel
const int RECOVER_REV = -25;           // lost line: inner wheel

// -------- Tunable parameters (menu / serial) ----------------
int   baseSpeed = 100;   // cruise PWM (+/- 5)
float Kp = 3.5f;         // PWM per mm of error (+/- 0.1); 6.5 overshot on track
float Ki = 0.0f;
float Kd = 30.0f;        // PWM per mm of change over 10 ms (+/- 1)

// Which line to follow when 2+ segments are under the array (junctions, tight S-bends)
enum BranchMode { BRANCH_LEFT, BRANCH_RIGHT, BRANCH_NEAREST };
BranchMode branchMode = BRANCH_NEAREST;  // keeps straight through 3-way splits
const char *BRANCH_NAMES[] = {"LEFT", "RIGHT", "NEAREST"};

// Line colour. AUTO switches to white-line-on-black when most sensors see black
// AND some see white; a solid black area (start box, crossing bar) has no white
// and does not switch it.
enum LineMode { LINE_AUTO, LINE_BLACK, LINE_WHITE };
LineMode lineMode = LINE_AUTO;
const char *LINE_NAMES[] = {"AUTO", "BLACK", "WHITE"};
const int INVERT_ENTER_BLACK = 10;     // >= this many black sensors -> white line on black
const int INVERT_EXIT_BLACK = 5;       // <= this many black sensors -> black line on white
const uint8_t INVERT_CONFIRM = 8;      // consecutive reads that must agree before switching
bool whiteLine = false;                // current detected colour

// -------- Sensor state --------------------------------------
uint16_t calMin[NUM_SENSORS], calMax[NUM_SENSORS];
int16_t  sensorValue[NUM_SENSORS];     // 0..1000 after calibration and floor
bool  isCalibrated = false;
bool  onLine = false;
float linePos = 0;                     // mm, + = line left of centre

// -------- PID state -----------------------------------------
float prevError = 0, integral = 0, dFilt = 0;
int   currentSpeed = START_SPEED;
int   lsp = 0, rsp = 0;
uint8_t rampTicks = 0;
unsigned long lastTick = 0;

// -------- Menu ----------------------------------------------
enum AppState { STATE_MENU, STATE_EDIT_PARAM, STATE_RUNNING, STATE_SENSOR_TEST };
AppState appState = STATE_MENU;

enum MenuItem { MENU_RUN, MENU_CALIBRATE, MENU_SPEED, MENU_KP, MENU_KD, MENU_SENSORS, MENU_ITEM_COUNT };
int currentMenuItem = MENU_RUN;
int editParamIndex = MENU_SPEED;

unsigned long upHoldTimer = 0, upRepeatTimer = 0;
unsigned long downHoldTimer = 0, downRepeatTimer = 0;
unsigned long enterHoldTimer = 0, backHoldTimer = 0;

bool posStream = false;

void motorRight(int speed);
void motorLeft(int speed);
void printCalibration();
void drawMenu();
void drawRunning();

// ============================================================
//  Fast ADC: configured once. analogRead() re-initialises the
//  ADC every call (~158 us); this takes ~3 us. Do not call
//  analogRead() on MUX_SIG, it would de-init this setup.
// ============================================================
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

uint16_t readSensorRaw(int i) {
  uint8_t ch = MUX_MAP[i];
  digitalWrite(MUX_S0, ch & 0x01);
  digitalWrite(MUX_S1, (ch >> 1) & 0x01);
  digitalWrite(MUX_S2, (ch >> 2) & 0x01);
  digitalWrite(MUX_S3, (ch >> 3) & 0x01);
  delayMicroseconds(MUX_SETTLE_US);
  return (fastRead() + fastRead()) / 2;
}

// ============================================================
//  Line position
// ============================================================
// Reads all sensors (~0.3 ms), normalises each to 0..1000 with the
// noise floor removed, and updates linePos (mm) and onLine.
// If more than one line segment is under the array, only the one picked
// by branchMode is used, instead of averaging onto the white between them.
void updateLineColour(int blackCount, int whiteCount) {
  if (lineMode != LINE_AUTO) {
    whiteLine = (lineMode == LINE_WHITE);
    return;
  }
  static uint8_t votes = 0;
  bool want = whiteLine;
  if (whiteCount > 0) {
    if (!whiteLine && blackCount >= INVERT_ENTER_BLACK) want = true;
    if (whiteLine && blackCount <= INVERT_EXIT_BLACK) want = false;
  }
  if (want != whiteLine && ++votes >= INVERT_CONFIRM) {
    whiteLine = want;
    votes = 0;
  } else if (want == whiteLine) {
    votes = 0;
  }
}

void readLine() {
  // Pass 1: 0..1000 where 1000 = black floor, count black / white sensors
  int blackCount = 0, whiteCount = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    uint16_t raw = readSensorRaw(i);
    if (SENSOR_DEAD[i]) {
      sensorValue[i] = 0;
      continue;
    }
    int range = max(1, (int)calMax[i] - (int)calMin[i]);
    int v = constrain(((int)raw - (int)calMin[i]) * 1000 / range, 0, 1000);
    sensorValue[i] = v;
    if (v > 600) blackCount++;
    else if (v < 300) whiteCount++;
  }
  updateLineColour(blackCount, whiteCount);

  // Pass 2: 1000 = line colour, noise floor removed
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (SENSOR_DEAD[i]) continue;
    int v = whiteLine ? 1000 - sensorValue[i] : sensorValue[i];
    sensorValue[i] = constrain((v - NOISE_FLOOR) * 1000 / (1000 - NOISE_FLOOR), 0, 1000);
  }

  // Segments are scanned left (index 0) to right. A disabled sensor does not
  // split a segment. A segment counts as a line only if it has a sensor above
  // ON_LINE_LEVEL.
  float sumW = 0, sumWX = 0, bestPos = 0;
  int peak = 0;
  onLine = false;
  for (int i = 0; i <= NUM_SENSORS; i++) {
    bool inSegment = i < NUM_SENSORS &&
                     (SENSOR_DEAD[i] ? sumW > 0 : sensorValue[i] > SEGMENT_LEVEL);
    if (inSegment) {
      sumW += sensorValue[i];
      sumWX += sensorValue[i] * SENSOR_X[i];
      peak = max(peak, (int)sensorValue[i]);
      continue;
    }
    if (sumW > 0 && peak > ON_LINE_LEVEL) {
      float pos = sumWX / sumW;
      bool take = !onLine ||                                   // first segment found
                  branchMode == BRANCH_RIGHT ||                // keep the right-most
                  (branchMode == BRANCH_NEAREST && fabsf(pos - linePos) < fabsf(bestPos - linePos));
      if (take) bestPos = pos;
      onLine = true;
    }
    sumW = sumWX = 0;
    peak = 0;
  }

  if (onLine) {
    linePos = bestPos;
  } else {
    // Report full deflection on the side the line was last seen
    linePos = (prevError >= 0) ? LOST_POS : -LOST_POS;
  }
}

// spin = true: robot rotates on the spot over the line.
// spin = false: motors stay off, slide the robot across the line by hand.
void calibrate(bool spin) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 10, "CALIBRATING (8s)...");
  u8g2.drawStr(0, 24, spin ? "Rotating on track" : "Slide across line");
  u8g2.sendBuffer();
  Serial.println(spin ? ">>> CALIBRATING (spin, 8 s)" : ">>> CALIBRATING (slide by hand, 8 s)");

  for (int i = 0; i < NUM_SENSORS; i++) {
    calMin[i] = calMax[i] = readSensorRaw(i);
  }

  unsigned long start = millis();
  while (millis() - start < CAL_MS) {
    if (spin) {
      motorRight(50);
      motorLeft(-50);
    }
    for (int i = 0; i < NUM_SENSORS; i++) {
      uint16_t v = readSensorRaw(i);
      if (v < calMin[i]) calMin[i] = v;
      if (v > calMax[i]) calMax[i] = v;
    }
  }
  motorRight(0);
  motorLeft(0);

  isCalibrated = true;
  printCalibration();

  u8g2.clearBuffer();
  u8g2.drawStr(0, 15, "CALIBRATION DONE!");
  u8g2.drawStr(0, 28, "Returning to menu...");
  u8g2.sendBuffer();
  delay(800);
}

void printCalibration() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    char buf[64];
    snprintf(buf, sizeof(buf), "CAL S%-2d white:%4u black:%4u range:%4d%s", i, calMin[i], calMax[i],
             calMax[i] - calMin[i], SENSOR_DEAD[i] ? "  (disabled)" : "");
    Serial.println(buf);
  }
}

// ============================================================
//  PID line following
// ============================================================
void startRun() {
  if (!isCalibrated) {
    Serial.println(">>> CALIBRATE FIRST");
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(0, 15, "CALIBRATE FIRST!");
    u8g2.sendBuffer();
    delay(1000);
    return;
  }
  prevError = 0; integral = 0; dFilt = 0;
  currentSpeed = START_SPEED;
  rampTicks = 0;
  lastTick = micros();
  appState = STATE_RUNNING;
  drawRunning();   // drawn once: the SW-I2C OLED is too slow to refresh mid-run
  Serial.println(">>> RUN");
}

void stopRun() {
  motorRight(0);
  motorLeft(0);
  appState = STATE_MENU;
  Serial.println(">>> STOPPED");
  drawMenu();
}

void followStep() {
  readLine();

  // Ramp up: +1 PWM every 4 ms
  if (currentSpeed < baseSpeed && ++rampTicks >= 4) {
    rampTicks = 0;
    currentSpeed++;
  }
  if (currentSpeed > baseSpeed) currentSpeed = baseSpeed;

  if (!onLine) {
    // Turn hard toward the side the line was last seen
    if (prevError > 0)      { lsp = RECOVER_REV; rsp = RECOVER_FWD; }
    else if (prevError < 0) { lsp = RECOVER_FWD; rsp = RECOVER_REV; }
    else                    { lsp = 0; rsp = 0; }
    motorLeft(lsp);
    motorRight(rsp);
    return;
  }

  float error = linePos;
  integral = constrain(integral + error, -1000.0f, 1000.0f);
  // Change in mm per 10 ms (independent of LOOP_US), lightly low-passed
  float d = (error - prevError) * (10000.0f / LOOP_US);
  dFilt = 0.7f * dFilt + 0.3f * d;
  prevError = error;

  int correction = (int)(Kp * error + Ki * integral + Kd * dFilt);
  lsp = constrain(currentSpeed - correction, MOTOR_MIN, MOTOR_MAX);
  rsp = constrain(currentSpeed + correction, MOTOR_MIN, MOTOR_MAX);
  motorLeft(lsp);
  motorRight(rsp);
}

// ============================================================
//  Motors (right = A pins, left = B pins)
// ============================================================
void motorRight(int speed) {
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

void motorLeft(int speed) {
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

// ============================================================
//  Parameter editing
// ============================================================
void adjustParam(int param, int dir) {
  if (param == MENU_SPEED) {
    baseSpeed = constrain(baseSpeed + 5 * dir, 30, 250);
  } else if (param == MENU_KP) {
    Kp = constrain(roundf((Kp + 0.1f * dir) * 10.0f) / 10.0f, 0.0f, 50.0f);
  } else if (param == MENU_KD) {
    Kd = constrain(roundf(Kd + 1.0f * dir), 0.0f, 300.0f);
  }
}

void printParams() {
  Serial.println("Speed=" + String(baseSpeed) + " | Kp=" + String(Kp, 1) + " | Kd=" + String(Kd, 0) +
                 " | Branch=" + BRANCH_NAMES[branchMode] + " | Line=" + LINE_NAMES[lineMode]);
}

// ============================================================
//  OLED UI
// ============================================================
void drawMenu() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 9, isCalibrated ? "MENU | C13:OK B5:ESC" : "MENU | NOT CALIBRATED");

  int firstVisible = constrain(currentMenuItem, 0, MENU_ITEM_COUNT - 2);
  for (int i = 0; i < 2; i++) {
    int idx = firstVisible + i;
    char lineBuf[32];
    char prefix = (idx == currentMenuItem) ? '>' : ' ';
    switch (idx) {
      case MENU_RUN:       snprintf(lineBuf, sizeof(lineBuf), "%c 1.RUN FOLLOWER", prefix); break;
      case MENU_CALIBRATE: snprintf(lineBuf, sizeof(lineBuf), "%c 2.CALIBRATE (8s)", prefix); break;
      case MENU_SPEED:     snprintf(lineBuf, sizeof(lineBuf), "%c 3.SPEED: %d", prefix, baseSpeed); break;
      case MENU_KP:        snprintf(lineBuf, sizeof(lineBuf), "%c 4.Kp: %s", prefix, String(Kp, 1).c_str()); break;
      case MENU_KD:        snprintf(lineBuf, sizeof(lineBuf), "%c 5.Kd: %s", prefix, String(Kd, 0).c_str()); break;
      default:             snprintf(lineBuf, sizeof(lineBuf), "%c 6.SENSOR TEST", prefix); break;
    }
    u8g2.drawStr(0, 20 + i * 11, lineBuf);
  }
  u8g2.sendBuffer();
}

void drawEditParam() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tr);
  char titleBuf[32], valBuf[32];
  if (editParamIndex == MENU_SPEED) {
    snprintf(titleBuf, sizeof(titleBuf), "TUNE SPEED (+/-5)");
    snprintf(valBuf, sizeof(valBuf), "<  %d  >", baseSpeed);
  } else if (editParamIndex == MENU_KP) {
    snprintf(titleBuf, sizeof(titleBuf), "TUNE Kp (+/-0.1)");
    snprintf(valBuf, sizeof(valBuf), "<  %s  >", String(Kp, 1).c_str());
  } else {
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
  u8g2.setFont(u8g2_font_6x10_tr);
  char buf[32];
  snprintf(buf, sizeof(buf), "RUNNING  Spd:%d", baseSpeed);
  u8g2.drawStr(0, 10, buf);
  snprintf(buf, sizeof(buf), "Kp:%s Kd:%s", String(Kp, 1).c_str(), String(Kd, 0).c_str());
  u8g2.drawStr(0, 21, buf);
  snprintf(buf, sizeof(buf), "Branch:%s  B5:STOP", BRANCH_NAMES[branchMode]);
  u8g2.drawStr(0, 32, buf);
  u8g2.sendBuffer();
}

// One bar per sensor (height = normalised value), X marks a disabled sensor
void drawSensorTest() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x7_tr);
  char buf[32];
  const char *col = whiteLine ? "WHT" : "BLK";
  if (onLine) snprintf(buf, sizeof(buf), "POS:%+6.1f %s B5:EXIT", linePos, col);
  else        snprintf(buf, sizeof(buf), "POS: LOST  %s B5:EXIT", col);
  u8g2.drawStr(0, 7, buf);

  for (int i = 0; i < NUM_SENSORS; i++) {
    int x = i * 9 + 1;
    if (SENSOR_DEAD[i]) {
      u8g2.drawLine(x, 12, x + 6, 31);
      u8g2.drawLine(x + 6, 12, x, 31);
      continue;
    }
    int h = sensorValue[i] * 20 / 1000;
    u8g2.drawFrame(x, 11, 7, 21);
    if (h > 0) u8g2.drawBox(x, 32 - h, 7, h);
  }
  u8g2.sendBuffer();
}

// ============================================================
//  Buttons
// ============================================================
bool checkButtonSingle(int pin, unsigned long &holdTimer) {
  if (digitalRead(pin) == LOW) {
    if (holdTimer == 0) {
      holdTimer = millis();
      return true;
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
      return true;
    }
    if (millis() - holdTimer > 350 && millis() - repeatTimer > 100) {
      repeatTimer = millis();
      return true;
    }
  } else {
    holdTimer = 0;
  }
  return false;
}

// ============================================================
//  Serial commands
// ============================================================
void handleSerial() {
  if (Serial.available() <= 0) return;
  char c = Serial.read();
  switch (c) {
    case 'r': case 'R': case 's': case 'S': startRun(); break;
    case 'x': case 'X': case ' ':           stopRun(); break;
    case 'c': case 'C': calibrate(true);  appState = STATE_MENU; drawMenu(); break;
    case 'm':           calibrate(false); appState = STATE_MENU; drawMenu(); break;
    case 'k': printCalibration(); break;
    case 'l': posStream = !posStream; break;
    case '+': adjustParam(MENU_SPEED, +1); printParams(); break;
    case '-': adjustParam(MENU_SPEED, -1); printParams(); break;
    case 'p': adjustParam(MENU_KP, +1); printParams(); break;
    case 'P': adjustParam(MENU_KP, -1); printParams(); break;
    case 'd': adjustParam(MENU_KD, +1); printParams(); break;
    case 'D': adjustParam(MENU_KD, -1); printParams(); break;
    case 'w': lineMode = (LineMode)((lineMode + 1) % 3); printParams(); break;
    case 'b': branchMode = (BranchMode)((branchMode + 1) % 3); printParams(); break;
  }
}

// POS,<ms>,<mm|lost>,v0..v13 at 50 Hz (outside a run)
void streamPos() {
  static unsigned long last = 0;
  if (millis() - last < 20) return;
  last = millis();
  readLine();
  char buf[160];
  int n = onLine ? snprintf(buf, sizeof(buf), "POS,%lu,%.2f,%c", last, linePos, whiteLine ? 'W' : 'B')
                 : snprintf(buf, sizeof(buf), "POS,%lu,lost,%c", last, whiteLine ? 'W' : 'B');
  for (int i = 0; i < NUM_SENSORS; i++) {
    n += snprintf(buf + n, sizeof(buf) - n, ",%d", sensorValue[i]);
  }
  Serial.println(buf);
}

// ============================================================
//  Setup / Loop
// ============================================================
void setup() {
  Serial.begin(115200);

  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  pinMode(MUX_SIG, INPUT_ANALOG);
  bool adcOk = fastAdcInit();

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  motorRight(0);
  motorLeft(0);

  pinMode(BTN_ENTER, INPUT_PULLUP);
  pinMode(BTN_BACK,  INPUT_PULLUP);
  pinMode(BTN_UP,    INPUT_PULLUP);
  pinMode(BTN_DOWN,  INPUT_PULLUP);

  for (int i = 0; i < NUM_SENSORS; i++) {
    calMin[i] = 0;
    calMax[i] = 4095;
  }

  u8g2.begin();
  drawMenu();

  Serial.println("\n=== APEX LFR - 14-sensor analog PID ===");
  if (!adcOk) Serial.println("!!! ADC INIT FAILED");
  printParams();
}

void loop() {
  handleSerial();

  switch (appState) {

    case STATE_MENU: {
      if (posStream) streamPos();

      if (checkButtonRepeat(BTN_UP, upHoldTimer, upRepeatTimer)) {
        currentMenuItem = (currentMenuItem - 1 + MENU_ITEM_COUNT) % MENU_ITEM_COUNT;
        drawMenu();
      }
      if (checkButtonRepeat(BTN_DOWN, downHoldTimer, downRepeatTimer)) {
        currentMenuItem = (currentMenuItem + 1) % MENU_ITEM_COUNT;
        drawMenu();
      }
      if (checkButtonSingle(BTN_ENTER, enterHoldTimer)) {
        if (currentMenuItem == MENU_RUN) {
          startRun();
          if (appState != STATE_RUNNING) drawMenu();
        } else if (currentMenuItem == MENU_CALIBRATE) {
          calibrate(true);
          drawMenu();
        } else if (currentMenuItem == MENU_SENSORS) {
          appState = STATE_SENSOR_TEST;
        } else {
          editParamIndex = currentMenuItem;
          appState = STATE_EDIT_PARAM;
          drawEditParam();
        }
      }
      break;
    }

    case STATE_EDIT_PARAM: {
      if (checkButtonRepeat(BTN_UP, upHoldTimer, upRepeatTimer)) {
        adjustParam(editParamIndex, +1);
        drawEditParam();
      }
      if (checkButtonRepeat(BTN_DOWN, downHoldTimer, downRepeatTimer)) {
        adjustParam(editParamIndex, -1);
        drawEditParam();
      }
      if (checkButtonSingle(BTN_ENTER, enterHoldTimer) || checkButtonSingle(BTN_BACK, backHoldTimer)) {
        printParams();
        appState = STATE_MENU;
        drawMenu();
      }
      break;
    }

    case STATE_RUNNING: {
      if (checkButtonSingle(BTN_BACK, backHoldTimer) || checkButtonSingle(BTN_ENTER, enterHoldTimer)) {
        stopRun();
        delay(200);
        break;
      }

      if (micros() - lastTick < LOOP_US) break;
      lastTick += LOOP_US;
      if (micros() - lastTick > LOOP_US) lastTick = micros();  // don't try to catch up after a stall

      followStep();

      static unsigned long lastLog = 0;
      if (millis() - lastLog >= 100) {
        lastLog = millis();
        char buf[80];
        if (onLine) snprintf(buf, sizeof(buf), "RUN pos:%+6.1f %s spd:%3d L:%4d R:%4d", linePos, whiteLine ? "WHT" : "BLK", currentSpeed, lsp, rsp);
        else        snprintf(buf, sizeof(buf), "RUN pos: LOST  %s spd:%3d L:%4d R:%4d", whiteLine ? "WHT" : "BLK", currentSpeed, lsp, rsp);
        Serial.println(buf);
      }
      break;
    }

    case STATE_SENSOR_TEST: {
      if (checkButtonSingle(BTN_BACK, backHoldTimer) || checkButtonSingle(BTN_ENTER, enterHoldTimer)) {
        appState = STATE_MENU;
        drawMenu();
        delay(200);
        break;
      }
      static unsigned long lastDraw = 0;
      if (millis() - lastDraw >= 100) {
        lastDraw = millis();
        readLine();
        drawSensorTest();
      }
      break;
    }
  }
}
