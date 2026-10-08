#include <Arduino.h>
#include <U8g2lib.h>

/*
  =============================================================================
  APEX LINE FOLLOWER - IR SENSOR ANALOG RESOLUTION & NOISE ANALYZER
  Target MCU: STM32G431CB (170MHz ARM Cortex-M4F)
  
  Purpose:
    1. Measure exact 12-bit ADC raw readings (0..4095) for all 8 IR sensors.
    2. Quantify optical contrast resolution: Delta = V_max (Black) - V_min (White).
    3. Measure analog noise floor / peak-to-peak jitter on static surfaces.
    4. Calculate optimal dynamic threshold: Mid = (V_min + V_max) / 2.
    5. Evaluate sensor signal quality (EXCELLENT, GOOD, FAIR, POOR).
    6. Real-time OLED visualization:
       - Mode 0: 8-Channel Normalized Dynamic Bars
       - Mode 1: Detailed Single-Sensor Deep Inspector
       - Mode 2: Multi-Sensor Contrast & Threshold Matrix
       - Mode 3: Real-Time Waveform Oscilloscope / Jitter Scope
    7. Dual Serial output: Formatted diagnostic table OR Arduino Serial Plotter.
  
  Controls:
    - PC13 (BTN_ENTER): Start/Stop 5s Min-Max Sweep Calibration
    - PB5  (BTN_MODE) : Cycle OLED Display Mode (0 -> 1 -> 2 -> 3)
    - PB3  (BTN_UP)   : Select Next Sensor (in Mode 1/3) / Hold to Jog Forward
    - PB4  (BTN_DOWN) : Select Prev Sensor (in Mode 1/3) / Hold to Jog Backward
  =============================================================================
*/

// ======================== HARDWARE PIN DEFINITIONS ========================
// 8-Channel IR Sensor Array (Physical left-to-right order: S0=PA7 ... S7=PA0)
const int SENSOR_PINS[8] = { PA7, PA6, PA5, PA4, PA3, PA2, PA1, PA0 };
const char* SENSOR_NAMES[8] = { "PA7", "PA6", "PA5", "PA4", "PA3", "PA2", "PA1", "PA0" };

// Pushbuttons (Active LOW with internal pull-up)
#define BTN_ENTER PC13
#define BTN_MODE  PB5
#define BTN_UP    PB3
#define BTN_DOWN  PB4

// TB6612FNG Motor Driver
#define AIN1  PB0     // Right motor direction 1
#define AIN2  PB1     // Right motor direction 2
#define BIN1  PB10    // Left motor direction 1
#define BIN2  PB11    // Left motor direction 2
#define PWMA  PA8     // Right motor PWM
#define PWMB  PA9     // Left motor PWM
#define STBY  PA10    // Standby enable (HIGH = active)

// OLED Display (SSD1306 128x32 via Software I2C on PB6/PB7)
#define OLED_SCL PB6
#define OLED_SDA PB7
U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2(U8G2_R0, OLED_SCL, OLED_SDA, U8X8_PIN_NONE);

// ======================== DATA STRUCTURES & STATS ========================
const int NUM_SENSORS = 8;
const int OVERSAMPLE_COUNT = 16;  // 16x oversampling for noise suppression

int rawValues[NUM_SENSORS];       // Filtered raw ADC (0..4095)
int minValues[NUM_SENSORS];       // Min reading recorded (White)
int maxValues[NUM_SENSORS];       // Max reading recorded (Black)
int deltaValues[NUM_SENSORS];     // Contrast dynamic range (Max - Min)
int midValues[NUM_SENSORS];       // Midpoint threshold (Min + Max) / 2
int jitterValues[NUM_SENSORS];    // Peak-to-peak jitter over recent samples

// Rolling window for jitter / noise analysis
#define JITTER_WINDOW 32
int jitterHistory[NUM_SENSORS][JITTER_WINDOW];
uint8_t jitterIdx = 0;

// Scope waveform history for selected sensor (Mode 3)
#define SCOPE_POINTS 120
uint8_t scopeHistory[SCOPE_POINTS];
uint8_t scopeHead = 0;

// Display and Inspection State
enum DisplayMode {
  MODE_BARS = 0,      // 8-channel normalized dynamic bar graph
  MODE_INSPECT = 1,   // Detailed single-sensor inspection
  MODE_TABLE = 2,     // 4-sensor threshold summary
  MODE_SCOPE = 3,     // Live waveform & noise scope
  MODE_COUNT = 4
};
DisplayMode currentMode = MODE_BARS;
int inspectedSensor = 0;   // Currently selected sensor (0..7)

// Calibration State
bool isCalibrating = false;
unsigned long calStartTime = 0;
const unsigned long CAL_DURATION_MS = 5000;

// Serial Output Configuration
bool serialPlotterMode = false;
unsigned long lastSerialPrint = 0;
unsigned long lastDisplayUpdate = 0;

// Button Debounce and Long-Press Timing
unsigned long lastBtnCheck = 0;
bool lastEnter = HIGH, lastMode = HIGH, lastUp = HIGH, lastDown = HIGH;
unsigned long upPressTime = 0, downPressTime = 0;
bool isJoggingFwd = false, isJoggingBwd = false;

// ======================== MOTOR FUNCTIONS ========================
void motorsInit() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

void motorsBrake() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

void motorsJog(int speed) {
  if (speed > 0) {
    // Forward
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  } else if (speed < 0) {
    // Backward
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
  } else {
    motorsBrake();
    return;
  }
  analogWrite(PWMA, abs(speed));
  analogWrite(PWMB, abs(speed));
}

// ======================== SENSOR READING & ANALYSIS ========================
void resetCalibrationStats() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    minValues[i] = 4095;
    maxValues[i] = 0;
    deltaValues[i] = 0;
    midValues[i] = 2048;
    jitterValues[i] = 0;
  }
  for (int p = 0; p < SCOPE_POINTS; p++) {
    scopeHistory[p] = 16;
  }
}

void readSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    long sum = 0;
    int sampleMin = 4095;
    int sampleMax = 0;

    for (int s = 0; s < OVERSAMPLE_COUNT; s++) {
      int val = analogRead(SENSOR_PINS[i]);
      sum += val;
      if (val < sampleMin) sampleMin = val;
      if (val > sampleMax) sampleMax = val;
    }
    int avg = (int)(sum / OVERSAMPLE_COUNT);
    rawValues[i] = avg;

    // Record sample in rolling jitter buffer
    jitterHistory[i][jitterIdx] = avg;

    // Track min/max if calibrating or updating bounds
    if (isCalibrating) {
      if (avg < minValues[i]) minValues[i] = avg;
      if (avg > maxValues[i]) maxValues[i] = avg;
      deltaValues[i] = maxValues[i] - minValues[i];
      midValues[i] = (minValues[i] + maxValues[i]) / 2;
    }
  }

  // Update peak-to-peak jitter across rolling window
  jitterIdx = (jitterIdx + 1) % JITTER_WINDOW;
  for (int i = 0; i < NUM_SENSORS; i++) {
    int wMin = 4095;
    int wMax = 0;
    for (int k = 0; k < JITTER_WINDOW; k++) {
      int v = jitterHistory[i][k];
      if (v > 0) {
        if (v < wMin) wMin = v;
        if (v > wMax) wMax = v;
      }
    }
    jitterValues[i] = (wMax >= wMin) ? (wMax - wMin) : 0;
  }

  // Update scope waveform history for currently inspected sensor
  int normScope = 0;
  if (deltaValues[inspectedSensor] > 50) {
    normScope = map(rawValues[inspectedSensor], minValues[inspectedSensor], maxValues[inspectedSensor], 30, 2);
  } else {
    normScope = map(rawValues[inspectedSensor], 0, 4095, 30, 2);
  }
  normScope = constrain(normScope, 2, 30);
  scopeHistory[scopeHead] = (uint8_t)normScope;
  scopeHead = (scopeHead + 1) % SCOPE_POINTS;
}

const char* getSensorQuality(int delta) {
  if (delta >= 1600) return "EXCELLENT";  // >= 39% dynamic range
  if (delta >= 1000) return "GOOD";       // >= 24% dynamic range
  if (delta >= 600)  return "ACCEPTABLE"; // >= 14% dynamic range
  return "WEAK / LOW RES";
}

// ======================== OLED DISPLAY MODES ========================
void drawOLED() {
  u8g2.clearBuffer();

  switch (currentMode) {
    // -------------------------------------------------------------
    // MODE 0: 8-Channel Dynamic Normalized Bar Graph
    // -------------------------------------------------------------
    case MODE_BARS: {
      u8g2.setFont(u8g2_font_5x7_tr);
      if (isCalibrating) {
        char buf[32];
        int remainSec = (CAL_DURATION_MS - (millis() - calStartTime) + 999) / 1000;
        snprintf(buf, sizeof(buf), "SWEEPING LINE... %ds", remainSec);
        u8g2.drawStr(0, 7, buf);
      } else {
        u8g2.drawStr(0, 7, "RES BARS | C13:CAL B5:MODE");
      }

      // 8 vertical bars (each 12px wide, 2px gap)
      for (int i = 0; i < NUM_SENSORS; i++) {
        int x = i * 16 + 1;
        int y = 9;
        int w = 14;
        int h = 22;

        // Draw outline box
        u8g2.drawFrame(x, y, w, h);

        // Calculate fill height based on calibrated contrast range
        int fillH = 0;
        if (deltaValues[i] > 50) {
          fillH = map(rawValues[i], minValues[i], maxValues[i], 0, h - 2);
        } else {
          fillH = map(rawValues[i], 0, 4095, 0, h - 2);
        }
        fillH = constrain(fillH, 0, h - 2);

        if (fillH > 0) {
          u8g2.drawBox(x + 1, y + h - 1 - fillH, w - 2, fillH);
        }

        // Draw midpoint line indicator inside frame
        int midY = y + (h / 2);
        u8g2.drawHLine(x + 1, midY, 2);
        u8g2.drawHLine(x + w - 3, midY, 2);
      }
      break;
    }

    // -------------------------------------------------------------
    // MODE 1: Single-Sensor Deep Inspector
    // -------------------------------------------------------------
    case MODE_INSPECT: {
      u8g2.setFont(u8g2_font_5x7_tr);
      int s = inspectedSensor;
      float volts = (rawValues[s] * 3.30f) / 4095.0f;
      float contrastPct = (deltaValues[s] * 100.0f) / 4095.0f;

      // Line 1: Header + Raw + Voltage
      char l1[32];
      snprintf(l1, sizeof(l1), "S%d(%s): %4d (%.2fV)", s, SENSOR_NAMES[s], rawValues[s], volts);
      u8g2.drawStr(0, 7, l1);

      // Line 2: Min, Max, Delta
      char l2[32];
      snprintf(l2, sizeof(l2), "MIN:%d MAX:%d D:%d", minValues[s], maxValues[s], deltaValues[s]);
      u8g2.drawStr(0, 15, l2);

      // Line 3: Midpoint, Jitter, Contrast %
      char l3[32];
      snprintf(l3, sizeof(l3), "MID:%d JIT:%d C:%.1f%%", midValues[s], jitterValues[s], contrastPct);
      u8g2.drawStr(0, 23, l3);

      // Line 4: Signal Quality + Navigation Hint
      char l4[32];
      snprintf(l4, sizeof(l4), "[%s] B3/B4:S+-", getSensorQuality(deltaValues[s]));
      u8g2.drawStr(0, 31, l4);
      break;
    }

    // -------------------------------------------------------------
    // MODE 2: Multi-Sensor Contrast & Threshold Matrix
    // -------------------------------------------------------------
    case MODE_TABLE: {
      u8g2.setFont(u8g2_font_5x7_tr);
      // Group: S0..S3 if inspected < 4, else S4..S7
      int startIdx = (inspectedSensor < 4) ? 0 : 4;
      char hBuf[32];
      snprintf(hBuf, sizeof(hBuf), "DELTA & MID (S%d-S%d)", startIdx, startIdx + 3);
      u8g2.drawStr(0, 7, hBuf);

      for (int r = 0; r < 3; r++) {
        int idx = startIdx + r;
        if (idx < NUM_SENSORS) {
          char rBuf[32];
          snprintf(rBuf, sizeof(rBuf), "S%d: R:%4d D:%4d M:%4d",
                   idx, rawValues[idx], deltaValues[idx], midValues[idx]);
          u8g2.drawStr(0, 15 + (r * 8), rBuf);
        }
      }
      break;
    }

    // -------------------------------------------------------------
    // MODE 3: Real-Time Waveform & Noise Oscilloscope
    // -------------------------------------------------------------
    case MODE_SCOPE: {
      u8g2.setFont(u8g2_font_5x7_tr);
      char sBuf[32];
      snprintf(sBuf, sizeof(sBuf), "SCOPE S%d | JIT:%d cnt", inspectedSensor, jitterValues[inspectedSensor]);
      u8g2.drawStr(0, 7, sBuf);

      // Draw horizontal baseline grid lines
      u8g2.drawHLine(4, 10, 120);
      u8g2.drawHLine(4, 20, 120);
      u8g2.drawHLine(4, 30, 120);

      // Plot waveform trace
      for (int x = 0; x < SCOPE_POINTS - 1; x++) {
        int idx1 = (scopeHead + x) % SCOPE_POINTS;
        int idx2 = (scopeHead + x + 1) % SCOPE_POINTS;
        int y1 = scopeHistory[idx1];
        int y2 = scopeHistory[idx2];
        u8g2.drawLine(4 + x, y1, 5 + x, y2);
      }
      break;
    }

    default: break;
  }

  u8g2.sendBuffer();
}

// ======================== SERIAL TELEMETRY & PLOTTER ========================
void printSerialTable() {
  Serial.println(F("\n========================================================================================="));
  Serial.println(F(" CH | PIN |  RAW  | VOLTAGE |   MIN   |   MAX   |  DELTA  |   MID   | JITTER |   QUALITY     "));
  Serial.println(F("----+-----+-------+---------+---------+---------+---------+---------+--------+---------------"));

  for (int i = 0; i < NUM_SENSORS; i++) {
    float volts = (rawValues[i] * 3.30f) / 4095.0f;
    float contrastPct = (deltaValues[i] * 100.0f) / 4095.0f;

    char row[128];
    snprintf(row, sizeof(row),
             " S%d | %s | %5d |  %1.2fV  |  %5d  |  %5d  |  %5d  |  %5d  | %3d cnt | %s (%.1f%%)",
             i, SENSOR_NAMES[i], rawValues[i], volts,
             minValues[i], maxValues[i], deltaValues[i], midValues[i],
             jitterValues[i], getSensorQuality(deltaValues[i]), contrastPct);
    Serial.println(row);
  }
  Serial.println(F("========================================================================================="));
  Serial.print(F("Status: "));
  if (isCalibrating) {
    Serial.println(F("CALIBRATING... (Sweep sensors across line)"));
  } else {
    Serial.println(F("READY. [c]=Calibrate  [m]=OLED Mode  [p]=Plotter Mode  [0-7]=Inspect S#  [?]=Help"));
  }
}

void printSerialPlotter() {
  // Comma-separated output for Arduino Serial Plotter
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(rawValues[i]);
    if (i < NUM_SENSORS - 1) Serial.print(F(","));
  }
  Serial.println();
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'c' || c == 'C') {
      isCalibrating = true;
      calStartTime = millis();
      resetCalibrationStats();
      Serial.println(F(">>> Calibration started! Sweep sensors over black line and white floor..."));
    } else if (c == 'm' || c == 'M') {
      currentMode = (DisplayMode)((currentMode + 1) % MODE_COUNT);
      Serial.print(F(">>> Display Mode switched to: ")); Serial.println(currentMode);
    } else if (c == 'p' || c == 'P') {
      serialPlotterMode = !serialPlotterMode;
      if (serialPlotterMode) {
        Serial.println(F(">>> SERIAL PLOTTER MODE ACTIVATED (CSV stream: S0,S1,S2,S3,S4,S5,S6,S7)"));
      } else {
        Serial.println(F(">>> TABLE TELEMETRY MODE ACTIVATED"));
      }
    } else if (c >= '0' && c <= '7') {
      inspectedSensor = c - '0';
      Serial.print(F(">>> Inspected sensor set to S")); Serial.println(inspectedSensor);
    } else if (c == 'r' || c == 'R') {
      resetCalibrationStats();
      Serial.println(F(">>> Calibration limits reset."));
    } else if (c == '?') {
      Serial.println(F("\n--- COMMAND LIST ---"));
      Serial.println(F("  c : Start 5s sweep calibration"));
      Serial.println(F("  m : Cycle OLED display mode"));
      Serial.println(F("  p : Toggle Serial Plotter CSV output"));
      Serial.println(F("  0-7 : Select sensor to inspect"));
      Serial.println(F("  r : Reset Min/Max statistics"));
      Serial.println(F("  ? : Print this help\n"));
    }
  }
}

// ======================== BUTTON INTERACTION & JOGGING ========================
void handleButtons() {
  unsigned long now = millis();
  if (now - lastBtnCheck < 20) return;  // 20ms debounce polling
  lastBtnCheck = now;

  bool curEnter = digitalRead(BTN_ENTER);
  bool curMode  = digitalRead(BTN_MODE);
  bool curUp    = digitalRead(BTN_UP);
  bool curDown  = digitalRead(BTN_DOWN);

  // PC13 (BTN_ENTER): Start / Reset 5s sweep calibration
  if (lastEnter == HIGH && curEnter == LOW) {
    isCalibrating = true;
    calStartTime = now;
    resetCalibrationStats();
    Serial.println(F(">>> [BUTTON] Calibration Sweep Started (5s)..."));
  }

  // PB5 (BTN_MODE): Cycle display modes
  if (lastMode == HIGH && curMode == LOW) {
    currentMode = (DisplayMode)((currentMode + 1) % MODE_COUNT);
  }

  // PB3 (BTN_UP): Short click = Select next sensor; Long hold (>400ms) = Jog Forward
  if (lastUp == HIGH && curUp == LOW) {
    upPressTime = now;
  }
  if (curUp == LOW) {
    if (now - upPressTime > 400) {
      isJoggingFwd = true;
      motorsJog(55);  // Smooth low-speed jog
    }
  } else if (lastUp == LOW && curUp == HIGH) {
    if (isJoggingFwd) {
      isJoggingFwd = false;
      motorsBrake();
    } else {
      inspectedSensor = (inspectedSensor + 1) % NUM_SENSORS;
    }
  }

  // PB4 (BTN_DOWN): Short click = Select prev sensor; Long hold (>400ms) = Jog Backward
  if (lastDown == HIGH && curDown == LOW) {
    downPressTime = now;
  }
  if (curDown == LOW) {
    if (now - downPressTime > 400) {
      isJoggingBwd = true;
      motorsJog(-55); // Smooth low-speed jog
    }
  } else if (lastDown == LOW && curDown == HIGH) {
    if (isJoggingBwd) {
      isJoggingBwd = false;
      motorsBrake();
    } else {
      inspectedSensor = (inspectedSensor + NUM_SENSORS - 1) % NUM_SENSORS;
    }
  }

  lastEnter = curEnter;
  lastMode  = curMode;
  lastUp    = curUp;
  lastDown  = curDown;
}

// ======================== ARDUINO SETUP & LOOP ========================
void setup() {
  Serial.begin(115200);

  // Set STM32 ADC resolution to 12 bits (0..4095)
  analogReadResolution(12);

  // Initialize Sensor Pins
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(SENSOR_PINS[i], INPUT_ANALOG);
  }

  // Initialize Buttons with internal pullups
  pinMode(BTN_ENTER, INPUT_PULLUP);
  pinMode(BTN_MODE,  INPUT_PULLUP);
  pinMode(BTN_UP,    INPUT_PULLUP);
  pinMode(BTN_DOWN,  INPUT_PULLUP);

  // Initialize Motors
  motorsInit();

  // Initialize OLED
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 12, "APEX SENSOR TEST");
  u8g2.drawStr(0, 26, "Resolution Analyzer");
  u8g2.sendBuffer();
  delay(1200);

  resetCalibrationStats();

  Serial.println(F("\n=========================================================="));
  Serial.println(F("  APEX LINE FOLLOWER - 8-CHANNEL SENSOR RESOLUTION ANALYZER"));
  Serial.println(F("  12-bit ADC | Dynamic Contrast | Peak-to-Peak Noise Jitter"));
  Serial.println(F("=========================================================="));
  Serial.println(F("Press PC13 or send 'c' to start 5s calibration sweep."));
}

void loop() {
  unsigned long now = millis();

  // Read sensors continuously
  readSensors();

  // Check buttons & jogging
  handleButtons();

  // Handle incoming Serial commands
  handleSerialCommands();

  // Check calibration timer
  if (isCalibrating && (now - calStartTime >= CAL_DURATION_MS)) {
    isCalibrating = false;
    Serial.println(F(">>> Calibration sweep complete! Final contrast resolution locked."));
  }

  // Update OLED Display at ~25 FPS (every 40ms)
  if (now - lastDisplayUpdate >= 40) {
    lastDisplayUpdate = now;
    drawOLED();
  }

  // Serial Output Telemetry
  if (serialPlotterMode) {
    if (now - lastSerialPrint >= 20) {  // 50Hz for smooth Plotter curves
      lastSerialPrint = now;
      printSerialPlotter();
    }
  } else {
    if (now - lastSerialPrint >= 300) { // 300ms for readable Table refresh
      lastSerialPrint = now;
      printSerialTable();
    }
  }
}
