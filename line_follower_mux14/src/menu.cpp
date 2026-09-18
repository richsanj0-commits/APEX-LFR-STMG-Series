// ============================================================
//  menu.cpp  –  Menu navigation logic
// ============================================================
#include "menu.h"
#include "display.h"
#include "motors.h"
#include "sensors.h"
#include "follower.h"
#include <Arduino.h>
#include <stdio.h>

// ── Config values (will be used by other modules later) ──────
float cfg_kP        = 0.03f;
float cfg_kI        = 0.00f;
float cfg_kD        = 0.20f;
int   cfg_speed     = 80;        // max motor speed 0-255
int   cfg_cal_time  = 5;         // calibration spin seconds
int   cfg_line_color = 0;        // 0 = Black, 1 = White

// ── Internal state ───────────────────────────────────────────
static Screen current = SCR_WELCOME;
static int    cursor  = 0;
static bool   dirty   = true;     // needs redraw

// ── Menu tables ──────────────────────────────────────────────
static const char* MAIN_ITEMS[] = { "Start", "Calibrate", "Tuning", "Testing" };
static const int   MAIN_COUNT   = 4;

static const char* TUNING_ITEMS[] = { "PID", "Motor Speed", "Cal Time", "Line Color" };
static const int   TUNING_COUNT   = 4;

static const char* PID_ITEMS[] = { "Kp", "Ki", "Kd" };
static const int   PID_COUNT   = 3;

static const char* TESTING_ITEMS[] = { "Motor Test", "Sensor Test" };
static const int   TESTING_COUNT   = 2;

static const char* MOTOR_TEST_ITEMS[] = { "Left Motor", "Right Motor", "Forward", "Backward" };
static const int   MOTOR_TEST_COUNT   = 4;

// ── Helpers ──────────────────────────────────────────────────
static void go(Screen s) { current = s; cursor = 0; dirty = true; }

// Draw a vertical list, max 3 items visible on 128x32 (rows 1-3, row 0 = title)
static void draw_list(const char* title, const char** items, int count) {
  display_clear();
  char buf[22];
  snprintf(buf, sizeof(buf), "  %s", title);
  display_text(0, buf);

  const int MAX_ITEMS = 3;
  int start_idx = 0;
  
  if (cursor >= MAX_ITEMS) {
    start_idx = cursor - MAX_ITEMS + 1;
  }
  
  int visible = min(count - start_idx, MAX_ITEMS);
  for (int i = 0; i < visible; i++) {
    int item_idx = start_idx + i;
    snprintf(buf, sizeof(buf), "%s %s", (item_idx == cursor) ? ">" : " ", items[item_idx]);
    display_text(i + 1, buf);
  }
  display_show();
}

// Draw an edit screen for a float value
static void draw_edit_f(const char* label, float val) {
  display_clear();
  display_text(0, label);
  display_text(1, "UP/DN to change");
  display_text(2, "ENT to confirm");
  char buf[16];
  snprintf(buf, sizeof(buf), "Value: %.3f", val);
  display_text(4, buf);
  display_show();
}

// Draw an edit screen for an int value
static void draw_edit_i(const char* label, int val, const char* unit = "") {
  display_clear();
  display_text(0, label);
  display_text(1, "UP/DN to change");
  display_text(2, "ENT to confirm");
  char buf[16];
  snprintf(buf, sizeof(buf), "Value: %d%s", val, unit);
  display_text(4, buf);
  display_show();
}

// Draw line color toggle
static void draw_line_color() {
  display_clear();
  display_text(0, "Line Color");
  display_text(1, "UP/DN to toggle");
  display_text(2, "ENT to confirm");
  display_text(4, cfg_line_color == 0 ? "Value: Black" : "Value: White");
  display_show();
}

// ── Public API ────────────────────────────────────────────────
void menu_init() {
  go(SCR_WELCOME);
}

Screen menu_current() { return current; }

void menu_draw() {
  if (!dirty) return;
  dirty = false;

  switch (current) {
    case SCR_WELCOME:
      display_clear();
      display_text(2, "  LINE FOLLOWER");
      display_text(4, " Press ENTER");
      display_show();
      break;

    case SCR_MAIN:
      draw_list("MAIN MENU", MAIN_ITEMS, MAIN_COUNT);
      break;

    case SCR_TUNING:
      draw_list("TUNING", TUNING_ITEMS, TUNING_COUNT);
      break;

    case SCR_PID_MENU:
      draw_list("PID", PID_ITEMS, PID_COUNT);
      break;

    case SCR_TESTING:
      draw_list("TESTING", TESTING_ITEMS, TESTING_COUNT);
      break;

    case SCR_TEST_MOTOR_MENU:
      draw_list("MOTOR TEST", MOTOR_TEST_ITEMS, MOTOR_TEST_COUNT);
      break;

    case SCR_EDIT_KP:   draw_edit_f("Kp (P gain)", cfg_kP);             break;
    case SCR_EDIT_KI:   draw_edit_f("Ki (I gain)", cfg_kI);             break;
    case SCR_EDIT_KD:   draw_edit_f("Kd (D gain)", cfg_kD);             break;
    case SCR_EDIT_SPEED: draw_edit_i("Motor Speed", cfg_speed, "");     break;
    case SCR_EDIT_CAL_TIME: draw_edit_i("Cal Time", cfg_cal_time, "s"); break;
    case SCR_EDIT_LINE_COLOR: draw_line_color();                         break;

    case SCR_START:
      display_clear();
      display_text(2, "  RUNNING");
      display_text(4, " EXIT to stop");
      display_show();
      follower_loop(cfg_kP, cfg_kI, cfg_kD, cfg_speed, cfg_line_color);
      dirty = true; // force continuous loop
      break;

    case SCR_CALIBRATE:
      display_clear();
      display_text(2, "  CALIBRATING");
      display_text(4, " Please wait...");
      display_show();
      
      // Perform calibration (blocking call)
      sensors_calibrate(cfg_cal_time);
      
      // Automatically return to main menu after calibration
      go(SCR_MAIN);
      break;

    case SCR_TEST_LEFT:
      display_clear();
      display_text(0, "LEFT MOTOR");
      display_text(2, "Spinning...");
      display_text(3, "EXIT to stop");
      display_show();
      motor_left(150);
      motor_right(0);
      break;

    case SCR_TEST_RIGHT:
      display_clear();
      display_text(0, "RIGHT MOTOR");
      display_text(2, "Spinning...");
      display_text(3, "EXIT to stop");
      display_show();
      motor_left(0);
      motor_right(150);
      break;

    case SCR_TEST_FWD:
      display_clear();
      display_text(0, "FORWARD");
      display_text(2, "Moving...");
      display_text(3, "EXIT to stop");
      display_show();
      motor_left(150);
      motor_right(150);
      break;

    case SCR_TEST_BWD:
      display_clear();
      display_text(0, "BACKWARD");
      display_text(2, "Moving...");
      display_text(3, "EXIT to stop");
      display_show();
      motor_left(-150);
      motor_right(-150);
      break;

    case SCR_TEST_SENSOR: {
      sensors_read_raw();
      display_clear();
      
      int d[14];
      for(int i=0; i<14; i++) {
         if (cfg_line_color == 0) {
           d[i] = (sensor_values[i] > sensor_threshold[i]) ? 1 : 0;
         } else {
           d[i] = (sensor_values[i] < sensor_threshold[i]) ? 1 : 0;
         }
      }

      char buf[32];
      display_text(0, "   DIGITAL SENSORS:");
      snprintf(buf, sizeof(buf), " S0-6: %d %d %d %d %d %d %d", d[0], d[1], d[2], d[3], d[4], d[5], d[6]);
      display_text(1, buf);
      snprintf(buf, sizeof(buf), "S7-13: %d %d %d %d %d %d %d", d[7], d[8], d[9], d[10], d[11], d[12], d[13]);
      display_text(2, buf);
      display_show();
      
      dirty = true; // force continuous redraw for live data
      break;
    }

    default: break;
  }
}

// ── Input handler ─────────────────────────────────────────────
void menu_handle(ButtonEvent evt) {
  dirty = true;   // almost every press needs a redraw

  switch (current) {

    // ── Welcome ──────────────────────────────────────────────
    case SCR_WELCOME:
      if (evt == EVT_ENTER) go(SCR_MAIN);
      else dirty = false;
      break;

    // ── Main menu ────────────────────────────────────────────
    case SCR_MAIN:
      if      (evt == EVT_UP)   { if (cursor > 0) cursor--; }
      else if (evt == EVT_DOWN) { if (cursor < MAIN_COUNT - 1) cursor++; }
      else if (evt == EVT_ENTER) {
        if      (cursor == 0) { follower_start(); go(SCR_START); }
        else if (cursor == 1) go(SCR_CALIBRATE);
        else if (cursor == 2) go(SCR_TUNING);
        else if (cursor == 3) go(SCR_TESTING);
      }
      break;

    // ── Tuning submenu ───────────────────────────────────────
    case SCR_TUNING:
      if      (evt == EVT_UP)   { if (cursor > 0) cursor--; }
      else if (evt == EVT_DOWN) { if (cursor < TUNING_COUNT - 1) cursor++; }
      else if (evt == EVT_EXIT) go(SCR_MAIN);
      else if (evt == EVT_ENTER) {
        if      (cursor == 0) go(SCR_PID_MENU);
        else if (cursor == 1) go(SCR_EDIT_SPEED);
        else if (cursor == 2) go(SCR_EDIT_CAL_TIME);
        else if (cursor == 3) go(SCR_EDIT_LINE_COLOR);
      }
      break;

    // ── PID submenu ───────────────────────────────────────
    case SCR_PID_MENU:
      if      (evt == EVT_UP)   { if (cursor > 0) cursor--; }
      else if (evt == EVT_DOWN) { if (cursor < PID_COUNT - 1) cursor++; }
      else if (evt == EVT_EXIT) go(SCR_TUNING);
      else if (evt == EVT_ENTER) {
        if      (cursor == 0) go(SCR_EDIT_KP);
        else if (cursor == 1) go(SCR_EDIT_KI);
        else if (cursor == 2) go(SCR_EDIT_KD);
      }
      break;

    // ── Testing submenu ──────────────────────────────────────
    case SCR_TESTING:
      if      (evt == EVT_UP)   { if (cursor > 0) cursor--; }
      else if (evt == EVT_DOWN) { if (cursor < TESTING_COUNT - 1) cursor++; }
      else if (evt == EVT_EXIT) go(SCR_MAIN);
      else if (evt == EVT_ENTER) {
        if      (cursor == 0) go(SCR_TEST_MOTOR_MENU);
        else if (cursor == 1) go(SCR_TEST_SENSOR);
      }
      break;

    // ── Motor Test submenu ───────────────────────────────────
    case SCR_TEST_MOTOR_MENU:
      if      (evt == EVT_UP)   { if (cursor > 0) cursor--; }
      else if (evt == EVT_DOWN) { if (cursor < MOTOR_TEST_COUNT - 1) cursor++; }
      else if (evt == EVT_EXIT) go(SCR_TESTING);
      else if (evt == EVT_ENTER) {
        if      (cursor == 0) go(SCR_TEST_LEFT);
        else if (cursor == 1) go(SCR_TEST_RIGHT);
        else if (cursor == 2) go(SCR_TEST_FWD);
        else if (cursor == 3) go(SCR_TEST_BWD);
      }
      break;

    // ── Edit: Kp ─────────────────────────────────────────────
    case SCR_EDIT_KP:
      if      (evt == EVT_UP)            cfg_kP += 0.01f;
      else if (evt == EVT_DOWN)          cfg_kP -= 0.01f;
      else if (evt == EVT_ENTER || evt == EVT_EXIT) go(SCR_PID_MENU);
      if (cfg_kP < 0) cfg_kP = 0;
      break;

    // ── Edit: Ki ─────────────────────────────────────────────
    case SCR_EDIT_KI:
      if      (evt == EVT_UP)            cfg_kI += 0.01f;
      else if (evt == EVT_DOWN)          cfg_kI -= 0.01f;
      else if (evt == EVT_ENTER || evt == EVT_EXIT) go(SCR_PID_MENU);
      if (cfg_kI < 0) cfg_kI = 0;
      break;

    // ── Edit: Kd ─────────────────────────────────────────────
    case SCR_EDIT_KD:
      if      (evt == EVT_UP)            cfg_kD += 0.01f;
      else if (evt == EVT_DOWN)          cfg_kD -= 0.01f;
      else if (evt == EVT_ENTER || evt == EVT_EXIT) go(SCR_PID_MENU);
      if (cfg_kD < 0) cfg_kD = 0;
      break;

    // ── Edit: Speed ───────────────────────────────────────────
    case SCR_EDIT_SPEED:
      if      (evt == EVT_UP)            { cfg_speed += 5; if (cfg_speed > 255) cfg_speed = 255; }
      else if (evt == EVT_DOWN)          { cfg_speed -= 5; if (cfg_speed < 0)   cfg_speed = 0; }
      else if (evt == EVT_ENTER || evt == EVT_EXIT) go(SCR_TUNING);
      break;

    // ── Edit: Cal Time ────────────────────────────────────────
    case SCR_EDIT_CAL_TIME:
      if      (evt == EVT_UP)            { cfg_cal_time++; if (cfg_cal_time > 30) cfg_cal_time = 30; }
      else if (evt == EVT_DOWN)          { cfg_cal_time--; if (cfg_cal_time < 1)  cfg_cal_time = 1; }
      else if (evt == EVT_ENTER || evt == EVT_EXIT) go(SCR_TUNING);
      break;

    // ── Edit: Line Color ──────────────────────────────────────
    case SCR_EDIT_LINE_COLOR:
      if      (evt == EVT_UP || evt == EVT_DOWN) cfg_line_color = !cfg_line_color;
      else if (evt == EVT_ENTER || evt == EVT_EXIT) go(SCR_TUNING);
      break;

    // ── Running / Calibrating / Testing ──────────────────────
    case SCR_START:
      if (evt == EVT_EXIT) {
        motor_left(0);
        motor_right(0);
        go(SCR_MAIN);
      } else {
        dirty = false;
      }
      break;

    case SCR_CALIBRATE:
    case SCR_TEST_SENSOR:
      if (evt == EVT_EXIT) go(SCR_MAIN);
      else dirty = false;
      break;

    case SCR_TEST_LEFT:
    case SCR_TEST_RIGHT:
    case SCR_TEST_FWD:
    case SCR_TEST_BWD:
      if (evt == EVT_EXIT) {
        motor_left(0);
        motor_right(0);
        go(SCR_TEST_MOTOR_MENU);
      } else {
        dirty = false;
      }
      break;

    default: break;
  }
}
