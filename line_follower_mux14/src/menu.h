// ============================================================
//  menu.h  –  Menu system declarations
// ============================================================
#pragma once
#include "buttons.h"

// All possible screens
enum Screen {
  SCR_WELCOME,
  SCR_MAIN,
  // Main actions
  SCR_START,
  SCR_CALIBRATE,
  // Tuning submenu
  SCR_TUNING,
  SCR_PID_MENU,
  SCR_EDIT_KP,
  SCR_EDIT_KI,
  SCR_EDIT_KD,
  SCR_EDIT_SPEED,
  SCR_EDIT_CAL_TIME,
  SCR_EDIT_LINE_COLOR,
  // Testing submenu
  SCR_TESTING,
  SCR_TEST_MOTOR_MENU,
  SCR_TEST_LEFT,
  SCR_TEST_RIGHT,
  SCR_TEST_FWD,
  SCR_TEST_BWD,
  SCR_TEST_SENSOR,
};

void menu_init();
void menu_handle(ButtonEvent evt);
Screen menu_current();          // which screen is active?
void menu_draw();               // call this whenever display needs refresh
