// ============================================================
//  main.ino  –  Entry point
// ============================================================
#include "pins.h"
#include "buttons.h"
#include "display.h"
#include "menu.h"
#include "motors.h"
#include "sensors.h"

void setup() {
  Serial.begin(9600);
  buttons_init();
  display_init();
  motors_init();
  sensors_init();
  menu_init();
}

void loop() {
  // Draw screen if something changed
  menu_draw();

  // Read buttons (non-blocking)
  ButtonEvent evt = buttons_read();
  if (evt == EVT_NONE) return;

  // Let the menu handle the button press
  menu_handle(evt);
}
