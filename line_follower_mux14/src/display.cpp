// ============================================================
//  display.cpp  –  Thin OLED wrapper (128x64 SSD1306)
// ============================================================
#include "display.h"
#include "pins.h"
#include <U8g2lib.h>

static U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2(U8G2_R0, /*SCL=*/PB6, /*SDA=*/PB7, U8X8_PIN_NONE);

void display_init() {
  u8g2.begin();
  u8g2.setFont(u8g2_font_5x7_tr);
}

void display_clear() {
  u8g2.clearBuffer();
}

void display_text(int row, const char* text) {
  // 5x7 font is 7 pixels tall. +1 for spacing = 8px per row.
  // u8g2 draws from bottom-left of the glyph, so row 0 needs y=8.
  u8g2.drawStr(0, (row * 8) + 8, text);
}

void display_show() {
  u8g2.sendBuffer();
}
