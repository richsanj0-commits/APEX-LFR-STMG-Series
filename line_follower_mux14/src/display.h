// ============================================================
//  display.h  –  Thin OLED wrapper (128x64 SSD1306)
// ============================================================
#pragma once

void display_init();
void display_clear();
void display_text(int row, const char* text);  // row 0–7
void display_show();
