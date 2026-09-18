// ============================================================
//  buttons.h  –  Debounced button reader
// ============================================================
#pragma once

enum ButtonEvent { EVT_NONE, EVT_ENTER, EVT_EXIT, EVT_UP, EVT_DOWN };

void buttons_init();
ButtonEvent buttons_read();   // non-blocking, returns EVT_NONE if no press
