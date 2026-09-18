// ============================================================
//  buttons.cpp  –  Debounced button reader
// ============================================================
#include "buttons.h"
#include "pins.h"
#include <Arduino.h>

void buttons_init() {
  pinMode(BTN_ENTER, INPUT_PULLUP);
  pinMode(BTN_EXIT,  INPUT_PULLUP);
  pinMode(BTN_UP,    INPUT_PULLUP);
  pinMode(BTN_DOWN,  INPUT_PULLUP);
}

// Blocking debounce: detects press, waits for release, returns event
ButtonEvent buttons_read() {
  struct { int pin; ButtonEvent evt; } map[] = {
    { BTN_ENTER, EVT_ENTER },
    { BTN_EXIT,  EVT_EXIT  },
    { BTN_UP,    EVT_UP    },
    { BTN_DOWN,  EVT_DOWN  },
  };

  for (auto& b : map) {
    if (digitalRead(b.pin) == LOW) {
      delay(30);                          // debounce
      if (digitalRead(b.pin) == LOW) {
        while (digitalRead(b.pin) == LOW);  // wait for release
        return b.evt;
      }
    }
  }
  return EVT_NONE;
}
