// ============================================================
//  pins.h  –  All hardware pin definitions in one place
// ============================================================
#pragma once

// ── TB6612FNG Motor Driver ──────────────────────────────────
#define AIN1  PB0     // Left motor direction 1
#define AIN2  PB1     // Left motor direction 2
#define BIN1  PB10    // Right motor direction 1
#define BIN2  PB11    // Right motor direction 2
#define PWMA  PA8     // Left motor PWM
#define PWMB  PA9     // Right motor PWM
#define STBY  PA10    // Motor driver standby (HIGH = enabled)

// ── CD74HC4067 MUX (14-sensor array) ───────────────────────
#define MUX_S0   PA7
#define MUX_S1   PA6
#define MUX_S2   PA5
#define MUX_S3   PA4
#define MUX_SIG  PA2   // Analog input from MUX signal pin

#define SENSOR_COUNT 14

// ── SSD1306 OLED (I2C) ─────────────────────────────────────
#define OLED_SDA  PB7
#define OLED_SCL  PB6
#define OLED_ADDR 0x3C
#define SCREEN_W  128
#define SCREEN_H  64

// ── Buttons (active LOW, other leg to GND) ──────────────────
#define BTN_ENTER PC13
#define BTN_EXIT  PB3
#define BTN_UP    PB5
#define BTN_DOWN  PB4
