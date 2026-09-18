#include "sensors.h"
#include "pins.h"
#include "motors.h"
#include <Arduino.h>

uint16_t sensor_values[14];
uint16_t sensor_min[14];
uint16_t sensor_max[14];
uint16_t sensor_threshold[14];

static float last_error = 0.0f;

void sensors_init() {
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  pinMode(MUX_SIG, INPUT);
  
  // Set ADC to 12-bit resolution for 0-4095 range
  analogReadResolution(12);

  // Initialize thresholds to mid-point just in case
  for (int i = 0; i < 14; i++) {
    sensor_threshold[i] = 2048;
  }
}

void sensors_read_raw() {
  for (int i = 0; i < 14; i++) {
    // Set MUX selection pins
    digitalWrite(MUX_S0, i & 0x01);
    digitalWrite(MUX_S1, (i >> 1) & 0x01);
    digitalWrite(MUX_S2, (i >> 2) & 0x01);
    digitalWrite(MUX_S3, (i >> 3) & 0x01);
    
    // Give MUX and ADC capacitor more time to switch and settle
    delayMicroseconds(50); 
    
    // Read and store
    sensor_values[i] = analogRead(MUX_SIG);
  }
}

void sensors_calibrate(int cal_time_sec) {
  // Init: read actual values first to prevent any edge-case logic failures 
  // (like a sensor stuck exactly at 4095 never updating sensor_min)
  sensors_read_raw();
  for (int i = 0; i < 14; i++) {
    sensor_min[i] = sensor_values[i];
    sensor_max[i] = sensor_values[i];
  }

  // Spin bot in place slowly
  motor_left(60);
  motor_right(-60);

  unsigned long start_ms = millis();
  unsigned long duration = cal_time_sec * 1000UL;

  while (millis() - start_ms < duration) {
    sensors_read_raw();
    for (int i = 0; i < 14; i++) {
      if (sensor_values[i] < sensor_min[i]) sensor_min[i] = sensor_values[i];
      if (sensor_values[i] > sensor_max[i]) sensor_max[i] = sensor_values[i];
    }
  }

  // Stop bot
  motor_left(0);
  motor_right(0);

  // Calculate thresholds
  for (int i = 0; i < 14; i++) {
    sensor_threshold[i] = (sensor_min[i] + sensor_max[i]) / 2;
  }
}

float sensors_get_error(int line_color) {
  sensors_read_raw();

  // Weights for sensors 0 to 13. Center is between 6 and 7.
  static const float weights[14] = {
    -150.0f, -100.0f, -70.0f, -40.0f, -20.0f, -10.0f, -5.0f,
      +5.0f,  +10.0f, +20.0f, +40.0f, +70.0f, +100.0f, +150.0f
  };

  float sum_value = 0.0f;
  float sum_weight = 0.0f;
  int active_sensors = 0;

  for (int i = 0; i < 14; i++) {
    int digital_val = 0;
    
    // Binary threshold logic (1 or 0)
    // We use the threshold calculated during calibration.
    if (line_color == 0) {
      // Black Line: Higher value = Black
      if (sensor_values[i] > sensor_threshold[i]) digital_val = 1;
    } else {
      // White Line: Lower value = White
      if (sensor_values[i] < sensor_threshold[i]) digital_val = 1;
    }

    if (digital_val == 1) {
      sum_value += weights[i];
      sum_weight += 1.0f;
      active_sensors++;
    }
  }

  // If no line is detected (all sensors = 0), return the last known error to make sharp turns
  if (active_sensors == 0) {
    if (last_error > 0) return 150.0f; // Lost to the right, turn hard right
    if (last_error < 0) return -150.0f; // Lost to the left, turn hard left
    return 0.0f;
  }

  float error = sum_value / sum_weight;
  last_error = error;
  return error;
}
