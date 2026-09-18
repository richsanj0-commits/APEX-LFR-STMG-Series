#pragma once
#include <stdint.h>

// Array holding the raw 12-bit analog values (0-4095)
extern uint16_t sensor_values[14];
extern uint16_t sensor_min[14];
extern uint16_t sensor_max[14];
extern uint16_t sensor_threshold[14];

void sensors_init();
void sensors_read_raw();
void sensors_calibrate(int cal_time_sec);
float sensors_get_error(int line_color);
