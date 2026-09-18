#pragma once

// Reset PID state variables (integral and last_error)
void follower_start();

// Run one iteration of the PID line following loop
void follower_loop(float kp, float ki, float kd, int base_speed, int line_color);
