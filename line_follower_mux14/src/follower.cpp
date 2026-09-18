#include "follower.h"
#include "sensors.h"
#include "motors.h"
#include <Arduino.h>

static float follower_integral = 0.0f;
static float follower_last_error = 0.0f;

void follower_start() {
  follower_integral = 0.0f;
  follower_last_error = 0.0f;
}

void follower_loop(float kp, float ki, float kd, int base_speed, int line_color) {
  float error = sensors_get_error(line_color);
  
  follower_integral += error;
  
  // Anti-windup (prevent integral from growing to infinity)
  if (follower_integral > 1000.0f) follower_integral = 1000.0f;
  if (follower_integral < -1000.0f) follower_integral = -1000.0f;

  float raw_derivative = error - follower_last_error;
  
  // Simple low-pass filter on derivative to prevent jitter from sensor noise
  static float filtered_derivative = 0.0f;
  filtered_derivative = (0.7f * filtered_derivative) + (0.3f * raw_derivative);
  
  follower_last_error = error;

  float pid_val = (error * kp) + (follower_integral * ki) + (filtered_derivative * kd);

  // Advanced: Dynamic Base Speed (Auto-braking on corners)
  // When error is high (sharp turn), slow down the forward momentum so it doesn't overshoot.
  int current_base_speed = base_speed;
  float abs_error = abs(error);
  if (abs_error > 10.0f) {
    // Reduce speed as error increases. At max error (150), speed reduces to 20% of base_speed.
    float speed_factor = 1.0f - ((abs_error - 10.0f) / 175.0f); 
    if (speed_factor < 0.2f) speed_factor = 0.2f;
    current_base_speed = (int)(base_speed * speed_factor);
  }

  // Calculate new motor speeds based on dynamic base speed and PID correction
  int left_speed = current_base_speed + (int)pid_val;
  int right_speed = current_base_speed - (int)pid_val;

  // Constrain speeds to physical limits
  left_speed = constrain(left_speed, -255, 255);
  right_speed = constrain(right_speed, -255, 255);

  motor_left(left_speed);
  motor_right(right_speed);
}
