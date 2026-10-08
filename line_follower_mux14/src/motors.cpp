#include "motors.h"
#include "pins.h"
#include <Arduino.h>

void motors_init() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  
  // Enable the TB6612 driver
  digitalWrite(STBY, HIGH);
}

void motor_left(int speed) {
  // The left motor is physically connected to the B pins.
  // BIN1=LOW, BIN2=HIGH drives it FORWARD.
  speed = constrain(speed, -255, 255);
  
  if (speed > 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, speed);
  } else if (speed < 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, -speed);
  } else {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, 0);
  }
}

void motor_right(int speed) {
  // The right motor is physically connected to the A pins.
  // Inverted: AIN1=LOW, AIN2=HIGH drives it FORWARD.
  speed = constrain(speed, -255, 255);
  
  if (speed > 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, speed);
  } else if (speed < 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, -speed);
  } else {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, 0);
  }
}
