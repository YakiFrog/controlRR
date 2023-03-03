#include <Arduino.h>
#include "pid_m2006.h"

PID_M2006::PID_M2006(float Kp, float Ki, float Kd, float dt) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->dt = dt;
  this->reset();
}

void PID_M2006::reset() {
  this->p = 0;
  this->i = 0;
  this->d = 0;
  this->last_error = 0;
  this->last_time = millis();
}

float PID_M2006::update(float error) {
  unsigned long now = millis();
  float dt = (now - this->last_time) / 1000.0;
  this->last_time = now;
  this->p = error;
  this->i += error * dt;
  if (dt > 0) {
    this->d = (error - this->last_error) / dt;
  } else {
    this->d = 0;
  }
  this->last_error = error;
  return this->Kp * this->p + this->Ki * this->i + this->Kd * this->d;
}
