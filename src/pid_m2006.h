#pragma once
#ifndef __PIDM2006_H__
#define __PIDM2006_H__

#include <Arduino.h>

// PID制御クラス
class PID_M2006 {
  public: // public メソッド
    PID_M2006(float Kp, float Ki, float Kd, float dt);
    void reset();
    float update(float error);

  private: // private メンバ変数
    float Kp;
    float Ki;
    float Kd;
    float dt;
    float p;
    float i;
    float d;
    float last_error;
    unsigned long last_time;
};

#endif // __PIDM2006_H__