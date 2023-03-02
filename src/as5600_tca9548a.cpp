#include <Arduino.h>
#include <Wire.h>
#include <TCA9548A.h>
#include <AS5600.h>
#include "as5600_tca9548a.h"

static AS5600 as5600_1;
static AS5600 as5600_2;
static AS5600 as5600_3;
static AS5600 as5600_4;
static AS5600 as5600[4] = {as5600_1, as5600_2, as5600_3, as5600_4};

static TCA9548A I2CMux;

// setup()で呼び出す
void as5600_tca9548a_init(int dir_pin){
    I2CMux.begin(Wire);
    I2CMux.closeAll();
    for (int i = 0; i < 4; i++) {
        as5600[i].begin(dir_pin);
        as5600[i].setDirection(AS5600_CLOCK_WISE);
    }
}

// offsetを取得する
void as5600_tca9548a_get_offset(float offset1[4]){
    for (int i = 0; i < 4; i++) {
        I2CMux.openChannel(i);
        offset1[i] = as5600[i].getCumulativePosition() * AS5600_RAW_TO_DEGREES / 9.555; // 9.555はDifferential Swerve Driveのギア比
        I2CMux.closeChannel(i);
    }
}

// 現在の角度を取得する(0 to 360)
void as5600_tca9548a_get_current_angle(float current_angle[4], float offset1[4], float offset2[4]){
    for (int i = 0; i < 4; i++){
      I2CMux.openChannel(i);
      as5600[i].getCumulativePosition(); // これがないと，getCumulativePosition()の値がおかしくなる
      // -360 to 0 to 360
      current_angle[i] = (as5600[i].getCumulativePosition()*AS5600_RAW_TO_DEGREES / 9.555) - offset1[i] - offset2[i];
      I2CMux.closeChannel(i);

      // 回転時の補正（これが曲者）
      if (current_angle[i] >= 360) { // 360度以上,時計回りしたらリセット
        offset2[i] += 360 - (as5600[i].resetPosition() * AS5600_RAW_TO_DEGREES / 9.555); 
      } else if (current_angle[i] <= -360) { // -360度以上,反時計回りしたらリセット
        offset2[i] += -360 - (as5600[i].resetPosition() * AS5600_RAW_TO_DEGREES / 9.555); 
      }
      
      // -360 to 0 -> 0 to 360: 反時計回りしたとき，マイナスになるので補正(例：-1 -> 359)
      if (current_angle[i] < 0) {
        current_angle[i] = 360 + current_angle[i];
      }

      // 0 to 360 -> -180 to 180
      current_angle[i] = map(current_angle[i], 0, 360, -180, 180);
    }
}