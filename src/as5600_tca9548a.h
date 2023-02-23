#ifndef __AS5600_TCA9548A_H__
#define __AS5600_TCA9548A_H__

#include <Arduino.h>
#include <Wire.h>
#include <TCA9548A.h>
#include <AS5600.h>

void as5600_tca9548a_init(int dir_pin);
void as5600_tca9548a_get_offset(float offset1[4]);
void as5600_tca9548a_get_current_angle(float current_angle[4], float offset1[4], float offset2[4]);

#endif // __AS5600_TCA9548A_H__