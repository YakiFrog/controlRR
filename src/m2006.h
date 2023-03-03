#ifndef __M2006_H__
#define __M2006_H__

#include <Arduino.h>
#include <CAN.h>

bool can_init(int rx, int tx, int speed);
void m2006_make_data(int16_t data_in[8], uint8_t data_out1[8], uint8_t data_out2[8]);
bool m2006_send_data(uint8_t data_out1[8], uint8_t data_out2[8]);
void m2006_read_data(int id, int16_t mangle[8], int16_t mrpm[8], int16_t mtorque[8]);

#endif // __M2006_H__