#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "device.h"


#define DIR_OFFSET_US   -75

void Motor_Init(void);
void motor_set_acce_us(int16_t us);
void motor_set_dir_us(int16_t us);
void motor_set(int16_t value);



#ifdef __cplusplus
}
#endif

#endif



