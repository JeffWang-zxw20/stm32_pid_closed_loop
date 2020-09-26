#ifndef __ENCODER_H
#define __ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "device.h"

typedef struct
{
    int32_t pulse;  //编码器转速，pulse/5ms
    int32_t cycle;  //累计圈数
} ENCODER, *P_ENCODER;

//向前返回1，向后返回-1
int8_t EncoderReadDir(void);
void EncoderInit(void);
void EncoderUpdatePulse(P_ENCODER phdl);

#ifdef __cplusplus
}
#endif

#endif



