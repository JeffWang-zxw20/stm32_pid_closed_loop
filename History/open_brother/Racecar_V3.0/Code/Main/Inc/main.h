#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp.h"
#include "device.h"
#include "tools.h"
#include "host.h"
#include "control.h"

// #define ROCKER_RANGE                (660.0f)     //摇杆范围为-660 ~ +660
// #define MANUAL_MAX_SPEED_PULSE      10000   //手动速度模式下的编码器最快转速，单位为5ms内的脉冲数

#define UART_DMA_RX_SIZE    (128)

typedef struct
{
    int16_t target_speed;
    int16_t acce_us;         //目标油门，pwm = 1500 + target_acce
    int16_t dir_us;          //目标方向，向右为正，pwm = 1500 + target_dir
} MAIN_CTRL, *P_MAIN_CTRL;

typedef struct
{
    uint32_t clock;
    uint32_t clock2ms;
    uint32_t clock5ms;
    uint32_t clock10ms;
    uint32_t clock50ms;
    uint32_t clock100ms;
    uint32_t count2ms;
    uint32_t count5ms;
    uint32_t count10ms;
    uint32_t count50ms;
    uint32_t count100ms;
} CLOCK;

extern DBUS_MSG dbus_msg;
extern DBUS_CTRL dbus_ctrl;
extern ENCODER encoder;
extern MAIN_CTRL main_ctrl;
extern CLOCK clock;

void clock_2ms_callback(void);
void clock_5ms_callback(void);
void clock_10ms_callback(void);
void clock_50ms_callback(void);
void clock_100ms_callback(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */


