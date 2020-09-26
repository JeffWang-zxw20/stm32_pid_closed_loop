#ifndef __DBUS_H
#define __DBUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "device.h"

typedef enum
{
    DBUS_SW_NONE    = 0,    //无
    DBUS_SW_UP      = 1,    //上
    DBUS_SW_MIDDLE  = 3,    //中
    DBUS_SW_DOWN    = 2     //下
} DBUS_SW;

typedef struct
{
    //-660~660
    int16_t ch0;    //右手左右      
    int16_t ch1;    //右手上下
    int16_t ch2;    //左手左右
    int16_t ch3;    //左手上下
    DBUS_SW s1;     //左手开关
    DBUS_SW s2;     //右手开关
    uint32_t count;
} DBUS_MSG, *P_DBUS_MSG;

typedef enum
{
    CTRL_MODE_STOP,             //停止模式
    CTRL_MODE_MANUAL_FAST,      //手动快速油门模式
    CTRL_MODE_MANUAL_SLOW,      //手动慢速油门模式
    CTRL_MODE_MANUAL_SPEED,     //手动定速模式
    CTRL_MODE_AUTO_DIR,         //自动模式，仅方向自动，油门为定速手动模式
    CTRL_MODE_AUTO_ALL          //全自动模式
} DBUS_CTRL_MODE;

typedef struct
{
    DBUS_CTRL_MODE mode;
    int16_t ch_acce;    //油门通道杆量
    int16_t ch_dir;     //方向通道杆量，向右为正
} DBUS_CTRL, *P_DBUS_CTRL;

void dbusGetMsg(P_DBUS_MSG pmsg, uint8_t data[]);
void dbusGetCtrl(P_DBUS_MSG pmsg, P_DBUS_CTRL pctrl);


#ifdef __cplusplus
}
#endif

#endif



