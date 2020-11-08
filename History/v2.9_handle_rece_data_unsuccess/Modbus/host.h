
#ifndef __HOST_H
#define __HOST_H

#include "main.h"

#define TABLE_LEN           0x16
#define MODBUS_WRITE        1
#define MODBUS_READ         2
#define MODBUS_FEEDBACK     3


typedef struct
{
    uint8_t hardware_version;
    uint8_t slave_id;
    int16_t pulse;              //编码器转速，5ms内编码器脉冲数
    int32_t cycle;              //累计圈数
    int16_t voltage1;           //电池1电压
    int16_t voltage2;           //电池2电压
    uint8_t dir_mode;           //转向舵机控制模式：0为中立位，1为PWM脉宽控制，2为转向角度控制
    uint8_t degree;          //油门控制模式，0为中立位，1为油门百分比控制，2为速度控制
    int16_t acce_x;         //舵机PWM脉宽值，-500~500
    int16_t acce_y;      //舵机角度=value/10
    int16_t acce_z;           //油门百分比=(value/100)%
    int16_t trg_speed;          //速度值对应编码器5ms脉冲数
} MODBUS_TABLE, *P_MODBUS_TABLE;

typedef struct
{
    uint8_t order;  //read 
    uint8_t offset;
    uint8_t len;
    uint8_t data[128];

    uint32_t count_correct;
    uint32_t count_error;  
} MODBUS_MSG, *P_MODBUS_MSG;

void table_write(uint16_t offset, uint16_t len, uint8_t wr[], P_MODBUS_TABLE ptable);

void modbus_feedback(P_MODBUS_TABLE ptable, uint8_t offset, uint8_t len, uint8_t buff[], uint32_t *bufflen);
int8_t modbus_pack_analize(uint8_t rxdata[], uint32_t rxlen, P_MODBUS_MSG pmsg);



#endif