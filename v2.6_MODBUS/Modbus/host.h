
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
    int16_t pulse;              //������ת�٣�5ms�ڱ�����������
    int32_t cycle;              //�ۼ�Ȧ��
    int16_t voltage1;           //���1��ѹ
    int16_t voltage2;           //���2��ѹ
    uint8_t dir_mode;           //ת��������ģʽ��0Ϊ����λ��1ΪPWM������ƣ�2Ϊת��Ƕȿ���
    uint8_t degree;          //���ſ���ģʽ��0Ϊ����λ��1Ϊ���Űٷֱȿ��ƣ�2Ϊ�ٶȿ���
    int16_t acce_x;         //���PWM����ֵ��-500~500
    int16_t acce_y;      //����Ƕ�=value/10
    int16_t acce_z;           //���Űٷֱ�=(value/100)%
    int16_t trg_speed;          //�ٶ�ֵ��Ӧ������5ms������
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