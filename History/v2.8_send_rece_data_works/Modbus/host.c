#include "host.h"


void table_write(uint16_t offset, uint16_t len, uint8_t wr[], P_MODBUS_TABLE ptable)
{
    uint8_t *arr = (uint8_t*)ptable;
    int i;
    
    if(len >= 1 && offset + len <= TABLE_LEN)
    {
        for(i = 0; i <= len - 1; i++)
        {
            arr[offset + i] = wr[i];
        }
    }
}

int8_t modbus_pack_analize(uint8_t rxdata[], uint32_t rxlen, P_MODBUS_MSG pmsg)
{
    int i;
    uint8_t sum = 0;

    //��ͷ����
    if((rxdata[0] != 0xAA) || (rxdata[1] != 0xAA) || (rxdata[2] != 0xAA) || (rxdata[3] != 0xAA))
    {
        pmsg->count_error++;
        return -1;
    }
    
    //�������ʹ���
    if(rxdata[4] != MODBUS_WRITE && rxdata[4] != MODBUS_READ && rxdata[4] != MODBUS_FEEDBACK)
    {
        pmsg->count_error++;
        return -2;
    }

    //���ݳ���Խ��
    if(rxdata[5] + rxdata[6] > TABLE_LEN)
    {
        pmsg->count_error++;
        return -3;
    }

    //���ݳ��������ݰ����ȴ���ì��
    if((rxdata[6] + 8) != rxlen)
    {
        pmsg->count_error++;
        return -4;
    }

    sum = 0;
    for(i = 0; i <= rxlen - 2; i++)
    {
        sum ^= rxdata[i];
    }

    //У�����
    if(sum != rxdata[rxlen - 1])
    {
        pmsg->count_error++;
        return -5;
    }

    pmsg->order = rxdata[4];
    pmsg->offset= rxdata[5];
    pmsg->len= rxdata[6];
    for(i = 0; i <= pmsg->len - 1; i++)
    {
        pmsg->data[i] = rxdata[i + 7];
    }

    pmsg->count_correct++;
    return 0;
}

void modbus_feedback(P_MODBUS_TABLE ptable, uint8_t offset, uint8_t len, uint8_t buff[], uint32_t *bufflen)
{
    uint8_t *table = (uint8_t*)ptable;
		int i;
    if(len >= 1 && offset + len <= TABLE_LEN)
    {
        //���ɵ����ݰ��ܳ���
        *bufflen = 8 + len;

        //����ͷ
        buff[0] = 0xaa;
        buff[1] = 0xaa;
        buff[2] = 0xaa;
        buff[3] = 0xaa;

        //��������
        buff[4] = MODBUS_FEEDBACK;

        //���ݳ���
        buff[5] = offset;

        //���ݳ���
        buff[6] = len;

        //������
        for(i = 0; i <= len - 1; i++)
        {
            buff[i + 7] = table[offset + i];
        }


        //��λ���У��
        buff[7 + len] = 0;
        for(i = 0; i <= 7 + len - 1; i++)
        {
            buff[7 + len] ^= buff[i];
        }
    }
}



