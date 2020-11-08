#include "dbus.h"

void dbusGetMsg(P_DBUS_MSG pmsg, uint8_t data[])
{
    pmsg->ch0 = ((int16_t)data[0] | ((int16_t)data[1] << 8)) & 0x07FF;
    pmsg->ch1 = (((int16_t)data[1] >> 3) | ((int16_t)data[2] << 5)) & 0x07FF;
    pmsg->ch2 = (((int16_t)data[2] >> 6) | ((int16_t)data[3] << 2) | ((int16_t)data[4] << 10)) & 0x07FF;
    pmsg->ch3 = (((int16_t)data[4] >> 1) | ((int16_t)data[5] << 7)) & 0x07FF;

    pmsg->s1 = (DBUS_SW)((data[5] >> 4) & 0x000C) >> 2;
    pmsg->s2 = (DBUS_SW)((data[5] >> 4) & 0x0003);

    pmsg->ch0 -= 1024;
    pmsg->ch1 -= 1024;
    pmsg->ch2 -= 1024;
    pmsg->ch3 -= 1024;

    pmsg->count++;
}


void dbusGetCtrl(P_DBUS_MSG pmsg, P_DBUS_CTRL pctrl)
{
    switch (pmsg->s1)
    {
    case DBUS_SW_UP:
        switch (pmsg->s2)
        {
        case DBUS_SW_UP:
            pctrl->mode = CTRL_MODE_MANUAL_FAST;
            break;
        case DBUS_SW_MIDDLE:
            pctrl->mode = CTRL_MODE_MANUAL_SLOW;
            break;
        case DBUS_SW_DOWN:
            pctrl->mode = CTRL_MODE_MANUAL_SPEED;
            break;
        default:
            break;
        }
        break;

    case DBUS_SW_MIDDLE:
        switch (pmsg->s2)
        {
        case DBUS_SW_UP:
            pctrl->mode = CTRL_MODE_AUTO_DIR;
            break;
        case DBUS_SW_MIDDLE:
            pctrl->mode = CTRL_MODE_AUTO_ALL;
            break;
        case DBUS_SW_DOWN:
            pctrl->mode = CTRL_MODE_AUTO_ALL;
            break;
        default:
            break;
        }
        break;    
    
    case DBUS_SW_DOWN:
        pctrl->mode = CTRL_MODE_STOP;
        break;
    
    default:
        break;
    }

    pctrl->ch_acce = pmsg->ch1;
    pctrl->ch_dir = pmsg->ch2;
}
