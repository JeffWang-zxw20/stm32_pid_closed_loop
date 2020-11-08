//创建一个serial类 
serial::Serial sp; 
uint8_t buff[128];
uint32_t bufflen;
int16_t dir_us;
int16_t speed_pulse;   
//2.5m  72 circle encoder(1024line)
//1m/s  ==>  72/2.5*1024/200=147 pause per 5ms 
int16_t speed2pulse(double speed_mps)
{
    return (int16_t)(speed_mps *147.46);
}
int16_t steerangle2dirus(double steerangle)
{
    return (int16_t)(-steerangle/0.349*500);
}
void send_cmd(planner::cmd2car cmd)
{
    double velocity = cmd.velocity;
    double steer_angle = cmd.steer_angle;

    //speed and angle convert
    speed_pulse = speed2pulse(velocity);
    dir_us = steerangle2dirus(steer_angle);
    // cout << "---------------------" << endl;
    // cout << "steer_angle: " << steer_angle << endl;
    // cout << "dir_us: " << dir_us << endl;
    // cout << "---------------------" << endl;

    modbus_ctrl(dir_us, speed_pulse, buff, &bufflen);
    
    // static int32_t ii=0;
    // ii++;
    // cout << "sent: " << ii << endl;
    if(serial_port_check()==1)
        sp.write(buff, bufflen);
}