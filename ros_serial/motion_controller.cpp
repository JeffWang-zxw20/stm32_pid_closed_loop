#include "motion_controller.h"

using namespace std;
using namespace Eigen;

//serial port
//=====================================================================

typedef enum   
{
    MODBUS_RS_HEADER = 0,
    MODBUS_RS_TYPE,
    MODBUS_RS_OFFSET,
    MODBUS_RS_LEN,
    MODBUS_RS_DATA,
    MODBUS_RS_CHECK
} MODBUS_RS;

typedef struct
{
    MODBUS_RS rs;
    int count_header;
    unsigned char data[32];    
    int count_data;

    int16_t pulse;
    int32_t cycle;

    int count_corret;
    int count_error;
}MODBUS_HANDLE, *P_MODBUS_HANDLE;

MODBUS_HANDLE hmodbus;

void modbus_iterator(P_MODBUS_HANDLE phdl, unsigned char data)
{
    unsigned char sum = 0;
    int i;

    if(data == 0xaa)
    {
        phdl->count_header++;
    }
    else
    {
        phdl->count_header = 0;
    }
    
    if(phdl->count_header == 4)
    {
        phdl->count_header = 0;
        phdl->rs = MODBUS_RS_TYPE;
        return;
    }
    
    switch (phdl->rs)
    {
        case MODBUS_RS_HEADER:
            break;

        case MODBUS_RS_TYPE:
            if(data != 0x03)
            {
                phdl->rs = MODBUS_RS_HEADER;
            }
            else
            {
                phdl->rs = MODBUS_RS_OFFSET;
            }
            break;

        case MODBUS_RS_OFFSET:
            if(data != 0x02)
            {
                phdl->rs = MODBUS_RS_HEADER;
            }
            else
            {
                phdl->rs = MODBUS_RS_LEN;
            }
            break;

        case MODBUS_RS_LEN:
            if(data != 0x06)
            {
                phdl->rs = MODBUS_RS_HEADER;
            }
            else
            {
                phdl->rs = MODBUS_RS_DATA;
                phdl->count_data = 0;
            }
            break;

        case MODBUS_RS_DATA:
            phdl->data[phdl->count_data] = data;
            phdl->count_data++;
            if(phdl->count_data >= 6)
            {
                phdl->rs = MODBUS_RS_CHECK;
            }
            break;

        case MODBUS_RS_CHECK:
            sum = 0;
            sum ^= 0xaa;
            sum ^= 0xaa;
            sum ^= 0xaa;
            sum ^= 0xaa;
            sum ^= 0x03;
            sum ^= 0x02;
            sum ^= 0x06;

            for(i = 0; i <= 5; i++)
            {
                sum ^= phdl->data[i];
            }
            if(sum == data)
            {
                phdl->pulse = phdl->data[0] | (phdl->data[1] << 8);
                phdl->cycle = phdl->data[2] | (phdl->data[3] << 8) | (phdl->data[4] << 16) | (phdl->data[5] << 24);
                phdl->count_corret++;
                phdl->rs = MODBUS_RS_HEADER;
            }
            else
            {
                // ROS_INFO("error\r\n");
                phdl->count_error++;

            }
            break;

        default:
            phdl->rs = MODBUS_RS_HEADER;
            break;
    }
}

void modbus_write(uint8_t offset, uint8_t len, uint8_t data[], uint8_t buff[], uint32_t *bufflen)
{
    int i;

    if(len >= 1 && offset + len <= 0x16)
    {
        //生成的数据包总长度
        *bufflen = 8 + len;

        //数据头
        buff[0] = 0xaa;
        buff[1] = 0xaa;
        buff[2] = 0xaa;
        buff[3] = 0xaa;

        //数据类型
        buff[4] = 0x01;

        buff[5] = offset;

        //数据长度
        buff[6] = len;

        //数据区
        for(i = 0; i <= len - 1; i++)
        {
            buff[i + 7] = data[i];
        }

        //按位异或校验
        buff[7 + len] = 0;
        for(i = 0; i <= 7 + len - 1; i++)
        {
            buff[7 + len] ^= buff[i];
        }
    }
}

//dir_us -500 500
//speed_pulse -5000 5000
void modbus_ctrl(int16_t dir_us, int16_t speed_pulse, uint8_t buff[], uint32_t *bufflen)
{
    uint8_t data[128];
    data[0] = 1;//dir pwm mode
    data[1] = 2;//speed mode
    data[2] = dir_us & 0xff;
    data[3] = dir_us >> 8;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    data[8] = speed_pulse & 0xff;
    data[9] = speed_pulse >> 8;
    
    modbus_write(0x0c, 10, data, buff, bufflen);
}

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
void send_cmd(planner::cmd2car cmd) //----------------------------------------------send command to stm32 !!!!!!!!!!!!!!!!!!!!!!!!!!!----------------------------------
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

int serial_port_init(string serial_port)
{
    //创建句柄
    ros::NodeHandle node; 

    // //创建一个serial类 
    // serial::Serial sp; 
    //创建timeout 
    sp.setPort(serial_port); 
    sp.setBaudrate(1000000); 
    serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
    sp.setTimeout(to);

    try 
    { 
        //打开串口 
        sp.open(); 
    } 
    catch(serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port.");  
        return -1; 
    } 

    
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        return -1;
    }
    return 1;

}
bool serial_offline = false;
int serial_port_check()
{

    // ROS_ERROR_STREAM("serial port is offline.");
    if(!serial_offline)
    {
        try 
        { 
            sp.available();
        } 
        catch(serial::IOException& e) 
        { 
            serial_offline = true;
            sp.close();
            sp.setPort(serial_port); 
            sp.setBaudrate(1000000); 
            serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
            sp.setTimeout(to);
            ROS_ERROR_STREAM("serial lost!");  
        }
    }
    if(serial_offline)
    {
        try 
        { 
            //打开串口 
            sp.open(); 
        } 
        catch(serial::IOException& e) 
        { 
            ROS_ERROR_STREAM("Unable to reopen port. Ignore cmd this time.");  
            // cout << -1 << endl;
            return -1; 
        } 
        serial_offline = false;
    }

    // cout << 1 << endl;
    return 1;

}
//=====================================================================

// pose Vector4d x, y, theta, v
// pose2d_now: the pose right now (x,y,theta,velocity_now)
// pose2d_target:the pose the next taget point (x,y,theta,velocity_next_target(not velocity_next_step))
// TODO: 角度方向问题需要分析
long int cnt_show = 0;
double motion_controller(Vector4d pose2d_now, Vector4d pose2d_target)
{
    static double alpha_last = 0, beta_last = 0, dis_last = 0;
    static double alpha_sum = 0, alpha_diff = 0, beta_sum = 0, beta_diff = 0;
    static double dis_sum = 0, dis_diff = 0;

    double delta_y = pose2d_target(1) - pose2d_now(1);
    double delta_x = pose2d_target(0) - pose2d_now(0);

    double taget_angle = atan2(delta_y, delta_x);
    double alpha = taget_angle - pose2d_now(2);
    double beta  = taget_angle - pose2d_target(2);
    //prevent alpha and beta change suddenly when euler from -Pi to Pi
    if(alpha > 1*PI) // 3/2*PI
        alpha -= 2*PI;
    if(alpha < -1*PI)
        alpha += 2*PI;
    if(beta > 1*PI)
        beta -= 2*PI;
    if(beta < -1*PI)
        beta += 2*PI;
    // if(abs(alpha) > PI/2)
    //     ROS_ERROR("alpha > PI/2!!");
    // if(abs(beta) > PI/2)
    //     ROS_ERROR("beta > PI/2!!");

    alpha_sum += alpha;
    alpha_diff = alpha - alpha_last;
    alpha_last = alpha;
    beta_sum += beta;
    beta_diff = beta - beta_last;
    beta_last = beta;

    double alpha_w = alpha_p*alpha + alpha_i*alpha_sum + alpha_d*alpha_diff;
    double beta_w = beta_p*beta + beta_i*beta_sum + beta_d*beta_diff;
    // cout << "============================" << endl;
    
// cout << "alpha_w: " << alpha_w << "   beta_w: " << beta_w << endl; 
    // double w = alpha_w+ beta_w;  //slow can faster can't
    double w = alpha_w;// + beta_w;
    // steer_gama = atan2(w*wheel_base, pose2d_now(3));  //-pi/2~pi/2
    // cout << "w " << w << endl; 
    steer_gama = w;  //-pi/2~pi/2
    // if(abs(steer_gama) > PI/2)
    //     ROS_ERROR("Steer angle Error");
    // ROS_ERROR("alpha: %lf, beta: %lf\n",alpha, beta);

    double dis_now2next = sqrt((delta_x*delta_x+delta_y*delta_y));
    dis_sum += dis_now2next;
    dis_diff = dis_now2next - dis_last;
    dis_last = dis_now2next;
    double dis_v = dis_p*dis_now2next + dis_i*dis_sum + dis_d*dis_diff;
    // velocity_next_step = pose2d_target(3) + dis_v;
    //limit the acceleration  
    double max_velocity_changing_step = 1;
    if(pose2d_target(3) >= pose2d_now(3))
        velocity_next_step = min(pose2d_now(3)+max_velocity_changing_step, pose2d_target(3));  
    else
        velocity_next_step = pose2d_target(3); // when slow down, use the target velocity directly
        // velocity_next_step = max(pose2d_now(3)-max_velocity_changing_step, pose2d_target(3)); //bug if pose2d_now(3) is wrong and suddenly become a large speed, it will set a large point of spped atrget!!!!!!!!
    // double soft_factor = 0.5;
    // velocity_next_step = pose2d_target(3)*soft_factor + pose2d_now(3)*(1-soft_factor);

    cnt_show++;
    if(cnt_show%10 == 0 && 1)
    {
        // cout << "target speed: " <<velocity_next_step << endl; 
        //cout << "pose2d_now: " <<endl << pose2d_now << endl;
        //cout << "pose2d_target: " << endl << pose2d_target << endl;
        //cout << "delta_y: " << delta_y << "  delta_x: " << delta_x << endl;
        //cout << "taget_angle:  " << taget_angle << endl;
        //cout << "alpha: " << alpha << "   beta: " << beta << endl; 
        //cout << "============================" << endl;
    }

    return dis_now2next;
}

Vector4d pose2d_now, pose2d_target;

int32_t cntt=0;
planner::cmd2car cmd;
double dis2goal_p;
bool begin_first_point = false;
bool point_arrive = false;
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

    pose2d_now(0) = msg->pose.pose.position.x;
    pose2d_now(1) = msg->pose.pose.position.y;
    pose2d_now(2) = quaternion2yaw(q);  //-pi -- pi
    // cout << "================pose2d: " << pose2d_now(2) << endl;
    pose2d_now(3) = sqrt(msg->twist.twist.linear.x*msg->twist.twist.linear.x
                        +msg->twist.twist.linear.y*msg->twist.twist.linear.y);
                        // +msg->twist.twist.linear.z*msg->twist.twist.linear.z);
    
    dis2goal_p = motion_controller(pose2d_now, pose2d_target);
    // cout << "dis2goal_p" << dis2goal_p << endl;

    if((!point_arrive) && begin_first_point)
    {
        // transform goal into car frame
        Quaterniond q(yaw2quaternion(pose2d_now(2)));
        Matrix3d Rc_w = q.toRotationMatrix();
        Matrix3d Rw_c = Rc_w.inverse();
        Vector3d Tc_w(pose2d_now(0), pose2d_now(1), 0);
        Vector3d Tw_c = -Rw_c*Tc_w;
        Vector3d Ptarget_w(pose2d_target(0), pose2d_target(1), 0);
        Vector3d Ptarget_c = Rw_c*Ptarget_w + Tw_c;

        if(dis2goal_p < tolerance) 
        {
            point_arrive = true;
            planner::status status;
            status.arrive = 1;
            movement_status_pub.publish(status);
            ROS_INFO("Arrive at a point!");
        }
        else if(Ptarget_c(0) < 0) //if the target is behind the car, give up it(x<0in car frame). //need to test 
        {
            point_arrive = true;
            planner::status status;
            status.arrive = 1;
            movement_status_pub.publish(status);
            ROS_INFO("pass at a point!");
        }
    }

    
    //nav_msgs::Path  nav_msgs::Odometry

    //-----------------------------------------------------
    //test
    // static int8_t i = 0;
    // // cmd.velocity = 3 * fabs(sin((float)i / 256.0 * 2 * PI));
    // cmd.velocity = 3;
    // cmd.steer_angle = 20 * sin((float)i / 256.0 * 2.0f * PI);
    // cntt++;
    // if(cntt%10==0)
    // {
    //     i++;
    //     // cmd_pub.publish(cmd);
    // }
    //-----------------------------------------------------

    
}
bool first_point = false;
void targetpose_callback(const planner::goal::ConstPtr &msg)
{
    if(!first_point)
    {
        planner::status status;
        status.arrive = 3;
        movement_status_pub.publish(status);
    }
    first_point = true;

    pose2d_target(0) = msg->x;
    pose2d_target(1) = msg->y;
    pose2d_target(2) = msg->theta;
    pose2d_target(3) = msg->velocity;

    point_arrive = false;
    
    // ROS_INFO("Receive a point!");
}

void motion_controller_init(ros::NodeHandle &n)
{
    n.getParam("serial_port", serial_port);

    n.getParam("alpha_p", alpha_p);
    n.getParam("alpha_i", alpha_i);
    n.getParam("alpha_d", alpha_d);
    n.getParam("beta_p", beta_p);
    n.getParam("beta_i", beta_i);
    n.getParam("beta_d", beta_d);
    n.getParam("dis_p", dis_p);
    n.getParam("dis_i", dis_i);
    n.getParam("dis_d", dis_d);

    n.getParam("tolerance", tolerance);
    n.getParam("controlrate", controlrate); 

    n.getParam("sim", sim);

}


//TODO: only considering move forward
int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_controller");
    ros::NodeHandle n("~");
    motion_controller_init(n);
    cmd_pub = n.advertise<planner::cmd2car>("/cmd2car",4);   //for simulation  
    ros::Subscriber odom_sub = n.subscribe("/ekf_odom", 100, odom_callback, ros::TransportHints().tcpNoDelay());   
    ros::Subscriber pose_target_sub = n.subscribe("/trajectory_generator/waypoints", 100, targetpose_callback, ros::TransportHints().tcpNoDelay());
    movement_status_pub = n.advertise<planner::status>("movement_status", 100);
    // ros::ServiceServer service = n.advertiseService("motion_controller", targetpose_server);
    ROS_INFO("motion_controller.");

    if(!sim) //not simulation
        serial_port_init(serial_port);

    ros::Rate rate(10);
    while((!first_point) && ros::ok())
    {
        ros::spinOnce();
        if(first_point)
        {
            ROS_INFO("Receive first point!");
            break;
        }
        planner::status status;
        status.arrive = 2;
        movement_status_pub.publish(status);
        
        rate.sleep();
    }
    begin_first_point = true;

    ros::Rate loop_rate(controlrate);
    // static int32_t ii=0;
    static uint8_t i = 0;
    // while(ros::ok()) 
    // {
    //     // cout << serial_port_check() << endl;
    //     loop_rate.sleep();
    // }
    while(ros::ok())
    {
        ros::spinOnce();
        if(!sim && (                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      ()==1))  //means not in simulation mode and serial port is ok
        {
            string data_temp;
            size_t n = sp.available();   //serial::Serial sp; 
            if(n != 0) 
            {
                data_temp = sp.read(n);
                size_t i;
                for(i = 0; i <= n - 1; i++)
                {
                    modbus_iterator(&hmodbus, data_temp[i]);  ///read data
                }
            }
        }
        // ROS_INFO("pulse=%d cycle=%d\r\n", hmodbus.pulse, hmodbus.cycle);

        // speed_pulse = 1000 * fabs(sin((float)i / 256.0 * 2 * PI));
        // dir_us = 500 * sin((float)i / 256.0 * 2.0f * PI);
        // ROS_INFO("pulse=%d dir_us=%d\r\n", speed_pulse, dir_us);
        // modbus_ctrl(dir_us, speed_pulse, buff, &bufflen);
        // i++;
        
        // ii++;
        // cout << "sent: " << ii << endl;
        // ROS_INFO("bufflen = %d\r\n", bufflen);
        // sp.write(buff, bufflen);
        // planner::cmd2car cmd;
        // cmd.velocity = 3;
        // cmd.steer_angle = 20 * sin((float)i / 256.0 * 2.0f * PI);
        //cout << hmodbus.cycle << endl;
        cmd.steer_angle = steer_gama;
        cmd.velocity = velocity_next_step;
        if(!sim)
        {   
            // cout << "sent cmd." << endl;
            send_cmd(cmd);
            cmd_pub.publish(cmd); //for simulation
        }
        else
            cmd_pub.publish(cmd);

        loop_rate.sleep();
    }
    
    // ros::spin();
    if(!sim)
        sp.close();
    return 0;
}

//进入server时 callback 是否还可以执行？？？
//改成msg执行