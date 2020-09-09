#ifndef Motion_Controller_H
#define Motion_Controller_H

#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "conversion.h"

#include <iostream>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <queue>
#include <cmath>

#include "planner/nextpose.h"
#include "planner/cmd2car.h"
#include "planner/goal.h"
#include "planner/status.h"

//serial port
#include <serial/serial.h>
#include <string>
#include "math.h"

using namespace std; 
using namespace Eigen;
//! @brief Common variables
const double PI = 3.141592653589793;
const double TAU = 6.283185307179587;

bool sim = true;

double alpha_p = 1, alpha_i = 0, alpha_d = 0;
double beta_p = 1, beta_i = 0, beta_d = 0;
double dis_p = 1, dis_i = 0, dis_d = 0;

//param of car
double wheel_base = 0.033; //meter

//control params
double steer_gama;
double velocity_now, velocity_next_step;

double motion_controller(Vector4d pose2d_now, Vector4d pose2d_target);

ros::Publisher cmd_pub;
ros::Publisher movement_status_pub;

//position tolerance
double tolerance = 0.01;

double controlrate = 50;

//serial_port
string serial_port;

int serial_port_check();


#endif

