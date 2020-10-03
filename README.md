##This project aims to run MPCC on a toy car. 
Currently, I am working on stm32 part --- try to set up a high speed PID closed loop speed control, 
So that the output from MPCC (eg desired velocity) can be immediately achieved by using PID closed loop control.

This repository is a BAD EXAMPLE of using Github. 
PLEASE DO NOT GIT CLONE THE WHOLE BRANCH AS THE SIZE 
IS ABOUT
3 GB.


There are many versions in this repository:

V5.0
Module	                  Status	Comments 
Pid speed 	           worked	slow but accurate enough. See video
Imu 	                   Worked	30-70hz slow can be improved by adding periodic callbacks
Rod command send to stm32  Worked	Dma receive works very well(used Dji’s code).  Need to add Dma send 

V3.5 
Can use a DJI Rc controller to control the car and use ANO software (in the folder “pack”) to read imu data and current motor speed.

--------------------------------------------
Different folders 
Pack: Has three folders. 
Usart contains different visualisation software e.g ANO
RobomasterF4: contains pdf user guides of DJI robomaster board. 
Stm32_f4:contains pdf user guides of stm32-f4 boards.

Useful links:  contains the links of some very good tutorials. VERY HELPFUL!!!!!!!!

Open-brother 
It contains the code written by 凯哥

Ros_serial
It contains some code example for ros-stm32 communication.		
