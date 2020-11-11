##This project aims to run MPCC on a toy car. 
Currently, I am working on stm32 part --- try to set up a high speed PID closed loop speed control, 
So that the output from MPCC (eg desired velocity) can be immediately achieved by using PID closed loop control.

The STM32 boards will communicate with PC which runs MPC calculations under ROS environment. The STM32 boards receive the desired speed (MPC output) from PC
and used PID loop to achieve this speed. The board also will send back IMU data which is fusioned with motion capture system data by EKF. The fused odemetry is then used as the input of MPC calcuation.  
