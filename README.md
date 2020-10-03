##This project aims to run MPCC on a toy car. 
Currently, I am working on stm32 part --- try to set up a high speed PID closed loop speed control, 
So that the output from MPCC (eg desired velocity) can be immediately achieved by using PID closed loop control.

This repository is a BAD EXAMPLE of using Github. 

There are many different versions in this repository:

V5.0
Module	                  Status	Comments 
Pid speed 	           worked	slow but accurate enough. See video
Imu 	                   Worked	30-70hz slow can be improved by adding periodic callbacks
Rod command send to stm32  Worked	Dma receive works very well(used Dji’s code).  Need to add Dma send 
		
