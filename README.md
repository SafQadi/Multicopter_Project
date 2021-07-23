# Multicopter_Project

In this project I am uisng an STM32F3 chip placed on SPRACINGF3 flight controller to controll the flight of a multicopter - Quadcopter. 
<br /> 


## Gyro/Accelerometer
An MPU6050 Gyro/accelerometer chip is placed on the SPRACINGF3 board.
The communication between gyro/accelerometer and the STM32F3 is achieved through an I2C.
<br />


## The transmitter
The transimitter uses PPM protocol with eigh channels at 2.4GHz to communicate with the copter. 
- Four channels are for Throttle, ROLL, Pitch, and Yaw. 
- One channel for selecting the flight mode
- Two channels are dedicated for adjusting the PIDs on flight. 
<br />


## Electic speed controller (ESC)
The ESC are controlled by PWM at 480Hz. Four timers of the STM32F3 are used to generat the PWM signals 
<br />


## Flight modes
- Gyro mode: the flight is controlled based on the gyro angle ratio form the gyro
- Level mode: the copter levels itself when no input is given by the transmitter. This mode is angle-based, where a complementry filter is used to 
calculate the angle from both the gyro and the accelerometer.

## Compiling
The code is compiled using Eclipse for C/C++
