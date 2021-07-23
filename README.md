# Multicopter_Project

In this project I am uisng an STM32F3 chip placed on SPRACINGF3 flight controller to controll a multicopter - Quadcopter. The code is built from scratch in C and C++. Timers are set to decode the reciever data and generate the PWM signals for the speed controlles of the motors. A PID unit controlles the reaction of the copter in two different modes: gyro and level mode. 
<br /> 
<br /> 

## Gyro/Accelerometer
An MPU6050 Gyro/accelerometer chip is placed on the SPRACINGF3 board.
The communication between gyro/accelerometer and the STM32F3 is achieved through an I2C.
<br />
<br /> 

## The transmitter
The transimitter uses PPM protocol with eigh channels at 2.4GHz to communicate with the copter. 
- Four channels for Throttle, Roll, Pitch, and Yaw. 
- One channel for selecting the flight mode
- Two channels are dedicated for adjusting the PIDs on flight. 
<br />

## Electronic speed controller (ESC)
The ESCs are controlled by PWM signals at 480Hz. Four timers of the STM32F3 are used to generat the PWM signals 
<br />
<br /> 

## Flight modes
- Gyro mode: the flight is controlled based on the angle change ratio measured by the gyro
- Level mode: the copter levels itself when no input is recieved from the transmitter. This mode is angle-based, where a complementry filter is used to 
calculate the angle from both the gyro and the accelerometer.
<br /> 

## Compiling
The code is compiled using Eclipse for C/C++

## License
< >
    Copyright (C) <2021>  <Safwan Al-Qadhi>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation version 3 of the License.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

	Contact:	Dr. Safwan Al-Qadhi
			Email: saf.qadi@gmail.com, 
