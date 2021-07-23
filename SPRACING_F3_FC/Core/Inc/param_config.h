/*
 * param_config.h
 *
 *  Created on: 27.01.2020
 *      Author: Dr. Safwan Al-Qadhi
 *      Email: saq.qadi@gmail.com
 */

#ifndef INC_PARAM_CONFIG_H_
#define INC_PARAM_CONFIG_H_


// I2C address of the IMU
//const int MPU6050_ADDRESS = 0x68;

//#include "main.h"


// PID constants for the angle PID controller

const float RC_cutoff_freq = 40.0f;
const float Gyro_cutoff_freq = 40.0f;
const float Dterm_cutoff_freq = 15.0f;

const float COM_G_GAIN = 0.999;
const float COM_A_GAIN = 0.001;


//float PID_SCALE = 1 - (20 / Gyro_cutoff_freq);

const float Roll_P = 0.91; //* PID_SCALE; //manual 1.06 Oszi. 0,49
const float Roll_I = 0.6;  // / 500  //manual 3.4 Oszi. 1,1
const float Roll_D = 5.1;  // 2,7
const float Max_roll = 400.0;

const float Pitch_P = 0.99; // * PID_SCALE ; //manual 1.27 Oszi. 0,57
const float Pitch_I = 0.9;  // / 500  // manual 3.8 Oszi. 1,4
const float Pitch_D = 5.7; //3,3
const float Max_pitch = 400.0;

const float Yaw_P = 0.94;// * PID_SCALE ; // manual 1.1 0,72
const float Yaw_I = 3.6; // / 500   3,7
const float Yaw_D = 0.0;
const float Max_yaw = 400.0;

const float LEVEL_ROLL_P = 0.54;  // 9.4 level Oszi - 0.42 no Oszi for 10 max. angle and 0.78 for 5 max angle
const float LEVEL_PITCH_P = 0.62;  // 9.4 level Oszi - 0.42 no Oszi for 10 max. angle and 0.78 for 5 max angle
const float MAX_ANGLE = 5.0; // 5 = 98.4deg., 10 = 49.2 deg ,  20 =  24.6deg , 30= 16,4deg.  492/angle = MAX_ANGLE value

const float RC_RATE = 1.0;

// it the battery voltage goes below this value, an alarm will go on.
const float MIN_BAT_VOLTAGE = 10.0f;
const float battery_compensation = 0.15f;

const uint16_t idle_speed = 1050;



#endif /* INC_PARAM_CONFIG_H_ */


