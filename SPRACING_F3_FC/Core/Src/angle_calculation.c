/*
 * angle_calculation.c
 *
 *  Created on: 26.01.2020
 *      Author: Dr. Safwan Al-Qadhi
 *      Email: saq.qadi@gmail.com
 */

#include "main.h"
#include "mpu6050.h"
#include "angle_calculation.h"
#include "param_config.h"
#include "math.h"
#include "stdlib.h"



#define DEG_TO_RAD      (M_PI / 180.0f)

float angle_gX, angle_gY, angle_gZ;
int32_t acc_total_vector;
float angle_aY, angle_aX;
float pitch_angle = 0, roll_angle = 0;
float pitch_angle_half = 0, roll_angle_half = 0;



void calc_angles (float dt){


      acc_total_vector = sqrt((accel_X() * accel_X()) + (accel_Y() * accel_Y()) + (accel_Z() * accel_Z()));    //Calculate the total accelerometer vector.

      if (abs(accel_Y()) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
        angle_aX = asin((float)accel_Y() / acc_total_vector) * 57.296;              //Calculate the pitch angle.
      }
      if (abs(accel_X()) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
        angle_aY = asin((float)accel_X() / acc_total_vector) * 57.296;               //Calculate the roll angle.
      }


// ------------------------------------ mode 2 ---------------------------------------------

       angle_gX += (gyro_X() *  dt);
       angle_gY += (gyro_Y() * -dt); // -dt to synchronize the gyro direction with the acc.
       angle_gZ += (gyro_Z() *  dt);

       angle_gY += angle_gX * sin(gyro_Z() * dt * DEG_TO_RAD);
       angle_gX -= angle_gY * sin(gyro_Z() * dt * DEG_TO_RAD);
       angle_gX  = COM_G_GAIN * angle_gX +  COM_A_GAIN * angle_aX ;
	   angle_gY  = COM_G_GAIN * angle_gY +  COM_A_GAIN * angle_aY ;
	   roll_angle  = angle_gX  ;
	   pitch_angle = angle_gY ;


// ------------------------------------ mode 3 ---------------------------------------------

	   roll_angle_half  = COM_G_GAIN * (roll_angle_half  + gyro_X() *  dt) +  COM_A_GAIN * angle_aX ;
	   pitch_angle_half = COM_G_GAIN * (pitch_angle_half + gyro_Y() * -dt) +  COM_A_GAIN * angle_aY ;

// ------------------------- or ----------------------------

//       roll_angle_half  = COM_G_GAIN * (roll_angle_half  + ((gyro_X() *  dt) + (angle_gY * sin(gyro_Z() * dt * DEG_TO_RAD )))) +  COM_A_GAIN * angle_aX ;
//       pitch_angle_half = COM_G_GAIN * (pitch_angle_half + ((gyro_Y() * -dt) + (angle_gX * sin(gyro_Z() * dt * DEG_TO_RAD )))) +  COM_A_GAIN * angle_aY ;

}



float get_roll_angle (void) { return roll_angle; }
float get_pitch_angle (void) { return pitch_angle; }
float get_yaw_angle (void) { return angle_gZ; }

float get_roll_angle_half (void) { return roll_angle_half; }
float get_pitch_angle_half (void) { return pitch_angle_half; }

float get_angle_aX (void) { return angle_aX; }
float get_angle_aY (void) { return angle_aY; }



