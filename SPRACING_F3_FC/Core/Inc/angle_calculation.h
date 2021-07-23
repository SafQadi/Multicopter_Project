/*
 * angle_calculation.h
 *
 *  Created on: 26.01.2020
 *      Author: Dr. Safwan Al-Qadhi
 *      Email: saq.qadi@gmail.com
 */

#ifndef INC_ANGLE_CALCULATION_H_
#define INC_ANGLE_CALCULATION_H_


#ifdef __cplusplus
extern "C" {
#endif

void calc_angles (float);

float get_roll_angle (void);
float get_pitch_angle (void);
float get_yaw_angle (void);

float get_roll_angle_half (void);
float get_pitch_angle_half (void);

float get_angle_aX (void);
float get_angle_aY (void);


#ifdef __cplusplus
}
#endif


#endif /* INC_ANGLE_CALCULATION_H_ */
