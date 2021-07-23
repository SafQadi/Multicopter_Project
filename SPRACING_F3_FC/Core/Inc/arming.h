/*
 * Ready.h
 *
 *  Created on: 28.01.2020
 *      Author: Dr. Safwan Al-Qadhi
 *      Email: saq.qadi@gmail.com
 */

#ifndef ARMING_H_
#define ARMING_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>


void control_motors(uint16_t);
uint8_t start_speed ();
uint8_t armed_s ();



#ifdef __cplusplus
}
#endif



#endif /* ARMING_H_ */
