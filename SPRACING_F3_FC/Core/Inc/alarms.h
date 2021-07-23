/*
 * alarms.h
 *
 *  Created on: Feb 11, 2020
 *      Author: Dr. Safwan Al-Qadhi
 *      Email: saq.qadi@gmail.com
 */

#ifndef INC_ALARMS_H_
#define INC_ALARMS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"



void error_signal(uint8_t);
void flight_mode_signal(uint8_t);
void buzzer_signal(uint8_t, uint16_t);




#ifdef __cplusplus
}
#endif

#endif /* INC_ALARMS_H_ */
