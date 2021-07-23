/*
 * Ready.cpp
 *
 *  Created on: 28.01.2020
 *      Author: Dr. Safwan Al-Qadhi
 *      Email: saq.qadi@gmail.com
 */

#include "main.h"
#include <arming.h>
#include "IC_ISR.h"


uint8_t armed = 0;
uint8_t speed = 0;


void control_motors(uint16_t val ){


	  if (throttle_ch() < 1030 && yaw_ch() < 1030)armed = 55;

	  if (armed == 55 && throttle_ch() < 1030 && yaw_ch() > 1488) {
	     speed = val - 1000; // max. value is 255, due to uint8_t. if val is 1050, speed = 50
	     armed = 1;
       }

	  if (armed == 1 && throttle_ch() < 1030 && yaw_ch() > 1950) {
		  armed = 0;
		  speed = 0;
}

}


uint8_t start_speed () {return speed;}
uint8_t armed_s () {return armed;}
