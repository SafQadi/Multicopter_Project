/*
 * RCrate.cpp
 *
 *  Created on: 27.01.2020
 *      Author: Dr. Safwan Al-Qadhi
 *      Email:  saq.qadi@gmail.com
 */

#include "RCrate.h"

RC_rate::RC_rate() {
	// TODO Auto-generated constructor stub
}
RC_rate::~RC_rate() {
	// TODO Auto-generated destructor stub
}

RC_rate::RC_rate(float level_P, float angle, float rate)
{
 _Level_P_Gain = level_P;
 _max_angle = angle;
 _RC_rate_param = rate;
}

float RC_rate::RC_rate_manual(int16_t channel){

	  _setpoint = 0;

	  if (channel > 1508)_setpoint = channel - 1508;
	  else if (channel < 1492)_setpoint = channel - 1492;

//	  _setpoint /= 1.23; // 492/400. the rate is 400 deg/s


	  return _setpoint;
}


void RC_rate::set_Level_P(float d){
	 _Level_P_Gain = d;
}


float RC_rate::get_Level_P(){
	  return _Level_P_Gain;
}



void RC_rate::set_MaxAngle(float a){
	_max_angle = a;
}


float RC_rate::get_MaxAngle(){
	  return _max_angle;
}


void RC_rate::set_RC_rate(float r){
	_RC_rate_param = r;
}


float RC_rate::get_RC_rate(){
	  return _RC_rate_param;
}



