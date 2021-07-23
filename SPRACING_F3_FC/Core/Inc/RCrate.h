/*
 * RCrate.h
 *
 *  Created on: 27.01.2020
 *      Author: Dr. Safwan Al-Qadhi
 *      Email: saq.qadi@gmail.com
 */

#ifndef SRC_RCRATE_H_
#define SRC_RCRATE_H_

#include "main.h"


class RC_rate {

	public:
	RC_rate();
	virtual ~RC_rate();

	RC_rate(float level_P, float angle, float rate);

	float RC_rate_manual(int16_t);

    void  set_Level_P(float d);
    float get_Level_P();
    void  set_MaxAngle(float a);
    float get_MaxAngle();
    void  set_RC_rate(float r);
    float get_RC_rate();

	private:
	float _setpoint;
    float _Level_P_Gain;
    float _max_angle;
    float _RC_rate_param;

};

#endif /* SRC_RCRATE_H_ */
