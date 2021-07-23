/*
 * filter.cpp
 *
 *  Created on: Feb 13, 2020
 *      Author: Dr. Safwan Al-Qadhi
 *      Email: saq.qadi@gmail.com
 */

#include "filter.h"
#include "math.h"


filter::filter() {
	// TODO Auto-generated constructor stub
}

filter::~filter() {
	// TODO Auto-generated destructor stub
}


filter::filter(float f, float dt){

	_cutoff_freq = f;
	_dT = dt;

//	---------------------- Biquad param ----------------------------

    omega = 2.0f * PI_FLOAT * _cutoff_freq * _dT; // dt = 2000.0 * 0.000001f; //  omega = 2.0f * M_PI_FLOAT * filterFreq * refreshRate * 0.000001f;
    sn = sin(omega);
    cs = cos(omega);
    alpha = sn / (2.0 * 0.7071); // Q=2


    //float b0 = 0, b1 = 0, b2 = 0, a0 = 0, a1 = 0, a2 = 0;
    b0 = (1 - cs) * 0.5f;
    b1 = 1 - cs;
    b2 = (1 - cs) * 0.5f;
    a0 = 1 + alpha;
    a1 = -2 * cs;
    a2 = 1 - alpha;

    // normalize the coefficients
    filter_b0 = b0 / a0;
    filter_b1 = b1 / a0;
    filter_b2 = b2 / a0;
    filter_a1 = a1 / a0;
    filter_a2 = a2 / a0;

    _x1 = 0;
    _x2 = 0;


    //	---------------------- pt1 param ----------------------------


    _RC = 1 / ( 2 * PI_FLOAT * _cutoff_freq);
	_Gain = _dT / (_RC + _dT);

	pt1_result = 0;

}


float filter::Biquad_filter(float input){

	 result = filter_b0 * input + _x1;
	 _x1 = filter_b1 * input - filter_a1 * result + _x2;
	 _x2 = filter_b2 * input - filter_a2 * result;
	 //result = result * 0.99;

	 return result;

}


float filter::Pt1_filter(float input){

	pt1_result = pt1_result + _Gain * (input - pt1_result);

	return pt1_result;
}



