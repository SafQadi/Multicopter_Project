/*
 * filter.h
 *
 *  Created on: Feb 13, 2020
 *      Author: Dr. Safwan Al-Qadhi
 *      Email: saq.qadi@gmail.com
 */

#ifndef FILTER_H_
#define FILTER_H_


#define PI_FLOAT  3.1415926f


class filter {
public:
	filter();
	virtual ~filter();


	filter(float f, float dT);

	float Biquad_filter(float);
	float Pt1_filter(float);

	private:

	float _cutoff_freq;
	float _dT;

    float omega;
    float sn;
    float cs;
    float alpha; // Q=2


    //float b0 = 0, b1 = 0, b2 = 0, a0 = 0, a1 = 0, a2 = 0;
    float b0;
    float b1;
    float b2;
    float a0;
    float a1;
    float a2;

    float filter_b0 ;
    float filter_b1 ;
    float filter_b2 ;
    float filter_a1 ;
    float filter_a2 ;


	float _x1;
	float _x2;

	float result;
	float pt1_result;

    float _RC;
	float _Gain;


};

#endif /* FILTER_H_ */
