/*
 *
 *      Author: Dr. Safwan Al-Qadhi
 *      Email: saq.qadi@gmail.com
 *
*/


#ifndef INC_PID_H_
#define INC_PID_H_



#include "main.h"


class Pid{
  public:
	Pid();
	virtual ~Pid();

    Pid(float p, float i, float d, float iLimit);
    float updatePID(float target, float current, float Dterm, float deltaTime, uint8_t flight_mode);
    void resetPID();

    void setP(float p);
    void setI(float i);
    void setD(float d);
    void setIlimit(float limit);

    float getError();
    float getP();
    float getI();
    float getD();

    float getP_part();
    float getI_part();
    float getD_part();
    float get_PIDsum();

    float getDterm();
    float getDterm_f();
    float getDgyro_f();
    float getIlimit();

  private:
    float _P;
    float _I;
    float _D;
    float _PID_sum;
    float _I_limit;
    float _lastError;

    float   error;
    float   pPart;
    float   iPart;
    float   dPart;
    float   dterm;
};


#endif /* INC_PID_H_ */

