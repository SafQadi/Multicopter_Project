/*
 *      Author: Dr. Safwan Al-Qadhi
 *      Email: saq.qadi@gmail.com
*/

#include "pid.h"


Pid::Pid() {
	// TODO Auto-generated constructor stub
}
Pid::~Pid() {
	// TODO Auto-generated destructor stub
}


Pid::Pid(float p, float i, float d, float iLimit)
{
  _P = p;
  _I = i;
  _D = d;
  _I_limit = iLimit;
  _lastError = 0;
  iPart = 0;

}

float Pid::updatePID(float target, float current, float Dgyro, float deltaTime, uint8_t flight_mode)
{
   error = (target - current); // * deltaTime;

   pPart = _P * error;

   iPart += error *_I * deltaTime;
   if (iPart >  _I_limit) iPart = _I_limit;
   else if (iPart <  -_I_limit) iPart = -_I_limit;


   if (flight_mode == 1 || flight_mode == 2 || flight_mode == 3){
   dterm =  - (Dgyro - _lastError); // minus so that Dpart works as a break
   dPart = _D * dterm;
   _lastError = Dgyro;
   }
//   else if (flight_mode == 2 || flight_mode == 3){
//	dterm =   error - _lastError;
//	dPart = _D * dterm;
//	_lastError = error;
//   }

  _PID_sum = pPart + iPart + dPart;
  if (_PID_sum >  _I_limit) _PID_sum = _I_limit;
  else if (_PID_sum <  -_I_limit) _PID_sum = -_I_limit;

  return (_PID_sum);
}

void Pid::resetPID()
{
   error = 0;
   iPart = 0;
   dPart = 0;
  _PID_sum = 0;
}

void Pid::setP(float p) {  _P = p; }
void Pid::setI(float i) {  _I = i; }
void Pid::setD(float d) {  _D = d; }

void Pid::setIlimit(float limit) { _I_limit = limit; }


float Pid::getError() {  return error; }

float Pid::getP() {  return _P; }
float Pid::getI() {  return _I; }
float Pid::getD() {  return _D; }

float Pid::getP_part() {  return pPart; }
float Pid::getI_part() {  return iPart; }
float Pid::getD_part() {  return dPart; }
float Pid::get_PIDsum() {  return _PID_sum; }



float Pid::getDterm()   {  return dterm; }

float Pid::getIlimit() {   return _I_limit; }


