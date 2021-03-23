#ifndef _TURN_H_
#define _TURN_H_
#include "base.hpp"

extern float imuDifference;
extern float target; 

extern PID turnPID;
extern SLEW turnSLEW;
extern LOOP turnLOOP;
extern LOGGER turnVec;

extern void turn(float fTarget, int maxVoltage);
extern void turn(float fTarget, int maxVoltage, int maxTime);

#endif
