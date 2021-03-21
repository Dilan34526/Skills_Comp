#ifndef _SENSOR_H_
#define _SENSOR_H_
#include "base.hpp"

extern PID distPID;
extern LOGGER goalVec;
extern void driveToGoal(float fTarget, int maxVoltage);
extern void driveBack(float fTarget, int maxVoltage);
#endif
