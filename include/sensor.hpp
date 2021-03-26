#ifndef _SENSOR_H_
#define _SENSOR_H_
#include "base.hpp"

extern PID backPID;
extern PID diagPID;
extern PID diagPID2;
extern LOGGER goalVec;
extern LOGGER backVec;
extern LOGGER diagVec;
extern void driveToGoal(float fTarget, int maxVoltage);
extern void driveBack(float fTarget, int maxVoltage);
extern void driveDiag(float fTarget, int maxVoltage);
#endif
