#ifndef _SENSOR_H_
#define _SENSOR_H_
#include "base.hpp"

extern PID backPID;
extern LOGGER goalVec;
extern LOGGER backVec;
extern LOGGER diagVec;
extern void driveToGoal(float fTarget, int maxVoltage, int smallVoltage);
extern void driveDiag(float fTarget, int maxVoltage, bool colorRestriction);
#endif
