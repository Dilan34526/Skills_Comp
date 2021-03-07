#ifndef _DRIVE_H_
#define _DRIVE_H_
#include "base.hpp"
#include "api.h"

extern float imuDifference;

/*
* @Prototypes: PID Structures
*/
extern PID drivePID;
extern PID turnPID;

/*
*  @Prototypes: SLEW Structures
*/
extern SLEW driveSLEW;
extern SLEW turnSLEW;

/*
* @Prototypes: LOOP Structures
*/
extern LOOP driveLOOP;
extern LOOP turnLOOP;

extern std::vector<float> timeOut;

/*
* @Prototypes: Drive Distance Functions
*/
extern void drive(float fTarget, int maxVoltage);
extern void turn(float fTarget, int maxVoltage, int maxTime);
extern void driveDist(float fTarget,int maxVoltage);
extern void driveToGoal(float fTarget, int maxVoltage);
extern void goToBall(float heading);

#endif
