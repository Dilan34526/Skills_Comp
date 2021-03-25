#ifndef _DRIVE_H_
#define _DRIVE_H_
#include "base.hpp"

/*
* @Prototypes: PID Structures
*/
extern PID drivePID;
extern PID headingPID;

/*
* @Prototypes: LOGGER Structures
*/
extern LOGGER driveVec;

/*
*  @Prototypes: SLEW Structures
*/
extern SLEW driveSLEW;


/*
* @Prototypes: LOOP Structures
*/
extern LOOP driveLOOP;

/*
* @Prototypes: Drive Distance Functions
*/
extern void drive(float fTarget, int maxVoltage);
extern void drive(float fTarget, int maxVoltage, int maxTime);
extern void drive(float fTarget, int maxVoltage, float distance, int smallVoltage);
extern void drive(float fTarget, int maxVoltage, float distance, int smallVoltage, int maxTime);
#endif
