#ifndef _DRIVE_H_
#define _DRIVE_H_
#include "base.hpp"
#include "api.h"

extern float imuDifference;

/*
* @Prototypes: PID Structures
*/
extern PID drivePID;
extern PID distPID;
extern PID turnPID;
extern PID headingPID;


extern struct LOGGER {
  std::vector<float> elapsed;
  std::vector<float> encoder;
  std::vector<float> target;
  std::vector<float> power;
  std::vector<float> grad;
  std::vector<float> error;
  std::vector<float> distance;
  std::vector<float> process;
  std::vector<float> motor;
} LOGGER_t;
/*
* @Prototypes: LOGGER Structures
*/
extern LOGGER goalVec;
extern LOGGER driveVec;
extern LOGGER turnVec;

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
extern void drive(float fTarget, int maxVoltage, int maxTime);
extern void turn(float fTarget, int maxVoltage, int maxTime);
extern void driveDist(float fTarget,int maxVoltage);
extern void driveToGoal(float fTarget, int maxVoltage);
extern void goToBall(float heading);
extern void correction(float heading, float bx, float by);
extern void lastCorrect(float heading, float by);
extern void lastCorrect(float heading, float bx, float by);

#endif
