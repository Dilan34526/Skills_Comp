#ifndef _BASE_H_
#define _BASE_H_
#include <vector>

/*
* @struct: PID Calculation Data
*   @ float kP: proportional gain
*   @ float kI: integral gain
*   @ float kD: derivative gain
*   @ float epsilonInner: inner bounds of integration
*   @ float epsilonOuter: outer bounds of integration
*   @ float sigma: integral of error
*   @ float lastValue: previously measured sensor value
*   @ unsigned long lastTime: previously measured system time
*/
extern struct PID{
	float kP;
	float kI;
	float kD;
	float epsilonInner;
	float epsilonOuter;
	float sigma;
	float lastVal;
	float error;
	float gradient;
	unsigned long lastTime;
} PID_t;

/*
* @struct: Slew Calculation Data
*   @ float aceelRate: acceleration rate
*   @ float zone: acceleration and deacceleration zone
*/
extern struct SLEW {
	float accelRate;
	float lastValMTR;
} SLEW_t;

/*
* @struct: 1D Voltage Feedback Loop (Straight Movements and Point Turns)
*   @float target: set point
*   @float processVariable: processVariable (i.e encoder counts for distance)
*   @float pidOut: proportional integral derivative controller output power
*   @float motorOut: heading output power
*   @float motionSegment: phases of motion (accel --> pure feedback)
*/
extern struct LOOP {
	float target;
  float processVariable;
  float pidOut;
  float motorOut;
  int motionSegment;
} LOOP_t;

/*
* @struct: Timer for loop
*   @bool bAtTarget: loop at target
*   @unsigned long loopStart: starting time
*   @unsigned long timer: elapsed time
*   @unsigned long atTargetTime: elapsed time settled wihin target bounds
*/
extern struct TIMER {
	bool atTarget;
	unsigned long startTime;
	unsigned long timer;
	unsigned long atTargetTime;
} TIMER_t;

extern struct LOGGER {
  std::vector<float> elapsed;
  std::vector<float> encoder;
  std::vector<float> target;
  std::vector<float> power;
  std::vector<float> prev;
  std::vector<float> error;
  std::vector<float> distance;
  std::vector<float> process;
  std::vector<float> motor;
	std::vector<float> timeOut;
	std::vector<float> turn;
	std::vector<float> pid;
	std::vector<float> accel;
} LOGGER_t;

/*
* @Prototypes: Initialize Functions
*/
extern void pidInit (PID&pid, float kP, float kI, float kD, float epsilonInner, float epsilonOuter);
extern void slewInit (SLEW&slew, float accelRate);
extern void loopInit(LOOP&loop, float target);
extern void timerInit(TIMER&timer);

/*
* @Prototypes: Feedback and Feedforward Functions
*/
extern float pidCalculate (PID&pid, float setPoint, float processVariable);
extern float slewCalculate (float lastVal, float newVal, float accel, float maxVoltage);

/*
* @Prototypes: Brake Mode Functions
*/
extern void coastDrive();
extern void brakeDrive();
extern void holdDrive();
extern void lockDrive();
extern void driveZero();

/*
* @Prototypes: Distance Functions
*/
extern float relativeDistanceBack(float origin, float sensor, bool okay);
extern float relativeDistanceDiag(float origin, float sensor, bool okay);
/*
* @Prototypes: Motor Specific Functions
*/
extern void resetEncoders();
extern float getAverageEncoderValues();
extern float signChecker(float input, float sameSign);
extern void resetSLEW();
extern void driveL(float input, unsigned int maxVoltage);
extern void driveR(float input, unsigned int maxVoltage);
#endif
