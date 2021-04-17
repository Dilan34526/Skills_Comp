#include "main.h"

Motor dlf(DLF,  E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
Motor dlb(DLB,  E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
Motor drf(DRF,  E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);
Motor drb(DRB,  E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);

ADIEncoder track(SIDE_1, SIDE_2);
Imu imu(DIMU);

Distance left(LFT);
Distance back(BCK);
Distance down(DOWN);
Distance diag(DIAG);

void pidInit (PID&pid, float kP, float kI, float kD, float epsilonInner, float epsilonOuter) {
	pid.kP = kP;
	pid.kI = kI;
	pid.kD = kD;
	pid.epsilonInner = epsilonInner; //bounds of integration
	pid.epsilonOuter = epsilonOuter;
	pid.sigma = 0;
	pid.error = 0;
	pid.gradient = 0;
	pid.lastVal = 0;
	pid.lastTime = millis();
}

void slewInit(SLEW&slew, float accelRate) {
	slew.accelRate = accelRate;
	slew.lastValMTR = 0;
}

void loopInit(LOOP&loop, float target) {
  loop.target = target;
  loop.processVariable = 0;
  loop.pidOut = 0;
  loop.motorOut = 0;
  loop.motionSegment = 0;
}

void timerInit(TIMER&timer) {
	timer.atTarget = false;
	timer.startTime = millis();
  timer.timer = millis();
  timer.atTargetTime = millis(); //changed from millis() to 0;
}

float	pidCalculate (PID&pid, float setPoint, float processVariable) {

  //  Calculate elpased time per loop
	float deltaTime = (float)(millis() - pid.lastTime) / 1000.0;
	pid.lastTime = millis();

	//  Calculate error
	pid.error = setPoint - processVariable;

  //  Rate of change (slope) in error per loop
	if(deltaTime > 0){
		pid.gradient = (processVariable - pid.lastVal) / deltaTime;
	}
	pid.lastVal = processVariable;

  //  Sum error within bounds of integration
	if(fabs(pid.error) > pid.epsilonInner && fabs(pid.error) < pid.epsilonOuter){
		pid.sigma += pid.error * deltaTime;
	}

  //  Reset integral outside of its bounds
	if (fabs (pid.error) > pid.epsilonOuter){
		pid.sigma = 0;
  }

  float output = 0;
	output = (pid.error * pid.kP) 	+ (pid.sigma * pid.kI)	- (pid.gradient * pid.kD);
  return output;
}

float slewCalculate (float lastVal, float newVal, float accel, float maxVoltage) {
	float out = 127;
	out = lastVal + accel;
	if(sign(lastVal) == sign(accel)) {
		if(fabs(out) > fabs(maxVoltage)) {
		out = maxVoltage;
		}
	} else {
		if(fabs(out) < fabs(maxVoltage)) {
		out = maxVoltage;
		}
	}
	if(lastVal == 0) {
		out = lastVal + accel;
	}
	if(fabs(newVal) < fabs(maxVoltage)) {
		out = newVal;
	}
	return out;
}

float slewCalculate (float lastVal, float accel, float maxVoltage) {
  float out = 127;
	out = lastVal + accel;
	if(fabs(out) > fabs(maxVoltage) && fabs(lastVal) < fabs(maxVoltage)) {
			out = maxVoltage;
	}
	if(lastVal == maxVoltage) {
		out = maxVoltage;
	}
	return out;
}

void resetEncoders() {
  dlf.tare_position();
  dlb.tare_position();
  drf.tare_position();
  drb.tare_position();
}

void coastDrive() {
	dlf.set_brake_mode(E_MOTOR_BRAKE_COAST);
  dlb.set_brake_mode(E_MOTOR_BRAKE_COAST);
  drf.set_brake_mode(E_MOTOR_BRAKE_COAST);
  drb.set_brake_mode(E_MOTOR_BRAKE_COAST);
}

void brakeDrive() {
	dlf.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
  dlb.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
  drf.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
  drb.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
}

void holdDrive() {
  dlf.set_brake_mode(E_MOTOR_BRAKE_HOLD);
  dlb.set_brake_mode(E_MOTOR_BRAKE_HOLD);
  drf.set_brake_mode(E_MOTOR_BRAKE_HOLD);
  drb.set_brake_mode(E_MOTOR_BRAKE_HOLD);
}

void lockDrive() {
	dlf.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	dlb.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	drf.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	drb.set_brake_mode(E_MOTOR_BRAKE_HOLD);

	dlf.move_velocity(0);
	dlb.move_velocity(0);
	drf.move_velocity(0);
	drb.move_velocity(0);
}

void driveZero() {
	dlf.move(0);
  dlb.move(0);
	drf.move(0);
  drb.move(0);
}

void driveL(float input, unsigned int maxVoltage){
  /*
  * @Ternerary: Is the abs input voltage greater than the max?
  *   @ yes:  limit voltage to max * direction
  *   @ no:   use original input
  */
  float finalPower = fabs(input) > maxVoltage ? (  maxVoltage * sign(input) ) : input;

  dlf.move(finalPower);
  dlb.move(finalPower);
}
void driveR(float input, unsigned int maxVoltage){
  /*
  * @Ternerary: Is the abs input voltage greater than the max?
  *   @ yes:  limit voltage to max * direction
  *   @ no:   use original input
  */
  float finalPower = fabs(input) > maxVoltage ? (  maxVoltage * sign(input) ) : input;

  drf.move(finalPower);
  drb.move(finalPower);
}

float getAverageEncoderValues(){
  float left = (dlb.get_position() + dlf.get_position())/2;
  float right = (drb.get_position() + drf.get_position())/2;
  return (left + right)/2;
}

float relativeDistanceBack(float origin, float sensor, bool okay) {
  if(okay) {
    if(sensor < 5000 || sensor > 400) {
    	float dist = (sensor - origin)/25.4;
			return dist;
  	} else {
    return (getAverageEncoderValues()/216 * WHEEL_CIRCMF);
  	}
  } else {
    return (getAverageEncoderValues()/216 * WHEEL_CIRCMF);
  }
}

float relativeDistanceDiag(float origin, float sensor, bool okay) {
	if(okay) {
    if(sensor < 5000 || sensor > 25) {
    	float dist = sensor;
			return dist;
  	} else {
    return (-((getAverageEncoderValues()/216 * WHEEL_CIRCMF)*25.4/sqrt(2)) + origin);
  	}
  } else {
    return (-((getAverageEncoderValues()/216 * WHEEL_CIRCMF)*25.4/sqrt(2)) + origin);
  }
}

void resetSLEW() {
	driveSLEW.lastValMTR = 0;
	turnSLEW.lastValMTR = 0;
}


void outtake() {
	intake.mode = INTK_OUT;
	indexer.mode = INDX_OUT;
	shooter.mode = SHOT_OUT;
}

void moveIn() {
	indexer.mode = INDX_MOVE_IN;
	shooter.mode = SHOT_MOVE_IN;
}
