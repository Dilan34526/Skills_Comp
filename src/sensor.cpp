#include "main.h"

PID distPID;
TIMER goalTimed;
TIMER backTimed;
LOGGER goalVec;
LOGGER distVec;

void driveToGoal(float fTarget, int maxVoltage) {

	fTarget = fTarget/WHEEL_CIRCMF * 360 / 5 * 3;
  float headingCorrection = 0.15 * maxVoltage;
  bool achieved = false;
  unsigned long maxTime = 1500;
  float range = 10;

	clearLCDLines();
	resetEncoders();
	loopInit(driveLOOP, fTarget);
  brakeDrive();
	loopInit(turnLOOP, imu.get_rotation());
	timerInit(goalTimed);


	while(!goalTimed.atTarget){
		 //Update elapsed time
		 goalTimed.timer = millis() - goalTimed.startTime;

		 //update sensor inputs
		 driveLOOP.processVariable = getAverageEncoderValues();
		 turnLOOP.processVariable = imu.get_rotation();

		 //update PID calculations
		 driveLOOP.pidOut = pidCalculate(drivePID, driveLOOP.target, driveLOOP.processVariable);
		 turnLOOP.pidOut = pidCalculate(turnPID, turnLOOP.target, turnLOOP.processVariable);

		 driveLOOP.motorOut = slewUp(driveSLEW.lastValMTR, driveLOOP.pidOut, driveSLEW.accelRate, maxVoltage);

		 if(fabs(turnLOOP.pidOut) > headingCorrection){
			 turnLOOP.motorOut = headingCorrection * sign(turnLOOP.pidOut);
		 }

		 if(driveLOOP.processVariable < driveLOOP.target + range && driveLOOP.processVariable > driveLOOP.target - range) {
			 achieved = true;
		 }

		 if(achieved) {
			 driveL(30, maxVoltage);
			 driveR(30, maxVoltage);
		 } else if(fabs(driveLOOP.motorOut) < 20) {
       driveLOOP.motorOut = sign(driveLOOP.motorOut) * 20;
       driveL(driveLOOP.motorOut, maxVoltage);
       driveR(driveLOOP.motorOut, maxVoltage);
     } else {
       driveL(driveLOOP.motorOut, maxVoltage);
       driveR(driveLOOP.motorOut, maxVoltage);
     }

      driveSLEW.lastValMTR = driveLOOP.motorOut;

		 if(goal.get() < 100) {
			 holdDrive();
			 driveZero();
			 lcd::print(1, "LOOP FINISHED at %5.2lu", goalTimed.timer);
			 goalTimed.atTarget = true;
		 }

		 //  break out of while loop if the elapsed time of the function is too long
		 if(goalTimed.timer > maxTime){
			 lockDrive();
			 lcd::print(1, "LOOP TIMED OUT at %lu", millis());
			 goalTimed.atTarget = true;
		 }


     goalVec.elapsed.push_back(goalTimed.timer);
     goalVec.process.push_back(driveLOOP.processVariable);
     goalVec.target.push_back(driveLOOP.target);
     goalVec.motor.push_back(driveLOOP.motorOut);

		 delay(10);
	}
}

void driveBack(float fTarget, int maxVoltage) {
  delay(10);
	float origin = back.get();

	bool good = true;
	if(origin > 5000 || origin < 100) {
		good = false;
	}
  fTarget = (fTarget - origin)/25.4;

  unsigned long maxTime = 3000;

  float range = 0.5;
  float headingCorrection = 0.2 * maxVoltage;

	clearLCDLines();
	resetEncoders();
	coastDrive();
	loopInit(driveLOOP, fTarget);
  loopInit(turnLOOP, floor(imu.get_rotation()));
	timerInit(backTimed);

	while(!backTimed.atTarget) {
		 //Update elapsed time
		 backTimed.timer = millis() - backTimed.startTime;

     float dist = relativeDistance(origin, back.get(), good);
     float enc = getAverageEncoderValues()/216 * WHEEL_CIRCMF;

		 //update sensor inputs
		 driveLOOP.processVariable = (dist + enc)/2;
     turnLOOP.processVariable = imu.get_rotation();

		 //update PID calculations
		 driveLOOP.pidOut = pidCalculate(distPID, driveLOOP.target, driveLOOP.processVariable);
     turnLOOP.pidOut = pidCalculate(headingPID, turnLOOP.target, turnLOOP.processVariable);

     if(fabs(turnLOOP.pidOut) > headingCorrection){
      turnLOOP.motorOut = headingCorrection * sign(turnLOOP.pidOut);
     }

     //calculate SLEW
     driveLOOP.motorOut = slewUp(driveSLEW.lastValMTR, driveLOOP.pidOut, driveSLEW.accelRate, maxVoltage);

		 driveL(driveLOOP.motorOut + turnLOOP.motorOut, maxVoltage);
		 driveR(driveLOOP.motorOut - turnLOOP.motorOut, maxVoltage);

     driveSLEW.lastValMTR = driveLOOP.motorOut;

		 if(driveLOOP.processVariable < driveLOOP.target + range && driveLOOP.processVariable > driveLOOP.target - range) {
       if(dlf.get_actual_velocity() > -10 && dlf.get_actual_velocity() < 10
          && drb.get_actual_velocity() > -10 && drb.get_actual_velocity() < 10) {
         lockDrive();
         driveZero();
         lcd::print(1, "LOOP FINISHED at %5.2lu", backTimed.timer);
         backTimed.atTarget = true;
       }
			}

		 //  break out of while loop if the elapsed time of the function is too long
		 if(backTimed.timer > maxTime){
			 lockDrive();
			 lcd::print(1, "LOOP TIMED OUT at %lu", millis());
			 backTimed.atTarget = true;
		 }

     distVec.elapsed.push_back(backTimed.timer);
     distVec.process.push_back(driveLOOP.processVariable);
     distVec.target.push_back(driveLOOP.target);
     distVec.motor.push_back(driveLOOP.motorOut);

		 delay(10);
	}
}
