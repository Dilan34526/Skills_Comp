#include "main.h"

PID backPID;
TIMER goalTimed;
TIMER backTimed;
TIMER diagTimed;
LOGGER goalVec;
LOGGER backVec;
LOGGER diagVec;

void driveToGoal(float fTarget, int maxVoltage) {

	fTarget = fTarget/WHEEL_CIRCMF * 360 * (3/5);
	float absVoltage = abs(maxVoltage);
  float headingCorrection = 0.15 * absVoltage;
  bool achieved = false;
  unsigned long maxTime = 1500;
  float range = 10;

	driveSLEW.accelRate = signChecker(driveSLEW.accelRate, fTarget);
  turnSLEW.accelRate = signChecker(turnSLEW.accelRate, absVoltage);

	clearLCDLines();
	resetEncoders();
	resetSLEW();
	loopInit(driveLOOP, fTarget);
	loopInit(turnLOOP, target);
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

		 driveLOOP.motorOut = slewCalculate(driveSLEW.lastValMTR, driveLOOP.pidOut, driveSLEW.accelRate, maxVoltage);

		 if(fabs(turnLOOP.pidOut) > headingCorrection){
			 turnLOOP.motorOut = headingCorrection * sign(turnLOOP.pidOut);
		 } else {
       turnLOOP.motorOut = turnLOOP.pidOut;
     }

		 if(driveLOOP.processVariable > driveLOOP.target - range) {
			 achieved = true;
		 }

		 if(achieved) {
			 driveL(30, absVoltage);
			 driveR(30, absVoltage);
		 } else if(fabs(driveLOOP.motorOut) < 20) {
       driveLOOP.motorOut = 20;
       driveL(driveLOOP.motorOut + turnLOOP.motorOut, absVoltage);
       driveR(driveLOOP.motorOut - turnLOOP.motorOut, absVoltage);
     } else {
       driveL(driveLOOP.motorOut + turnLOOP.motorOut, absVoltage);
       driveR(driveLOOP.motorOut - turnLOOP.motorOut, absVoltage);
     }

      driveSLEW.lastValMTR = driveLOOP.motorOut;

		 if(down.get() < 100) {
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

	float absVoltage = abs(maxVoltage);
  float range = 0.5;
	float velocity = 4;
  float headingCorrection = 0.2 * maxVoltage;

	driveSLEW.accelRate = signChecker(driveSLEW.accelRate, maxVoltage);
  turnSLEW.accelRate = signChecker(turnSLEW.accelRate, absVoltage);

	clearLCDLines();
	resetEncoders();
	resetSLEW();
	loopInit(driveLOOP, fTarget);
  loopInit(turnLOOP, target);
	timerInit(backTimed);

	while(!backTimed.atTarget) {
		 //Update elapsed time
		 backTimed.timer = millis() - backTimed.startTime;

     float dist = relativeDistanceBack(origin, back.get(), good);
     float enc = getAverageEncoderValues()/216 * WHEEL_CIRCMF;

		 //update sensor inputs
		 driveLOOP.processVariable = (dist + enc)/2;
     turnLOOP.processVariable = imu.get_rotation();

		 //update PID calculations
		 driveLOOP.pidOut = pidCalculate(backPID, driveLOOP.target, driveLOOP.processVariable);
     turnLOOP.pidOut = pidCalculate(headingPID, turnLOOP.target, turnLOOP.processVariable);

     if(fabs(turnLOOP.pidOut) > headingCorrection){
      turnLOOP.motorOut = headingCorrection * sign(turnLOOP.pidOut);
     } else {
      turnLOOP.motorOut = turnLOOP.pidOut;
     }

     //calculate SLEW
     driveLOOP.motorOut = slewCalculate(driveSLEW.lastValMTR, driveLOOP.pidOut, driveSLEW.accelRate, maxVoltage);

		 driveL(driveLOOP.motorOut + turnLOOP.motorOut, absVoltage);
		 driveR(driveLOOP.motorOut - turnLOOP.motorOut, absVoltage);

     driveSLEW.lastValMTR = driveLOOP.motorOut;

		 if(driveLOOP.processVariable < driveLOOP.target + range && driveLOOP.processVariable > driveLOOP.target - range) {
       if(dlf.get_actual_velocity() > -velocity && dlf.get_actual_velocity() < velocity
          && drb.get_actual_velocity() > -velocity && drb.get_actual_velocity() < velocity) {
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

     backVec.elapsed.push_back(backTimed.timer);
     backVec.process.push_back(driveLOOP.processVariable);
     backVec.target.push_back(driveLOOP.target);
     backVec.motor.push_back(driveLOOP.motorOut);

		 delay(10);
	}
}


void driveDiag(float fTarget, int maxVoltage) {
  delay(15);
	float origin = diag.get();

	bool good = true;
	while (origin > 5000 || origin < 100) {
		origin = diag.get();
		delay(2);
	}

  int maxTime = 3500;

	float absVoltage = abs(maxVoltage);
  float range = 10;
	float velocity = 4;
  float headingCorrection = 0.2 * absVoltage;
	float prevPID = -832;

  float first = (fTarget - origin)/25.4 * sqrt(2);

	drive(-first, -100);

	origin = diag.get();
	float second = (fTarget - origin)/25.4 * sqrt(2);

	drive(-second, 50, 300);

	origin = diag.get();
	float third = (fTarget - origin)/25.4 * sqrt(2);

	drive(-third, 50, 300);



	// driveSLEW.accelRate = signChecker(driveSLEW.accelRate, maxVoltage);
	// turnSLEW.accelRate = signChecker(turnSLEW.accelRate, absVoltage);
	//
	// clearLCDLines();
	// resetEncoders();
	// resetSLEW();
	// loopInit(driveLOOP, fTarget);
  // loopInit(turnLOOP, target);
	// timerInit(diagTimed);
	//
	// while(!diagTimed.atTarget) {
	// 	 //Update elapsed time
	// 	 diagTimed.timer = millis() - diagTimed.startTime;
	//
	// 	 //update sensor inputs
	// 	 driveLOOP.processVariable = getAverageEncoderValues();
  //    turnLOOP.processVariable = imu.get_rotation();
	//
	// 	 //update PID calculations
	// 	 driveLOOP.pidOut = pidCalculate(diagPID, driveLOOP.target, driveLOOP.processVariable);
  //    turnLOOP.pidOut = pidCalculate(headingPID, turnLOOP.target, turnLOOP.processVariable);
	//
  //    if(fabs(turnLOOP.pidOut) > headingCorrection){
  //     turnLOOP.motorOut = headingCorrection * sign(turnLOOP.pidOut);
  //    } else {
  //     turnLOOP.motorOut = turnLOOP.pidOut;
  //    }
	//
	// 	 diagVec.prev.push_back(prevPID);
	// 	 prevPID = driveLOOP.pidOut;
	//
  //    //calculate SLEW
  //    driveLOOP.motorOut = slewCalculate(driveSLEW.lastValMTR, driveLOOP.pidOut, driveSLEW.accelRate, maxVoltage);
	// 	 //
	// 	 driveL(driveLOOP.motorOut + turnLOOP.motorOut, absVoltage);
	// 	 driveR(driveLOOP.motorOut - turnLOOP.motorOut, absVoltage);
	//
  //    driveSLEW.lastValMTR = driveLOOP.motorOut;
	//
	// 	 if(driveLOOP.processVariable < driveLOOP.target + range && driveLOOP.processVariable > driveLOOP.target - range) {
  //      if(dlf.get_actual_velocity() > -velocity && dlf.get_actual_velocity() < velocity
  //         && drb.get_actual_velocity() > -velocity && drb.get_actual_velocity() < velocity) {
  //        lockDrive();
  //        driveZero();
  //        lcd::print(1, "LOOP FINISHED at %5.2lu", diagTimed.timer);
  //        diagTimed.atTarget = true;
  //      }
	// 		}
	//
	// 	 //  break out of while loop if the elapsed time of the function is too long
	// 	 if(diagTimed.timer > maxTime){
	// 		 lockDrive();
	// 		 lcd::print(1, "LOOP TIMED OUT at %lu", millis());
	// 		 diagTimed.atTarget = true;
	// 	 }
	//
	// 	 lcd::print(2, "MOTOR %f", driveLOOP.motorOut);
	// 	 lcd::print(3, "TARGET %f", driveLOOP.target);
	// 	 lcd::print(4, "PID OUT %f", driveLOOP.pidOut);
	//
  //    diagVec.elapsed.push_back(diagTimed.timer);
  //    diagVec.process.push_back(driveLOOP.processVariable);
  //    diagVec.target.push_back(driveLOOP.target);
  //    diagVec.motor.push_back(driveLOOP.motorOut);
	// 	 diagVec.pid.push_back(driveLOOP.pidOut);
	// 	 diagVec.distance.push_back(driveLOOP.processVariable);
	//
	// 	 delay(10);
	// }
}
