#include "main.h"

PID drivePID;
PID headingPID;
LOOP driveLOOP;
SLEW driveSLEW;
TIMER timed;
LOGGER driveVec;


void drive(float fTarget, int maxVoltage, float distance, int smallVoltage, int maxTime) {
  fTarget = fTarget/WHEEL_CIRCMF * 360 / 5 * 3;
  distance = distance/WHEEL_CIRCMF * 360 / 5 * 3;
  int absVoltage = abs(maxVoltage);

  float range = 10;
  float velocity = 4;

  driveSLEW.accelRate = signChecker(driveSLEW.accelRate, fTarget);
  turnSLEW.accelRate = signChecker(turnSLEW.accelRate, absVoltage);

  float headingCorrection = 0.2 * absVoltage;

  clearLCDLines();
  resetEncoders();
  brakeDrive();
  loopInit(driveLOOP, fTarget);
  loopInit(turnLOOP, target);
  timerInit(timed);

  while(!timed.atTarget) {
    //Update elapsed time
    timed.timer = millis() - timed.startTime;

    //update sensor inputs
    driveLOOP.processVariable = getAverageEncoderValues();
    turnLOOP.processVariable = imu.get_rotation();

    //update PID calculations
    driveLOOP.pidOut = pidCalculate(drivePID, driveLOOP.target, driveLOOP.processVariable);
    turnLOOP.pidOut = pidCalculate(headingPID, turnLOOP.target, turnLOOP.processVariable);

    //calculate the SLEWs
    if(fabs(driveLOOP.processVariable) < fabs(distance)) {
      driveLOOP.motorOut = slewCalculate(driveSLEW.lastValMTR, driveLOOP.pidOut, driveSLEW.accelRate, maxVoltage);
    } else {
      driveLOOP.motorOut = slewCalculate(driveSLEW.lastValMTR, driveLOOP.pidOut, driveSLEW.accelRate, smallVoltage);
    }

    if(headingCorrection < fabs(turnLOOP.pidOut)) {
      turnLOOP.motorOut = headingCorrection * sign(turnLOOP.pidOut);
    }

    driveL(driveLOOP.motorOut + turnLOOP.motorOut, absVoltage);
    driveR(driveLOOP.motorOut - turnLOOP.motorOut, absVoltage);

    driveSLEW.lastValMTR = driveLOOP.motorOut;

    if(driveLOOP.processVariable < driveLOOP.target + range && driveLOOP.processVariable > driveLOOP.target - range) {
      if(dlf.get_actual_velocity() > -velocity && dlf.get_actual_velocity() < velocity
         && drb.get_actual_velocity() > -velocity && drb.get_actual_velocity() < velocity) {
        lockDrive();
        driveZero();
        lcd::print(1, "LOOP FINISHED at %5.2lu", timed.timer);
        timed.atTarget = true;
      }
     }

    //  break out of while loop if the elapsed time of the function is too long
    if(timed.timer > maxTime){
      lockDrive();
      lcd::print(1, "LOOP TIMED OUT at %lu", millis());
      timed.atTarget = true;
    }

    driveVec.elapsed.push_back(timed.timer);
    driveVec.process.push_back(driveLOOP.processVariable);
    driveVec.target.push_back(driveLOOP.target);
    driveVec.motor.push_back(driveLOOP.motorOut);

    delay(10);
  }
}

void drive(float fTarget, int maxVoltage, float distance, int smallVoltage) {
  drive(fTarget, maxVoltage, distance, smallVoltage, 5000);
}

void drive(float fTarget, int maxVoltage, int maxTime) {

   fTarget = fTarget/WHEEL_CIRCMF * 360 / 5 * 3;

   float range = 10;
   float velocity = 4;
   float absVoltage = abs(maxVoltage);


   driveSLEW.accelRate = signChecker(driveSLEW.accelRate, fTarget);
   turnSLEW.accelRate = signChecker(turnSLEW.accelRate, absVoltage);

   float headingCorrection = 0.2 * absVoltage;

   clearLCDLines();
	 resetEncoders();
	 brakeDrive();
   loopInit(driveLOOP, fTarget);
   loopInit(turnLOOP, target);
   timerInit(timed);

   while(!timed.atTarget) {
      //Update elapsed time
      timed.timer = millis() - timed.startTime;

      //update sensor inputs
  	  driveLOOP.processVariable = getAverageEncoderValues();
      turnLOOP.processVariable = imu.get_rotation();

      //update PID calculations
  	  driveLOOP.pidOut = pidCalculate(drivePID, driveLOOP.target, driveLOOP.processVariable);
      turnLOOP.pidOut = pidCalculate(headingPID, turnLOOP.target, turnLOOP.processVariable);

      //calculate the SLEW
      driveLOOP.motorOut = slewCalculate(driveSLEW.lastValMTR, driveLOOP.pidOut, driveSLEW.accelRate, maxVoltage);

      if(headingCorrection < fabs(turnLOOP.pidOut)) {
        turnLOOP.motorOut = headingCorrection * sign(turnLOOP.pidOut);
      }

      driveL(driveLOOP.motorOut + turnLOOP.motorOut, absVoltage);
      driveR(driveLOOP.motorOut - turnLOOP.motorOut, absVoltage);

      driveSLEW.lastValMTR = driveLOOP.motorOut;

			if(driveLOOP.processVariable < driveLOOP.target + range && driveLOOP.processVariable > driveLOOP.target - range) {
        if(dlf.get_actual_velocity() > -velocity && dlf.get_actual_velocity() < velocity
           && drb.get_actual_velocity() > -velocity && drb.get_actual_velocity() < velocity) {
          lockDrive();
				  driveZero();
				  lcd::print(1, "LOOP FINISHED at %5.2lu", timed.timer);
				  timed.atTarget = true;
        }
			 }

      //  break out of while loop if the elapsed time of the function is too long
		  if(timed.timer > maxTime){
        lockDrive();
        lcd::print(1, "LOOP TIMED OUT at %lu", millis());
			  timed.atTarget = true;
		  }

      driveVec.elapsed.push_back(timed.timer);
      driveVec.process.push_back(driveLOOP.processVariable);
      driveVec.target.push_back(driveLOOP.target);
      driveVec.motor.push_back(driveLOOP.motorOut);
      driveVec.turn.push_back(turnLOOP.motorOut);


      lcd::print(0, "PROCESS: %f", turnLOOP.processVariable);
      lcd::print(1, "TARGET %f", turnLOOP.target);
      lcd::print(2, "PID OUT %f", turnLOOP.pidOut);
      lcd::print(3, "MOTOR %f", turnLOOP.motorOut);

      delay(10);
   }
}

void drive(float fTarget, int maxVoltage) {
  drive(fTarget, maxVoltage, 5000);
}
