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

  float range = 10;
  float velocity = 10;

  float headingCorrection = 0.2 * maxVoltage;

  clearLCDLines();
  resetEncoders();
  brakeDrive();
  loopInit(driveLOOP, fTarget);
  loopInit(turnLOOP, imu.get_rotation());
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
      driveLOOP.motorOut = slewUp(driveSLEW.lastValMTR, driveLOOP.pidOut, driveSLEW.accelRate, maxVoltage);
    } else {
      driveLOOP.motorOut = slewDown(driveSLEW.lastValMTR, driveLOOP.pidOut, driveSLEW.accelRate, smallVoltage);
    }

    if(headingCorrection < fabs(turnLOOP.pidOut)) {
      turnLOOP.motorOut = headingCorrection * sign(turnLOOP.pidOut);
    }

    driveL(driveLOOP.motorOut + turnLOOP.motorOut, maxVoltage);
    driveR(driveLOOP.motorOut - turnLOOP.motorOut, maxVoltage);

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
      // timeOut.push_back(1.0);
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

void drive(float fTarget, int maxVoltage, int maxTime) {

   fTarget = fTarget/WHEEL_CIRCMF * 360 / 5 * 3;

   float range = 10;
   float velocity = 10;

   float headingCorrection = 0.2 * maxVoltage;

   clearLCDLines();
	 resetEncoders();
	 brakeDrive();
   loopInit(driveLOOP, fTarget);
   loopInit(turnLOOP, floor(imu.get_rotation()));
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
      driveLOOP.motorOut = slewUp(driveSLEW.lastValMTR, driveLOOP.pidOut, driveSLEW.accelRate, maxVoltage);

      if(headingCorrection < fabs(turnLOOP.pidOut)) {
        turnLOOP.motorOut = headingCorrection * sign(turnLOOP.pidOut);
      }

      driveL(driveLOOP.motorOut + turnLOOP.motorOut, maxVoltage);
      driveR(driveLOOP.motorOut - turnLOOP.motorOut, maxVoltage);

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
        // timeOut.push_back(1.0);
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

void drive(float fTarget, int maxVoltage) {
  drive(fTarget, maxVoltage, 5000);
}
