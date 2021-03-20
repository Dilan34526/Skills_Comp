#include "main.h"

float headingCorrection = 15;
float imuDifference = 65;

PID drivePID;
PID headingPID;
PID distPID;
PID turnPID;

LOOP driveLOOP;
LOOP turnLOOP;

SLEW driveSLEW;
SLEW turnSLEW;

TIMER timed;

LOGGER driveVec;
LOGGER goalVec;
LOGGER distVec;
LOGGER turnVec;

void drive(float fTarget, int maxVoltage) {
  drive(fTarget, maxVoltage, 5000);
}

void drive(float fTarget, int maxVoltage, int maxTime) {

   fTarget = fTarget/WHEEL_CIRCMF * 360 / 5 * 3;

   float range = 10;

   headingCorrection = 0.2 * maxVoltage;

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

      //calculate the SLEW
      driveLOOP.motorOut = slewCalculate(driveSLEW.lastValMTR, driveLOOP.pidOut, driveSLEW.accelRate, maxVoltage);

      if(headingCorrection < fabs(turnLOOP.pidOut)) {
        turnLOOP.motorOut = headingCorrection * sign(turnLOOP.pidOut);
      }

      driveL(driveLOOP.motorOut + turnLOOP.motorOut, maxVoltage);
      driveR(driveLOOP.motorOut - turnLOOP.motorOut, maxVoltage);

      driveSLEW.lastValMTR = driveLOOP.motorOut;

			if(driveLOOP.processVariable < driveLOOP.target + range && driveLOOP.processVariable > driveLOOP.target - range) {
        if(dlf.get_actual_velocity() > -10 && dlf.get_actual_velocity() < 10
           && drb.get_actual_velocity() > -10 && drb.get_actual_velocity() < 10) {
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

			// lcd::print(3, "PROCESS %f", driveLOOP.processVariable);
			// lcd::print(2, "MOTOR %f", limit(driveLOOP.motorOut, maxVoltage));
			// lcd::print(4, "TRGT %f", driveLOOP.target);

      driveVec.elapsed.push_back(timed.timer);
      driveVec.process.push_back(driveLOOP.processVariable);
      driveVec.target.push_back(driveLOOP.target);
      driveVec.motor.push_back(driveLOOP.motorOut);

      delay(10);
   }

}

void driveToGoal(float fTarget, int maxVoltage) {

	fTarget = fTarget/WHEEL_CIRCMF * 360 / 5 * 3;
  headingCorrection = 0.15 * maxVoltage;
  bool achieved = false;
  unsigned long maxTime = 1500;
  float range = 10;

	clearLCDLines();
	resetEncoders();
	loopInit(driveLOOP, fTarget);
  brakeDrive();
	loopInit(turnLOOP, imu.get_rotation());
	timerInit(timed);


	while(!timed.atTarget){
		 //Update elapsed time
		 timed.timer = millis() - timed.startTime;

		 //update sensor inputs
		 driveLOOP.processVariable = getAverageEncoderValues();
		 turnLOOP.processVariable = imu.get_rotation();

		 //update PID calculations
		 driveLOOP.pidOut = pidCalculate(drivePID, driveLOOP.target, driveLOOP.processVariable);
		 turnLOOP.pidOut = pidCalculate(turnPID, turnLOOP.target, turnLOOP.processVariable);

		 driveLOOP.motorOut = slewCalculate(driveSLEW.lastValMTR, driveLOOP.pidOut, driveSLEW.accelRate, maxVoltage);

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
			 lcd::print(1, "LOOP FINISHED at %5.2lu", timed.timer);
			 timed.atTarget = true;
		 }

		 //  break out of while loop if the elapsed time of the function is too long
		 if(timed.timer > maxTime){
			 lockDrive();
			 lcd::print(1, "LOOP TIMED OUT at %lu", millis());
			 timed.atTarget = true;
		 }


     goalVec.elapsed.push_back(timed.timer);
     goalVec.process.push_back(driveLOOP.processVariable);
     goalVec.target.push_back(driveLOOP.target);
     goalVec.motor.push_back(driveLOOP.motorOut);

		 delay(10);
	}
}

void driveDist(float fTarget, int maxVoltage) {
  delay(10);
	float origin = back.get();

	bool good = true;
	if(origin > 5000 || origin < 100) {
		good = false;
	}
  fTarget = (fTarget - origin)/25.4;

  unsigned long maxTime = 3000;

  float range = 0.5;
  headingCorrection = 0.2 * maxVoltage;

	clearLCDLines();
	resetEncoders();
	coastDrive();
	loopInit(driveLOOP, fTarget);
  loopInit(turnLOOP, floor(imu.get_rotation()));
	timerInit(timed);

	while(!timed.atTarget) {
		 //Update elapsed time
		 timed.timer = millis() - timed.startTime;

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
     driveLOOP.motorOut = slewCalculate(driveSLEW.lastValMTR, driveLOOP.pidOut, driveSLEW.accelRate, maxVoltage);

		 driveL(driveLOOP.motorOut + turnLOOP.motorOut, maxVoltage);
		 driveR(driveLOOP.motorOut - turnLOOP.motorOut, maxVoltage);

     driveSLEW.lastValMTR = driveLOOP.motorOut;

		 if(driveLOOP.processVariable < driveLOOP.target + range && driveLOOP.processVariable > driveLOOP.target - range) {
       if(dlf.get_actual_velocity() > -10 && dlf.get_actual_velocity() < 10
          && drb.get_actual_velocity() > -10 && drb.get_actual_velocity() < 10) {
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

     // elapsed.push_back(timed.timer);
     // target.push_back(driveLOOP.target);
     // encoder.push_back(driveLOOP.processVariable);
     // power.push_back(driveLOOP.pidOut);
     // grad.push_back(drivePID.gradient);
     // lastValue.push_back(driveLOOP.motorOut);
     // error.push_back(drivePID.error);
     // velocity.push_back(dlf.get_actual_velocity());

     // lcd::print(2, "MOTOR %f", limit(driveLOOP.motorOut, maxVoltage));
     // lcd::print(3, "PROCESS %f", driveLOOP.processVariable);
     // lcd::print(4, "DIST %f", dist);
     // lcd::print(5, "ENC %f", enc);
     // lcd::print(6, "TRGT %f", driveLOOP.target);
     //
     // elapsed.push_back(timed.timer);
     // target.push_back(driveLOOP.target);
     // encoder.push_back(enc);
     // distance.push_back(dist);
     // process.push_back(driveLOOP.processVariable);

     distVec.elapsed.push_back(timed.timer);
     distVec.process.push_back(driveLOOP.processVariable);
     distVec.target.push_back(driveLOOP.target);
     distVec.motor.push_back(driveLOOP.motorOut);
     // distVec.power.push_back(driveLOOP.pidOut);
     // distVec.grad.push_back(drivePID.gradient);

     // distVec.error.push_back(drivePID.error);

		 delay(10);
	}
  // std::vector<std::pair<std::string, std::vector<float>>> dataset
  // = {{"Elapsed", distVec.elapsed}, {"Encoder", distVec.encoder}, {"Target", distVec.target},{"Error", distVec.error}, {"Gradient", distVec.grad}, {"Motor", distVec.lastValue}, {"Power", distVec.power}};
  // write_csv("/usd/Dist.csv", dataset);
}

void turn(float fTarget, int maxVoltage, int maxTime) {

 float range = 0.5;

 clearLCDLines();
 resetEncoders();
 brakeDrive();
 loopInit(turnLOOP, fTarget+imuDifference);
 timerInit(timed);

 while(!timed.atTarget){

   //  Update elapsed time
   timed.timer = millis() - timed.startTime;

   // update sensor inputs
   turnLOOP.processVariable = imu.get_rotation();


   // update PID calculations
   turnLOOP.pidOut = pidCalculate(turnPID, turnLOOP.target, turnLOOP.processVariable);

   turnLOOP.motorOut = slewCalculate(turnSLEW.lastValMTR, turnLOOP.pidOut, turnSLEW.accelRate, maxVoltage);

   driveL(turnLOOP.motorOut, maxVoltage);
	 driveR(-turnLOOP.motorOut, maxVoltage);

   turnSLEW.lastValMTR = turnLOOP.motorOut;

   //  refresh the at time target if error is too large
	 if(turnLOOP.processVariable < turnLOOP.target + range && turnLOOP.processVariable > turnLOOP.target - range) {
     if(dlf.get_actual_velocity() > -10 && dlf.get_actual_velocity() < 10
        && drb.get_actual_velocity() > -10 && drb.get_actual_velocity() < 10) {
       lockDrive();
       driveZero();
       lcd::print(1, "LOOP FINISHED at %5.2lu", timed.timer);
       timed.atTarget = true;
     }
		}

   //  break out of while loop if the elapsed time of the function is too long
	 if(timed.timer > maxTime){
		 holdDrive();
		 driveZero();
		 lcd::print(1, "LOOP TIMED OUT at %5.2lu", timed.timer);
		 timed.atTarget = true;
	 }

   turnVec.elapsed.push_back(timed.timer);
   turnVec.process.push_back(turnLOOP.processVariable - 65);
   turnVec.target.push_back(fTarget);
   turnVec.motor.push_back(turnLOOP.motorOut);

   delay(10);
 }

}

void goToBall(float heading) {
  delay(5);
  float rx = left.get();
  float ry = back.get();

  while(left.get() > 5000 || left.get() < 100) {
		rx = left.get();
    delay(2);
	}

  while(back.get() > 5000 || back.get() < 100) {
		ry = left.get();
    delay(2);
	}

  lcd::set_text(0, "LEFT: " + std::to_string(rx));
  lcd::set_text(1, "BACK: " + std::to_string(ry));

  float bx = 1070;
  float by = 1710;

  float x_error  = (bx - rx)/25.4;
  float y_error =  (by - ry)/25.4;

  float angle_relative = atan(y_error/x_error) * 180/M_PI;
  float dist = sqrtf((y_error*y_error) + (x_error * x_error));

  float angle_absolute;

  if(bx > rx) {
    angle_absolute = (heading + 90) - angle_relative;
  } else if (bx < rx) {
    angle_absolute = (heading - 90) + fabs(angle_relative);
  } else {
    angle_absolute = heading;
  }
  if(fabs(heading - angle_absolute) > 3) {
    turn(angle_absolute, 20, 300);
    drive(dist, 115);
  } else {
    pidInit(distPID, 9.5, 0, 0.125, 0, 0);
    driveDist(by, 115);
  }
}


void correction(float heading, float bx, float by) {
  delay(5);
  float rx = left.get();


  while(left.get() > 5000 || left.get() < 100) {
		rx = left.get();
    delay(2);
	}

  float x_error = bx - rx;

  if(fabs(x_error) > 80) {
    turn(heading + 90, 125, 1600);
    pidInit(drivePID, 9.5, 0, 0.3, 0, 0);
    driveDist(1088, 50);
    turn(heading, 125, 1600);
  } else if (fabs(x_error) > 30) {
    turn(heading + 90, 125, 1600);
    pidInit(drivePID, 1, 0, 0.10, 0, 0);
    drive(x_error/25.4, 50);
    turn(heading, 125, 1600);
  }

  if(fabs(bx - left.get()) < 26) {
    lcd::set_text(2, "APPROVED!");
  } else {
    lcd::set_text(2, "NOT APPROVED!");
  }

  intake.mode = INTK_IN;
  pidInit(headingPID, 0.1, 0, 0, 0, 0);
  pidInit(drivePID, 9.5, 0, 0.3, 0, 0);
  driveDist(by, 85);
  pidInit(headingPID, 0.1, 0, 0, 0, 0);
  pidInit(drivePID, 1, 0, 0, 0, 0);
}

void lastCorrect(float heading, float bx, float by) {
  delay(5);
  float rx = back.get();


  while(back.get() > 1650 || back.get() < 100) {
		rx = back.get();
    delay(2);
	}

  float x_error = bx - rx;

  if(fabs(x_error) > 80) {
    pidInit(drivePID, 9.5, 0, 0.3, 0, 0);
    pidInit(headingPID, 0.1, 0, 0, 0, 0);
    driveDist(1088, 50);
  } else if (fabs(x_error) > 30) {
    pidInit(drivePID, 1, 0, 0.10, 0, 0);
    drive(x_error/25.4, 50);
  }

  delay(5);
  float ry = back.get();


  while(back.get() > 1800 || back.get() < 100) {
    ry = back.get();
    delay(2);
  }

  turn(heading - 90, 125, 1600);
  intake.mode = INTK_IN;
  slewInit(driveSLEW, 17);
  drive((by - ry)/25.4, 85);
  slewInit(driveSLEW, 25);
}

void lastCorrect(float heading, float by) {
  lastCorrect(heading, 1088, by);
}
