#include "main.h"

float headingCorrection = 15;
float imuDifference = 65;

PID drivePID;
PID headingPID;
PID turnPID;

LOOP driveLOOP;
LOOP turnLOOP;

SLEW driveSLEW;
SLEW turnSLEW;

TIMER timed;
//
// std::vector<float> elapsed;
// std::vector<float> target;
// std::vector<float> encoder;
// std::vector<float> power;
// std::vector<float> grad;
// std::vector<float> error;
// std::vector<float> distance;
// std::vector<float> process;
// std::vector<float> lastValue;
std::vector<float> timeOut;
// std::vector<float> velocity;

void drive(float fTarget, int maxVoltage) {

   fTarget = fTarget/WHEEL_CIRCMF * 360 / 5 * 3;

   unsigned long maxTime = 5000;
   float range = 10;

   // elapsed.clear();
   // target.clear();
   // encoder.clear();
   // power.clear();
   // grad.clear();
   // lastValue.clear();
   // error.clear();
   // velocity.clear();
   headingCorrection = 0.2 * maxVoltage;

   clearLCDLines();
	 resetEncoders();
	 brakeDrive();
   pidInit(headingPID, 1, 0, 0, 0, 0);
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
          timeOut.push_back(0.0);
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

      // elapsed.push_back(timed.timer);
      // target.push_back(driveLOOP.target);
      // encoder.push_back(driveLOOP.processVariable);
      // power.push_back(driveLOOP.pidOut);
      // grad.push_back(drivePID.gradient);
      // lastValue.push_back(driveLOOP.motorOut);
      // error.push_back(drivePID.error);
      // velocity.push_back(dlf.get_actual_velocity());

      delay(10);
   }
   // std::vector<std::pair<std::string, std::vector<float>>> dataset
   // = {{"Elapsed", elapsed}, {"Target", target}, {"Encoder", encoder}, {"Error", error}, {"Gradient", grad}, {"Motor", lastValue}, {"Power", power}, {"Velocity", velocity}};
   // write_csv("/usd/Drive.csv", dataset);
}

void driveToGoal(float fTarget, int maxVoltage) {

	fTarget = fTarget/WHEEL_CIRCMF * 360 / 5 * 3;
  headingCorrection = 0.15 * maxVoltage;
  bool achieved = false;
  unsigned long maxTime = 4000;
  float range = 10;

  // elapsed.clear();
  // target.clear();
  // encoder.clear();
  // power.clear();
  // grad.clear();
  // lastValue.clear();
  // error.clear();
  // velocity.clear();

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


     // elapsed.push_back(timed.timer);
     // target.push_back(driveLOOP.target);
     // encoder.push_back(driveLOOP.processVariable);
     // power.push_back(driveLOOP.pidOut);
     // grad.push_back(drivePID.gradient);
     // lastValue.push_back(driveLOOP.motorOut);
     // error.push_back(drivePID.error);
     // velocity.push_back(dlf.get_actual_velocity());

		 // lcd::print(3, "PROCESS %f", driveLOOP.processVariable);
		 // lcd::print(2, "MOTOR %f", limit(driveLOOP.motorOut, maxVoltage));
		 // lcd::print(4, "TRGT %f", driveLOOP.target);

		 delay(10);
	}
  // std::vector<std::pair<std::string, std::vector<float>>> dataset
  // = {{"Elapsed", elapsed}, {"Target", target}, {"Encoder", encoder}, {"Error", error}, {"Gradient", grad}, {"Motor", lastValue}, {"Power", power}, {"Velocity", velocity}};
  // write_csv("/usd/Goal.csv", dataset);
}

void driveDist(float fTarget, int maxVoltage) {
  delay(10);
	float origin = back.get();

	bool good = true;
	if(origin > 5000 || origin < 100) {
		good = false;
	}
  fTarget = (fTarget - origin)/25.4;
  unsigned long maxTime = 5000;
  float range = 0.5;
  headingCorrection = 0.15 * maxVoltage;

  // elapsed.clear();
  // target.clear();
  // encoder.clear();
  // power.clear();
  // grad.clear();
  // lastValue.clear();
  // error.clear();
  // velocity.clear();

	clearLCDLines();
	resetEncoders();
	coastDrive();
	loopInit(driveLOOP, fTarget);
	timerInit(timed);

	while(!timed.atTarget) {
		 //Update elapsed time
		 timed.timer = millis() - timed.startTime;

     float dist = relativeDistance(origin, back.get(), good);
     float enc = getAverageEncoderValues()/216 * WHEEL_CIRCMF;

		 //update sensor inputs
		 driveLOOP.processVariable = (dist + enc)/2;

		 //update PID calculations
		 driveLOOP.pidOut = pidCalculate(drivePID, driveLOOP.target, driveLOOP.processVariable);

     //calculate SLEW
     driveLOOP.motorOut = slewCalculate(driveSLEW.lastValMTR, driveLOOP.pidOut, driveSLEW.accelRate, maxVoltage);

		 driveL(driveLOOP.motorOut, maxVoltage);
		 driveR(driveLOOP.motorOut, maxVoltage);

     driveSLEW.lastValMTR = driveLOOP.motorOut;

		 if(driveLOOP.processVariable < driveLOOP.target + range && driveLOOP.processVariable > driveLOOP.target - range) {
       if(dlf.get_actual_velocity() > -10 && dlf.get_actual_velocity() < 6
          && drb.get_actual_velocity() > -10 && drb.get_actual_velocity() < 6) {
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

		 delay(10);
	}
  // std::vector<std::pair<std::string, std::vector<float>>> dataset
  // = {{"Elapsed", elapsed}, {"Target", target}, {"Encoder", encoder}, {"Error", error}, {"Gradient", grad}, {"Motor", lastValue}, {"Power", power}, {"Velocity", velocity}};
  // write_csv("/usd/Dist.csv", dataset);
}

void turn(float fTarget, int maxVoltage, int maxTime) {

 float range = 0.5;

 // elapsed.clear();
 // target.clear();
 // encoder.clear();
 // power.clear();
 // grad.clear();
 // lastValue.clear();
 // error.clear();
 // velocity.clear();

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
     if(dlf.get_actual_velocity() > -10 && dlf.get_actual_velocity() < 6
        && drb.get_actual_velocity() > -10 && drb.get_actual_velocity() < 10) {
       timeOut.push_back(0.0);
       lockDrive();
       driveZero();
       lcd::print(1, "LOOP FINISHED at %5.2lu", timed.timer);
       timed.atTarget = true;
     }
		}

   //  break out of while loop if the elapsed time of the function is too long
	 if(timed.timer > maxTime){
     timeOut.push_back(1.0);
		 holdDrive();
		 driveZero();
		 lcd::print(1, "LOOP TIMED OUT at %5.2lu", timed.timer);
		 timed.atTarget = true;
	 }

	 lcd::print(3, "IMU %f", imu.get_rotation());
	 lcd::print(2, "PID %f", limit(turnLOOP.pidOut, maxVoltage));

   // elapsed.push_back(timed.timer);
   // target.push_back(turnLOOP.target);
   // encoder.push_back(turnLOOP.processVariable);
   // power.push_back(turnLOOP.pidOut);
   // grad.push_back(turnPID.gradient);
   // lastValue.push_back(turnLOOP.motorOut);
   // error.push_back(turnPID.error);

   delay(10);
 }
 // std::vector<std::pair<std::string, std::vector<float>>> dataset
 // = {{"Elapsed", elapsed}, {"Target", target}, {"Encoder", encoder}, {"Error", error}, {"Gradient", grad}, {"Motor", lastValue}, {"Power", power}};
 // write_csv("/usd/Turn.csv", dataset);
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

  float bx = 1105;
  float by = 1710;

  // if(heading == 180 && bx < rx) {
  //   bx = 1100;
  // }
  // if(heading == 90) {
  //   by = 1698;
  // }

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
    pidInit(turnPID, 3, 0, 0.25, 0, 0);
    turn(angle_absolute, 125, 1600);
    pidInit(drivePID, 1, 0, 0.125, 0, 0);
    drive(dist, 100);
  } else {
    pidInit(drivePID, 7.5, 0, 0.125, 0, 0);
    driveDist(by, 90);
  }
}
