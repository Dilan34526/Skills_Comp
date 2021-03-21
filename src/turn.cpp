#include "main.h"

float imuDifference = 65;

PID turnPID;
LOOP turnLOOP;
SLEW turnSLEW;
LOGGER turnVec;
TIMER turnTimed;

void turn(float fTarget, int maxVoltage, int maxTime) {

 float range = 0.5;

 clearLCDLines();
 resetEncoders();
 brakeDrive();
 loopInit(turnLOOP, fTarget+imuDifference);
 timerInit(turnTimed);

 while(!turnTimed.atTarget){

   //  Update elapsed time
   turnTimed.timer = millis() - turnTimed.startTime;

   // update sensor inputs
   turnLOOP.processVariable = imu.get_rotation();


   // update PID calculations
   turnLOOP.pidOut = pidCalculate(turnPID, turnLOOP.target, turnLOOP.processVariable);

   turnLOOP.motorOut = slewUp(turnSLEW.lastValMTR, turnLOOP.pidOut, turnSLEW.accelRate, maxVoltage);

   driveL(turnLOOP.motorOut, maxVoltage);
	 driveR(-turnLOOP.motorOut, maxVoltage);

   turnSLEW.lastValMTR = turnLOOP.motorOut;

   //  refresh the at time target if error is too large
	 if(turnLOOP.processVariable < turnLOOP.target + range && turnLOOP.processVariable > turnLOOP.target - range) {
     if(dlf.get_actual_velocity() > -10 && dlf.get_actual_velocity() < 10
        && drb.get_actual_velocity() > -10 && drb.get_actual_velocity() < 10) {
       lockDrive();
       driveZero();
       lcd::print(1, "LOOP FINISHED at %5.2lu", turnTimed.timer);
       turnTimed.atTarget = true;
     }
		}

   //  break out of while loop if the elapsed time of the function is too long
	 if(turnTimed.timer > maxTime){
		 holdDrive();
		 driveZero();
		 lcd::print(1, "LOOP TIMED OUT at %5.2lu", turnTimed.timer);
		 turnTimed.atTarget = true;
	 }

   turnVec.elapsed.push_back(turnTimed.timer);
   turnVec.process.push_back(turnLOOP.processVariable - imuDifference);
   turnVec.target.push_back(fTarget);
   turnVec.motor.push_back(turnLOOP.motorOut);

   delay(10);
 }

}

void turn(float fTarget, int maxVoltage) {
  turn(fTarget, maxVoltage, 1600);
}
