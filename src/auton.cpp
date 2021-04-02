#include "main.h"

int time;

void firstGoal() {
  drive(35.5, 125, 1650);
  turn(-135, 125, 850);
  intake.mode = INTK_COAST;
  pidInit(headingPID, 3, 0, 0, 0, 0);
  driveToGoal(18, 125);
  shoot(1);
  driveDiag(1100, -100);
  turn(0, 125, 1150);
  intake.mode = INTK_IN;
}

void secondGoal() {
  goToBall(0, true);
  turn(-90, 125, 1000);
  drive(26, 125, 1050);
  intake.mode = INTK_COAST;
  driveToGoal(5, 110);
  shoot(1);
  drive(-20, -125, 950);
  turn(0, 125, 950);
  intake.mode = INTK_IN;
}

void thirdGoal() {
  drive(50, 125, 36, 80, 1650);
  drive(-3, 50, 650);
  turn(-90, 125, 1000);
  drive(13, 125, 800);//low
  intake.mode = INTK_COAST;
  turn(-45, 125, 950);
  driveToGoal(13, 125);
  shoot(1);
  driveDiag(1100, -100);
  turn(90, 125, 1250);
  intake.mode = INTK_IN;
}

void fourthGoal() {
  goToBall(90);
  turn(0, 125, 1150);
  intake.mode = INTK_COAST;
  driveToGoal(32, 125);
  shoot(1);
  delay(400);
  shoot(1);
  drive(-6, -100, 650);
  turn(90, 125, 950);
  intake.mode = INTK_IN;
}

void fifthGoal() {
  drive(48, 100, 1650);
  turn(45, 125, 850);
  intake.mode = INTK_COAST;
  driveToGoal(13, 125);
  shoot(1);
  driveDiag(1100, -100);
  turn(180, 125, 1150);
  intake.mode = INTK_IN;
}

void sixthGoal() {
  goToBall(180, true);
  turn(90, 125, 1150);
  drive(23, 125, 1300);
  intake.mode = INTK_COAST;
  driveToGoal(5, 110);
  shoot(1);
  drive(-20, -125, 1050);
  turn(180, 125, 950);
  intake.mode = INTK_IN;
}

void seventhGoal() {
  pidInit(headingPID, 4.5, 0, 0, 0, 0);
  drive(50, 125, 36, 80, 1600);
  pidInit(headingPID, 3, 0, 0, 0, 0);
  drive(-3, 50, 650);
  turn(90, 125, 1000);
  drive(13, 125, 950);
  intake.mode = INTK_COAST;
  turn(135, 125, 950);
  driveToGoal(13, 125);
  shoot(1);
  driveDiag(1100, -100);
  turn(270, 125, 1150);
  intake.mode = INTK_IN;
}

void eighthGoal() {
  pidInit(headingPID, 4.5, 0, 0, 0, 0);
  goToBall(270, true);
  pidInit(headingPID, 0.5, 0, 0, 0, 0);
  turn(360, 125, 1000);
  intake.mode = INTK_OUT;
  bool there = filled[1];
  pidInit(headingPID, 0, 0, 0, 0, 0);
  drive(24, 125, 1500);
  intake.mode = INTK_COAST;
  float elapsed = millis() - time;
  if(elapsed < 59000 && there) {
    centerShoot(1);
    delay(750);
    centerShoot(1);
  } else {
    centerShoot(1);
  }
  drive(-12, -125);
}







void auton(){
  time = millis();
  int elapsed = 0;
  brakeDrive();
  shooter.mode = SHOT_IN;
  delay(150);
  shooter.mode = SHOT_MOVE_IN;
  intake.mode = INTK_IN;
  indexer.mode = INDX_MOVE_IN;
  pidInit(headingPID, 0, 0, 0, 0, 0);
  pidInit(turnPID, 4.9, 0, 0.3, 0, 0);
  pidInit(drivePID, 1, 0, 0, 0, 0);
  pidInit(backPID, 9.5, 0, 0.02, 0, 0);
  pidInit(diagPID, 1.67, 0, 0.02, 0, 0);
  pidInit(diagPID2, 1.67, 0, 0.02, 0, 0);
  slewInit(driveSLEW, 15);
  slewInit(turnSLEW, 15);


  // firstGoal();
  // secondGoal();
  // thirdGoal();
  // fourthGoal();
  // fifthGoal();
  // sixthGoal();
  // seventhGoal();
  // eighthGoal();

  imuDifference = 0;

  drive(48, 125, 100000000);

 // drive(35.5, 125, 1650);
 // imuDifference = 0;
 // turn(90, 125);






  // delay(500);
  // drive(-5, 85);
  // turn(75, 100, 400);
  // drive(5, 85);
  // centerShoot(1);
  // drive(-20, 126);

  // clearLCDLines();
  elapsed = millis() - time;
  lcd::print(7, "Elapsed %d", elapsed);

  //{"PID", driveVec.pid}, {"Accel", driveVec.accel},

  std::vector<std::pair<std::string, std::vector<float>>> drive
  = {{"Elapsed", driveVec.elapsed}, {"Process", driveVec.process},  {"Target", driveVec.target}, {"Motor", driveVec.motor},  {"PID", driveVec.pid}, {"Turn", driveVec.turn},
     {"DLF", driveVec.dlf}, {"DLB", driveVec.dlb}, {"DRF", driveVec.drf}, {"DRB", driveVec.drb}};
  write_csv("/usd/Drive.csv", drive);

  std::vector<std::pair<std::string, std::vector<float>>> goal
  = {{"Elapsed", goalVec.elapsed}, {"Process", goalVec.process}, {"Target", goalVec.target}, {"Motor", goalVec.motor}};
  write_csv("/usd/Goal.csv", goal);

  std::vector<std::pair<std::string, std::vector<float>>> turn
  = {{"Elapsed", turnVec.elapsed}, {"Encoder", turnVec.process}, {"Target", turnVec.target}, {"Motor", turnVec.motor}};
  write_csv("/usd/Turn.csv", turn);

  std::vector<std::pair<std::string, std::vector<float>>> diag
  = {{"Elapsed", diagVec.elapsed}, {"Dist", diagVec.distance}, {"Process", diagVec.process}, {"Target", diagVec.target}, {"Motor", diagVec.motor}, {"PID", diagVec.pid}, {"Prev PID", diagVec.prev}};
  write_csv("/usd/Diag.csv", diag);
}

//first goal
// intake.mode = INTK_COAST;
// driveToGoal(18, 125);
// shoot(1);
// pidInit(headingPID, 1, 0, 0, 0, 0);
// drive(-47.5, 110);
// pidInit(headingPID, 3, 0, 0, 0, 0);
// pidInit(turnPID, 5, 0, 0.3, 0, 0);
// turn(0, 125, 1600);
// intake.mode = INTK_IN;


//second goal
// goToBall(0);
// pidInit(turnPID, 5, 0, 0.3, 0, 0);
// turn(-90, 125, 1600);
// drive(26, 125);
// intake.mode = INTK_COAST;
// driveToGoal(5, 110);
// shoot(1);
// drive(-18, 125);
// pidInit(turnPID, 5, 0, 0.3, 0, 0);
// turn(0, 125, 1600);
// intake.mode = INTK_IN;

//third goal
// pidInit(drivePID, 1, 0, 0.125, 0, 0);
// pidInit(headingPID, 2, 0, 0, 0, 0);
// drive(50, 85);
// pidInit(headingPID, 3, 0, 0, 0, 0);
// pidInit(turnPID, 5, 0, 0.3, 0, 0);
// turn(-90, 125, 1600);
// drive(13, 125);
// intake.mode = INTK_COAST;
// pidInit(turnPID, 5, 0, 0.25, 0, 0);
// turn(-45, 125, 1600);
// driveToGoal(13, 125);
// shoot(1);
// pidInit(headingPID, 2, 0, 0, 0, 0);
// drive(-47.5, 110);
// pidInit(headingPID, 3, 0, 0, 0, 0);
// intake.mode = INTK_IN;
// pidInit(turnPID, 5, 0, 0.25, 0, 0);
// turn(180, 125, 1600);

//fourth goal
// lastCorrect(180, 1095, 1780);
// pidInit(turnPID, 5, 0, 0.25, 0, 0);
// turn(0, 125, 1600);
// intake.mode = INTK_COAST;
// pidInit(drivePID, 1, 0, 0.125, 0, 0);
// drive(30, 125);
// driveToGoal(2, 50);
// shoot(1);
// drive(-5, 100);
// turn(90, 125, 1600);
// intake.mode = INTK_IN;

//fifth goal
// pidInit(drivePID, 1, 0, 0.125, 0, 0);
// drive(50, 100);//prev 48
// turn(45, 125, 1600);
// intake.mode = INTK_COAST;
// driveToGoal(13, 125);
// shoot(2);
// pidInit(headingPID, 2, 0, 0, 0, 0);
// drive(-47.5, 110);
// pidInit(headingPID, 3, 0, 0, 0, 0);
// intake.mode = INTK_IN;
// turn(270, 110, 1600);

//sixth goal
// lastCorrect(270, 1150, 1710);
// slewInit(driveSLEW, 10);
// pidInit(turnPID, 5, 0, 0.25, 0, 0);
// turn(90, 125, 1600);
// pidInit(drivePID, 1, 0, 0.125, 0, 0);
// pidInit(headingPID, 4.5, 0, 0, 0, 0);
// drive(-23, 115, 1100);
// turn(80, 50, 300);
// turn(90, 50, 300);
// drive(12.5, 115);//prev 10
// turn(90, 60, 400);
// drive(-13, 115, 1500);
// drive(18, 115);
// turn(90, 60, 400);
// drive(15, 125);
// //----
// pidInit(headingPID, 3, 0, 0, 0, 0);
// intake.mode = INTK_COAST;
// driveToGoal(10, 125);
// shoot(1);
// drive(-36, 125);
// turn(180, 125, 1600);
// intake.mode = INTK_IN;


//v2
// void v2() {
//   drive(22, 125);//24
//   turn(0, 125, 1600);
//   lastCorrect(0, 1100, 1700);
//   pidInit(turnPID, 5, 0, 0.3, 0, 0);
//   turn(0, 125, 1600);
//   intake.mode = INTK_COAST;
//   drive(7, 110);
//   shoot(2);//1
//   drive(-15, 126);
// }

//seventh goal
// pidInit(drivePID, 1, 0, 0.125, 0, 0);
// drive(48, 90);
// turn(90, 115, 1600);
// drive(14, 125);//prev 12.5
// intake.mode = INTK_COAST;
// turn(135, 125, 1600); //undershoot by 0.63
// driveToGoal(13, 125);
// shoot(1);
// pidInit(headingPID, 2, 0, 0, 0, 0);
// drive(-47.5, 110);
// pidInit(headingPID, 3, 0, 0, 0, 0);
// turn(0, 125, 1600);
// intake.mode = INTK_IN;



//eighth goal
// lastCorrect(0, 1100, 1850);
// pidInit(turnPID, 5, 0, 0.3, 0, 0);
// turn(0, 115, 1600);
// intake.mode = INTK_COAST;
// drive(8.5, 110);//prev 7
// shoot(2);
// drive(-15, 126);
