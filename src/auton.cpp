#include "main.h"

void firstGoal() {
  drive(35.5, 125);
  turn(-135, 125, 1600);
  intake.mode = INTK_COAST;
  driveToGoal(18, 125);
  shoot(1);
  pidInit(headingPID, 1, 0, 0, 0, 0);
  drive(-47.5, 110);
  pidInit(headingPID, 3, 0, 0, 0, 0);
  pidInit(turnPID, 5, 0, 0.3, 0, 0);
  turn(0, 125, 1600);
  intake.mode = INTK_IN;
}

void secondGoal() {
  goToBall(0);
  pidInit(turnPID, 5, 0, 0.3, 0, 0);
  turn(-90, 125, 1600);
  drive(26, 125);
  intake.mode = INTK_COAST;
  driveToGoal(5, 110);
  shoot(1);
  drive(-18, 125);
  pidInit(turnPID, 5, 0, 0.3, 0, 0);
  turn(0, 125, 1600);
  intake.mode = INTK_IN;
}

void thirdGoal() {
  pidInit(drivePID, 1, 0, 0.125, 0, 0);
  pidInit(headingPID, 2, 0, 0, 0, 0);
  drive(50, 85);
  pidInit(headingPID, 3, 0, 0, 0, 0);
  pidInit(turnPID, 5, 0, 0.3, 0, 0);
  turn(-90, 125, 1600);
  drive(13, 125);
  intake.mode = INTK_COAST;
  pidInit(turnPID, 5, 0, 0.25, 0, 0);
  turn(-45, 125, 1600);
  driveToGoal(13, 125);
  shoot(1);
  pidInit(headingPID, 2, 0, 0, 0, 0);
  drive(-47.5, 110);
  pidInit(headingPID, 3, 0, 0, 0, 0);
  intake.mode = INTK_IN;
  pidInit(turnPID, 5, 0, 0.25, 0, 0);
  turn(180, 125, 1600);
}

void fourthGoal() {
  lastCorrect(180, 1095, 1780);
  pidInit(turnPID, 5, 0, 0.25, 0, 0);
  turn(0, 125, 1600);
  intake.mode = INTK_COAST;
  pidInit(drivePID, 1, 0, 0.125, 0, 0);
  drive(30, 125);
  driveToGoal(2, 50);
  shoot(1);
  drive(-5, 100);
  turn(90, 125, 1600);
  intake.mode = INTK_IN;
}
//
void fifthGoal() {
  pidInit(drivePID, 1, 0, 0.125, 0, 0);
  drive(50, 100);//prev 48
  turn(45, 125, 1600);
  intake.mode = INTK_COAST;
  driveToGoal(13, 125);
  shoot(2);
  pidInit(headingPID, 2, 0, 0, 0, 0);
  drive(-47.5, 110);
  pidInit(headingPID, 3, 0, 0, 0, 0);
  intake.mode = INTK_IN;
  turn(270, 110, 1600);
}

void sixthGoal() {
  lastCorrect(270, 1150, 1710);
  slewInit(driveSLEW, 10);
  pidInit(turnPID, 5, 0, 0.25, 0, 0);
  turn(90, 125, 1600);
  pidInit(drivePID, 1, 0, 0.125, 0, 0);
  pidInit(headingPID, 4.5, 0, 0, 0, 0);
  drive(-23, 115, 1100);
  turn(80, 50, 300);
  turn(90, 50, 300);
  drive(12.5, 115);//prev 10
  turn(90, 60, 400);
  drive(-13, 115, 1500);
  drive(18, 115);
  turn(90, 60, 400);
  drive(15, 125);
  //----
  pidInit(headingPID, 3, 0, 0, 0, 0);
  intake.mode = INTK_COAST;
  driveToGoal(10, 125);
  shoot(1);
  drive(-36, 125);
  turn(180, 125, 1600);
  intake.mode = INTK_IN;
}

void v2() {
  drive(22, 125);//24
  turn(0, 125, 1600);
  lastCorrect(0, 1100, 1700);
  pidInit(turnPID, 5, 0, 0.3, 0, 0);
  turn(0, 125, 1600);
  intake.mode = INTK_COAST;
  drive(7, 110);
  shoot(2);//1
  drive(-15, 126);
}

void seventhGoal() {
  pidInit(drivePID, 1, 0, 0.125, 0, 0);
  drive(48, 90);
  turn(90, 115, 1600);
  drive(14, 125);//prev 12.5
  intake.mode = INTK_COAST;
  turn(135, 125, 1600); //undershoot by 0.63
  driveToGoal(13, 125);
  shoot(1);
  pidInit(headingPID, 2, 0, 0, 0, 0);
  drive(-47.5, 110);
  pidInit(headingPID, 3, 0, 0, 0, 0);
  turn(0, 125, 1600);
  intake.mode = INTK_IN;
}

void eighthGoal() {
  lastCorrect(0, 1100, 1850);
  pidInit(turnPID, 5, 0, 0.3, 0, 0);
  turn(0, 115, 1600);
  intake.mode = INTK_COAST;
  drive(8.5, 110);//prev 7
  shoot(2);
  drive(-15, 126);

}





void auton(){
  int time = millis();
  int elapsed = 0;
  shooter.mode = SHOT_IN;
  delay(150);
  shooter.mode = SHOT_MOVE_IN;
  intake.mode = INTK_IN;
  indexer.mode = INDX_MOVE_IN;
  pidInit(headingPID, 1, 0, 0, 0, 0);
  pidInit(turnPID, 5, 0, 0.25, 0, 0);
  pidInit(drivePID, 1, 0, 0.125, 0, 0);
  pidInit(distPID, 9.5, 0, 0.125, 0, 0);
  slewInit(driveSLEW, 20);
  slewInit(turnSLEW, 40);

  firstGoal();
  secondGoal();
  thirdGoal();
  fourthGoal();
  fifthGoal();
  sixthGoal();
  v2();
  seventhGoal();
  eighthGoal();


  // delay(500);
  // drive(-5, 85);
  // turn(75, 100, 400);
  // drive(5, 85);
  // centerShoot(1);
  // drive(-20, 126);

  clearLCDLines();
  elapsed = millis() - time;
  lcd::print(7, "Elapsed %d", elapsed);

  std::vector<std::pair<std::string, std::vector<float>>> drive
  = {{"Elapsed", driveVec.elapsed}, {"Process", driveVec.process},  {"Target", driveVec.target}, {"Motor", driveVec.motor}};
  write_csv("/usd/Drive.csv", drive);

  std::vector<std::pair<std::string, std::vector<float>>> goal
  = {{"Elapsed", goalVec.elapsed}, {"Process", goalVec.process}, {"Target", goalVec.target}, {"Motor", goalVec.motor}};
  write_csv("/usd/Goal.csv", goal);

  std::vector<std::pair<std::string, std::vector<float>>> turn
  = {{"Elapsed", turnVec.elapsed}, {"Encoder", turnVec.process}, {"Target", turnVec.target}, {"Motor", turnVec.motor}};
  write_csv("/usd/Turn.csv", turn);

  //
  //
  //   while(true) {
  //       lcd::print(5, "LEFT: %d", left.get());
  //       lcd::print(6, "BACK: %d", back.get());
  //       delay(10);
  //   }



    // while(true)
    // {
    //   lcd::print(1, "BCKLFT: %d", backLeft.get());
    //   lcd::print(2, "BCKRGT: %d", backRight.get());
    //   lcd::print(3, "IMU %f", imu.get_rotation());
    //   delay(30);
    // }

}
