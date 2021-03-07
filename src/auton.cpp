#include "main.h"

void firstGoal() {
  drive(36.5, 125);
  turn(-135, 125, 1600);
  driveToGoal(18, 125);
  intake.mode = INTK_COAST;
  shoot(1);
  pidInit(drivePID, 1, 0, 0.125, 0, 0);
  drive(-46.5, 125);
  turn(0, 125, 1600);
  intake.mode = INTK_IN;
}

void secondGoal() {
  goToBall(0);
  turn(-90, 100, 1600);
  pidInit(drivePID, 1, 0, 0.125, 0, 0);
  drive(26, 125);
  intake.mode = INTK_COAST;
  driveToGoal(5, 90);
  shoot(2);
  drive(-18, 125);
  turn(0, 100, 1600);
  intake.mode = INTK_IN;
}

void thirdGoal() {
  pidInit(drivePID, 1, 0, 0.125, 0, 0);
  drive(48, 85);
  turn(-90, 100, 1600);
  drive(12.5, 125);
  intake.mode = INTK_COAST;
  pidInit(turnPID, 2.55, 0, 0.2, 0, 0);
  turn(-45, 125, 1600);
  driveToGoal(10, 125);
  shoot(1);
  drive(-43, 125);
  intake.mode = INTK_IN;
  pidInit(turnPID, 3.5, 0, 0.15, 0, 0);
  turn(90, 100, 1600);
}

void fourthGoal() {
  goToBall(90);
  pidInit(turnPID, 3.5, 0, 0.15, 0, 0);
  turn(0, 100, 1600);
  intake.mode = INTK_COAST;
  pidInit(drivePID, 1, 0, 0.125, 0, 0);
  driveToGoal(32, 100);
  shoot(1);
  drive(-7, 100);
  turn(90, 100, 1600);
  intake.mode = INTK_IN;
}
//
void fifthGoal() {
  pidInit(drivePID, 1, 0, 0.125, 0, 0);
  drive(48, 80);
  pidInit(turnPID, 2.55, 0, 0.2, 0, 0);
  turn(45, 100, 1600);
  intake.mode = INTK_COAST;
  driveToGoal(10, 90);
  shoot(1);
  drive(-43, 100);
  intake.mode = INTK_IN;
  pidInit(turnPID, 3.5, 0, 0.15, 0, 0);
  turn(180, 100, 1600);
}

void sixthGoal() {
  goToBall(180);
  pidInit(turnPID, 3.5, 0, 0.15, 0, 0);
  turn(90, 100, 1600);
  pidInit(drivePID, 1, 0, 0.125, 0, 0);
  drive(-14, 100);
  drive(14, 100);
  drive(-14, 100);
  turn(90, 100, 1600);
  drive(31, 125);
  intake.mode = INTK_COAST;
  driveToGoal(5, 90);
  shoot(2);
  drive(-18, 125);
  turn(180, 100, 1600);
  intake.mode = INTK_IN;
}

void seventhGoal() {
  pidInit(drivePID, 1, 0, 0.125, 0, 0);
  drive(48, 100);
  turn(90, 100, 1600);
  drive(12.5, 100);
  intake.mode = INTK_COAST;
  pidInit(turnPID, 2.55, 0, 0.2, 0, 0);
  turn(135, 100, 1600);
  driveToGoal(9, 100);
  shoot(1);
  drive(-43, 100);
  pidInit(turnPID, 3.5, 0, 0.15, 0, 0);
  turn(270, 100, 1600);
  intake.mode = INTK_IN;
}

void eighthGoal() {
  goToBall(270);
  turn(0, 100, 1600);
  intake.mode = INTK_COAST;
  pidInit(drivePID, 1, 0, 0.15, 0, 0);
  drive(10, 100);
  shoot(1);
}





void auton(){
  int time = millis();
  int elapsed = 0;
  shooter.mode = SHOT_IN;
  delay(150);
  shooter.mode = SHOT_MOVE_IN;
  intake.mode = INTK_IN;
  indexer.mode = INDX_MOVE_IN;
  pidInit(turnPID, 3.5, 0, 0.15, 0, 0);
  pidInit(drivePID, 1, 0, 0.10, 0, 0); // dist - change to 10 kP
  slewInit(driveSLEW, 10);
  slewInit(turnSLEW, 10);

  // imuDifference = -90;
  // goToBall(90);

  firstGoal();
  secondGoal();
  thirdGoal();
  fourthGoal();
  fifthGoal();
  sixthGoal();
  seventhGoal();
  eighthGoal();

    // sixthGoal();
    // seventhGoal();

    // std::vector<std::pair<std::string, std::vector<float>>> data = {{"TimeOut", timeOut}};
    // write_csv("/usd/TimeOut.csv", data);
    //
  clearLCDLines();
  // lcd::set_text(6, std::to_string(left.get()));
  elapsed = millis() - time;
  lcd::print(7, "Elapsed %d", elapsed);
    // while(true) {
    //     lcd::print(6, "IMU: %f", imu.get_rotation());
    //     lcd::print(5, "DIST: %d", back.get());
    //     delay(10);
    // }



    // while(true)
    // {
    //   lcd::print(1, "BCKLFT: %d", backLeft.get());
    //   lcd::print(2, "BCKRGT: %d", backRight.get());
    //   lcd::print(3, "IMU %f", imu.get_rotation());
    //   delay(30);
    // }

}
