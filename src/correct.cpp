#include "main.h"

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
    pidInit(backPID, 9.5, 0, 0.125, 0, 0);
    driveBack(by, 115);
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
    driveBack(1088, 50);
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
  driveBack(by, 85);
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
    driveBack(1088, 50);
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
