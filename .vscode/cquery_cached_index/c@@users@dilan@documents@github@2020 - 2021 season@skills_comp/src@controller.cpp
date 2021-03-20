#include "main.h"
Controller m(E_CONTROLLER_MASTER);

bool pressed;
bool move;
bool oldFillIndx = false;
bool oldFillShot = false;

void drive() {
  brakeDrive();
  float y = m.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
  float t = m.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);

  if(sign(y) == -1) {
    driveL(y + t, 101);
    driveR(y - t, 101);
  } else {
    driveL(y + t, 127);
    driveR(y - t, 127);
  }
}

void keys() {
  if(m.get_digital(E_CONTROLLER_DIGITAL_R2)) {
    intake.mode = INTK_IN;
    lcd::set_text(2, "IN");
  } else if(m.get_digital(E_CONTROLLER_DIGITAL_R1)) {
    intake.mode = INTK_OUT;
    lcd::set_text(3, "Out");
  } else {
    intake.mode = INTK_COAST;
    lcd::set_text(4, "Coast");
  }
}

bool stop() {
  if(((goalLeft.get_hue() > 300 || goalLeft.get_hue() < 75 ) && (goalRight.get_hue() > 300 || goalRight.get_hue() < 75))) {
    return true;
  } else {
    return false;
  }
}
void driveShoot(int mode) {
 bool achieved = false;
 bool coast = false;
 bool one = true;
 indexer.mode = INDX_DO_NOTHING;
 shooter.mode = SHOT_DO_NOTHING;
 int many = count.shotOut;
 many = many + 1;
 bool detected = false;
 if(balls[0] == "B") {
   achieved = true;
 }
 while(!achieved) {

 if(count.shotOut == many && one) {
   for(int i = 0; i < 150; i++) {
     shooter.mode = SHOT_OUT;
     delay(2);
   }
   shooter.mode = mode;
   one = false;
 }


 if(balls[1] == "B") {
   detected = false;
   indx.move_velocity(0);
   shooter.mode = mode;
 } else if(!detected){
   indx.move(indexer.maxVol);
   shooter.mode = mode;
 }

 drive();
 if(stop()) {
   coast = true;
 }
 if(!coast) {
   keys();
 } else {
   intake.mode = INTK_COAST;
 }

 if(!m.get_digital(E_CONTROLLER_DIGITAL_L2)) {
   achieved = true;
 }
 delay(30);
}
}

void controllerInput() {

  if(m.get_digital(E_CONTROLLER_DIGITAL_L1)) {
    intake.mode = INTK_OUT;
    indexer.mode = INDX_OUT;
    shooter.mode = SHOT_OUT;
    pressed = true;
  } else if(m.get_digital(E_CONTROLLER_DIGITAL_R1)) {
    driveShoot(SHOT_SLOW_IN);
    pressed = false;
  } else if(m.get_digital(E_CONTROLLER_DIGITAL_L2)) {
    driveShoot(SHOT_IN);
    pressed = false;
  } else if(m.get_digital(E_CONTROLLER_DIGITAL_X)) {
    shooter.mode = SHOT_IN;
    indexer.mode = INDX_IN;
    pressed = false;
  } else {
    shooter.mode = SHOT_MOVE_IN;
    indexer.mode = INDX_MOVE_IN;
    pressed = false;
  }

  if(m.get_digital(E_CONTROLLER_DIGITAL_R2)) {
    intake.mode = INTK_IN;
  } else if(!pressed) {
    intake.mode = INTK_COAST;
  }
  drive();

  lcd::set_text(6, "Left " + std::to_string(left.get()));
}
