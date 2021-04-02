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
  } else if(m.get_digital(E_CONTROLLER_DIGITAL_R1)) {
    intake.mode = INTK_OUT;
  } else {
    intake.mode = INTK_COAST;
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
 if(color[0] == "B") {
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

 if(color[1] == "B") {
   indx.move_velocity(0);
 } else {
   indx.move(indexer.maxVol);
 }
 shooter.mode = mode;

 drive();
 keys();

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
  } else if(m.get_digital(E_CONTROLLER_DIGITAL_X)) {
    indexer.mode = INDX_IN;
    shooter.mode = SHOT_IN;
    pressed = false;
  } else if(m.get_digital(E_CONTROLLER_DIGITAL_L2)) {
    driveShoot(SHOT_IN);
    pressed = false;
  } else {
    shooter.mode = SHOT_MOVE_IN;
    indexer.mode = INDX_MOVE_IN;
    pressed = false;
  }

  if(m.get_digital(E_CONTROLLER_DIGITAL_R2)) {
    intake.mode = INTK_IN;
  } else if (m.get_digital(E_CONTROLLER_DIGITAL_R1)) {
    intake.mode = INTK_OUT;
  } else if(!pressed) {
    intake.mode = INTK_COAST;
  }
  drive();

  // lcd::set_text(6, "Left " + std::to_string(left.get()));
}
