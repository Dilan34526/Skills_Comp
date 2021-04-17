#include "main.h"

Motor shot(SHOT,  E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
ADIAnalogIn line(LINE);

SHOOTER shooter;

void initShooter(){
  shot.set_brake_mode(E_MOTOR_BRAKE_COAST);
  shooter.mode = SHOT_DO_NOTHING;
  shooter.maxVol = 127;
  shooter.slowVol = 30;
  shooter.minVol = -127;
  shooter.shoot = false;
}

void shootOne() {
  bool achieved = false;
  bool once = true;
  indexer.mode = INDX_DO_NOTHING;
  shooter.mode = SHOT_DO_NOTHING;
  int many = count.shotOut;
  many = many + 1;
  int beginTime = millis();
  int time = 0;
  while(!achieved) {
    time = millis() - beginTime;
    shooter.mode = SHOT_IN;
    if(many == count.shotOut || time > 1400) {
      achieved = true;
    }
    delay(10);
  }
  shooter.mode = SHOT_MOVE_IN;
  indexer.mode = INDX_MOVE_IN;
}

void shootTwo() {
  bool achieved = false;
  bool once = true;
  indexer.mode = INDX_COAST;
  shooter.mode = SHOT_DO_NOTHING;
  int two = count.shotOut;
  two = two + 2;
  int one = count.shotOut;
  one = one + 1;
  int beginTime = millis();
  int time = 0;
  while(!achieved) {
    time = millis() - beginTime;
    shooter.mode = SHOT_IN;
    indexer.mode = INDX_IN;
    if(one == count.shotOut && once) {
      shot.tare_position();
      while(shot.get_position() > - 1500 || time > 2000) {
        time = millis() - beginTime;
        shooter.mode = SHOT_OUT;
        delay(10);
      }
      once = false;
    }
    if(two == count.shotOut || time > 2800) {
      achieved = true;
    }
    delay(10);
  }
  shooter.mode = SHOT_MOVE_IN;
  indexer.mode = INDX_MOVE_IN;
}

void centerShootOne() {
  bool achieved = false;
  bool once = true;
  indexer.mode = INDX_DO_NOTHING;
  shooter.mode = SHOT_DO_NOTHING;
  int many = count.shotOut;
  many = many + 1;
  int beginTime = millis();
  int time = 0;
  while(!achieved) {
    time = millis() - beginTime;
    shooter.mode = SHOT_SLOW_IN;
    if(many == count.shotOut || time > 1200) {
      achieved = true;
    }
    delay(10);
  }
  shooter.mode = SHOT_MOVE_IN;
  indexer.mode = INDX_MOVE_IN;
}

void CenterShootWithIndexer() {
  bool achieved = false;
  bool once = true;
  indexer.mode = INDX_DO_NOTHING;
  shooter.mode = SHOT_DO_NOTHING;
  int many = count.shotOut;
  many = many + 1;
  int beginTime = millis();
  int time = 0;
  while(!achieved) {
    time = millis() - beginTime;
    shooter.mode = SHOT_SLOW_IN;
    indexer.mode = INDX_IN;
    if(many == count.shotOut || time > 1200) {
      achieved = true;
    }
    delay(10);
  }
  shooter.mode = SHOT_MOVE_IN;
  indexer.mode = INDX_MOVE_IN;
}

void shooterTask(void* param){
  initShooter();

  while(1){

    switch (shooter.mode) {

      case SHOT_IN:
        shot.move(shooter.maxVol);
  			break;

      case SHOT_OUT:
        shot.move(shooter.minVol);
    		break;

      case SHOT_SLOW_IN: {
        shot.move(90);
        break;
      }

      case SHOT_MOVE_IN: {
        if(filled[0]) {
          shot.move_velocity(0);
        } else {
          shot.move(shooter.slowVol);
        }
      	break;
      }

      case SHOT_MOVE_OUT: {
        if(!filled[1]) {
          shot.move(-70);
        } else if (!filled[0]) {
          shot.move(-30);
        } else {
          shot.move_velocity(0);
        }
        shot.move(-70);
        break;
      }


      case SHOT_PREP: {
        shot.tare_position();
        while(shot.get_position() < 360) {
          shot.move(50);
        }
        shooter.mode = SHOT_MOVE_IN;
      }

      case SHOT_COAST:
      shot.move(0);
      break;

      case SHOT_DO_NOTHING:
      break;

    }

    delay(35);
  }
}
