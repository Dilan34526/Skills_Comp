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

void shoot(int num) {
  bool achieved = false;
  bool once = true;
  indexer.mode = INDX_DO_NOTHING;
  shooter.mode = SHOT_DO_NOTHING;
  int many = count.shotOut;
  many = many + num;
  int beginTime = millis();
  int time = 0;
  while(!achieved) {
    time = millis() - beginTime;
    if(num == 1) {
      shooter.mode = SHOT_IN;
    } else if (num == 2 && many + 1 == count.shotOut && once) {
      shot.tare_position();
      while(shot.get_position() > - 1500 || time > 2000) {
        time = millis() - beginTime;
        shooter.mode = SHOT_OUT;
        delay(10);
      }
      once = false;
    } else if (num == 2 && !once) {
      shooter.mode = SHOT_SLOW_IN;
      indexer.mode = INDX_IN;
    } else if(num == 2 && once) {
      shooter.mode = SHOT_IN;
      indexer.mode = INDX_IN;
    }
    if(many == count.shotOut || time > 1400 * num) {
      achieved = true;
    }
    delay(10);
  }
  shooter.mode = SHOT_MOVE_IN;
  indexer.mode = INDX_MOVE_IN;
}

void centerShoot(int num) {
  bool achieved = false;
  bool once = true;
  indexer.mode = INDX_DO_NOTHING;
  shooter.mode = SHOT_DO_NOTHING;
  int many = count.shotOut;
  many = many + num;
  int beginTime = millis();
  int time = 0;
  while(!achieved) {
    time = millis() - beginTime;
    if(num == 1) {
      shooter.mode = SHOT_SLOW_IN;
    }
    if(many == count.shotOut || time > 1400 * num) {
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
        shot.move(100);
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

      case SHOT_COAST:
      shot.move(0);
      break;

      case SHOT_DO_NOTHING:
      break;

    }

    delay(35);
  }
}
