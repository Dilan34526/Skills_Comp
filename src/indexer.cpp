#include "main.h"

Motor indx(INDX,  E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);


INDEXER indexer;

void initIndexer(){
  indx.set_brake_mode(E_MOTOR_BRAKE_COAST);
  indexer.mode = INDX_DO_NOTHING;
  indexer.maxVol = 90;
  indexer.minVol = -127;
  indexer.first = false;
}

void indexerTask(void* param){
  initIndexer();
  while(1){

    switch (indexer.mode) {

      case INDX_IN:
        indx.move(indexer.maxVol);
        break;

      case INDX_OUT: {
        indx.move(indexer.minVol);
    		break;
      }

      case INDX_MOVE_IN: {
        if(!filled[0]) {
          indx.move(70);
        } else if (!filled[1]) {
          indx.move(60);
        } else {
          indx.move(10);
        }
        break;
      }

      case INDX_MOVE_OUT: {
        if(!filled[1]) {
          indx.move(-40);
        } else {
          indx.move_velocity(0);
        }
        break;
      }


      case INDX_COAST: {
        indx.move(0);
    		break;
      }

      case INDX_DO_NOTHING:
      break;
    }

    delay(35);
  }
}
