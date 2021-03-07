#include "main.h"

Motor inl(INL, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
Motor inr(INR, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
INTAKE intake;

void initIntake(){
  inl.set_brake_mode(E_MOTOR_BRAKE_COAST);
  inr.set_brake_mode(E_MOTOR_BRAKE_COAST);
  intake.mode = INTK_DO_NOTHING;
  intake.maxVol = 127;
  intake.minVol = -127;
  intake.intaked = false;
}

void intakeTask(void* param){
  initIntake();
  while(1){

    switch (intake.mode) {

      case INTK_IN:

        inl.move(intake.maxVol);
        inr.move(intake.maxVol);
  			break;

      case INTK_OUT:
        inl.move(intake.minVol);
        inr.move(intake.minVol);
    		break;

      case INTK_COAST:
        inl.move(0);
        inr.move(0);
        break;

      case INTK_DO_NOTHING:

        break;

    }

    delay(35);
  }
}
