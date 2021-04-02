#ifndef _INTAKE_H_
#define _INTAKE_H_

/*
* @struct: Intake Data
*  @int mode: state of the intake
*  @int maxVol: maximum voltage
*  @int minVol: minimum voltage
*/
extern struct INTAKE{
  int mode;
  int maxVol;
  int minVol;
  bool intaked;
} INTAKE_t;
extern INTAKE intake;

/*
* @enum: Intake Behaviors
*  @state INTK_IN: intake rolls forward
*  @state INTK_OUT: intake rolls backward
*  @state INTK_COAST: intake coasts
*  @state INTK_DO_NOTHING: intake does nothing
*/
enum intakeModes{
  INTK_IN = 0,
  INTK_OUT,
  INTK_COAST,
  INTK_DO_NOTHING,
};

/*
* @task: State Machine for Intake Motors
*/
extern void intakeTask(void* param);

#endif
