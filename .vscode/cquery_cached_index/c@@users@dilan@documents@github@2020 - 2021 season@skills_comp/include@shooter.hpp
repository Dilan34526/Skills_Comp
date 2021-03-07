#ifndef _SHOOTER_H_
#define _SHOOTER_H_

/*
* @struct: Shooter Data
*  @int mode: state of the shooter
*  @int maxVol: maximum voltage
*  @int minVol: minimum voltage
*/
extern struct SHOOTER {
  int mode;
  int maxVol;
  int minVol;
  int slowVol;
  bool shoot;
} SHOOTER_t;
extern SHOOTER shooter;

/*
* @enum: Shooter Behaviors
*  @state SHOT_COAST: shooter coasts
*  @state SHOT_IN: shooter rolls forward
*  @state SHOT_OUT: shooter rolls backward
*/
enum shooterModes{
  SHOT_IN,
  SHOT_OUT,
  SHOT_MOVE_IN,
  SHOT_MOVE_OUT,
  SHOT_COAST,
  SHOT_DO_NOTHING,
};

extern void shoot(int num);

/*
* @task: State Machine for Shooter Motor
*/
extern void shooterTask(void* param);

// extern void shoot(int num);

#endif
