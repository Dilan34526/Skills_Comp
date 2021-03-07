#include "main.h"
#include <iterator>
#include <queue>

Distance shotDist(SHOT_DIST);
Optical goalLeft(GOLFT_OPT);
Optical goalRight(GORGT_OPT);
Optical indxOpt(INDX_OPT);
Optical shotOpt(SHOT_OPT);


std::array<std::string, 2> balls {"N", "N"};
std::array<bool, 2> filled {false, false};

OUT out;
COUNT count;

void initOut() {
  out.down = 150;
  out.up = 190;
  out.wentDownShot = false;
}

void initCount() {
  count.shotOut = 0;
  indxOpt.set_led_pwm(50);
  shotOpt.set_led_pwm(50);
  goalLeft.set_led_pwm(50);
  goalRight.set_led_pwm(50);
}

void countBalls() {


  if(shot.get_actual_velocity() > 0 && shotDist.get() < out.down) {
    out.wentDownShot = true;
  }



  if(out.wentDownShot) {
    if(shotDist.get() > out.up) {
      out.wentDownShot = false;
      count.shotOut++;
    }
  }

  if(line.get_value() < 2850) {
    filled[0] = true;
    delay(5);
  } else {
    filled[0] = false;
  }

  if(shotOpt.get_hue() < 50 || shotOpt.get_hue() > 300) {
    balls[0] = "R";
  } else if (shotOpt.get_hue() > 150 && shotOpt.get_hue() < 300){
    balls[0] = "B";
  } else {
    balls[0] = "N";
  }

  if(indxOpt.get_proximity() > 200) {
    filled[1] = true;
    delay(5);
  } else {
    filled[1] = false;
  }

  if(indxOpt.get_hue() < 100 || indxOpt.get_hue() > 325) {
    balls[1] = "R";
  } else if (indxOpt.get_hue() > 150 && indxOpt.get_hue() < 300) {
    balls[1] = "B";
  } else {
    balls[1] = "N";
  }
}

void countTask(void* param){
  initCount();
  initOut();
  int beginTime = millis();
  while(true){


    countBalls();

    delay(10);
  }
}
