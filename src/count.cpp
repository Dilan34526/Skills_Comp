#include "main.h"
#include <iterator>
#include <queue>

Distance shotDist(SHOT_DIST);
Optical indxOpt(INDX_OPT);


std::string color = "N";
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

  if(indxOpt.get_proximity() > 200) {
    filled[1] = true;
    delay(5);
  } else {
    filled[1] = false;
  }

  if(indxOpt.get_hue() < 100 || indxOpt.get_hue() > 325) {
    color = "R";
  } else if (indxOpt.get_hue() > 150 && indxOpt.get_hue() < 300) {
    color = "B";
  } else {
    color = "N";
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
