#include "main.h"
#include <iterator>
#include <queue>

Distance empty(EMPTY);
Optical indxOpt(INDX_OPT);
Optical shotOpt(SHOT_OPT);


std::array<std::string, 2> color {"N", "N"};
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
}

// bool intkWentDown = true;
// int intaked = 0;
void countBalls() {


  if(shot.get_actual_velocity() > 0 && empty.get() < out.down) {
    out.wentDownShot = true;
  }

  if(out.wentDownShot) {
    if(empty.get() > out.up) {
      out.wentDownShot = false;
      count.shotOut++;
    }
  }

  // if(inl.get_direction() == 1 && inl.get_actual_velocity() < 175) {
  //   intkWentDown = true;
  // }
  //
  // if(intkWentDown) {
  //   if(inl.get_actual_velocity() > 175) {
  //     intkWentDown = false;
  //     intaked++;
  //   }
  // }

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

  if(shotOpt.get_hue() < 50 || shotOpt.get_hue() > 300) {
    color[0] = "R";
  } else if (shotOpt.get_hue() > 150 && shotOpt.get_hue() < 300){
    color[0] = "B";
  } else {
    color[0] = "N";
  }

  if(indxOpt.get_hue() < 100 || indxOpt.get_hue() > 325) {
    color[1] = "R";
  } else if (indxOpt.get_hue() > 150 && indxOpt.get_hue() < 300) {
    color[1] = "B";
  } else {
    color[1] = "N";
  }
}

void countTask(void* param){
  initCount();
  initOut();
  float beginTime = millis();
  std::vector<float> elapsed;
  std::vector<float> inlVel;
  std::vector<float> inrVel;
  std::vector<float> inlCur;
  std::vector<float> inrCur;

  while(true){


    countBalls();

    delay(10);
  }
}
