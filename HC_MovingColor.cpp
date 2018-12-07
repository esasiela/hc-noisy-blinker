/***************************************************************************
  This is a library for randomly changing RGB values for pretty colors
 ***************************************************************************/
#include "Arduino.h"
#include "HC_MovingColor.h"
 
#define MOVING_MAX 255
#define MOVING_MIN 50
#define MOVING_STEP 10
#define MOVING_DIR_CHANGE_PCT 10


MovingColor::MovingColor() {
  valR = random(MOVING_MIN, MOVING_MAX+1);
  valG = random(MOVING_MIN, MOVING_MAX+1);
  valB = random(MOVING_MIN, MOVING_MAX+1);

  dirR = random(0, 2)?true:false;
  dirG = random(0, 2)?true:false;
  dirB = random(0, 2)?true:false;
}

uint8_t MovingColor::red() {
  return (uint8_t)valR;
}
uint8_t MovingColor::green() {
  return (uint8_t)valG;
}
uint8_t MovingColor::blue() {
  return (uint8_t)valB;
}

void MovingColor::move() {

  if (random(0,100)<MOVING_DIR_CHANGE_PCT) {
    dirR != dirR;
  }
  valR += (dirR?1:-1) * random(0, MOVING_STEP+1);
  if (valR>=MOVING_MAX) {
    valR = MOVING_MAX;
    dirR = false;
  } else if (valR <= MOVING_MIN) {
    valR = MOVING_MIN;
    dirR = true;
  }

  if (random(0,100)<MOVING_DIR_CHANGE_PCT) {
    dirG != dirG;
  }
  valG += (dirG?1:-1) * random(0, MOVING_STEP+1);
  if (valG>=MOVING_MAX) {
    valG = MOVING_MAX;
    dirG = false;
  } else if (valG <= MOVING_MIN) {
    valG = MOVING_MIN;
    dirG = true;
  }

  if (random(0,100)<MOVING_DIR_CHANGE_PCT) {
    dirB != dirB;
  }
  valB += (dirB?1:-1) * random(0, MOVING_STEP+1);
  if (valB>=MOVING_MAX) {
    valB = MOVING_MAX;
    dirB = false;
  } else if (valB <= MOVING_MIN) {
    valB = MOVING_MIN;
    dirB = true;
  }
}
