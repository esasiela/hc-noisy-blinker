/***************************************************************************
  This is a library for randomly changing RGB values for pretty colors
 ***************************************************************************/
#ifndef __HC_MOVING_COLOR__
#define __HC_MOVING_COLOR__

#include "Arduino.h"


#define MOVING_MAX 255
#define MOVING_MIN 50
#define MOVING_STEP 10
#define MOVING_DIR_CHANGE_PCT 10


class MovingColor {
  public:
    MovingColor();
    byte red();
    byte green();
    byte blue();
    void move();
  private:
    // specifically using 16 bits, because our calculations would temporarily overflow 8 bits and break
    uint16_t valR;
    uint16_t valG;
    uint16_t valB;
    bool dirR;
    bool dirG;
    bool dirB;
};

#endif
