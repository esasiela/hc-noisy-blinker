/*
 * Mode is cycled with the push button on Pin D4.
 * 
 * all modes include blinking to the music, user can push the threshold way high to enable CONSTANT ON
 * 1 - independently migrate colors
 * 2 - migrate as one color
 * 3 - independently migrate colors - 4 LEDs on peaks, 4 on threshold
 * 4 - migrate as one - 4 LEDs on peaks, 4 on threshold
 * 5 - independently migrate colors - 3 LEDs on peaks, 3 on threshold, 2 on constantly
 * 6 - migrate as one - 3 LEDs on peaks, 3 on threshold, 2 on constantly
 * 7 - user defined color - 2 on constant, 3 on peaks, 3 on threshold
 * 8 - user defined color - 2 on constant, 6 on threshold
 * 
 * future consideration:
 * x - audio linear graph, migrating colors as one
 * 
 */
#include <EEPROM.h>
#include <HC_BouncyButton.h>
#include "HC_MovingColor.h"

//#define __SERIAL_DEBUG__ 1

#include <FastLED.h>


#define EEPROM_ADDR_MODE 0
#define EEPROM_ADDR_RED 1
#define EEPROM_ADDR_GREEN 2
#define EEPROM_ADDR_BLUE 3


#define PIN_SOUND_DIGITAL 3
#define PIN_MODE_BUTTON 4

#define PIN_SOUND_ANALOG A0
#define PIN_SOUND_THRESH A1
#define PIN_BRIGHTNESS A2
#define PIN_RANDOM_SEED A3



#define PIN_PIXEL 5
#define PIXEL_COUNT 8

#define BRIGHTNESS_MAX 255
#define BRIGHTNESS_MIN 0

#define BRIGHTNESS_CHANGE_THRESHOLD 4

#define MAX_SYSTEM_MODE 8
uint8_t systemMode=1;

uint8_t userDefRed, userDefGreen, userDefBlue;

boolean showingModeChange=false;
unsigned long modeChangeMillis;
#define MODE_CHANGE_DURATION 2000

BouncyButton modeButton = BouncyButton(PIN_MODE_BUTTON);


MovingColor pixelColors[PIXEL_COUNT];

unsigned long lastMoveMillis;
#define MOVE_DURATION_MILLIS 100



#define BRIGHTNESS_DURATION_MILLIS 100
unsigned long lastBrightnessMillis;

int brightness=255;
int threshold = 530;
int volA;
int volD;

#define MAX_ANALOG_READ 1023
int sampleHigh;
int sampleLow=MAX_ANALOG_READ;
int sampleCount;
long sampleAcc;

int sampleD0, sampleD1;

#define SAMPLE_DURATION_MILLIS 250
unsigned long lastReportMillis;

#define THRESH_DURATION_MILLIS 100
unsigned long lastThreshMillis;

unsigned long nowMillis;



CRGB fastled[PIXEL_COUNT];


void printPrompt() {
  Serial.println(F("Supply a RGB value for magic mode 8 (three space separated numbers 0-255): "));
}

void loadUserDefinedColors() {
  userDefRed = EEPROM.read(EEPROM_ADDR_RED);
  userDefGreen = EEPROM.read(EEPROM_ADDR_GREEN);
  userDefBlue = EEPROM.read(EEPROM_ADDR_BLUE);
#ifdef __SERIAL_DEBUG__
  Serial.print(F("USER DEFINED Color (R,G,B): "));
  Serial.print(userDefRed);
  Serial.print(F(", "));
  Serial.print(userDefGreen);
  Serial.print(F(", "));
  Serial.print(userDefBlue);
  Serial.println("");
#endif
}

void setup() {

  Serial.begin(9600);

  pinMode(PIN_SOUND_DIGITAL, INPUT);

  FastLED.addLeds<NEOPIXEL, PIN_PIXEL>(fastled, PIXEL_COUNT);

  systemMode = EEPROM.read(EEPROM_ADDR_MODE);
  if (systemMode==0) {
    systemMode=1;
  } else if (systemMode>MAX_SYSTEM_MODE) {
    systemMode=MAX_SYSTEM_MODE;
  }

  pinMode(PIN_MODE_BUTTON, INPUT_PULLUP);
  modeButton.init();

  randomSeed(analogRead(PIN_RANDOM_SEED));

  // MovingColors were already initialized before this call to randomSeed, so it only impacts future calls to move(), not the starting conditions

  loadUserDefinedColors();

  printPrompt();
}

void loop() {
  nowMillis = millis();

  while (Serial.available()>0) {
    int tmpR = Serial.parseInt();
    int tmpG = Serial.parseInt();
    int tmpB = Serial.parseInt();
    
    Serial.print("Red: ");
    Serial.println(tmpR);
    Serial.print("Green: ");
    Serial.println(tmpG);
    Serial.print("Blue: ");
    Serial.println(tmpB);

    if (Serial.read() == '\n') {
      // process the stuff
      Serial.println("Done reading RGB, updating system.");

      userDefRed = constrain(tmpR, 0, 255);
      userDefGreen = constrain(tmpG, 0, 255);
      userDefBlue = constrain(tmpB, 0, 255);

      EEPROM.write(EEPROM_ADDR_RED, userDefRed);
      EEPROM.write(EEPROM_ADDR_GREEN, userDefGreen);
      EEPROM.write(EEPROM_ADDR_BLUE, userDefBlue);

      loadUserDefinedColors();
     
      printPrompt();
    }
  }


  if (modeButton.update() && !modeButton.getState()) {
    // button pressed
    systemMode++;
    if (systemMode>MAX_SYSTEM_MODE) {
      // yes, we number at 1, not zero, deal with it
      systemMode=1;
    }

    EEPROM.write(EEPROM_ADDR_MODE, systemMode);
    
    if (systemMode==7 || systemMode==8) {
      loadUserDefinedColors();
    }

    
    // show the new mode number
    showingModeChange = true;
    modeChangeMillis = nowMillis;

    for (int i=0; i<PIXEL_COUNT; i++) {
      if (i+1 == systemMode) {
        fastled[i] = CRGB::Red;
      } else {
        fastled[i] = CRGB::Black;
      }
    }

    FastLED.show();
  }



  if (showingModeChange) {
    if ((nowMillis-modeChangeMillis)>=MODE_CHANGE_DURATION) {
        // done showing mode change, now the system will resume
        showingModeChange=false;
#ifdef __SERIAL_DEBUG__
        Serial.print(F("done showing mode change after duration "));
        Serial.println(nowMillis-modeChangeMillis);
#endif
    }

  } else {

    /***********************
     * SAMPLE AUDIO DATA
     ***********************/
    delay(1);
    volA = analogRead(PIN_SOUND_ANALOG);
    volD = digitalRead(PIN_SOUND_DIGITAL);  

    if (volD == HIGH) {
      sampleD1++;
    } else {
      sampleD0++;
    }

    sampleAcc += volA;
    sampleCount++;
  
    if (volA>sampleHigh) {
      sampleHigh = volA;
    }
    if (volA<sampleLow) {
      sampleLow = volA;
    }
  
    if ((nowMillis-lastThreshMillis)>=THRESH_DURATION_MILLIS) {
      lastThreshMillis = nowMillis;

      threshold = analogRead(PIN_SOUND_THRESH);
    }

    if ((nowMillis-lastReportMillis)>=SAMPLE_DURATION_MILLIS) {
      lastReportMillis = nowMillis;
    
#ifdef __SERIAL_DEBUG__
      Serial.print("low=");
      Serial.print(sampleLow);
      Serial.print(" avg=");
      if (sampleCount>0) {
        Serial.print((int)(sampleAcc/sampleCount));
      } else {
        Serial.print("NaN");
      }
      Serial.print(" high=");
      Serial.print(sampleHigh);

      Serial.print(" digital LOW=");
      Serial.print(sampleD0);
      Serial.print(" HIGH=");
      Serial.print(sampleD1);

      Serial.print(" thresh=");
      Serial.print(threshold);

      Serial.print(" brightness=");
      Serial.print(brightness);

      Serial.print(" mapped=");
      Serial.print(map(brightness, 0, 1023, BRIGHTNESS_MIN, BRIGHTNESS_MAX));
    
      Serial.println("");
#endif

      // reset the sample trackers
      sampleHigh=0;
      sampleLow=MAX_ANALOG_READ;
      sampleCount=0;
      sampleAcc=0;

      sampleD0=0;
      sampleD1=0;
    }
    







    if (volA >= threshold) {
      // Sound is loud
      for (int i=0; i<PIXEL_COUNT; i++) {

// 5 - independently migrate colors - 3 LEDs on peaks, 3 on threshold, 2 on constantly

        if (systemMode==1 || systemMode==3 || systemMode==5) {
          fastled[i] = CRGB(pixelColors[i].red(), pixelColors[i].green(), pixelColors[i].blue());
        } else if (systemMode==2 || systemMode==4 || systemMode==6) {
          fastled[i] = CRGB(pixelColors[0].red(), pixelColors[0].green(), pixelColors[0].blue());
        } else if (systemMode==7 || systemMode==8) {
          fastled[i] = CRGB(userDefRed, userDefGreen, userDefBlue);
        }
      }
    
      if (volA == sampleHigh) {
        // this is a peak sample, maybe do some special display depending on mode???
      } else {
        // turn off some pixels that are not the PEAK
        if (systemMode==3 || systemMode==4) {
          for (int i=0; i<PIXEL_COUNT; i++) {
            if (i==0 || i==1 || i==6 || i==7) {
              fastled[i] = CRGB::Black;
            }
          } 
        } else if (systemMode==5 || systemMode==6 || systemMode==7) {
          for (int i=0; i<PIXEL_COUNT; i++) {
            if (i==2 || i==4 || i==6) {
              fastled[i] = CRGB::Black;
            }
          }
        }
      }
    
    } else {
      // Sound is quiet
      for (int i=0; i<PIXEL_COUNT; i++) {
        fastled[i] = CRGB::Black;
      }
    }

    // modes 5-8 keep pixels 0&7 always on
    if (systemMode==5) {
      fastled[0] = CRGB(pixelColors[0].red(), pixelColors[0].green(), pixelColors[0].blue());
      fastled[7] = CRGB(pixelColors[7].red(), pixelColors[7].green(), pixelColors[7].blue());
    } else if (systemMode==6) {
      fastled[0] = CRGB(pixelColors[0].red(), pixelColors[0].green(), pixelColors[0].blue());
      fastled[7] = CRGB(pixelColors[0].red(), pixelColors[0].green(), pixelColors[0].blue());
    } else if (systemMode==7 || systemMode==8) {
      fastled[0] = CRGB(userDefRed, userDefGreen, userDefBlue);
      fastled[7] = CRGB(userDefRed, userDefGreen, userDefBlue);
    }

    FastLED.show();

    if ((nowMillis-lastMoveMillis)>=MOVE_DURATION_MILLIS) {
      lastMoveMillis=nowMillis;

      for (int i=0; i<PIXEL_COUNT; i++) {
        pixelColors[i].move();
      }
    }


    if ((nowMillis-lastBrightnessMillis)>=BRIGHTNESS_DURATION_MILLIS) {
      lastBrightnessMillis = nowMillis;

      if (analogRead(PIN_BRIGHTNESS)!=brightness) {
        brightness = analogRead(PIN_BRIGHTNESS);
        FastLED.setBrightness(map(brightness, 0, 1023, BRIGHTNESS_MIN, BRIGHTNESS_MAX));
      }
    }
  }
}
