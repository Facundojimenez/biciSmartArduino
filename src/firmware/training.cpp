#include<Arduino.h>
#include "definitions.h"

tTraining setTraining;
intensity_t previousIntensity = NOINTENSITY;
tSummary summary = { 0, 0, 0 };
bool trainingReceived;
//updateTime y updateDistance
unsigned long lctMetersCalculated;
unsigned long lastTimeCalculatedTime;
//bool lctWaitingSummaryConfirmation;
bool summarySent;

void defaultTraining() 
{
  setTraining.setTime = DEFAULTTIME;
  setTraining.setMeters = DEFAULTMETERS;
  setTraining.dynamicMusic = DEFAULTDYNAMICMUSIC;
  trainingReceived = true;
  lctMetersCalculated = millis();
  lastTimeCalculatedTime = millis();
}

void resetTraining()
{
  showTrainingState("Restarting");
  //lctWaitingSummaryConfirmation = 0;
  previousIntensity = NOINTENSITY;
  trainingReceived = false;
  summarySent = false;

  // lcd.setRGB(RGB_LOW, RGB_LOW, RGB_HIGH);
  lcd.setRGB(RGB_HIGH, RGB_HIGH, RGB_LOW);

  setTraining.setMeters = 0;
  setTraining.setTime = 0;
  summary.averageSpeed = 0;
  summary.metersDone = 0;
  summary.timeDone = 0;
  rang25 = false;
  rang50 = false;
  rang75 = false;
  rang100 = false;
}

void resumeTraining()
{
  showTrainingState("Resumed");
  lastTimeCalculatedTime = millis();
  lctMetersCalculated = millis();
  lcd.clear();
}

void updateTrainingState() 
{
  updateDistance();
  updateTime();
  showSpeed();
  turnOnIntensityLed();
  turnOnDynamicMusic();
  turnOnBuzzer();
}

void trainingFinished(const char *mensaje) 
{
  showTrainingState(mensaje);
  sendSummary();
  //lctWaitingSummaryConfirmation = millis();
  summarySent = true;
}

void startTraining() 
{
  showTrainingState("Started");
  lctMetersCalculated = millis();
  lastTimeCalculatedTime = millis();
  lcd.clear();
}