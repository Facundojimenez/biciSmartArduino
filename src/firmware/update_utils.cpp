#include<Arduino.h>
#include "definitions.h"

float volumeTimeout;
float previousVolumeTimeout;

void updateDistance() 
{
  unsigned long currentTime = millis();
  float timeElapsedSeconds = ((float)(currentTime - lctMetersCalculated)) / ONE_SEC;
  float distanceIncrement = speed_MS * timeElapsedSeconds;
  summary.metersDone += distanceIncrement;
  lctMetersCalculated = currentTime;
}

void updateTime()
{
  long currentTime = millis();
  float timePassed = ((float)(currentTime - lastTimeCalculatedTime)) / ONE_SEC;
  summary.timeDone += timePassed;
  lastTimeCalculatedTime = currentTime;
}

void updateVolume() 
{
  Serial.print("Asignando Volumen a: ");
  Serial.println(lastVolumeValue);
  volumeTimeout = millis();
  if (volumeTimeout - lastActivationTime > 1000)
  {
  BT.print("VOL ");
  BT.println(lastVolumeValue);
  }
  lastActivationTime = volumeTimeout;
}