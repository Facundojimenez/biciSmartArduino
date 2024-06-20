#include <Arduino.h>
#include "definitions.h"

// External Variables/ Calculo de velocidad
volatile unsigned long lastActivationTime = 0;
volatile unsigned long currentActivationTime = 0;
volatile unsigned long newacttime  = 0;
volatile unsigned long timediff = 0;
bool acabaDePedalear = false;
float lowSpeed;
float highSpeed;
//Serial serial;

void checkSpeedSensor() 
{

  newacttime = millis();
  if (newacttime - lastActivationTime > BIKE_IS_STOPPED_TIME) 
  {
    speed_MS = 0;
  }

  int sensorValue = analogRead(HALL_SENSOR_PIN);

  if (!acabaDePedalear && sensorValue < LOWER_SENSOR_HALL_THRESHOLD) 
  {
    currentActivationTime = millis();
    if (lastActivationTime > 0) 
    {
      timediff = currentActivationTime - lastActivationTime;
      speed_MS = BIKE_WHEEL_CIRCUNFERENCE_PERIMETER_MM / (float)timediff;
    }
    acabaDePedalear = true;
  }
  else 
  {
    acabaDePedalear = false;
  }

  lastActivationTime = currentActivationTime;
  Serial.println(speed_MS);
}

void checkMediaButtonSensor() 
{
  int buttonState = digitalRead(MEDIA_MOVEMENT_SENSOR_PIN);
  if (buttonState == HIGH) 
  {
    currentEvent = EVENT_NEXT_MEDIA_BUTTON;
  } else 
  {
    currentEvent = EVENT_CONTINUE;
  }
}

void checkPlayStopButtonSensor() 
{
  int buttonState = digitalRead(PLAY_STOP_MEDIA_SENSOR_PIN);
  if (buttonState == HIGH) 
  {
    currentEvent = EVENT_PLAY_STOP_MEDIA_BUTTON;
  } else 
  {
    currentEvent = EVENT_CONTINUE;
  }
}

void checkCancelButtonSensor() 
{
  int buttonState = digitalRead(TRAINING_CANCEL_PIN);
  if (buttonState == HIGH) 
  {
    currentEvent = EVENT_TRAINING_CANCELLED;
  } else 
  {
    currentEvent = EVENT_CONTINUE;
  }
}

void checkTrainingButtonSensor() 
{
  int buttonState = digitalRead(TRAINING_CONTROL_PIN);

  if (buttonState == HIGH) 
  {
    currentEvent = EVENT_TRAINING_BUTTON;
  } else 
  {
    currentEvent = EVENT_CONTINUE;
  }
}

void checkTrainingBluetoothInterface() 
{
  if (!trainingReceived) 
  {
    if (BT.available() > 0) 
    {
      String consoleCommand = BT.readString();
      //int dynamicMusic;
      //sscanf(consoleCommand.c_str(), "%d %d %d", &(setTraining.setTime), &(setTraining.setMeters), &(setTraining.dynamicMusic));
      sscanf(consoleCommand.c_str(), "%d %d %d %d %d %d", &(setTraining.setTime), &(setTraining.setMeters), 
                                                      &(setTraining.dynamicMusic), &(setTraining.enableBuzzer),
                                                      &(setTraining.enableMusicButtons), &(setTraining.intensity));
      setTraining.personalizedTraining = true;

      switch (setTraining.intensity)
      {
        case 1:
        lowSpeed = LOW_SPEED_LOW;
        highSpeed = HIGH_SPEED_LOW;
        break;
        case 2:
        lowSpeed = LOW_SPEED_MEDIA;
        highSpeed = HIGH_SPEED_MEDIA;
        break;
        case 3:
        lowSpeed = LOW_SPEED_HIGH;
        highSpeed = HIGH_SPEED_HIGH;
        break;
        default:
        break;
      }
      // if ((setTraining.setMeters != 0 && setTraining.setTime != 0) || (setTraining.setMeters == 0 && setTraining.setTime == 0)) 
      // {
      //   Serial.println("Entrenamiento Invalido");
      //   setTraining.setMeters = 0;
      //   setTraining.setTime = 0;
      //   return;
      // }
      // if (dynamicMusic)
      //   setTraining.dynamicMusic = true;
      // else
      //   setTraining.dynamicMusic = false;

      currentEvent = EVENT_TRAINING_RECEIVED;
      trainingReceived = true;
    }
  } else 
  {
    if (BT.available() > 0) 
    {
      String consoleCommand = BT.readString();
      if(consoleCommand == "RESUME" || consoleCommand == "PAUSE")
        currentEvent = EVENT_TRAINING_BUTTON;
      if(consoleCommand == "CANCEL")
        currentEvent = EVENT_TRAINING_CANCELLED;
    }else
    {
      currentEvent = EVENT_CONTINUE;
    } 
  }
}

void checkSummaryBluetooth() 
{
  if (summarySent) 
  {
    // unsigned long currentTime = millis();
    // if ((currentTime - lctWaitingSummaryConfirmation) < MAX_TIME_WAITTING_CONFIRMATION) 
    // {
    //   if (Serial.available() > 0) 
    //   {
    //     String consoleCommand = Serial.readString();
    //     if (strcmp(consoleCommand.c_str(), "OK") == 0) 
    //     {
    //       currentEvent = EVENT_TRAINING_RESTARTED;
    //     }
    //   }
    // } else 
    // {
    //   currentEvent = EVENT_TRAINING_RESTARTED;
    // }
    currentEvent = EVENT_TRAINING_RESTARTED;
  } else 
  {
    currentEvent = EVENT_CONTINUE;
  }
}

void checkProgress() 
{
  if (summary.timeDone == 0) 
  {
    currentEvent = EVENT_CONTINUE;
    return;
  }

  if (setTraining.setTime != 0) 
  {
    if (summary.timeDone >= (setTraining.setTime)) 
    {
      currentEvent = EVENT_TRAINING_CONCLUDED;
    }
  } else 
  {
    if (summary.metersDone >= setTraining.setMeters) 
    {
      currentEvent = EVENT_TRAINING_CONCLUDED;
    }
  }
}

void checkVolumeSensor() 
{
  if (summary.timeDone == 0) 
  {
    currentEvent = EVENT_CONTINUE;
    return;
  }

  int value = analogRead(VOLUME_SENSOR_PIN);
  int currentVolumeValue = map(value, MIN_POT_VALUE, MAX_POT_VALUE, MIN_VOLUME, MAX_VOLUME);
  Serial.print("vol read:" );
  Serial.println(currentVolumeValue);

  if (currentVolumeValue != lastVolumeValue) 
  {
    currentEvent = EVENT_VOLUME_CHANGE;
    lastVolumeValue = currentVolumeValue;
    
  } else 
  {
    currentEvent = EVENT_CONTINUE;
  }
}

void (*check_sensor[NUMBER_OF_SENSORS])() = 
{
  checkSpeedSensor,
  checkCancelButtonSensor,
  checkTrainingButtonSensor,
  checkPlayStopButtonSensor,
  checkMediaButtonSensor,
  checkTrainingBluetoothInterface,
  checkSummaryBluetooth,
  checkProgress,
  checkVolumeSensor
};

void get_event() {
  unsigned long currentTime = millis();
  if ((currentTime - previousTime) > MAX_SENSOR_LOOP_TIME) 
  {
    check_sensor[index]();
    index = ++index % NUMBER_OF_SENSORS;
    previousTime = currentTime;
  } else 
  {
    currentEvent = EVENT_CONTINUE;
  }
}
