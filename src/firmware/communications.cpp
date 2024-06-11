#include <Arduino.h>
#include "definitions.h"

SoftwareSerial BT(BLUETOOTH_RXD, BLUETOOTH_TXD);

void initBT()
{
  BT.begin(BLUETOOTH_SPEED);
}

void sendMusicComand(const char *command)
{
  if (!setTraining.dynamicMusic)
  {
    Serial.print("Enviando Comando: ");
    Serial.println(command);

    BT.print("Enviando Comando: ");
    BT.println(command);
  }
}

void sendSummary()
{
  summary.averageSpeed = summary.metersDone / summary.timeDone;
  Serial.print("Tiempo: ");
  Serial.println((summary.timeDone));
  Serial.print("Metros Recorridos: ");
  Serial.println(summary.metersDone);
  Serial.print("Velocidad Media: ");
  Serial.println(summary.averageSpeed);

  BT.print("Tiempo: ");
  BT.println((summary.timeDone));
  BT.print("Metros Recorridos: ");
  BT.println(summary.metersDone);
  BT.print("Velocidad Media: ");
  BT.println(summary.averageSpeed);
}

void turnOnDynamicMusic()
{
  if (setTraining.dynamicMusic)
  {
    if (speed_MS <= LOW_SPEED)
    {
      if (previousIntensity != LOWINTENSITY)
      {
        Serial.println("Sad Music");
        BT.println("Sad Music");
        previousIntensity = LOWINTENSITY;
      }
    }
    else if (speed_MS < HIGH_SPEED)
    {
      if (previousIntensity != MIDINTENSITY)
      {
        Serial.println("Neutral Music");
        BT.println("Neutral Music");
        previousIntensity = MIDINTENSITY;
      }
    }
    else
    {
      if (previousIntensity != HIGHINTENSITY)
      {
        Serial.println("Motivational Music");

        BT.println("Motivational Music");
        previousIntensity = HIGHINTENSITY;
      }
    }
  }
}
