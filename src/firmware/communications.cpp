#include <Arduino.h>
#include "definitions.h"

SoftwareSerial BT(BLUETOOTH_RXD, BLUETOOTH_TXD);

void initBT()
{
  BT.begin(BLUETOOTH_SPEED);
}

void sendMusicComand(const char *command)
{
  if (setTraining.enableMusicButtons)
  {
    if(setTraining.dynamicMusic && strcmp(command, "NEXT") == 0)
      return;

    Serial.print("Enviando Comando: ");
    Serial.println(command);

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



  char bufferMensaje[100];
  
  //Se utilizan cadenas de caracteres auxiliares para poder printear los numeros decimales, porque sprintf() no soporta flotantes en Arduino 
  char time_str[6];
  char meters_str[6];
  char avg_speed_str[6];

	dtostrf(summary.timeDone, 2, 2, time_str);
  dtostrf(summary.metersDone, 2, 2, meters_str);
  dtostrf(summary.averageSpeed, 2, 2, avg_speed_str);
  
  sprintf(bufferMensaje, "ENDED|Tiempo: %s|Metros Recorridos: %s|Velocidad Media: %s", time_str, meters_str, avg_speed_str);
  BT.println(bufferMensaje);

  // BT.println("ENDED");
  // BT.print("Tiempo: ");
  // BT.print((summary.timeDone));
  // BT.print("\nMetros Recorridos: ");
  // BT.print(summary.metersDone);
  // BT.print("\nVelocidad Media: ");
  // BT.println(summary.averageSpeed);
}

void sendTrainningState(const char* comand)
{
  if(setTraining.personalizedTraining)
    BT.println(comand);
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
