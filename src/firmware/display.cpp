#include<Arduino.h>
#include"definitions.h"

void showSpeed()
{
  currentTimeLcd = millis();
  if ((currentTimeLcd - previousTimeLcd) > LED_UPDATE_SPEED_TIME)
  {
    lcd.setCursor(COLUMN_0, ROW_0);

    lcd.print("Tiempo:        ");
    lcd.setCursor(COLUMN_11, ROW_0);
    lcd.print(summary.timeDone);

    lcd.setCursor(COLUMN_0, ROW_1);
    lcd.print("speed(M/S)     ");
    lcd.setCursor(COLUMN_11, ROW_1);
    lcd.print(speed_MS);
    // Serial.println(speed_MS);

    previousTimeLcd = currentTimeLcd;
  }
}

void showTrainingState(const char *event)
{
  if (strcmp(event, currentLcd.c_str()) != 0)
  {
    lcd.clear();
    lcd.setCursor(COLUMN_0, ROW_0);
    lcd.print("Training");
    lcd.setCursor(COLUMN_0, ROW_1);
    lcd.print(event);
    currentLcd = event;
  }
}

void turnOnIntensityLed()
{
  if (speed_MS <= lowSpeed)
  {
    ledLowSpeed();
  }
  else if (speed_MS < highSpeed)
  {
    ledNormalSpeed();
  }
  else
  {
    ledHighSpeed();
  }
}

void ledOn()
{
  analogWrite(BLUE_LED_PIN, RGB_HIGH);
  analogWrite(GREEN_LED_PIN, RGB_LOW);
  analogWrite(RED_LED_PIN, RGB_HIGH);
}

void ledLowSpeed()
{
  analogWrite(BLUE_LED_PIN, RGB_HIGH);
  analogWrite(GREEN_LED_PIN, RGB_LOW);
  analogWrite(RED_LED_PIN, RGB_LOW);
  lcd.setRGB(RGB_LOW, RGB_LOW, RGB_HIGH);
}

void ledNormalSpeed()
{
  analogWrite(BLUE_LED_PIN, RGB_LOW);
  analogWrite(GREEN_LED_PIN, RGB_HIGH);
  analogWrite(RED_LED_PIN, RGB_LOW);
  lcd.setRGB(RGB_LOW, RGB_HIGH, RGB_LOW);
}

void ledHighSpeed()
{
  analogWrite(BLUE_LED_PIN, RGB_LOW);
  analogWrite(GREEN_LED_PIN, RGB_LOW);
  analogWrite(RED_LED_PIN, RGB_HIGH);
  lcd.setRGB(RGB_HIGH, RGB_LOW, RGB_LOW);
}

void offLed()
{
  analogWrite(BLUE_LED_PIN, RGB_LOW);
  analogWrite(GREEN_LED_PIN, RGB_LOW);
  analogWrite(RED_LED_PIN, RGB_LOW);
}

void turnOnBuzzer()
{
  if(!setTraining.enableBuzzer)
    return;
    
  float percent;
  if (setTraining.setTime != 0)
  {
    percent = (summary.timeDone * PERCENT_100 / (float)(setTraining.setTime));
  }
  else
  {
    percent = (summary.metersDone * PERCENT_100 / (float)setTraining.setMeters);
  }

  if (percent >= PERCENT_25 && !rang25)
  {
    tone(BUZZER_PIN, LOW_FRECUENCY, TONE_DURATION);
    rang25 = true;
  }
  else if (percent >= PERCENT_50 && !rang50)
  {
    tone(BUZZER_PIN, MID_FRECUENCY, TONE_DURATION);
    rang50 = true;
  }
  else if (percent >= PERCENT_75 && !rang75)
  {
    tone(BUZZER_PIN, MID_FRECUENCY, TONE_DURATION);
    rang75 = true;
  }
  else if (percent >= PERCENT_100 && !rang100)
  {
    tone(BUZZER_PIN, HIGH_FRECUENCY, TONE_DURATION);
    rang100 = true;
  }
}
