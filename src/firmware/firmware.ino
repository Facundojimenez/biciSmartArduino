/*
    Hello World.ino
    2013 Copyright (c) Seeed Technology Inc.  All right reserved.

    Author:Loovee
    2013-9-18

    Grove - Serial LCD RGB Backlight demo.

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <Wire.h>
#include "rgb_lcd.h"
#include <SoftwareSerial.h>

rgb_lcd lcd;




#define NUMBER_OF_SENSORS 9
#define VOLUME_SENSOR_PIN A3
#define PLAY_STOP_MEDIA_SENSOR_PIN 8
#define MEDIA_MOVEMENT_SENSOR_PIN 7
#define HALL_SENSOR_PIN A2
#define TRAINING_CONTROL_PIN 2
#define TRAINING_CANCEL_PIN 4

#define RED_LED_PIN 9
#define GREEN_LED_PIN 6
#define BLUE_LED_PIN 5
#define BUZZER_PIN 3

#define BLUETOOTH_TXD 11
#define BLUETOOTH_RXD 10

#define MAX_PERIOD_VALUE 1150
#define MIN_PERIOD_VALUE 250
#define MAXIMUM_PERIOD_THRESHOLD 950

#define LOW_SPEED 7
#define HIGH_SPEED 20

#define SERIAL_SPEED 9600
#define ONE_SEC 1000
#define COMMON_WHEEL_CIRCUNFERENCE 2.1
#define MS_TO_KMH 3.6

#define DEFAULTTIME 5
#define DEFAULTMETERS 0
#define DEFAULTDYNAMICMUSIC 1
#define MAX_TIME_WAITTING_TRAINING 3000

#define MAX_SENSOR_LOOP_TIME 10

#define MAX_TIME_WAITTING_CONFIRMATION 3000

#define LCD_ROWS 2
#define LCD_COLS 16
#define LCD_DIR 0x20
#define ROW_1 1
#define ROW_0 0
#define COLUMN_11 11
#define COLUMN_0 0

#define RGB_HIGH 255
#define RGB_LOW 0

#define MIN_VOLUME 0
#define MAX_VOLUME 10

#define MIN_POT_VALUE 0
#define MAX_POT_VALUE 1023

#define PERCENT_25 25
#define PERCENT_50 50
#define PERCENT_75 75
#define PERCENT_100 100

#define LOW_FRECUENCY 200
#define MID_FRECUENCY 300
#define HIGH_FRECUENCY 500

#define TONE_DURATION 500

SoftwareSerial BT(BLUETOOTH_RXD, BLUETOOTH_TXD);

enum state_t {
  STATE_WAITING_FOR_TRAINING,
  STATE_READY_FOR_TRAINING,
  STATE_TRAINING_IN_PROGRESS,
  STATE_PAUSED_TRAINING,
  STATE_TRAINING_FINISHED
};

enum event_t {
  EVENT_TRAINING_RECEIVED,
  EVENT_TRAINING_BUTTON,
  EVENT_TRAINING_CANCELLED,
  EVENT_PLAY_STOP_MEDIA_BUTTON,
  EVENT_NEXT_MEDIA_BUTTON,
  EVENT_TRAINING_CONCLUDED,
  EVENT_TRAINING_RESTARTED,
  EVENT_CONTINUE,
  EVENT_MONITORING_TRAINING,
  EVENT_VOLUME_CHANGE
};

enum intensity_t {
  NOINTENSITY,
  LOWINTENSITY,
  MIDINTENSITY,
  HIGHINTENSITY
};

struct tTraining {
  unsigned int setTime;
  unsigned int setMeters;
  bool dynamicMusic;
};

struct tSummary {
  float timeDone;
  float metersDone;
  float averageSpeed;
};


event_t currentEvent;
state_t currentState;

//LiquidCrystal_I2C lcd(LCD_DIR, LCD_COLS, LCD_ROWS);
String currentLcd;

intensity_t previousIntensity = NOINTENSITY;

unsigned long CTPedalling;
unsigned long LCTPedalling;
float pedallingPeriodMs;
float speed_MS;
float speed_KMH;
int index;
bool bikeStopped;
unsigned long lctMetersCalculated;

unsigned long currentTime;
unsigned long previousTime;

unsigned long currentTimeLcd;
unsigned long previousTimeLcd;

tTraining setTraining;
unsigned long lastTimeCalculatedTime;

bool trainingReceived = false;

tSummary summary = { 0, 0, 0 };

bool summarySent = false;
bool lctWaitingSummaryConfirmation;

bool rang25 = false;
bool rang50 = false;
bool rang75 = false;
bool rang100 = false;

int lastVolumeValue;

void showSpeed();
void showTrainingState(char *event);
void turnOnIntensityLed();
void ledLowSpeed();
void ledNormalSpeed();
void ledHighSpeed();
void offLed();
void sendMusicComand(char *comand);
void turnOnBuzzer();
void turnOnDynamicMusic();
void sendSummary();
void updateDistance();
void updateTime();
void updateVolume();

void defaultTraining();
void resetTraining();
void resumeTraining();
void updateTrainingState();
void trainingState();
void trainingFinished(char *mensaje);
void startTraining();

void ledOn() {
  analogWrite(BLUE_LED_PIN, RGB_HIGH);
  analogWrite(GREEN_LED_PIN, RGB_LOW);
  analogWrite(RED_LED_PIN, RGB_HIGH);
}

#define BLUETOOTH_SPEED 38400
#define LED_NUMBER_OF_ROWS 2
#define LED_NUMBER_OF_COLUMNS 16

void do_init() {
  pinMode(PLAY_STOP_MEDIA_SENSOR_PIN, INPUT);
  pinMode(MEDIA_MOVEMENT_SENSOR_PIN, INPUT);
  pinMode(TRAINING_CANCEL_PIN, INPUT);
  pinMode(TRAINING_CONTROL_PIN, INPUT);
  pinMode(HALL_SENSOR_PIN, INPUT);

  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  ledOn();


  //lcd.display();
  lcd.begin(LED_NUMBER_OF_COLUMNS, LED_NUMBER_OF_ROWS);

  lcd.setRGB(RGB_HIGH, RGB_HIGH, RGB_LOW);





  BT.begin(BLUETOOTH_SPEED);
  Serial.begin(SERIAL_SPEED);

  currentState = STATE_WAITING_FOR_TRAINING;
  currentEvent = EVENT_CONTINUE;
  currentLcd = "";

  previousTime = millis();
}


volatile unsigned long lastActivationTime = 0;
volatile unsigned long currentActivationTime = 0;
volatile unsigned long newacttime  = 0;
volatile unsigned long timediff = 0;
float speed_MM_MS = 0;

#define UPPER_SENSOR_HALL_THRESHOLD 600
#define LOWER_SENSOR_HALL_THRESHOLD 400
#define BIKE_WHEEL_CIRCUNFERENCE_PERIMETER_MM 2200


// si
// si

// si

// no

// si

// no
//
// lee 5ms positivo toma
// lee 10ms positivo no toma
// lee 15ms negativo toma
// lee 20ms negativo no toma
// lee 25ms positivo toma
// lee 30ms negativo toma
// lee 35ms negativo no toma

// si la lectura anterior es un positivo pero la lectura actual es un positivo -> pasar
// si la lectura anterior es positivo y la lectura actual es negativo -> cambiar lectura anterior a negativo
// si la lectura anterior es negativo y la lectura actual es positivo -> ingresa



#define BIKE_IS_STOPPED_TIME 2000

bool acabaDePedalear = false;
// si
#define MS_TO_KM_H_CONVERSION_CONSTANT  3.6
void checkSpeedSensor() {
  newacttime = millis();
  if (newacttime - lastActivationTime > BIKE_IS_STOPPED_TIME) {
    speed_MS = 0;
    speed_KMH = 0;
    speed_MM_MS = 0;
  }
  int sensorValue = analogRead(HALL_SENSOR_PIN);
  //Serial.println(sensorValue);
  //  if ((sensorValue > UPPER_SENSOR_HALL_THRESHOLD || sensorValue < LOWER_SENSOR_HALL_THRESHOLD)) {
  if (!acabaDePedalear && sensorValue < LOWER_SENSOR_HALL_THRESHOLD) {
    currentActivationTime = millis();
    if (lastActivationTime > 0) {
      timediff = currentActivationTime - lastActivationTime;
      speed_MS = BIKE_WHEEL_CIRCUNFERENCE_PERIMETER_MM / (float)timediff;
      speed_KMH = speed_MM_MS * MS_TO_KM_H_CONVERSION_CONSTANT;
      //speed_MS = speed_MM_MS;

    }
    acabaDePedalear = true;
  }
  else {
    acabaDePedalear = false;
  }
  lastActivationTime = currentActivationTime;
  Serial.println(speed_MS);
}



void checkMediaButtonSensor() {
  int buttonState = digitalRead(MEDIA_MOVEMENT_SENSOR_PIN);
  if (buttonState == HIGH) {
    currentEvent = EVENT_NEXT_MEDIA_BUTTON;
  } else {
    currentEvent = EVENT_CONTINUE;
  }
}

void checkPlayStoptButtonSensor() {
  int buttonState = digitalRead(PLAY_STOP_MEDIA_SENSOR_PIN);
  if (buttonState == HIGH) {
    currentEvent = EVENT_PLAY_STOP_MEDIA_BUTTON;
  } else {
    currentEvent = EVENT_CONTINUE;
  }
}

void checkCancelButtonSensor() {
  int buttonState = digitalRead(TRAINING_CANCEL_PIN);
  if (buttonState == HIGH) {
    currentEvent = EVENT_TRAINING_CANCELLED;
  } else {
    currentEvent = EVENT_CONTINUE;
  }
}

void checkTrainingButtonSensor() {
  int buttonState = digitalRead(TRAINING_CONTROL_PIN);

  if (buttonState == HIGH) {
    currentEvent = EVENT_TRAINING_BUTTON;
  } else {
    currentEvent = EVENT_CONTINUE;
  }
}

void checkTrainingBluetoothInterface() {
  if (!trainingReceived) {
    if (BT.available() > 0) {
      String consoleCommand = BT.readString();
      int dynamicMusic;
      sscanf(consoleCommand.c_str(), "%d %d %d", &(setTraining.setTime), &(setTraining.setMeters), &dynamicMusic);
      if ((setTraining.setMeters != 0 && setTraining.setTime != 0) || (setTraining.setMeters == 0 && setTraining.setTime == 0)) {
        Serial.println("Entrenamiento Invalido");
        setTraining.setMeters = 0;
        setTraining.setTime = 0;
        return;
      }
      if (dynamicMusic)
        setTraining.dynamicMusic = true;
      else
        setTraining.dynamicMusic = false;

      currentEvent = EVENT_TRAINING_RECEIVED;
      trainingReceived = true;
    }
  } else {
    currentEvent = EVENT_CONTINUE;
  }
}

void checkSummaryBluetooth() {
  if (summarySent) {
    long currentTime = millis();
    if ((currentTime - lctWaitingSummaryConfirmation) < MAX_TIME_WAITTING_CONFIRMATION) {
      if (Serial.available() > 0) {
        String consoleCommand = Serial.readString();
        if (strcmp(consoleCommand.c_str(), "OK") == 0) {
          currentEvent = EVENT_TRAINING_RESTARTED;
        }
      }
    } else {
      currentEvent = EVENT_TRAINING_RESTARTED;
    }
  } else {
    currentEvent = EVENT_CONTINUE;
  }
}

void checkProgress() {
  if (summary.timeDone == 0) {
    currentEvent = EVENT_CONTINUE;
    return;
  }

  if (setTraining.setTime != 0) {
    if (summary.timeDone >= (setTraining.setTime)) {
      currentEvent = EVENT_TRAINING_CONCLUDED;
    }
  } else {
    if (summary.metersDone >= setTraining.setMeters) {
      currentEvent = EVENT_TRAINING_CONCLUDED;
    }
  }
}

void checkVolumeSensor() {
  if (summary.timeDone == 0 || setTraining.dynamicMusic) {
    currentEvent = EVENT_CONTINUE;
    return;
  }

  int value = analogRead(VOLUME_SENSOR_PIN);
  int currentVolumeValue = map(value, MIN_POT_VALUE, MAX_POT_VALUE, MIN_VOLUME, MAX_VOLUME);

  if (currentVolumeValue != lastVolumeValue) {
    currentEvent = EVENT_VOLUME_CHANGE;
    lastVolumeValue = currentVolumeValue;
  } else {
    currentEvent = EVENT_CONTINUE;
  }
}

void (*check_sensor[NUMBER_OF_SENSORS])() = {
  checkSpeedSensor,
  checkCancelButtonSensor,
  checkTrainingButtonSensor,
  checkPlayStoptButtonSensor,
  checkMediaButtonSensor,
  checkTrainingBluetoothInterface,
  checkSummaryBluetooth,
  checkProgress,
  checkVolumeSensor
};

void get_event() {
  currentTime = millis();
  if ((currentTime - previousTime) > MAX_SENSOR_LOOP_TIME) {
    check_sensor[index]();
    index = ++index % NUMBER_OF_SENSORS;
    previousTime = currentTime;
  } else {
    currentEvent = EVENT_CONTINUE;
  }
}

void state_machine() {
  get_event();

  switch (currentState) {
    case STATE_WAITING_FOR_TRAINING:
      switch (currentEvent) {
        case EVENT_TRAINING_RECEIVED:
          showTrainingState("Received");
          currentState = STATE_READY_FOR_TRAINING;
          break;
        case EVENT_CONTINUE:
          showTrainingState("Not Received");
          currentState = STATE_WAITING_FOR_TRAINING;
          break;
        case EVENT_TRAINING_BUTTON:
          defaultTraining();
          currentState = STATE_TRAINING_IN_PROGRESS;
          break;
        default:
          break;
      }
      break;
    case STATE_READY_FOR_TRAINING:
      switch (currentEvent) {
        case EVENT_TRAINING_BUTTON:
          startTraining();
          currentState = STATE_TRAINING_IN_PROGRESS;
          break;
        case EVENT_CONTINUE:
          showTrainingState("Waiting to Start");
          currentState = STATE_READY_FOR_TRAINING;
          break;
        default:
          break;
      }
      break;
    case STATE_TRAINING_IN_PROGRESS:
      switch (currentEvent) {
        case EVENT_TRAINING_CONCLUDED:
          trainingFinished("Concluded");
          currentState = STATE_TRAINING_FINISHED;
          break;
        case EVENT_TRAINING_BUTTON:
          showTrainingState("Paused");
          currentState = STATE_PAUSED_TRAINING;
          break;
        case EVENT_TRAINING_CANCELLED:
          trainingFinished("Cancelled");
          currentState = STATE_TRAINING_FINISHED;
          break;
        case EVENT_PLAY_STOP_MEDIA_BUTTON:
          sendMusicComand("PLAY/STOP");
          currentState = STATE_TRAINING_IN_PROGRESS;
          break;
        case EVENT_NEXT_MEDIA_BUTTON:
          sendMusicComand("NEXT");
          currentState = STATE_TRAINING_IN_PROGRESS;
          break;
        case EVENT_CONTINUE:
          updateTrainingState();
          currentState = STATE_TRAINING_IN_PROGRESS;
          break;
        case EVENT_VOLUME_CHANGE:
          updateVolume();
          break;
        default:
          break;
      }
      break;
    case STATE_PAUSED_TRAINING:
      switch (currentEvent) {
        case EVENT_TRAINING_BUTTON:
          resumeTraining();
          currentState = STATE_TRAINING_IN_PROGRESS;
          break;
        case EVENT_TRAINING_CANCELLED:
          trainingFinished("Cancelled");
          currentState = STATE_TRAINING_FINISHED;
          break;
        case EVENT_CONTINUE:
          offLed();
          currentState = STATE_PAUSED_TRAINING;
          break;
        default:
          break;
      }
      break;
    case STATE_TRAINING_FINISHED:
      switch (currentEvent) {
        case EVENT_TRAINING_RESTARTED:
          resetTraining();
          currentState = STATE_WAITING_FOR_TRAINING;
          break;
        case EVENT_CONTINUE:
          currentState = STATE_TRAINING_FINISHED;
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}

void setup() {
  do_init();
}

void loop() {
  state_machine();
}
#define LED_UPDATE_SPEED_TIME 500
void showSpeed() {
  currentTimeLcd = millis();
  if ((currentTimeLcd - previousTimeLcd) > LED_UPDATE_SPEED_TIME) {
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

void showTrainingState(char *event) {
  if (strcmp(event, currentLcd.c_str()) != 0) {
    lcd.clear();
    lcd.setCursor(COLUMN_0, ROW_0);
    lcd.print("Training");
    lcd.setCursor(COLUMN_0, ROW_1);
    lcd.print(event);
    currentLcd = event;
  }
}

void turnOnIntensityLed() {
  if (speed_KMH <= LOW_SPEED) {
    ledLowSpeed();
  } else if (speed_KMH < HIGH_SPEED) {
    ledNormalSpeed();
  } else {
    ledHighSpeed();
  }
}

void ledLowSpeed() {
  analogWrite(BLUE_LED_PIN, RGB_HIGH);
  analogWrite(GREEN_LED_PIN, RGB_LOW);
  analogWrite(RED_LED_PIN, RGB_LOW);
  lcd.setRGB(RGB_LOW, RGB_LOW, RGB_HIGH);
}

void ledNormalSpeed() {
  analogWrite(BLUE_LED_PIN, RGB_LOW);
  analogWrite(GREEN_LED_PIN, RGB_HIGH);
  analogWrite(RED_LED_PIN, RGB_LOW);
  lcd.setRGB(RGB_LOW, RGB_HIGH, RGB_LOW);
}

void ledHighSpeed() {
  analogWrite(BLUE_LED_PIN, RGB_LOW);
  analogWrite(GREEN_LED_PIN, RGB_LOW);
  analogWrite(RED_LED_PIN, RGB_HIGH);
  lcd.setRGB(RGB_HIGH, RGB_LOW, RGB_LOW);
}

void offLed() {
  analogWrite(BLUE_LED_PIN, RGB_LOW);
  analogWrite(GREEN_LED_PIN, RGB_LOW);
  analogWrite(RED_LED_PIN, RGB_LOW);
}

void sendMusicComand(char *command) {
  if (!setTraining.dynamicMusic) {
    Serial.print("Enviando Comando: ");
    Serial.println(command);

    BT.print("Enviando Comando: ");
    BT.println(command);
  }
}

void turnOnBuzzer() {
  float percent;
  if (setTraining.setTime != 0) {
    percent = (summary.timeDone * PERCENT_100 / (float)(setTraining.setTime));
  } else {
    percent = (summary.metersDone * PERCENT_100 / (float)setTraining.setMeters);
  }

  if (percent >= PERCENT_25 && !rang25) {
    tone(BUZZER_PIN, LOW_FRECUENCY, TONE_DURATION);
    rang25 = true;
  } else if (percent >= PERCENT_50 && !rang50) {
    tone(BUZZER_PIN, MID_FRECUENCY, TONE_DURATION);
    rang50 = true;
  } else if (percent >= PERCENT_75 && !rang75) {
    tone(BUZZER_PIN, MID_FRECUENCY, TONE_DURATION);
    rang75 = true;
  } else if (percent >= PERCENT_100 && !rang100) {
    tone(BUZZER_PIN, HIGH_FRECUENCY, TONE_DURATION);
    rang100 = true;
  }
}

void turnOnDynamicMusic() {
  if (setTraining.dynamicMusic) {
    if (speed_KMH <= LOW_SPEED) {
      if (previousIntensity != LOWINTENSITY)
        Serial.println("Sad Music");
      previousIntensity = LOWINTENSITY;
    } else if (speed_KMH < HIGH_SPEED) {
      if (previousIntensity != MIDINTENSITY)
        Serial.println("Neutral Music");
      previousIntensity = MIDINTENSITY;
    } else {
      if (previousIntensity != HIGHINTENSITY)
        Serial.println("Motivational Music");
      previousIntensity = HIGHINTENSITY;
    }

    BT.print("ESTADO ACTUAL MUSICA DINAMICA: ");
    BT.println(previousIntensity);
  }
}

void updateDistance() {
  unsigned long currentTime = millis();
  float timeElapsedSeconds = ((float)(currentTime - lctMetersCalculated)) / ONE_SEC;
  float distanceIncrement = speed_MS * timeElapsedSeconds;

  summary.metersDone += distanceIncrement;

  lctMetersCalculated = currentTime;
}

void sendSummary() {
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

void updateTime() {
  long currentTime = millis();
  float timePassed = ((float)(currentTime - lastTimeCalculatedTime)) / ONE_SEC;
  summary.timeDone += timePassed;
  lastTimeCalculatedTime = currentTime;
}

void updateVolume() {
  Serial.print("Asignando Volumen a: ");
  Serial.println(lastVolumeValue);
  BT.print("Asignando Volumen a: ");
  BT.println(lastVolumeValue);
}

//////////// IMPLEMENTACION FUNCIONES STATE MACHINE

void defaultTraining() {
  setTraining.setTime = DEFAULTTIME;
  setTraining.setMeters = DEFAULTMETERS;
  setTraining.dynamicMusic = DEFAULTDYNAMICMUSIC;
  trainingReceived = true;
  lctMetersCalculated = millis();
  lastTimeCalculatedTime = millis();
}

void resetTraining() {
  showTrainingState("Restarting");
  lctWaitingSummaryConfirmation = 0;
  previousIntensity = NOINTENSITY;
  trainingReceived = false;
  summarySent = false;

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

void resumeTraining() {
  showTrainingState("Resumed");
  lastTimeCalculatedTime = millis();
  lctMetersCalculated = millis();
  lcd.clear();
}

void updateTrainingState() {
  updateDistance();
  updateTime();
  showSpeed();
  turnOnIntensityLed();
  turnOnDynamicMusic();
  turnOnBuzzer();
}

void trainingFinished(char *mensaje) {
  showTrainingState(mensaje);
  sendSummary();
  lctWaitingSummaryConfirmation = millis();
  summarySent = true;
}

void startTraining() {
  showTrainingState("Started");
  lctMetersCalculated = millis();
  lastTimeCalculatedTime = millis();
  lcd.clear();
}
