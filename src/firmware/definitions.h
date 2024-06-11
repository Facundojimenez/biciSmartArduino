#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <Wire.h>
#include "rgb_lcd.h"
#include <SoftwareSerial.h>

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

#define LOW_SPEED 4
#define HIGH_SPEED 8

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

#define UPPER_SENSOR_HALL_THRESHOLD 600
#define LOWER_SENSOR_HALL_THRESHOLD 400
#define BIKE_WHEEL_CIRCUNFERENCE_PERIMETER_MM 2200

#define BIKE_IS_STOPPED_TIME 2000

#define BLUETOOTH_SPEED 38400
#define LED_NUMBER_OF_ROWS 2
#define LED_NUMBER_OF_COLUMNS 16

#define MS_TO_KM_H_CONVERSION_CONSTANT 3.6
#define LED_UPDATE_SPEED_TIME 500

enum state_t
{
    STATE_WAITING_FOR_TRAINING,
    STATE_READY_FOR_TRAINING,
    STATE_TRAINING_IN_PROGRESS,
    STATE_PAUSED_TRAINING,
    STATE_TRAINING_FINISHED
};

enum event_t
{
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

enum intensity_t
{
    NOINTENSITY,
    LOWINTENSITY,
    MIDINTENSITY,
    HIGHINTENSITY
};

struct tTraining
{
    unsigned int setTime;
    unsigned int setMeters;
    bool dynamicMusic;
};

struct tSummary
{
    float timeDone;
    float metersDone;
    float averageSpeed;
};

void showSpeed();
void showTrainingState(const char *event);
void turnOnIntensityLed();
void ledLowSpeed();
void ledNormalSpeed();
void ledHighSpeed();
void offLed();
void sendMusicComand(const char *comand);
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
void trainingFinished(const char *mensaje);
void startTraining();

void checkSpeedSensor();
void checkCancelButtonSensor();
void checkTrainingButtonSensor();
void checkMediaButtonSensor();
void checkPlayStopButtonSensor();
void checkTrainingBluetoothInterface();
void checkSummaryBluetooth();
void checkProgress();
void checkVolumeSensor();

void get_event();

void initBT();

void ledOn();

void state_machine();

extern rgb_lcd lcd;
extern SoftwareSerial BT;
extern event_t currentEvent;
extern state_t currentState;

extern String currentLcd;

extern intensity_t previousIntensity;

extern unsigned long CTPedalling;
extern unsigned long LCTPedalling;
extern float pedallingPeriodMs;
extern float speed_MS;
extern float speed_KMH;
extern int index;
extern bool bikeStopped;
extern unsigned long lctMetersCalculated;

extern unsigned long currentTime;
extern unsigned long previousTime;

extern unsigned long currentTimeLcd;
extern unsigned long previousTimeLcd;

extern tTraining setTraining;
extern unsigned long lastTimeCalculatedTime;

extern bool trainingReceived;

extern tSummary summary;

extern bool summarySent;
extern bool lctWaitingSummaryConfirmation;

extern bool rang25;
extern bool rang50;
extern bool rang75;
extern bool rang100;

extern int lastVolumeValue;

extern volatile unsigned long lastActivationTime;
extern volatile unsigned long currentActivationTime;
extern volatile unsigned long newacttime;
extern volatile unsigned long timediff;
extern float speed_MM_MS;

extern bool acabaDePedalear;

#endif // DEFINITIONS_H