#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <Wire.h>
#include "rgb_lcd.h"
#include <SoftwareSerial.h>
//se usa en el array checkSensors
#define NUMBER_OF_SENSORS 9
//pines
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
//intensidad y musica dinamica
#define LOW_SPEED 4
#define HIGH_SPEED 8
//updateTime y updateDistance
#define ONE_SEC 1000
//entrenamiento por default
#define DEFAULTTIME 120
#define DEFAULTMETERS 0
#define DEFAULTDYNAMICMUSIC 0
#define DEFAULTBUZZER 1
#define DEFAULTMUSCIBUTTONS 0
#define DEFAULTTRAININGTYPE 0
//getEvent
#define MAX_SENSOR_LOOP_TIME 10
//enviar resumen
//#define MAX_TIME_WAITTING_CONFIRMATION 3000
//lcd
#define LCD_ROWS 2
#define LCD_COLS 16
#define LCD_DIR 0x20
#define ROW_1 1
#define ROW_0 0
#define COLUMN_11 11
#define COLUMN_0 0
#define LED_NUMBER_OF_ROWS 2
#define LED_NUMBER_OF_COLUMNS 16
#define LED_UPDATE_SPEED_TIME 500
//led rgb
#define RGB_HIGH 255
#define RGB_LOW 0
//volumen y potenciometro
#define MIN_VOLUME 0
#define MAX_VOLUME 10
#define MIN_POT_VALUE 0
#define MAX_POT_VALUE 1023
//buzzer
#define PERCENT_25 25
#define PERCENT_50 50
#define PERCENT_75 75
#define PERCENT_100 100
#define LOW_FRECUENCY 200
#define MID_FRECUENCY 300
#define HIGH_FRECUENCY 500
#define TONE_DURATION 500
//calcular velocidad
#define UPPER_SENSOR_HALL_THRESHOLD 600
#define LOWER_SENSOR_HALL_THRESHOLD 400
#define BIKE_WHEEL_CIRCUNFERENCE_PERIMETER_MM 2200
#define BIKE_IS_STOPPED_TIME 2000
#define MS_TO_KM_H_CONVERSION_CONSTANT 3.6
//begin serial y bt
#define BLUETOOTH_SPEED 38400
#define SERIAL_SPEED 9600

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
    bool enableBuzzer;
    bool enableMusicButtons;
    bool personalizedTraining;
};

struct tSummary
{
    float timeDone;
    float metersDone;
    float averageSpeed;
};
//actuadores
void showSpeed();
void showTrainingState(const char *event);
void turnOnIntensityLed();
void ledLowSpeed();
void ledNormalSpeed();
void ledHighSpeed();
void ledOn();
void offLed();
void sendMusicComand(const char *comand);
void sendSummary();
void sendTrainningState(const char* comand);
void turnOnBuzzer();
void turnOnDynamicMusic();
void updateDistance();
void updateTime();
void updateVolume();
//trainning
void defaultTraining();
void resetTraining();
void resumeTraining();
void updateTrainingState();
void trainingState();
void trainingFinished(const char *mensaje);
void startTraining();
//sensores
void checkSpeedSensor();
void checkCancelButtonSensor();
void checkTrainingButtonSensor();
void checkMediaButtonSensor();
void checkPlayStopButtonSensor();
void checkTrainingBluetoothInterface();
void checkSummaryBluetooth();
void checkProgress();
void checkVolumeSensor();
//arduino
void initBT();
void get_event();
void state_machine();
//lcd
extern rgb_lcd lcd;
extern String currentLcd;
extern unsigned long currentTimeLcd;
extern unsigned long previousTimeLcd;
//bt
extern SoftwareSerial BT;
//enum
extern event_t currentEvent;
extern state_t currentState;
extern intensity_t previousIntensity;
//estructuras
extern tTraining setTraining;
extern tSummary summary;
//getEvent
extern unsigned long previousTime;
extern int index;
//updateTime y updateDistance
extern unsigned long lctMetersCalculated;
extern unsigned long lastTimeCalculatedTime;
//entrenamiento
extern bool trainingReceived;
//resumen
extern bool summarySent;
extern bool lctWaitingSummaryConfirmation;
//buzzer
extern bool rang25;
extern bool rang50;
extern bool rang75;
extern bool rang100;
//volumen
extern int lastVolumeValue;
//calcular velocidad
extern float speed_MS;
extern volatile unsigned long lastActivationTime;
extern volatile unsigned long currentActivationTime;
extern volatile unsigned long newacttime;
extern volatile unsigned long timediff;
extern bool acabaDePedalear;

#endif // DEFINITIONS_H