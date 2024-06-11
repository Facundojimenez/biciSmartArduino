#include "definitions.h"
#include "Arduino.h"

rgb_lcd lcd;

event_t currentEvent;
state_t currentState;

String currentLcd;

unsigned long CTPedalling;
unsigned long LCTPedalling;
float pedallingPeriodMs;
float speed_MS;
float speed_KMH;
int index;
bool bikeStopped;

unsigned long currentTime;
unsigned long previousTime;

unsigned long currentTimeLcd;
unsigned long previousTimeLcd;

bool rang25 = false;
bool rang50 = false;
bool rang75 = false;
bool rang100 = false;

int lastVolumeValue;



void do_init()
{
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

  lcd.begin(LED_NUMBER_OF_COLUMNS, LED_NUMBER_OF_ROWS);

  lcd.setRGB(RGB_HIGH, RGB_HIGH, RGB_LOW);

  initBT();
  Serial.begin(SERIAL_SPEED);

  currentState = STATE_WAITING_FOR_TRAINING;
  currentEvent = EVENT_CONTINUE;
  currentLcd = "";

  previousTime = millis();
}




void setup()
{
  do_init();
}

void loop()
{
  state_machine();
}




//////////// IMPLEMENTACION FUNCIONES STATE MACHINE
