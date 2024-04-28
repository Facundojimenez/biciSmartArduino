/*
  LINK AL TINKERCAD: https://www.tinkercad.com/things/4jSW9r8r8Eu-fantastic-crift/editel?sharecode=Mbjg4XzH3n2pD3k0aG0lXuejTw5PJrB85NU4hLtyJUw
*/

// -------------- IMPORTACIÓN DE LIBRERIAS  --------------

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// -------------- DECLARACIÓN DE DIRECTIVAS DE PRECOMPILACIÓN --------------

// NOMBRES DE LOS PINES DE LOS SENSORES
#define NUMBER_OF_SENSORS 9 // 6 + 2(BLUETOOTH) + 1 (Progreso) + 1 (pausarAutomaticamente)
#define VOLUME_SENSOR_PIN A2
#define PLAY_STOP_MEDIA_SENSOR_PIN 8
#define MEDIA_MOVEMENT_SENSOR_PIN 7
#define HALL_SENSOR_PIN A3
#define TRAINING_CONTROL_PIN 2
#define TRAINING_CANCEL_PIN 4

// NOMBRES DE LOS PINES DE LOS ACTUADORES
#define RED_LED_PIN 11
#define GREEN_LED_PIN 6
#define BLUE_LED_PIN 10
#define BUZZER_PIN 3

// CONSTANTES DE VELOCIDAD
#define MAX_PERIOD_VALUE 1150
#define MIN_PERIOD_VALUE 250
#define MAXIMUM_PERIOD_THRESHOLD 950

// UMBRALES DE VELOCIDAD
#define LOW_SPEED 7
#define HIGH_SPEED 20

// CALCULO DE PEDALEADAS Y DISTANCIA RECORRIDA
#define SERIAL_SPEED 9600
#define ONE_SEC 1000 // 1000 MILIS = 1SEG
#define COMMON_WHEEL_CIRCUNFERENCE 2.1
#define MS_TO_KMH 3.6

// CONSTANTES DE ENTRENAMIENTO POR DEFAULT
#define DEFAULTTIME 5
#define DEFAULTMETERS 0
#define DEFAULTDINAMICMUSIC 1
#define MAX_TIME_WAITTING_TRAINING 3000 // 3SEG

// LECTURA DE SENSORES
#define MAX_SENSOR_LOOP_TIME 50 // 50 MILISEGUNDOS

// TIMEOUT DE CONFIRMACION DE ENVIO DE RESUMEN
#define MAX_TIME_WAITTING_CONFIRMATION 3000 // 3SEG

// -------------- DEFINICION DE ENUMS Y TIPOS DE DATOS --------------
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
  EVENT_PAUSE_START_MEDIA_BUTTON,
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
  unsigned int setTime;   // SEGUNDOS
  unsigned int setMeters; // METROS
  bool dynamicMusic;
};

struct tSummary
{
  float timeDone;
  float metersDone;
  float averageSpeed;
};

// -------------- DECLARACIÓN E INICIALIZACIÓN DE VARIABLES GLOBALES --------------

// EVENTOS Y ESTADOS GLOBALES DEL SISTEMA
event_t currentEvent;
state_t currentState;

// INICIALIZACION DE LED DISPLAY
LiquidCrystal_I2C lcd(0x20, 16, 2);

// INTENSIDAD DE ENTRENAMIENTO
intensity_t previousIntensity = NOINTENSITY;

// CONTROLAR VELOCIDAD
unsigned long CTPedalling;
unsigned long LCTPedalling;
float pedallingPeriodMs;
float speed_MS;
float speedKm;
int index;
bool bikeStopped;
unsigned long lctMetersCalculated;

// TIMEOUT PARA LEER SENSORES
unsigned long currentTime;
unsigned long previousTime;

// ENTRENAMIENTO
tTraining setTrainning;
unsigned long lastTimeCalculatedTime;

// TIMEOUT ESPERANDO ENTRENAMIENTO
bool trainingReceived = false;

// RESUMEN
tSummary summary = {0, 0, 0};

// TIMEOUT ESPERANDO CONFIRMACION
bool summarySent = false;
bool lctWaitingSummaryConfirmation;

// flags buzzer
bool rang25 = false;
bool rang50 = false;
bool rang75 = false;
bool rang100 = false;

//MANEJO DE VOLUMEN
int lastVolumeValue;

// CONSTANTES DE DEBUG DE EVENTOS Y ESTADOS (A ELIMINAR EN LA ENTREGA FINAL)
String arrStates[5] = {"STATE_WAITING", "STATE_READY", "STATE_TRAINING", "STATE_PAUSED", "STATE_FINISHED"};
String arrEvents[10] = {"EVENT_TRAINING_RECEIVED", "EVENT_TRAINING_BUTTON", "EVENT_TRAINING_CANCELLED", "EVENT_PAUSE_START_MEDIA_BUTTON", "EVENT_NEXT_MEDIA_BUTTON",
                        "EVENT_CONCLUDED", "EVENT_RESTARTED", "EVENT_CONTINUE", "EVENT_MONITORING", "EVENT_VOLUME_CHANGE"};

// -------------- DECLARACIÓN DE PROTOTIPOS DE FUNCIONES DE SENSORES Y ACTUADORES --------------
void showSpeed();
void showTrainignState(char *event);
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

void printEvent(int eventIndex)
{
  Serial.print("Current Event: ");
  Serial.println(arrEvents[eventIndex]);
}

void printState(int stateIndex)
{
  Serial.print("Current State: ");
  Serial.println(arrStates[stateIndex]);
}

void ledOn()
{
  analogWrite(BLUE_LED_PIN, 255);
  analogWrite(GREEN_LED_PIN, 0);
  analogWrite(RED_LED_PIN, 255);
}

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

  lcd.init();
  lcd.backlight();

  Serial.begin(SERIAL_SPEED);

  // Inicializo el primer estado
  currentState = STATE_WAITING_FOR_TRAINING;
  currentEvent = EVENT_CONTINUE;

  // Inicializa el tiempo
  previousTime = millis();
}

void checkSpeedSensor()
{
  bikeStopped = false;
  CTPedalling = millis();

  int valorPot = analogRead(HALL_SENSOR_PIN);
  float frecuency = 0;

  pedallingPeriodMs = map(valorPot, 0, 1023, MAX_PERIOD_VALUE, MIN_PERIOD_VALUE);
  if (pedallingPeriodMs > MAXIMUM_PERIOD_THRESHOLD)
  {
    bikeStopped = true;
    speedKm = 0;
    speedKm = 0;
  }
  else
  {
    // V = W * R
    // W = F * 2 * PI
    // V = F * 2 * PI * R
    // 2 * PI * R = Diametro
    // V = F * D
    // V = F X D
    // F = 1/T
    // pero nuestro T esta en milisegundos...
    // Entonces lo llevamos a segundos!
    // f = 1/T = 1 / (value/1000)
    // f = 1000 / T (regla de la oreja)
    frecuency = ONE_SEC / pedallingPeriodMs;
    speed_MS = frecuency * COMMON_WHEEL_CIRCUNFERENCE;
    speedKm = speed_MS * MS_TO_KMH;

    if (((CTPedalling - LCTPedalling) >= pedallingPeriodMs) && !bikeStopped)
    {
      LCTPedalling = CTPedalling;
    }
  }
  currentEvent = EVENT_CONTINUE;
}

void checkMediaButtonSensor()
{
  int buttonState = digitalRead(MEDIA_MOVEMENT_SENSOR_PIN);
  if (buttonState == HIGH)
  {
    currentEvent = EVENT_NEXT_MEDIA_BUTTON;
  }
  else
  {
    currentEvent = EVENT_CONTINUE;
  }
}

void checkPlayStoptButtonSensor()
{
  int buttonState = digitalRead(PLAY_STOP_MEDIA_SENSOR_PIN);
  if (buttonState == HIGH)
  {
    currentEvent = EVENT_PAUSE_START_MEDIA_BUTTON;
  }
  else
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
  }
  else
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
  }
  else
  {
    currentEvent = EVENT_CONTINUE;
  }
}

void checkTrainingBluetoothInterface()
{
  if (!trainingReceived)
  {
    if (Serial.available() > 0)
    {
      // read the incoming byte:
      // reemplazar Seria con el obj bluetooth una vez en la prueba de hardware
      String consoleCommand = Serial.readString();
      int dynamicMusic;
      Serial.print("Comando recibido: "); // TRAINING: 5SEG 0M DIN.MUSIC: 1
                                          // TRAINING: 0SEG 50M DIN.MUSIC: 1
      Serial.println(consoleCommand);
      sscanf(consoleCommand.c_str(), "TRAINING: %dSEG %dM DIN.MUSIC: %d", &(setTrainning.setTime), &(setTrainning.setMeters), &dynamicMusic);
      if ((setTrainning.setMeters != 0 && setTrainning.setTime != 0) || (setTrainning.setMeters == 0 && setTrainning.setTime == 0))
      {
        Serial.print("Entrenamiento Invalido");
        setTrainning.setMeters = 0;
        setTrainning.setTime = 0;
        return;
      }
      if (dynamicMusic)
        setTrainning.dynamicMusic = true;
      else
        setTrainning.dynamicMusic = false;

      Serial.println("Tiempo Segundos:");
      Serial.println(setTrainning.setTime);
      Serial.println("Metros:");
      Serial.println(setTrainning.setMeters);
      Serial.println("Dinamic Music:");
      Serial.println(setTrainning.dynamicMusic);

      currentEvent = EVENT_TRAINING_RECEIVED;
      trainingReceived = true;
    }
  }
  else
  {
    currentEvent = EVENT_CONTINUE;
  }
}

void checkSummaryBluetooth()
{
  if (summarySent)
  {
    long currentTime = millis();
    if ((currentTime - lctWaitingSummaryConfirmation) < MAX_TIME_WAITTING_CONFIRMATION)
    {
      if (Serial.available() > 0)
      {
        String consoleCommand = Serial.readString();
        Serial.print("Comando recibido: ");
        Serial.println(consoleCommand);
        if (strcmp(consoleCommand.c_str(), "OK") == 0)
        {
          currentEvent = EVENT_TRAINING_RESTARTED;
        }
      }
    }
    else
    {
      currentEvent = EVENT_TRAINING_RESTARTED;
    }
  }
  else
  {
    currentEvent = EVENT_CONTINUE;
  }
}

void checkProgress()
{
  if (summary.timeDone == 0)
  { // validación para que no se disparen eventos relacionados al monitoreo del entrenamiento cuando todavia no empezó.
    currentEvent = EVENT_CONTINUE;
    return;
  }

  if (setTrainning.setTime != 0) // Si seteo por tiempo
  {
    if (summary.timeDone >= (setTrainning.setTime))
    {
      currentEvent = EVENT_TRAINING_CONCLUDED;
    }
  }
  else // Si seteo por KM
  {
    if (summary.metersDone >= setTrainning.setMeters)
    {
      currentEvent = EVENT_TRAINING_CONCLUDED;
    }
  }
}

void checkVolumeSensor()
{
  if (summary.timeDone == 0)
  { // validación para que no se disparen eventos relacionados al monitoreo del entrenamiento cuando todavia no empezó.
    currentEvent = EVENT_CONTINUE;
    return;
  }

  int value = analogRead(VOLUME_SENSOR_PIN);
  int currentVolumeValue = map(value, 0, 1023, 0, 10);

  if (currentVolumeValue != lastVolumeValue)
  {
    currentEvent = EVENT_VOLUME_CHANGE;
    lastVolumeValue = currentVolumeValue;
  }
  else
  {
    currentEvent = EVENT_CONTINUE;
  }
}

void (*check_sensor[NUMBER_OF_SENSORS])() =
{
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

// Tomar Eventos
void get_event()
{
  // verificar sensores
  currentTime = millis();
  if ((currentTime - previousTime) > MAX_SENSOR_LOOP_TIME)
  {
    check_sensor[index]();
    index = ++index % NUMBER_OF_SENSORS;
    previousTime = currentTime;
  }
  else
  {
    currentEvent = EVENT_CONTINUE;
  }
}

// Maquina de estados
void state_machine()
{
  get_event();
  printState(currentState);
  printEvent(currentEvent);

  switch (currentState)
  {
  case STATE_WAITING_FOR_TRAINING:
    switch (currentEvent)
    {
    case EVENT_TRAINING_RECEIVED:
      showTrainignState("Received");
      currentState = STATE_READY_FOR_TRAINING;
      break;
    case EVENT_CONTINUE:
      showTrainignState("Not Received");
      currentState = STATE_WAITING_FOR_TRAINING;
      break;
    case EVENT_TRAINING_BUTTON:
      // Iniciar entrenamiento predeterminado
      setTrainning.setTime = DEFAULTTIME;
      setTrainning.setMeters = DEFAULTMETERS;
      setTrainning.dynamicMusic = DEFAULTDINAMICMUSIC;
      trainingReceived = true;
      lctMetersCalculated = millis();
      lastTimeCalculatedTime = millis();
      currentState = STATE_TRAINING_IN_PROGRESS;
      break;
    default:
      Serial.println("UNKNOWN_EVENT");
      break;
    }
    break;
  case STATE_READY_FOR_TRAINING:
    switch (currentEvent)
    {
    case EVENT_TRAINING_BUTTON:
      showTrainignState("Started");
      lctMetersCalculated = millis();
      lastTimeCalculatedTime = millis();
      currentState = STATE_TRAINING_IN_PROGRESS;
      break;
    case EVENT_CONTINUE:
      showTrainignState("Waiting to Start");
      currentState = STATE_READY_FOR_TRAINING;
      break;
    default:
      Serial.println("UNKNOWN_EVENT");
      break;
    }
    break;
  case STATE_TRAINING_IN_PROGRESS:
    switch (currentEvent)
    {
    case EVENT_TRAINING_CONCLUDED:
      showTrainignState("Concluided");
      sendSummary();
      lctWaitingSummaryConfirmation = millis();
      summarySent = true;
      currentState = STATE_TRAINING_FINISHED;
      break;
    case EVENT_TRAINING_BUTTON:
      showTrainignState("Paused");
      currentState = STATE_PAUSED_TRAINING;
      break;
    case EVENT_TRAINING_CANCELLED:
      showTrainignState("Cancelled");
      sendSummary();
      lctWaitingSummaryConfirmation = millis();
      summarySent = true;
      currentState = STATE_TRAINING_FINISHED;
      break;
    case EVENT_PAUSE_START_MEDIA_BUTTON:
      sendMusicComand("STOP");
      currentState = STATE_TRAINING_IN_PROGRESS;
      break;
    case EVENT_NEXT_MEDIA_BUTTON:
      sendMusicComand("NEXT");
      currentState = STATE_TRAINING_IN_PROGRESS;
      break;
    case EVENT_CONTINUE:
      updateDistance();
      updateTime();
      showSpeed();
      turnOnIntensityLed();
      turnOnDynamicMusic();
      turnOnBuzzer();
      currentState = STATE_TRAINING_IN_PROGRESS;
      break;
    case EVENT_VOLUME_CHANGE:
      updateVolume();
      break;
    default:
      Serial.println("UNKNOWN_EVENT");
      break;
    }
    break;
  case STATE_PAUSED_TRAINING:
    switch (currentEvent)
    {
    case EVENT_TRAINING_BUTTON:
      showTrainignState("Resumed");
      lastTimeCalculatedTime = millis();
      lctMetersCalculated = millis();
      currentState = STATE_TRAINING_IN_PROGRESS;
      break;
    case EVENT_TRAINING_CANCELLED:
      showTrainignState("Cancelled");
      sendSummary();
      lctWaitingSummaryConfirmation = millis();
      summarySent = true;
      currentState = STATE_TRAINING_FINISHED;
      break;
    case EVENT_CONTINUE:
      offLed();
      currentState = STATE_PAUSED_TRAINING;
      break;
    default:
      Serial.println("UNKNOWN_EVENT");
      break;
    }
    break;
  case STATE_TRAINING_FINISHED:
    switch (currentEvent)
    {
    case EVENT_TRAINING_RESTARTED:
      showTrainignState("Restarting");
      lctWaitingSummaryConfirmation = 0;
      previousIntensity = NOINTENSITY;
      trainingReceived = false;
      summarySent = false;
      setTrainning.setMeters = 0;
      setTrainning.setTime = 0;
      summary.averageSpeed = 0;
      summary.metersDone = 0;
      summary.timeDone = 0;
      rang25 = false;
      rang50 = false;
      rang75 = false;
      rang100 = false;
      currentState = STATE_WAITING_FOR_TRAINING;
      break;
    case EVENT_CONTINUE:
      currentState = STATE_TRAINING_FINISHED;
      break;
    default:
      Serial.println("UNKNOWN_EVENT");
      break;
    }
    break;
  default:
    Serial.println("UNKNOWN_STATE");
    break;
  }
}
// Sistema Embebido
void setup()
{
  do_init();
}

void loop()
{
  state_machine();
}

// Funciones Actuadores
void showSpeed()
{
  lcd.clear();
  lcd.setCursor(0, 0);

  lcd.print("Tiempo:");
  lcd.setCursor(11, 0);
  lcd.print(summary.timeDone);

  lcd.setCursor(0, 1);
  lcd.print("speed(M/S)");
  lcd.setCursor(11, 1);
  lcd.print((int)speed_MS);
}

void showTrainignState(char *event)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Training");
  lcd.setCursor(0, 1);
  lcd.print(event);
}

void turnOnIntensityLed()
{
  // Revisar la actualizacion de la intensidad
  if (speedKm <= LOW_SPEED)
  {
    ledLowSpeed();
  }
  else if (speedKm < HIGH_SPEED)
  {
    ledNormalSpeed();
  }
  else
  {
    ledHighSpeed();
  }
}

void ledLowSpeed()
{
  analogWrite(BLUE_LED_PIN, 255);
  analogWrite(GREEN_LED_PIN, 0);
  analogWrite(RED_LED_PIN, 0);
}

void ledNormalSpeed()
{
  analogWrite(BLUE_LED_PIN, 0);
  analogWrite(GREEN_LED_PIN, 255);
  analogWrite(RED_LED_PIN, 0);
}

void ledHighSpeed()
{
  analogWrite(BLUE_LED_PIN, 0);
  analogWrite(GREEN_LED_PIN, 0);
  analogWrite(RED_LED_PIN, 255);
}

void offLed()
{
  analogWrite(BLUE_LED_PIN, 0);
  analogWrite(GREEN_LED_PIN, 0);
  analogWrite(RED_LED_PIN, 0);
}

void sendMusicComand(char *comand)
{
  if (!setTrainning.dynamicMusic)
  {
    Serial.print("Enviando Comando: ");
    Serial.println(comand);
  }
}

void turnOnBuzzer()
{
  float percent;
  if (setTrainning.setTime != 0)
  {
    percent = (summary.timeDone * 100 / (float)(setTrainning.setTime));
  }
  else
  {
    percent = (summary.metersDone * 100 / (float)setTrainning.setMeters);
  }

  Serial.println("Porcentaje");
  Serial.println(percent);

  if (percent >= 25 && !rang25)
  {
    Serial.println("suena 25");
    tone(BUZZER_PIN, 200, 500);
    rang25 = true;
  }
  else if (percent >= 50 && !rang50)
  {
    Serial.println("suena 50");
    tone(BUZZER_PIN, 300, 500);
    rang50 = true;
  }
  else if (percent >= 75 && !rang75)
  {
    Serial.println("suena 75");
    tone(BUZZER_PIN, 400, 500);
    rang75 = true;
  }
  else if (percent >= 100 && !rang100)
  {
    Serial.println("suena 100");
    tone(BUZZER_PIN, 500, 500);
    rang100 = true;
  }
}

void turnOnDynamicMusic()
{
  if (setTrainning.dynamicMusic)
  {
    if (speedKm <= LOW_SPEED)
    {
      if (previousIntensity != LOWINTENSITY)
        Serial.println("Sad Music");
      previousIntensity = LOWINTENSITY;
    }
    else if (speedKm < HIGH_SPEED)
    {
      if (previousIntensity != MIDINTENSITY)
        Serial.println("Neutral Music");
      previousIntensity = MIDINTENSITY;
    }
    else
    {
      if (previousIntensity != HIGHINTENSITY)
        Serial.println("Motivational Music");
      previousIntensity = HIGHINTENSITY;
    }
  }
}

void updateDistance()
{
  unsigned long currentTime = millis();

  // Calculate the time elapsed since the start (in hours)
  float timeElapsedSeconds = ((float)(currentTime - lctMetersCalculated)) / ONE_SEC; // / 3600.0; //Dejarlo a metros por segundos

  // Calculate the distance traveled using the formula
  float distanceIncrement = speed_MS * timeElapsedSeconds;

  // Update the total distance traveled
  summary.metersDone += distanceIncrement;

  // Update the start time for the next calculation
  lctMetersCalculated = currentTime;
}

void sendSummary()
{
  summary.averageSpeed = summary.metersDone / summary.timeDone;
  Serial.println("Tiempo: ");
  Serial.println((summary.timeDone));
  Serial.println("Metros Recorridos: ");
  Serial.println(summary.metersDone);
  Serial.println("Velocidad Media: ");
  Serial.println(summary.averageSpeed);
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
  Serial.println(lastVolumeValue);
}