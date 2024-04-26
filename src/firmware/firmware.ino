/*
  TINKERCAD: https://www.tinkercad.com/things/4jSW9r8r8Eu-fantastic-crift/editel?sharecode=Mbjg4XzH3n2pD3k0aG0lXuejTw5PJrB85NU4hLtyJUw

TO DO:
  *Para que es la cantidad de pedaleadas?
  *Las pedaleadas suben muy poco

  *Agregar el potenciometro
  *El contador de vueltas vuelve a aumentar aun no estando en estado de entrenamiento
  *La velocidad de actualizacion del contador de vueltas posiblemente no refleje la velocidad esperada debido a la espera en el simulador por la necesidad 
  de checkear todos los otros sensores
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Crear el objeto lcd dirección 0x3F y 16 columnas x 2 Filas
LiquidCrystal_I2C lcd(0x20, 16, 2);

//--CONSTANTES CON NOMBRES DE LOS PINES--
// SENSORES
#define NUMBER_OF_SENSORS 10 // 6 + 2(BLUETOOTH) + 1 (Progreso) + 1 (pausarAutomaticamente)

#define VOLUME_SENSOR_PIN A2
#define PLAY_STOP_MEDIA_SENSOR_PIN 8
#define MEDIA_MOVEMENT_SENSOR_PIN 7
#define HALL_SENSOR_PIN A3
#define TRAINING_CONTROL_PIN 2
#define TRAINING_CANCEL_PIN 4

// ACTUADORES
#define RED_LED_PIN 11
#define GREEN_LED_PIN 6
#define BLUE_LED_PIN 10
#define BUZZER_PIN 3

// CONTROLAR VELOCIDAD
//float mDone;
//float metersDone;
unsigned long CTPedalling;
unsigned long LCTPedalling;
//int pedalCounter;
float pedallingPeriodMs;
float speed_MS;
float speedKm;
int index;
#define MAX_PERIOD_VALUE 1150
#define MIN_PERIOD_VALUE 250
//#define MS_TO_SECONDS_RATIO 1000
#define MAXIMUM_PERIOD_THRESHOLD 950
bool bikeStopped;
// float rpm = 0;

unsigned long lctMetersCalculated;


// UMBRALES DE VELOCIDAD
#define LOW_SPEED 7
#define HIGH_SPEED 20
// UMBRALES ADAPTADO A METROS POR SEGUNDO
//#define LOW_SPEED 3
//#define HIGH_SPEED 6


//--CONSTANTES EXTRAS--
#define SERIAL_SPEED 9600
//#define ONE_MINUTE 60000 // 60.000 MILIS = 1MIN
#define ONE_SEC 1000 // 1000 MILIS = 1SEG
//#define CONST_CONV_CM 0.01723
#define COMMON_WHEEL_CIRCUNFERENCE 2.1
#define MS_TO_KMH 3.6

// TIMEOUT PARA LEER SENSORES
//const unsigned long MAX_SENSOR_LOOP_TIME = 50; // 
#define MAX_SENSOR_LOOP_TIME 50 // 50 MILISEGUNDOS
unsigned long currentTime;
unsigned long previousTime;

// ENTRENAMIENTO
struct tTraining
{
  unsigned int settedTime; //SEGUNDOS
  unsigned int settedM; //METROS
  bool dynamicMusic;
};
tTraining settedTrainning;
//unsigned long startTimeTraining;
unsigned long lastTimeCalculatedTime;

// TIMEOUT ESPERANDO ENTRENAMIENTO
#define MAX_TIME_WAITTING_TRAINING 3000 // 3SEG
unsigned long lctWaitingTraining;
bool trainingReceived = false;

// RESUMEN
struct tSummary
{
  float timeDone;
  float mDone;
  float averageSpeed;
  unsigned int cantPed;
};
tSummary summary = {0, 0, 0, 0};

// TIMEOUT ESPERANDO CONFIRMACION 
#define MAX_TIME_WAITTING_CONFIRMATION 3000 // 3SEG
bool summarySent = false;
bool lctWaitingSummaryConfirmation;

// flags buzzer
bool rang25 = false;
bool rang50 = false;
bool rang75 = false;
bool rang100 = false;

// ESTADOS Y EVENTOS
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
};

event_t currentEvent;
state_t currentState;

String arrStates[5] = {"STATE_WAITING", "STATE_READY", "STATE_TRAINING", "STATE_PAUSED", "STATE_FINISHED"};
String arrEvents[9] = {"EVENT_TRAINING_RECEIVED", "EVENT_TRAINING_BUTTON", "EVENT_TRAINING_CANCELLED", "EVENT_PAUSE_START_MEDIA_BUTTON", "EVENT_NEXT_MEDIA_BUTTON",
                        "EVENT_CONCLUDED", "EVENT_RESTARTED", "EVENT_CONTINUE", "EVENT_MONITORING"};

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

// CONFIGURACION
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
  lctWaitingTraining = millis();
}

// Funciones de atención a los sensores
// Ver las modificaciones necesarias para el calculo de velocidad
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
      summary.cantPed++;
    }
  }
  currentEvent = EVENT_CONTINUE;
}

void checkStopMusicWhenLowSpeed()
{
  //if(startTimeTraining == 0)
  if(summary.timeDone == 0)
  { //validación para que no se disparen eventos relacionados al monitoreo del entrenamiento cuando todavia no empezó.
    currentEvent = EVENT_CONTINUE;
    return;
  }

  if (!settedTrainning.dynamicMusic && speed_MS <= LOW_SPEED) // Si esta con su propia musica y va lento, se pausa su musica
    currentEvent = EVENT_PAUSE_START_MEDIA_BUTTON;
}

void checkMediaButtonSensor()
{
  int buttonState = digitalRead(MEDIA_MOVEMENT_SENSOR_PIN);
  if(buttonState == HIGH)
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
    long ctWaitingTraining = millis();
    if ((ctWaitingTraining - lctWaitingTraining) < MAX_TIME_WAITTING_TRAINING)
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
        sscanf(consoleCommand.c_str(), "TRAINING: %dSEG %dM DIN.MUSIC: %d", &(settedTrainning.settedTime), &(settedTrainning.settedM), &dynamicMusic);
        if( (settedTrainning.settedM != 0 && settedTrainning.settedTime !=0) || (settedTrainning.settedM == 0 && settedTrainning.settedTime == 0))
        {
          Serial.print("Entrenamiento Invalido");
          settedTrainning.settedM = 0;
          settedTrainning.settedTime = 0;
          return ;
        }
        if (dynamicMusic)
          settedTrainning.dynamicMusic = true;
        else
          settedTrainning.dynamicMusic = false;

        Serial.println("Tiempo Segundos:");
        Serial.println(settedTrainning.settedTime);
        Serial.println("Metros:");
        Serial.println(settedTrainning.settedM);
        Serial.println("Dinamic Music:");
        Serial.println(settedTrainning.dynamicMusic);
        currentEvent = EVENT_TRAINING_RECEIVED;
        trainingReceived = true;
      }
    }
    else
    {
      settedTrainning.settedTime = 10;
      settedTrainning.settedM = 0;
      settedTrainning.dynamicMusic = true;
      trainingReceived = true;
      currentEvent = EVENT_TRAINING_RECEIVED;
    }
  }
  else
  {
    currentEvent = EVENT_CONTINUE;
  }
}

void checkSummaryBluetooth()
{
  if(summarySent)
  {
    long currentTime = millis();
    if((currentTime - lctWaitingSummaryConfirmation) < MAX_TIME_WAITTING_CONFIRMATION)
    {
      if (Serial.available() > 0)
      {
        String consoleCommand = Serial.readString();
        Serial.print("Comando recibido: ");
        Serial.println(consoleCommand);
        if(strcmp(consoleCommand.c_str(), "OK") == 0)
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

void checkProgress() // Verifica si termino o no, solo si el 
{
  //if(startTimeTraining == 0)
  if(summary.timeDone == 0)
  { //validación para que no se disparen eventos relacionados al monitoreo del entrenamiento cuando todavia no empezó.
    currentEvent = EVENT_CONTINUE;
    return;
  }

  if (settedTrainning.settedTime != 0) // Si seteo por tiempo
  {
    //long currentTime = millis();
    //long trainingTime = (currentTime - startTimeTraining) / ONE_SEC;
    //trainingTime /= ONE_MINUTE Para Minutos
    if (summary.timeDone >= (settedTrainning.settedTime))
    {
      currentEvent = EVENT_TRAINING_CONCLUDED;
    }
  }
  else // Si seteo por KM
  {
    if(summary.mDone >= settedTrainning.settedM)
    {
      currentEvent = EVENT_TRAINING_CONCLUDED;
    }
  }
}

void checkVolumeSensor()
{
  //TODO
}

void (*check_sensor[NUMBER_OF_SENSORS])() =
{
  checkVolumeSensor,
  checkSpeedSensor,
  checkCancelButtonSensor,
  checkTrainingButtonSensor,
  checkPlayStoptButtonSensor,
  checkMediaButtonSensor,
  checkTrainingBluetoothInterface,
  checkSummaryBluetooth,
  checkProgress,
  checkStopMusicWhenLowSpeed
};

// Funciones Actuadores

void showSpeed();
void showTrainignState(char* event);
void turnOnIntensityLed();
void ledLowSpeed();
void ledNormalSpeed();
void ledHighSpeed();
void offLed();
void sendMusicComand(char* comand);
void turnOnBuzzer();
void turnOnDynamicMusic();
void sendSummary();
void updatePedallingCounter();
void updateDistance();
void updateTime();

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
      showTrainignState("Not Received"); // Training not received -> Waiting for trainning
      currentState = STATE_WAITING_FOR_TRAINING;
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
      //startTimeTraining = millis();
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
      trainingReceived = false;
      //currentState = STATE_WAITING_FOR_TRAINING;
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
      trainingReceived = false;
      //currentState = STATE_WAITING_FOR_TRAINING;
      currentState = STATE_TRAINING_FINISHED;
      break;
    // case EVENT_MONITORING_TRAINING:
    //  currentState = STATE_TRAINING_IN_PROGRESS;
    // break;
    case EVENT_PAUSE_START_MEDIA_BUTTON:
      sendMusicComand("STOP");
      currentState = STATE_TRAINING_IN_PROGRESS;
      break;
    case EVENT_NEXT_MEDIA_BUTTON:
      sendMusicComand("NEXT");
      currentState = STATE_TRAINING_IN_PROGRESS;
      break;
    case EVENT_CONTINUE:
      //updatePedallingCounter();
      updateDistance();
      updateTime();
      showSpeed();
      turnOnIntensityLed();
      turnOnDynamicMusic();
      turnOnBuzzer();
      currentState = STATE_TRAINING_IN_PROGRESS;
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
      trainingReceived = false;
      sendSummary();
      lctWaitingSummaryConfirmation = millis();
      summarySent = true;
      //currentState = STATE_WAITING_FOR_TRAINING;
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
      //startTimeTraining = 0;
      lctWaitingSummaryConfirmation = 0;
      lctWaitingTraining = millis();
      //pedalCounter = 0;
      //mDone = 0;
      trainingReceived = false;
      summarySent = false;
      settedTrainning.settedM = 0;
      settedTrainning.settedTime = 0;
      summary.cantPed = 0;
      summary.averageSpeed = 0;
      summary.mDone = 0;
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

  //lcd.print("laps");
  //lcd.setCursor(11, 0);
  ////lcd.print(pedalCounter);
  //lcd.print(summary.cantPed);

  // lcd.print("Metros:");
  // lcd.setCursor(11,0);
  // lcd.print(summary.mDone);

  lcd.print("Tiempo:");
  lcd.setCursor(11,0);
  lcd.print(summary.timeDone);

  lcd.setCursor(0, 1);
  lcd.print("speed(M/S)");
  lcd.setCursor(11, 1);
  lcd.print((int)speed_MS);
}

void showTrainignState(char* event)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Training");
  lcd.setCursor(0, 1);
  lcd.print(event);
}

void turnOnIntensityLed()
{
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

void sendMusicComand(char* comand)
{
  if (!settedTrainning.dynamicMusic)
  {
    Serial.print("Enviando Comando: ");
    Serial.println(comand);
  }
}

void turnOnBuzzer()
{
  //long currentTime = millis();
  //float trainingTime = ((float)(currentTime - startTimeTraining)) / ONE_SEC;
  float percent;
  if (settedTrainning.settedTime != 0)
  {
    // float trainingTimeMin = ((float)trainingTime/ONE_MINUTE) Para Minutos;
    percent = (summary.timeDone * 100 / (float)(settedTrainning.settedTime));
  }
  else
  {
    percent = (summary.mDone * 100 / (float)settedTrainning.settedM);
  }

  Serial.println("Porcentaje");
  Serial.println(percent);
  //Agregar flags por si se pasa y no suena el buzzer
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
  if (settedTrainning.dynamicMusic)
  {
    if (speedKm <= LOW_SPEED)
    {
      Serial.println("Sad Music");
    }
    else if (speedKm < HIGH_SPEED)
    {
      Serial.println("Neutral Music");
    }
    else
    {
      Serial.println("Motivational Music");
    }
  }
}

void updatePedallingCounter()
{
  //pedalCounter += 1;
  summary.cantPed += 1;
}

void updateDistance()
{
  unsigned long currentTime = millis();

  // Calculate the time elapsed since the start (in hours)
  float timeElapsedSeconds = ((float)(currentTime - lctMetersCalculated)) / ONE_SEC; // / 3600.0; //Dejarlo a metros por segundos

  // Calculate the distance traveled using the formula
  float distanceIncrement = speed_MS * timeElapsedSeconds;

  // Update the total distance traveled
  summary.mDone += distanceIncrement;

  // Update the start time for the next calculation
  lctMetersCalculated = currentTime;
}

void sendSummary()
{
  //long currentTime = millis();
  //summary.cantPed = pedal_counter;
  //summary.timeDone = (currentTime - startTimeTraining) / ONE_SEC;
  //summary.mDone = mDone;
  summary.averageSpeed = summary.mDone / summary.timeDone;
  Serial.println("Cantidad Pedaleadas: ");
  Serial.println(summary.cantPed);
  Serial.println("Tiempo: ");
  Serial.println((summary.timeDone));
  Serial.println("Metros Recorridos: ");
  Serial.println(summary.mDone);
  Serial.println("Velocidad Media: ");
  Serial.println(summary.averageSpeed);
}

void updateTime()
{
  long currentTime = millis();
  float timePassed =  ((float)(currentTime - lastTimeCalculatedTime)) / ONE_SEC;
  summary.timeDone += timePassed;
  lastTimeCalculatedTime = currentTime;
}