/*
  TINKERCAD: https://www.tinkercad.com/things/4jSW9r8r8Eu-fantastic-crift/editel?sharecode=Mbjg4XzH3n2pD3k0aG0lXuejTw5PJrB85NU4hLtyJUw
*/

/*
NOTAS DEL PROFE:
- Implementar logica de play/stop de musica bluetooth segun la intensidad.

TO DO:
  *Cuando se pausa, que no cuente para el tiempo en entrenamiento
  *Que empiece a contar cuando tocamos el boton, no cuando prende
  *Activar el buzzer por porcentaje de KM
  *Reseteo de las variables para el proximo entrenamiento
  *Prender RGB cuando prende el sistema
  *Nada genera EVENT_RESTARTED
  *Por como recorremos los sensores, el sistema sigue corriendo aunque haya terminado el entrenamiento porque tienne que recorrer
  las demas funciones
  *No se calcula bien la distancia recorrida, queda un numero muy chico que lo considera 0

  *Modificar el BT para recibir el resumen
  *Modificar los tiempos para que todo quede en m/s y seg
  *Modificar de calculo de velocidad
  *Agregar el potenciometro
  *Actualizar los LCD con menos frecuencia
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Crear el objeto lcd dirección 0x3F y 16 columnas x 2 Filas
LiquidCrystal_I2C lcd(0x20, 16, 2);

//--CONSTANTES CON NOMBRES DE LOS PINES--
// SENSORES
const unsigned int NUMBER_OF_SENSORS = 8; // 5 + 1(BLUETOOTH) + 1 (Progreso) + 1 (pausarAutomaticamente)

const int PLAY_STOP_MEDIA_SENSOR_PIN = 8;
const int MEDIA_MOVEMENT_SENSOR_PIN = 7;
const int HALL_SENSOR_PIN = A3;
const int TRAINING_CONTROL_PIN = 2;
const int TRAINING_CANCEL_PIN = 4;

// ACTUADORES
const int RED_LED_PIN = 11;
const int GREEN_LED_PIN = 6;
const int BLUE_LED_PIN = 10;
const int BUZZER_PIN = 3;

// CONTROLAR VELOCIDAD
unsigned long actual_pedalling_time;
unsigned long previous_pedalling_time;
int pedal_counter = 0;
float pedaling_frecuency_mHz = 0;
float speed_KMH = 0;
int index = 0;
float rpm = 0;

unsigned long lctKmCalculated;
float kmDone;

const unsigned LOW_SPEED = 7;
const unsigned HIGH_SPEED = 20;

//--CONSTANTES EXTRAS--
#define SERIAL_SPEED 9600
#define ONE_MINUTE 60000
const double CONST_CONV_CM = 0.01723;
const double COMMON_WHEEL_CIRCUNFERENCE = 2.1;
const double MS_TO_KMH = 3.6;

// TIMEOUT PARA LEER SENSORES
const unsigned long MAX_SENSOR_LOOP_TIME = 50; // #define TIEMPO_MAX_MILIS 50 // MEDIO SEGUNDO
unsigned long currentTime;
unsigned long previousTime;

// ENTRENAMIENTO
struct tTraining
{
  unsigned int settedTime; // Minutos/Milisegundos?
  unsigned int settedKm;
  bool dynamicMusic;
};
tTraining settedTrainning;
unsigned long startTimeTraining; // medir el tiempo que lleva timePassed - startTimeTraining < settedTime

// TIMEOUT ESPERANDO ENTRENAMIENTO
#define MAX_TIME_WAITTING_TRAINING 1000 // 1min
//unsigned long ctWaitingTraining;
unsigned long lctWaitingTraining;
bool trainingReceived = false;

// RESUMEN
struct tSummary
{
  unsigned int timeDone;
  float kmDone;
  unsigned averageSpeed;
  unsigned int cantPed;
};
tSummary summary = {0, 0, 0, 0};

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
  EVENT_PREVIOUS_MEDIA_BUTTON,
  EVENT_NEXT_MEDIA_BUTTON,
  EVENT_TRAINING_CONCLUDED,
  EVENT_TRAINING_RESTARTED,
  EVENT_CONTINUE,
  EVENT_MONITORING_TRAINING,
};

event_t currentEvent;
state_t currentState;

String arrStates[5] = {"STATE_WAITING", "STATE_READY", "STATE_TRAINING", "STATE_PAUSED", "STATE_FINISHED"};
String arrEvents[9] = {"EVENT_TRAINING_RECEIVED", "EVENT_TRAINING_BUTTON", "EVENT_TRAINING_CANCELLED", "EVENT_PREVIOUS_MEDIA_BUTTON", "EVENT_NEXT_MEDIA_BUTTON",
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
  // TOMO EL TIEMPO ACTUAL
  actual_pedalling_time = millis();
  // VERIFICA CUANTO TRANSCURRIO DE TIEMPO

  int valorPot = analogRead(HALL_SENSOR_PIN);

  // map the value to mHz (since only maps to integer values)
  pedaling_frecuency_mHz = map(valorPot, 0, 1023, 0, 4000);

  int frecuency = 1000000 / pedaling_frecuency_mHz;

  speed_KMH = ((pedaling_frecuency_mHz / 1000) * COMMON_WHEEL_CIRCUNFERENCE) * (MS_TO_KMH);

  // Serial.println(speed_KMH);
  if ((actual_pedalling_time - previous_pedalling_time) >= frecuency && pedaling_frecuency_mHz > 0)
  {
    previous_pedalling_time = actual_pedalling_time;
  }

  currentEvent = EVENT_CONTINUE;
}

void checkStopMusicWhenLowSpeed()
{
  if(currentState != STATE_TRAINING_IN_PROGRESS){ //validación para que no se disparen eventos relacionados al monitoreo del entrenamiento cuando todavia no empezó.
    currentEvent = EVENT_CONTINUE;
    return;
  }

  if (!settedTrainning.dynamicMusic && speed_KMH <= LOW_SPEED) // Si esta con su propia musica y va lento, se pausa su musica
    currentEvent = EVENT_PREVIOUS_MEDIA_BUTTON;
}

void checkMediaButtonSensor()
{
  int buttonState = digitalRead(MEDIA_MOVEMENT_SENSOR_PIN);
  if(buttonState == HIGH)
  {
    currentEvent = EVENT_NEXT_MEDIA_BUTTON;
  }
  else{
    currentEvent = EVENT_CONTINUE;
  }
}

void checkPlayStoptButtonSensor()
{
  int buttonState = digitalRead(PLAY_STOP_MEDIA_SENSOR_PIN);
  if (buttonState == HIGH)
  {
    currentEvent = EVENT_PREVIOUS_MEDIA_BUTTON;
  }
  else{
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
  else{
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
  else{
    currentEvent = EVENT_CONTINUE;
  }
}

void checkBluetoothInterface()
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
        Serial.print("Comando recibido: "); // TRAINING: 3000MIL 1 - TRAINING: 3KM 0
        Serial.println(consoleCommand);
        if (sscanf(consoleCommand.c_str(), "TRAINING: %ldMIL %d", &(settedTrainning.settedTime), &dynamicMusic))
        {
          settedTrainning.settedKm = 0;
        }
        else
        {
          sscanf(consoleCommand.c_str(), "TRAINING: %dKM %d", &(settedTrainning.settedKm), dynamicMusic);
          settedTrainning.settedTime = 0;
        }
        if (dynamicMusic)
          settedTrainning.dynamicMusic = true;
        else
          settedTrainning.dynamicMusic = false;

        Serial.println("Tiempo:");
        Serial.println(settedTrainning.settedTime);
        Serial.println("KM:");
        Serial.println(settedTrainning.settedKm);
        Serial.println("Dinamic Music:");
        Serial.println(settedTrainning.dynamicMusic);
        currentEvent = EVENT_TRAINING_RECEIVED;
        trainingReceived = true;
      }
    }
    else
    {
      settedTrainning.settedTime = 10000;
      settedTrainning.settedKm = 0;
      settedTrainning.dynamicMusic = true;
      trainingReceived = true;
      currentEvent = EVENT_TRAINING_RECEIVED;
    }
  }
  else
  {
    //Cuando se manda el resumen, mandar un recibido que genere EVENT_RESTARTED
    currentEvent = EVENT_CONTINUE;
  }
}

void checkProgress() // Verifica si termino o no, solo si el 
{
  if(currentState != STATE_TRAINING_IN_PROGRESS){ //validación para que no se disparen eventos relacionados al monitoreo del entrenamiento cuando todavia no empezó.
    currentEvent = EVENT_CONTINUE;
    return;
  }

  if (settedTrainning.settedTime != 0) // Si seteo por tiempo
  {
    long currentTime = millis();
    long trainingTime = currentTime - startTimeTraining;
    //trainingTime /= ONE_MINUTE Para Minutos
    if (trainingTime >= settedTrainning.settedTime)
    {
      currentEvent = EVENT_TRAINING_CONCLUDED;
    }
  }
  else // Si seteo por KM
  {
    if(kmDone >= settedTrainning.settedKm)
    {
      currentEvent = EVENT_TRAINING_CONCLUDED;
    }
  }
}

void (*check_sensor[NUMBER_OF_SENSORS])() =
{
  checkSpeedSensor,
  checkCancelButtonSensor,
  checkTrainingButtonSensor,
  checkPlayStoptButtonSensor,
  checkMediaButtonSensor,
  checkBluetoothInterface,
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
      startTimeTraining = millis();
      lctKmCalculated = millis();
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
      trainingReceived = false;
      currentState = STATE_WAITING_FOR_TRAINING;
      //currentState = STATE_TRAINING_FINISHED;
      break;
    case EVENT_TRAINING_BUTTON:
      showTrainignState("Paused");
      currentState = STATE_PAUSED_TRAINING;
      break;
    case EVENT_TRAINING_CANCELLED:
      showTrainignState("Cancelled");
      sendSummary();
      trainingReceived = false;
      currentState = STATE_WAITING_FOR_TRAINING;
      //currentState = STATE_TRAINING_FINISHED;
      break;
    // case EVENT_MONITORING_TRAINING:
    //  currentState = STATE_TRAINING_IN_PROGRESS;
    // break;
    case EVENT_PREVIOUS_MEDIA_BUTTON:
      sendMusicComand("STOP");
      currentState = STATE_TRAINING_IN_PROGRESS;
      break;
    case EVENT_NEXT_MEDIA_BUTTON:
      sendMusicComand("NEXT");
      currentState = STATE_TRAINING_IN_PROGRESS;
      break;
    case EVENT_CONTINUE:
      turnOnIntensityLed();
      turnOnBuzzer();
      turnOnDynamicMusic();
      showSpeed();
      updatePedallingCounter();
      updateDistance();
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
      currentState = STATE_TRAINING_IN_PROGRESS;
      break;
    case EVENT_TRAINING_CANCELLED:
      showTrainignState("Cancelled");
      trainingReceived = false;
      sendSummary();
      currentState = STATE_WAITING_FOR_TRAINING;
      //currentState = STATE_TRAINING_FINISHED;
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
      sendSummary();
      trainingReceived = false;
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
  lcd.print("laps");
  lcd.setCursor(10, 0);
  lcd.print(pedal_counter);
  lcd.setCursor(0, 1);
  lcd.print("speed_KMH");
  lcd.setCursor(10, 1);
  lcd.print((int)speed_KMH);
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
  if (speed_KMH <= LOW_SPEED)
  {
    ledLowSpeed();
  }
  else if (speed_KMH < HIGH_SPEED)
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
  long currentTime = millis();
  long trainingTime = currentTime - startTimeTraining;
  float percent;
  if (settedTrainning.settedTime != 0)
  {
    // float trainingTimeMin = ((float)trainingTime/ONE_MINUTE) Para Minutos;
    percent = (trainingTime * 100 / (float)settedTrainning.settedTime);
  }
  else
  {
    percent = (kmDone * 100 / (float)settedTrainning.settedKm);
  }

  Serial.println("Porcentaje");
  Serial.println(percent);
  //Agregar flags por si se pasa y no suena el buzzer
  if (percent >= 25 && percent <= 26)
  {
    Serial.println("suena 25");
    tone(BUZZER_PIN, 200, 500);
  }
  else if (percent >= 50 && percent <= 51)
  {
    Serial.println("suena 50");
    tone(BUZZER_PIN, 300, 500);
  }
  else if (percent >= 75 && percent <= 76)
  {
    Serial.println("suena 75");
    tone(BUZZER_PIN, 400, 500);
  }
  else if (percent >= 99 && percent <= 101)
  {
    Serial.println("suena 100");
    tone(BUZZER_PIN, 500, 500);
  }
}


void turnOnDynamicMusic()
{
  if (settedTrainning.dynamicMusic)
  {
    if (speed_KMH <= LOW_SPEED)
    {
      Serial.println("Sad Music");
    }
    else if (speed_KMH < HIGH_SPEED)
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
  pedal_counter += 1;
}

void updateDistance()
{
  unsigned long currentTime = millis();

  // Calculate the time elapsed since the start (in hours)
  float timeElapsedHours = ((float)(currentTime - lctKmCalculated)) / 1000.0; // / 3600.0; //Dejarlo a metros por segundos

  // Calculate the distance traveled using the formula
  float distanceIncrement = speed_KMH * timeElapsedHours;

  // Update the total distance traveled
  kmDone += distanceIncrement;

  // Update the start time for the next calculation
  lctKmCalculated = currentTime;
}

void sendSummary()
{
  summary.cantPed = pedal_counter;
  summary.timeDone = currentEvent - startTimeTraining;
  summary.kmDone = kmDone;
  Serial.println("Cantidad Pedaleadas: ");
  Serial.println(summary.cantPed);
  Serial.println("Tiempo: ");
  Serial.println(summary.timeDone);
  Serial.println("Kilometros Recorridos: ");
  Serial.println(summary.kmDone);
}
