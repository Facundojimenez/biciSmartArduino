/*
  TINKERCAD: https://www.tinkercad.com/things/4jSW9r8r8Eu-fantastic-crift/editel?sharecode=Mbjg4XzH3n2pD3k0aG0lXuejTw5PJrB85NU4hLtyJUw
*/

/*
NOTAS DEL PROFE:
- Está bien tener el bluetooth como un sensor más.
- Sacar los switches de las funciones de los LCD
- Sacar toda la logica de que se leen estados en las funciones de los sensores.
- Crear eventos en la maquina de estados para los botones
- MEjorar las funciones de los actuadores
- Hacer los prints de los mensajes de bluetooth.
- Implementar logica de play/stop de musica bluetooth segun la intensidad.

TO DO:
  *Definir la estructura entrenamiento
  *Que empiece a contar cuando tocamos el boton, no cuando prende
  *Envio del resumen por BT al finalizar el entrenamiento, hayq eu ver como calcular cuantos km hizo, la vel media
  *Activar el buzzer por porcentaje de KM (por tiempo ya esta hecho, falta por km)
  *Timeout esperando entrenamiento (hecho, revisar!)
  *Entrenamiento predeterminado (hecho, revisar!)
  *Reseteo de las variables para el proximo entrenamiento
  *Prender RGB cuando prende el sistema

ENTRENAMIENTO:

- Por km o Vel (una de las dos), check si quiere musica dinamica o no, si lo quiere, se desactivan los botones de la media?
- Como sabemos cuanto tiempo estuvo entrenando o cuantos km hizo -> activar el buzzer -> resumen
- Resumen: Duracion, Vel Media, KM recorridos, Cant Pedaleadas
- Banda sonora dinamica? choca con la funcionalidad de poner en pausa la musica cuando va lento

*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Crear el objeto lcd dirección 0x3F y 16 columnas x 2 Filas
LiquidCrystal_I2C lcd(0x20, 16, 2);

//--CONSTANTES CON NOMBRES DE LOS PINES--
// SENSORES
const unsigned int NUMBER_OF_SENSORS = 8; // 5 + 1(BLUETOOTH) + 1 (Progreso) + 1 (dynamicMusic)

const int PLAY_STOP_MEDIA_SENSOR_PIN = 8;
const int MEDIA_MOVEMENT_SENSOR_PIN = 7;
const int HALL_SENSOR_PIN = A3;
const int TRAINING_CONTROL_PIN = 2;
const int TRAINING_CANCEL_PIN = 4;

/// PREVIOUS NEXT BUTTON
int buttonStatePrevious = LOW;                   // previousstate of the switch
unsigned long minButtonLongPressDuration = 1000; // Time we wait before we see the press as a long press
unsigned long buttonLongPressMillis;             // Time in ms when we the button was pressed
bool buttonStateLongPress = false;               // True if it is a long press
const int intervalButton = 50;                   // Time between two readings of the button state
unsigned long previousButtonMillis;              // Timestamp of the latest reading
unsigned long buttonPressDuration;
unsigned long currentButtonMillis;

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

float distanceTraveled = 0;
unsigned long startTime;

const unsigned LOW_SPEED = 7;
const unsigned MEDIUM_SPEED = 13;
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
unsigned long ctTraining;
unsigned long lctTraining;
bool trainingReceived = false;

// RESUMEN
struct tSummary
{
  unsigned timeDone;
  unsigned int kmDone;
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
  EVENT_PS_MEDIA_BUTTON,
  EVENT_PREVIOUS_MEDIA_BUTTON,
  EVENT_NEXT_MEDIA_BUTTON,
  EVENT_TRAINING_CONCLUDED,
  EVENT_TRAINING_CANCELLED,
  EVENT_TRAINING_RESTARTED,
  EVENT_CONTINUE,
  EVENT_MONITORING_TRAINING,
};

event_t currentEvent;
state_t currentState;

String arrStates[5] = {"STATE_WAITING_FOR_TRAINING", "STATE_READY_FOR_TRAINING", "STATE_TRAINING_IN_PROGRESS", "STATE_PAUSED_TRAINING", "STATE_TRAINING_FINISHED"};
String arrEvents[10] = {"EVENT_TRAINING_RECEIVED", "EVENT_TRAINING_BUTTON", "EVENT_TRAINING_CANCELLED", "EVENT_PS_MEDIA_BUTTON", "EVENT_PREVIOUS_MEDIA_BUTTON", "EVENT_NEXT_MEDIA_BUTTON",
                        "EVENT_TRAINING_CONCLUDED", "EVENT_TRAINING_RESTARTED", "EVENT_CONTINUE", "EVENT_MONITORING_TRAINING"};

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
  lctTraining = millis();
  startTimeTraining = millis();
}

// Funciones de atención a los sensores
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
    currentEvent = EVENT_CONTINUE;
  }
}

void checkDynamicMusic()
{

  if (!settedTrainning.dynamicMusic) // Si esta con su propia musica y va lento, se pausa su musica
  {
    if (speed_KMH <= LOW_SPEED)
      currentEvent = EVENT_PS_MEDIA_BUTTON;
  }
}

void checkMediaButtonSensor()
{
  int buttonState = digitalRead(MEDIA_MOVEMENT_SENSOR_PIN);
  // if(buttonState == HIGH)
  // {
  //   currentEvent = EVENT_NEXT_MEDIA_BUTTON;
  // }
  currentButtonMillis = millis();

  // // If the difference in time between the previous reading is larger than intervalButton
  if (currentButtonMillis - previousButtonMillis > intervalButton)
  {

    //   // Read the digital value of the button (LOW/HIGH)
    int buttonState = digitalRead(MEDIA_MOVEMENT_SENSOR_PIN);

    //   // If the button has been pushed AND
    //   // If the button wasn't pressed before AND
    //   // IF there was not already a measurement running to determine how long the button has been pressed
    if (buttonState == HIGH && buttonStatePrevious == LOW && !buttonStateLongPress)
    {
      buttonLongPressMillis = currentButtonMillis;
      buttonStatePrevious = HIGH;
      //     // Serial.println("Button pressed");
    }

    //   // Calculate how long the button has been pressed
    buttonPressDuration = currentButtonMillis - buttonLongPressMillis;

    //   // If the button is pressed AND
    //   // If there is no measurement running to determine how long the button is pressed AND
    //   // If the time the button has been pressed is larger or equal to the time needed for a long press
    if (buttonState == HIGH && !buttonStateLongPress && buttonPressDuration >= minButtonLongPressDuration)
    {
      buttonStateLongPress = true;
      Serial.println("Button long pressed");
      currentEvent = EVENT_PREVIOUS_MEDIA_BUTTON;
    }

    //   // If the button is released AND
    //   // If the button was pressed before
    if (buttonState == LOW && buttonStatePrevious == HIGH)
    {
      buttonStatePrevious = LOW;
      buttonStateLongPress = false;
      //     // Serial.println("Button released");

      //     // If there is no measurement running to determine how long the button was pressed AND
      //     // If the time the button has been pressed is smaller than the minimal time needed for a long press
      //     // Note: The video shows:
      if (!buttonStateLongPress && buttonPressDuration < minButtonLongPressDuration)
      {
        //     //       since buttonStateLongPress is set to FALSE on line 263, !buttonStateLongPress is always TRUE
        //     //       and can be removed.
        if (buttonPressDuration < minButtonLongPressDuration)
        {
          Serial.println("Button pressed shortly");
          currentEvent = EVENT_NEXT_MEDIA_BUTTON;
        }
      }

      //   // store the current timestamp in previousButtonMillis
      previousButtonMillis = currentButtonMillis;
    }
  }
}

void checkPlayStoptButtonSensor()
{
  int buttonState = digitalRead(PLAY_STOP_MEDIA_SENSOR_PIN);
  if (buttonState == HIGH)
  {
    currentEvent = EVENT_PS_MEDIA_BUTTON;
  }
}

void checkCancelButtonSensor()
{
  int buttonState = digitalRead(TRAINING_CANCEL_PIN);
  if (buttonState == HIGH)
  {
    currentEvent = EVENT_TRAINING_CANCELLED;
  }
}

void checkTrainingButtonSensor()
{
  int buttonState = digitalRead(TRAINING_CONTROL_PIN);

  if (buttonState == HIGH)
  {
    currentEvent = EVENT_TRAINING_BUTTON;
  }
}

void checkBluetoothInterface()
{
  ctTraining = millis();
  if (!trainingReceived)
  {
    if ((ctTraining - lctTraining) < MAX_TIME_WAITTING_TRAINING)
    {
      if (Serial.available() > 0)
      {
        // read the incoming byte:
        // reemplazar Seria con el obj bluetooth una vez en la prueba de hardware
        String consoleCommand = Serial.readString();
        int dynamicMusic;
        Serial.print("Comando recibido: "); // TRAINING: 30MIN TRUE - TRAINING: 3KM FALSE
        Serial.println(consoleCommand);
        if (sscanf(consoleCommand.c_str(), "TRAINING: %ldMIN %d", &(settedTrainning.settedTime), &dynamicMusic))
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
    currentEvent = EVENT_CONTINUE;
  }
}

void checkProgress() // Verifica si termino o no
{
  if (settedTrainning.settedTime != 0) // Si seteo por tiempo
  {
    long trainingTime = currentTime - startTimeTraining;
    if ((trainingTime / ONE_MINUTE) >= settedTrainning.settedTime)
    {
      currentEvent = EVENT_TRAINING_CONCLUDED;
    }
  }
  else // Si seteo por KM
  {
    // Calcular los KM recorridos para saber si termino o no
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
        checkDynamicMusic};

// Funciones Actuadores

void showSpeed();
void showTrainignState(String event);
void turnOnIntensityLed();
void ledLowSpeed();
void ledNormalSpeed();
void ledHighSpeed();
void offLed();
void sendMusicComand(String comand);
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
      currentState = STATE_TRAINING_FINISHED;
      break;
    case EVENT_TRAINING_BUTTON:
      showTrainignState("Paused");
      currentState = STATE_PAUSED_TRAINING;
      break;
    case EVENT_TRAINING_CANCELLED:
      showTrainignState("Cancelled");
      currentState = STATE_TRAINING_FINISHED;
      break;
    // case EVENT_MONITORING_TRAINING:
    //  currentState = STATE_TRAINING_IN_PROGRESS;
    // break;
    case EVENT_PS_MEDIA_BUTTON:
      sendMusicComand("STOP");
      currentState = STATE_TRAINING_IN_PROGRESS;
      break;
    case EVENT_NEXT_MEDIA_BUTTON:
      sendMusicComand("NEXT");
      currentState = STATE_TRAINING_IN_PROGRESS;
      break;
    case EVENT_PREVIOUS_MEDIA_BUTTON:
      sendMusicComand("PREVIOUS");
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
      showTrainignState("Cancelles");
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
      sendSummary();
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

void showTrainignState(String event)
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
  else if (speed_KMH < MEDIUM_SPEED)
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

void sendMusicComand(String comand)
{
  if (!settedTrainning.dynamicMusic)
  {
    Serial.print("Enviando Comando: ");
    Serial.println(comand);
  }
}

void turnOnBuzzer()
{
  long trainingTime = currentTime - startTimeTraining;
  if (settedTrainning.settedTime != 0)
  {
    // Suena el Buzzer segun progreso del entrenamiento
    // float trainingTimeMin = ((float)trainingTime/ONE_MINUTE);
    float timePercent = (trainingTime * 100 / (float)settedTrainning.settedTime);
    Serial.println("Porcentaje");
    Serial.println(timePercent);
    Serial.println(trainingTime);
    Serial.println(settedTrainning.settedTime);
    if (timePercent >= 25 && timePercent <= 26)
    {
      Serial.println("suena 25");
      tone(BUZZER_PIN, 200, 2000);
    }
    else if (timePercent >= 50 && timePercent <= 51)
    {
      Serial.println("suena 50");
      tone(BUZZER_PIN, 300, 2000);
    }
    else if (timePercent >= 75 && timePercent <= 76)
    {
      tone(BUZZER_PIN, 400, 2000);
    }
    else if (timePercent >= 99 && timePercent <= 101)
    {
      tone(BUZZER_PIN, 500, 2000);
    }
  }
  else
  {

    // Calculate the time elapsed since the start (in hours)
    float timeElapsedHours = trainingTime / 1000.0 / 3600.0;

    // Calculate the distance traveled using the formula
    float distanceIncrement = speed_KMH * timeElapsedHours;

    // Update the total distance traveled
    distanceTraveled += distanceIncrement;

    float travellingPercent = (distanceTraveled * 100 / (float)settedTrainning.settedKm);

    // Update the start time for the next calculation
    startTime = currentTime;

    Serial.println("Porcentaje");
    Serial.println(travellingPercent);
    Serial.println(trainingTime);
    Serial.println(settedTrainning.settedTime);
    if (travellingPercent >= 25 && travellingPercent <= 26)
    {
      Serial.println("suena 25");
      tone(BUZZER_PIN, 200, 2000);
    }
    else if (travellingPercent >= 50 && travellingPercent <= 51)
    {
      Serial.println("suena 50");
      tone(BUZZER_PIN, 300, 2000);
    }
    else if (travellingPercent >= 75 && travellingPercent <= 76)
    {
      tone(BUZZER_PIN, 400, 2000);
    }
    else if (travellingPercent >= 99 && travellingPercent <= 101)
    {
      tone(BUZZER_PIN, 500, 2000);
    }
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
  float timeElapsedHours = (currentTime - startTime) / 1000.0 / 3600.0;

  // Calculate the distance traveled using the formula
  float distanceIncrement = speed_KMH * timeElapsedHours;

  // Update the total distance traveled
  distanceTraveled += distanceIncrement;

  // Update the start time for the next calculation
  startTime = currentTime;
}

void sendSummary()
{
}
