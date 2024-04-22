#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Crear el objeto lcd dirección 0x3F y 16 columnas x 2 Filas
LiquidCrystal_I2C lcd(0x20, 16, 2);

//--CONSTANTES CON NOMBRES DE LOS PINES--
// SENSORES

const int PLAY_STOP_MOVEMENT_SENSOR_PIN = 7;
const int PREVIOUS_NEXT_MEDIA_MOVEMENT_SENSOR_PIN = 4;
const int HALL_SENSOR_PIN = A3;
const int TRAINING_CONTROL_PIN = 2;

// ACTUADORES
const int RED_LED_PIN = 11;
const int GREEN_LED_PIN = 6;
const int BLUE_LED_PIN = 10;
// const int BUZZER_PIN = 3;

unsigned long actual_pedalling_time;
unsigned long previous_pedalling_time;

int pedal_counter = 0;
float pedaling_frecuency_mHz = 0;
float speed = 0;
int index = 0;
float rpm = 0;

//--CONSTANTES EXTRAS--
const double CONST_CONV_CM = 0.01723;
const double COMMON_WHEEL_CIRCUNFERENCE = 2.1;

#define SERIAL_SPEED 9600
#define TIEMPO_MAX_MILIS 50 // MEDIO SEGUNDO
const unsigned LOW_SPEED = 7;
const unsigned MEDIUM_SPEED = 13;
const unsigned HIGH_SPEED = 20;

const unsigned long MAX_SENSOR_LOOP_TIME = 50;
const unsigned int NUMBER_OF_SENSORS = 5;
unsigned long currentTime;
unsigned long previousTime;

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
  EVENT_TRAINING_STARTED,
  EVENT_TRAINING_PAUSED,
  EVENT_TRAINING_RESUMED,
  EVENT_TRAINING_CONCLUDED,
  EVENT_TRAINING_CANCELLED,
  EVENT_TRAINING_RESTARTED,
  EVENT_CONTINUE,
  EVENT_MONITORING_TRAINING
};

event_t currentEvent;
state_t currentState;

String arrStates[5] = {"STATE_WAITING_FOR_TRAINING", "STATE_READY_FOR_TRAINING", "STATE_TRAINING_IN_PROGRESS", "STATE_PAUSED_TRAINING", "STATE_TRAINING_FINISHED"};
String arrEvents[9] = {"EVENT_TRAINING_RECEIVED", "EVENT_TRAINING_STARTED", "EVENT_TRAINING_PAUSED", "EVENT_TRAINING_RESUMED",
                       "EVENT_TRAINING_CONCLUDED", "EVENT_TRAINING_CANCELLED", "EVENT_TRAINING_RESTARTED", "EVENT_CONTINUE", "EVENT_MONITORING_TRAINING"};

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

void do_init()
{
  // Acá puntualmente NO se inicializan los pines de los sensores de movimiento,
  // porque la funcion que los maneja los va mapeando dinamicamente como input y output
  pinMode(TRAINING_CONTROL_PIN, INPUT);
  pinMode(HALL_SENSOR_PIN, INPUT);

  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  // pinMode(BUZZER_PIN, OUTPUT);

  lcd.init();
  lcd.backlight();

  Serial.begin(SERIAL_SPEED);

  // Inicializo el primer estado
  currentState = STATE_WAITING_FOR_TRAINING;
  currentEvent = EVENT_CONTINUE;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Waiting For");
  lcd.setCursor(0, 1);
  lcd.print("Training!");
  // Inicializa el tiempo
  previousTime = millis();
}

// Funciones de atención a los sensores

void checkPlayStopSensor()
{
  Serial.println("CHECK SENSOR PLAY/STOP");
  readPlayStopSensor(PLAY_STOP_MOVEMENT_SENSOR_PIN, PLAY_STOP_MOVEMENT_SENSOR_PIN);
}

void checkMediaControlSensor()
{
  Serial.println("CHECK SENSOR MEDIA CONTROL");
  readMediaControlSensor(PREVIOUS_NEXT_MEDIA_MOVEMENT_SENSOR_PIN, PREVIOUS_NEXT_MEDIA_MOVEMENT_SENSOR_PIN);
}

void checkSpeedSensor()
{
  Serial.println("CHECK SENSOR SPEED");
  // TOMO EL TIEMPO ACTUAL
  actual_pedalling_time = millis();
  //     // Serial.println("Hola");
  //     // VERIFICA CUANTO TRANSCURRIO DE TIEMPO

  int valorPot = analogRead(HALL_SENSOR_PIN);

  // common pedalling frecuency varies from 60 to 120 rpm which is 1 to 2 hz

  // map the value to mHz (since only maps to integer values)
  pedaling_frecuency_mHz = map(valorPot, 0, 1023, 0, 4000);

  int frecuency = 1000000 / pedaling_frecuency_mHz;

  speed = ((pedaling_frecuency_mHz / 1000) * COMMON_WHEEL_CIRCUNFERENCE) * (3.6);

  // Serial.println(speed);
  if ((actual_pedalling_time - previous_pedalling_time) >= frecuency && pedaling_frecuency_mHz > 0)
  {
    previous_pedalling_time = actual_pedalling_time;
    pedal_counter += 1;
  }
}

void checkTrainingButtonSensor()
{
  int buttonState = digitalRead(TRAINING_CONTROL_PIN);

  if (buttonState == HIGH)
  {
    switch (currentState)
    {
    case STATE_READY_FOR_TRAINING:
      currentEvent = EVENT_TRAINING_STARTED;
      break;
    case STATE_TRAINING_IN_PROGRESS:
      currentEvent = EVENT_TRAINING_PAUSED;
      break;
    case STATE_PAUSED_TRAINING:
      currentEvent = EVENT_TRAINING_RESUMED;

    default:
      break;
    }
  }
}

// NOTA IMPORRRRRRTANTE: Preguntar si hay que dejar como sensor a la interfaz bluetooth o no.
void checkBluetoothInterface()
{
  if (Serial.available() > 0)
  {
    // read the incoming byte:
    // reemplazar Seria con el obj bluetooth una vez en la prueba de hardware
    String consoleCommand = Serial.readString();
    Serial.print("Comando recibido: ");
    Serial.println(consoleCommand);
    if (consoleCommand.equals("training"))
    {
      currentEvent = EVENT_TRAINING_RECEIVED;
      Serial.println("Entrenamiento recibido");
    }
  }
  else
  {
    currentEvent = EVENT_CONTINUE;
    Serial.println("estoy esperando comando");
  }
}

void (*check_sensor[NUMBER_OF_SENSORS])() = {
    checkSpeedSensor,
    checkTrainingButtonSensor,
    checkPlayStopSensor,
    checkMediaControlSensor,
    checkBluetoothInterface};

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
      updateLCD(currentEvent);
      currentState = STATE_READY_FOR_TRAINING;
      break;
    case EVENT_CONTINUE:
      updateLCD(currentEvent);
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
    case EVENT_TRAINING_STARTED:
      updateLCD(currentEvent);
      currentState = STATE_TRAINING_IN_PROGRESS;
      break;
    case EVENT_CONTINUE:
      currentState = STATE_READY_FOR_TRAINING;
      break;
    case EVENT_TRAINING_RECEIVED:
      updateLCD(currentEvent);
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
      currentState = STATE_TRAINING_FINISHED;
      break;
    case EVENT_TRAINING_PAUSED:
      updateLCD(currentEvent);
      updateRGB(currentEvent);
      currentState = STATE_PAUSED_TRAINING;
      break;
    case EVENT_TRAINING_CANCELLED:
      currentState = STATE_TRAINING_FINISHED;
      break;
    case EVENT_MONITORING_TRAINING:
      // currentState = STATE_TRAINING_IN_PROGRESS;
      break;

    case EVENT_CONTINUE:
      updateRGB(EVENT_MONITORING_TRAINING);
      updateLCD(EVENT_MONITORING_TRAINING);
      // currentState = STATE_TRAINING_IN_PROGRESS;
      break;

    case EVENT_TRAINING_RESUMED:
      updateLCD(currentEvent);
      updateRGB(currentEvent);
      break;

    default:
      Serial.println("UNKNOWN_EVENT");
      break;
    }
    break;
  case STATE_PAUSED_TRAINING:
    switch (currentEvent)
    {
    case EVENT_TRAINING_RESUMED:
      updateLCD(currentEvent);
      updateRGB(currentEvent);
      currentState = STATE_TRAINING_IN_PROGRESS;
      break;
    case EVENT_TRAINING_CANCELLED:
      currentState = STATE_TRAINING_FINISHED;
      break;
    case EVENT_CONTINUE:
      updateRGB(EVENT_TRAINING_PAUSED);
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

void setup()
{
  do_init();
}

void loop()
{
  state_machine();
}

void updateLCD(int event)
{
  switch (event)
  {
  case EVENT_TRAINING_RECEIVED:
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Training");
    lcd.setCursor(0, 1);
    lcd.print("Received!");
    break;
  case EVENT_TRAINING_STARTED:
    break;
  case EVENT_TRAINING_PAUSED:
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Training");
    lcd.setCursor(0, 1);
    lcd.print("Paused!");

    break;
  case EVENT_TRAINING_RESUMED:
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Training");
    lcd.setCursor(0, 1);
    lcd.print("Resumed!");
    break;
  case EVENT_TRAINING_CONCLUDED:
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Training");
    lcd.setCursor(0, 1);
    lcd.print("Concluded!");
    break;
  case EVENT_TRAINING_CANCELLED:
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Training");
    lcd.setCursor(0, 1);
    lcd.print("Canceled!");
    break;
  case EVENT_TRAINING_RESTARTED:
    break;
  case EVENT_CONTINUE:
    break;
  case EVENT_MONITORING_TRAINING:
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("laps");
    lcd.setCursor(10, 0);
    lcd.print(pedal_counter);
    lcd.setCursor(0, 1);
    lcd.print("speed");
    lcd.setCursor(10, 1);
    lcd.print((int)speed);
    break;
  default:
    break;
  }
}

void updateRGB(int event)
{
  switch (event)
  {
  case EVENT_TRAINING_RESUMED:
  case EVENT_MONITORING_TRAINING:
    if (speed <= LOW_SPEED)
    {
      analogWrite(BLUE_LED_PIN, 255);
      analogWrite(GREEN_LED_PIN, 0);
      analogWrite(RED_LED_PIN, 0);
    }
    else if (speed > LOW_SPEED && speed < HIGH_SPEED)
    {
      analogWrite(BLUE_LED_PIN, 0);
      analogWrite(GREEN_LED_PIN, 255);
      analogWrite(RED_LED_PIN, 0);
    }
    else if (speed >= HIGH_SPEED)
    {
      analogWrite(BLUE_LED_PIN, 0);
      analogWrite(GREEN_LED_PIN, 0);
      analogWrite(RED_LED_PIN, 255);
    }
    break;
  case EVENT_TRAINING_PAUSED:
    analogWrite(BLUE_LED_PIN, 0);
    analogWrite(GREEN_LED_PIN, 0);
    analogWrite(RED_LED_PIN, 0);

    break;

  default:
    break;
  }
}

long readPlayStopSensor(int triggerPin, int echoPin)
{
  pinMode(triggerPin, OUTPUT);

  // limpio el TRIGGER
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);

  // pongo HIGH el trigger por 10 microsegundos
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  pinMode(echoPin, INPUT);

  // Leo la señal ECHO y retorno el tiempo del sondio
  return pulseIn(echoPin, HIGH);
}

long readMediaControlSensor(int triggerPin, int echoPin)
{
  pinMode(triggerPin, OUTPUT);

  // limpio el TRIGGER
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);

  // pongo HIGH el trigger por 10 microsegundos
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  pinMode(echoPin, INPUT);

  // Leo la señal ECHO y retorno el tiempo del sondio
  return pulseIn(echoPin, HIGH);
}
