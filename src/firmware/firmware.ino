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

unsigned long TIEMPO_ACTUAL;
unsigned long TIEMPO_ANTERIOR;

int contador = 0;

//--CONSTANTES EXTRAS--
const double CONST_CONV_CM = 0.01723;

// define
#define SERIAL_SPEED 9600
#define TIEMPO_MAX_MILIS 1 // MEDIO SEGUNDO
#define VELOCIDAD_BAJA 30
#define VELOCIDAD_MEDIA 60
#define VELOCIDAD_ALTA 90

//long readPlayStopSensor(int triggerPin, int echoPin)
//{
  
//}

enum state_t { 
  STATE_WAITING_FOR_TRAINING, 
  STATE_READY_FOR_TRAINING, 
  STATE_TRAINING_IN_PROGRESS, 
  STATE_PAUSED_TRAINING, 
  STATE_TRAINING_FINISHED
};

enum event_t { 
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


int index = 0;
const unsigned long MAX_SENSOR_LOOP_TIME = 300;
const unsigned int NUMBER_OF_SENSORS = 5;
unsigned long currentTime;
unsigned long previousTime;

String arrEstados[5] = { "STATE_WAITING_FOR_TRAINING", "STATE_READY_FOR_TRAINING", "STATE_TRAINING_IN_PROGRESS", "STATE_PAUSED_TRAINING", "STATE_TRAINING_FINISHED"};
String arrEventos[9] = { "EVENT_TRAINING_RECEIVED", "EVENT_TRAINING_STARTED", "EVENT_TRAINING_PAUSED", "EVENT_TRAINING_RESUMED",
"EVENT_TRAINING_CONCLUDED", "EVENT_TRAINING_CANCELLED", "EVENT_TRAINING_RESTARTED", "EVENT_CONTINUE", "EVENT_MONITORING_TRAINING"};

void printEvento(int eventoIndex){
  Serial.print("Evento actual: ");
  Serial.println(arrEventos[eventoIndex]);
}

void printEstado(int estadoIndex){
  Serial.print("Estado actual: ");
  Serial.println(arrEstados[estadoIndex]);
}

void do_init() {
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


    //Inicializo el primer estado
    currentState = STATE_WAITING_FOR_TRAINING;
    currentEvent = EVENT_CONTINUE;

    //Inicializa el tiempo
    previousTime = millis();
}


//Funciones de atención a los sensores

void checkPlayStopSensor(){
  Serial.println("CHECK SENSOR PLAY/STOP");
}

void checkMediaControlSensor(){
    Serial.println("CHECK SENSOR MEDIA CONTROL");
}

void checkSpeedSensor(){
  Serial.println("CHECK SENSOR SPEED");
}

void checkTrainingButtonSensor(){
  Serial.println("CHECK SENSOR TRAINING SENSOR");
}


//NOTA IMPORRRRRRTANTE: Preguntar si hay que dejar como sensor a la interfaz bluetooth o no.
void checkBluetoothInterface(){
  if (Serial.available() > 0) {
    // read the incoming byte:
    String consoleCommand = Serial.readString();
    Serial.print("Comando recibido: ");
    Serial.println(consoleCommand);
    currentEvent = EVENT_TRAINING_RECEIVED;
    Serial.println("Entrenamiento comenzado");
  }
  else{
    currentEvent = EVENT_CONTINUE;
    Serial.println("estoy esperando comando");
  }
}

void (*check_sensor[NUMBER_OF_SENSORS])() = { 
  checkSpeedSensor, 
  checkTrainingButtonSensor,
  checkPlayStopSensor, 
  checkMediaControlSensor,
  checkBluetoothInterface
};


void get_event(){

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



void state_machine(){
  get_event();
  printEvento(currentEvent);
  printEstado(currentState);

  switch(currentState){
    case STATE_WAITING_FOR_TRAINING:
      switch(currentEvent){
        case EVENT_TRAINING_RECEIVED:
          currentState = STATE_READY_FOR_TRAINING;
          break;
        case EVENT_CONTINUE:
          currentState = STATE_WAITING_FOR_TRAINING;
          break;
        default: 
          Serial.println("EVENTO_DESCONOCIDO");
          break;
      }
      break;
    case STATE_READY_FOR_TRAINING:
      switch(currentEvent){
        case EVENT_TRAINING_STARTED:
          currentState = STATE_TRAINING_IN_PROGRESS;
          break;
        case EVENT_CONTINUE:
          currentState = STATE_READY_FOR_TRAINING;
          break;
        default:
          Serial.println("EVENTO_DESCONOCIDO");
          break;
      }
      break;
    case STATE_TRAINING_IN_PROGRESS:
      switch(currentEvent){
        case EVENT_TRAINING_CONCLUDED:
          currentState = STATE_TRAINING_FINISHED;
          break;
        case EVENT_TRAINING_PAUSED:
          currentState = STATE_PAUSED_TRAINING;
          break;
        case EVENT_TRAINING_CANCELLED:
          currentState = STATE_TRAINING_FINISHED;
          break;
        case EVENT_MONITORING_TRAINING:
          currentState = STATE_TRAINING_IN_PROGRESS;
          break;
        default:
          Serial.println("EVENTO_DESCONOCIDO");
          break;
      }
      break;
    case STATE_PAUSED_TRAINING:
      switch(currentEvent){
        case EVENT_TRAINING_RESUMED:
          currentState = STATE_TRAINING_IN_PROGRESS;
          break;
        case EVENT_TRAINING_CANCELLED:
          currentState = STATE_TRAINING_FINISHED;
          break;
        case EVENT_CONTINUE:
          currentState = STATE_PAUSED_TRAINING;
          break;
        default:
          Serial.println("EVENTO_DESCONOCIDO");
          break;
      }
      break;
    case STATE_TRAINING_FINISHED:
      switch(currentEvent){
        case EVENT_TRAINING_RESTARTED:
          currentState = STATE_WAITING_FOR_TRAINING;
          break;
        case EVENT_CONTINUE:
          currentState = STATE_TRAINING_FINISHED;
          break;
        default:
          Serial.println("EVENTO_DESCONOCIDO");
          break;
      }
      break;
    default:
      Serial.println("ESTADO_DESCONOCIDO");
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

// void loop()
// {
//     // TOMO EL TIEMPO ACTUAL
//     TIEMPO_ACTUAL = millis();
//     // Serial.println("Hola");
//     // VERIFICA CUANTO TRANSCURRIO DE TIEMPO

//     int valorPot = analogRead(HALL_SENSOR_PIN);

//     int velocidad_pedaleada = map(valorPot, 0, 1023, 0, 100);
//     Serial.println(velocidad_pedaleada);

//     if ((TIEMPO_ACTUAL - TIEMPO_ANTERIOR) >= velocidad_pedaleada && velocidad_pedaleada > 0)
//     {
//         TIEMPO_ANTERIOR = TIEMPO_ACTUAL;
//         lcd.setCursor(0, 0);
//         lcd.print("Vueltas");
//         lcd.setCursor(10, 0);
//         contador += 1;
//         lcd.print(contador);
//       	lcd.setCursor(0,1);
//         lcd.print("Velocidad");
//       	lcd.setCursor(10,1);
//       	lcd.print(velocidad_pedaleada);
//       	lcd.clear();
    
//     }

//     if (velocidad_pedaleada <= VELOCIDAD_BAJA)
//     {
//         analogWrite(BLUE_LED_PIN, 255);
//         analogWrite(GREEN_LED_PIN, 0);
//         analogWrite(RED_LED_PIN, 0);
//     }
//     else if (velocidad_pedaleada > VELOCIDAD_BAJA && velocidad_pedaleada < VELOCIDAD_MEDIA)
//     {
//         analogWrite(BLUE_LED_PIN, 0);
//         analogWrite(GREEN_LED_PIN, 255);
//         analogWrite(RED_LED_PIN, 0);
//     }
//     else
//     {
//         analogWrite(BLUE_LED_PIN, 0);
//         analogWrite(GREEN_LED_PIN, 0);
//         analogWrite(RED_LED_PIN, 255);
//     }


// }



//void limpiarFila(int fila, int columna)