/*
  NOTA: Si al hacer nuevos cambios en el codigo de tinkercad les aparece "invalid header file", se trata de un error
  generico, y no tiene nada que ver con el header en si, si no de un error como un punto y coma o que alguna variable sin declarar,
  parentesis que no cierran, etc.
*/

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

//Crear el objeto lcd  dirección  0x3F y 16 columnas x 2 filas
LiquidCrystal_I2C lcd(0x20,16,2); 


//--CONSTANTES CON NOMBRES DE LOS PINES--
//SENSORES
const int PLAY_STOP_MOVEMENT_SENSOR_PIN = 7;
const int NEXT_MEDIA_MOVEMENT_SENSOR_PIN = 4;
const int HALL_SENSOR_PIN = A3;
const int TRAINING_CONTROL_PIN = 2;

//ACTUADORES
const int RED_LED_PIN = 11;
const int GREEN_LED_PIN = 6;
const int BLUE_LED_PIN = 10;
const int BUZZER_PIN = 3;


//--CONSTANTES EXTRAS--
const double CONST_CONV_CM = 0.01723;

void setup()
{
  //Acá puntualmente NO se inicializan los pines de los sensores de movimiento, porque la funcion que los maneja los va mapeando dinamicamente como input y output
  pinMode(TRAINING_CONTROL_PIN, INPUT);
  pinMode(HALL_SENSOR_PIN, INPUT);

  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  lcd.init();                     
  lcd.backlight();

  Serial.begin(9600);
}

long readPlayStopSensor(int triggerPin, int echoPin)
{
  pinMode(triggerPin, OUTPUT); 
  
  // limpio el TRIGGER
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  
  //pongo HIGH el trigger por 10 microsegundos
   digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  pinMode(echoPin, INPUT);
  
  //Leo la señal ECHO y retorno el tiempo del sondio
  return pulseIn(echoPin, HIGH);
}

long readNextMediaSensor(int triggerPin, int echoPin)
{
  pinMode(triggerPin, OUTPUT); 
  
  // limpio el TRIGGER
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  
  //pongo HIGH el trigger por 10 microsegundos
   digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  pinMode(echoPin, INPUT);
  
  //Leo la señal ECHO y retorno el tiempo del sondio
  return pulseIn(echoPin, HIGH);
}

void loop()
{
  //EJEMPLO DE USO DE SENSORES DE PROXIMIDAD (PARA LOS CONTROLES DE LA MUSICA EN EL MANUBRIO)
  //leo el tiempo obtiendo y lo convierto a cm
  // a partir de la formula del sonido
  int playStopSensorDistanceCm = CONST_CONV_CM * readPlayStopSensor(PLAY_STOP_MOVEMENT_SENSOR_PIN, PLAY_STOP_MOVEMENT_SENSOR_PIN);
  int nextMediaSensorDistanceCm = CONST_CONV_CM * readNextMediaSensor(NEXT_MEDIA_MOVEMENT_SENSOR_PIN, NEXT_MEDIA_MOVEMENT_SENSOR_PIN);
  
  //LOGGEO DE SENSORES DE PROXIMIDAD )
  /*
  Serial.print("Distancia del sensor de PLAY/STOP: ");
  Serial.print(playStopSensorDistanceCm);
  Serial.println(" cm");

  Serial.print("Distancia del sensor de NEXT_MEDIA: ");
  Serial.print(nextMediaSensorDistanceCm);
  Serial.println(" cm\n");
  */
  
  //LOGGEO DEL BOTON DE ENTRENAMIENTO (ARRANCAR, PAUSAR O CANCELAR)
  int trainingButtonReadValue = digitalRead(TRAINING_CONTROL_PIN);
  Serial.print("boton de entrenamiento: ");
  Serial.println(trainingButtonReadValue);
  
  //EJEMPLO DE USO DE PANTALLA LED
  lcd.print("Hola Mundo");
  lcd.setCursor(0,1);

  //EJEMPLO DE USO DE COLORES RGB (Valores del 0-255, combinando los 3 valores de los LED se forman nuevos colores)
  //SE USA PARA MOSTRAR EL VALOR DE LA INTENSIDAD ACTUAL DEL ENTRENAMIENTO 
  analogWrite(RED_LED_PIN, 200);
  analogWrite(GREEN_LED_PIN, 150);
  analogWrite(BLUE_LED_PIN, 100);
  
  //EJEMPLO DE LECTURA DE SENSOR HALL (simulado con potenciometro)
  int hallSensorValue = analogRead(HALL_SENSOR_PIN);
  Serial.print("\nSensor hall (potenciometro): ");
  Serial.println(hallSensorValue);
  
  //EJEMPLO DE USO DE BUZZER (LO DEJO COMENTADO PORQUE TE VIOLA EL OIDO)
  //Se le setea como segundo parametro la frecuencia a la que vibra, mientras mas alta, mas agudo, y viceversa.
  //tone(BUZZER_PIN, 400);

  
  
  delay(200); // espero 200 milisegudos
}