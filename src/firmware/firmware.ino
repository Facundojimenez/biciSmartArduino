
int cm = 0;
const double CONST_CONV_CM = 0.01723;
const int PLAY_STOP_SENSOR_PIN = 7;
const int NEXT_MEDIA_SENSOR_PIN = 4;


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

void setup()
{
  Serial.begin(9600);

}

void loop()
{
  //leo el tiempo obtiendo y lo convierto a cm
  // a partir de la formula del sonido
  int playStopSensorDistanceCm = CONST_CONV_CM * readPlayStopSensor(PLAY_STOP_SENSOR_PIN, PLAY_STOP_SENSOR_PIN);
  int nextMediaSensorDistanceCm = CONST_CONV_CM * readNextMediaSensor(NEXT_MEDIA_SENSOR_PIN, NEXT_MEDIA_SENSOR_PIN);
  
  Serial.print("Distancia del sensor de PLAY/STOP: ");
  Serial.print(playStopSensorDistanceCm);
  Serial.println(" cm");

  Serial.print("Distancia del sensor de NEXT_MEDIA: ");
  Serial.print(nextMediaSensorDistanceCm);
  Serial.println(" cm\n");
  
  delay(1000); // espero 10 milisegudos
}