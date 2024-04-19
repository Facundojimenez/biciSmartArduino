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


void setup()
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

    // TOMA LA PRIMERA MEDICION DE TIEMPO
    TIEMPO_ANTERIOR = millis();
}

void loop()
{
    // TOMO EL TIEMPO ACTUAL
    TIEMPO_ACTUAL = millis();
    // Serial.println("Hola");
    // VERIFICA CUANTO TRANSCURRIO DE TIEMPO

    int valorPot = analogRead(HALL_SENSOR_PIN);

    int velocidad_pedaleada = map(valorPot, 0, 1023, 0, 100);
    Serial.println(velocidad_pedaleada);

    if ((TIEMPO_ACTUAL - TIEMPO_ANTERIOR) >= velocidad_pedaleada && velocidad_pedaleada > 0)
    {
        TIEMPO_ANTERIOR = TIEMPO_ACTUAL;
        lcd.setCursor(0, 0);
        lcd.print("Vueltas");
        lcd.setCursor(10, 0);
        contador += 1;
        lcd.print(contador);
      	lcd.setCursor(0,1);
        lcd.print("Velocidad");
      	lcd.setCursor(10,1);
      	lcd.print(velocidad_pedaleada);
      	lcd.clear();
    
    }

    if (velocidad_pedaleada <= VELOCIDAD_BAJA)
    {
        analogWrite(BLUE_LED_PIN, 255);
        analogWrite(GREEN_LED_PIN, 0);
        analogWrite(RED_LED_PIN, 0);
    }
    else if (velocidad_pedaleada > VELOCIDAD_BAJA && velocidad_pedaleada < VELOCIDAD_MEDIA)
    {
        analogWrite(BLUE_LED_PIN, 0);
        analogWrite(GREEN_LED_PIN, 255);
        analogWrite(RED_LED_PIN, 0);
    }
    else
    {
        analogWrite(BLUE_LED_PIN, 0);
        analogWrite(GREEN_LED_PIN, 0);
        analogWrite(RED_LED_PIN, 255);
    }


}

//void limpiarFila(int fila, int columna)