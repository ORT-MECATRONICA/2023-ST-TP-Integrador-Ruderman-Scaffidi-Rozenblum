#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

#define PIN_LDR 
#define PIN_BUZZER
#define PIN_RELE 
#define BMP_SCK  ()
#define BMP_MISO ()
#define BMP_MOSI ()
#define BMP_CS   ()
#define BOTON_CONFIG  
#define BOTON_SUMA 
#define BOTON_RESTA
#define BOTON_SELECCIONAR
#define BOTON_ALARMA

#define PANTALLA_PRINCIPAL 1   //estados de la maquina
#define CHEQUEO_BOTONES 2
#define CONFIGURACION_TEMP 3
#define CONFIGURACION_HUM 4
#define CONFIGURACION_MQTT 5

  int estado_pantalla = 1;
  bool botonSuma = 0;
  bool botonResta = 0;
  bool botonConfig = 0;
  bool botonSeleccionar = 0;
  
Adafruit_BMP280 bmp; // I2C

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  
  pinMode(PIN_LDR, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_RELE, OUTPUT);
  pinMode(BOTON_CONFIG, INPUT_PULLUP);
  pinMode(BOTON_SUMA, INPUT_PULLUP);
  pinMode(BOTON_RESTA, INPUT_PULLUP);
  pinMode(BOTON_SELECCIONAR, INPUT_PULLUP);
  pinMode(BOTON_ALARMA, INPUT_PULLUP);

  bmp.begin();
  Serial.begin (9600);
  lcd.begin();
  lcd.backlight();
 
}

void loop() {
  
  int humedad = analogRead(PIN_HUMEDAD);
  int luz = analogRead(PIN_LDR);
  int temp = bmp.readTemperature();

  switch (estado_pantalla)
  {
    case PANTALLA_PRINCIPAL:
    
      Serial.print ("pantalla principal");
      lcd.setCursor(0, 0);
      lcd.print("Temp:");

      lcd.setCursor(5,0);
      lcd.print(temp);
  
      lcd.setCursor(9,0);
      lcd.print("Hum:");
  
      lcd.setCursor(13,0);
      lcd.print(humedad);

      lcd.setCursor(0,1);
      lcd.print("LED:");

      lcd.setCursor(4,1);
      lcd.print(luz);

      if (digitalRead (BOTON_CONFIG) == LOW){
        estado_pantalla = CHEQUEO_BOTONES;
      }

      break;

    case CHEQUEO_BOTONES:

      Serial.print ("chequeo de botones");

      if (digitalRead (BOTON_CONFIG) == HIGH && botonConfig == 0){
        lcd.clear();
        botonConfig = 1;
        estado_pantalla = CONFIGURACION_TEMP;     
      }

      if (digitalRead (BOTON_CONFIG) == HIGH && botonConfig == 1){
        lcd.clear();
        botonConfig = 0;
        estado_pantalla = PANTALLA_PRINCIPAL;     
      }     

      if (digitalRead (BOTON_SELECCIONAR) == HIGH && estado_pantalla == CONFIGURACION_TEMP){
        lcd.clear();
        estado_pantalla = CONFIGURACION_HUM;     
      }

      if (digitalRead (BOTON_SELECCIONAR) == HIGH && estado_pantalla == CONFIGURACION_HUM){
        lcd.clear();
        estado_pantalla = CONFIGURACION_MQTT;     
      }

      if (digitalRead (BOTON_SELECCIONAR) == HIGH && estado_pantalla == CONFIGURACION_MQTT){
        lcd.clear();
        estado_pantalla = CONFIGURACION_TEMP;     
      }

      break;

  }
    
}
