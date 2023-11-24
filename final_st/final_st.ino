#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "AsyncMqttClient.h"
#include "time.h"
#include "Arduino.h"
#include <UniversalTelegramBot.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

////


////
#define PRINCIPAL 0
#define PASAJE_SETTINGS 1
#define SETTINGS 2
#define PASAJE_PRINCIPAL 4
#define RESTA_CONTADOR 4
#define SUMA_CONTADOR 5

////::::::::::::::::

//////wifi
const char* ssid = "ORT-IoT";
const char* password = "OrtIOTnew22$2";

const char name_device = 22;  ////device numero de grupo 5A 1x siendo x el numero de grupo
///5B 2x siendo x el numero de grupo

// Timers auxiliar variables//////////////////////////
unsigned long now = millis(); ///valor actual
unsigned long lastMeasure1 = 0; ///variable para contar el tiempo actual
unsigned long lastMeasure2 = 0; ///variable para contar el tiempo actual

const unsigned long interval_envio = 5000;//Intervalo de envio de datos mqtt
const unsigned long interval_leeo =  10000;//Intervalo de lectura de datos y guardado en la cola  // tiempo de envio, modificar las 2 // este x2 del envio
int i = 0;

///time
int GMT = -3;
int GMT_Normal = -3;
long unsigned int timestamp ;  // hora
const char* ntpServer = "south-america.pool.ntp.org";
long  gmtOffset_sec = GMT * 3600; //seting cambiar gmt /declarar nuevamente la variable 3600
const int   daylightOffset_sec = 0;
int cambioHorario = 3600;
///variables ingresar a la cola struct
int indice_entra = 0;
int indice_saca = 0;
bool flag_vacio = 1;

/////mqqtt
#define MQTT_HOST IPAddress(10, 162, 24, 47)
#define MQTT_PORT 1884
#define MQTT_USERNAME "esp32"
#define MQTT_PASSWORD "mirko15"
char mqtt_payload[150] ;  /////
// Test MQTT Topic
#define MQTT_PUB "/esp32/datos_sensores"
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

/////::::::::::::

//#define SUMA_LDR 6
//#define SUMA_HORAS 7
//#define RESTA_TEMP 8
//#define RESTA_HUMEDAD 9
//#define RESTA_LDR 10
//#define RESTA_HORAS 11
//#define SUMA_SETTINGS 12
//#define RESTA_SETTINGS 13
#define BOTON_ARRIBA 12
#define BOTON_ABAJO 14
#define BOTON_DERECHA 27
#define BOTON_IZQUIERDA 26
#define BOTON_ENTER 25
#define LED_1 0
#define LED_2 2
#define LED_3 15
#define PIN_LDR 32


int contadorSettings = 0;
const int pinSensorHumedad = 35;
int estadoMaquina = 0;

int lcdColumns = 20;
int lcdRows = 4;
int estadoBotonArriba = 0;
int estadoBotonAbajo = 0;
int estadoBotonIzquierda = 0;
int estadoBotonDerecha = 0;
int estadoBotonEnter = 0;
int lecturaBotonArriba;
int lecturaBotonAbajo;
int lecturaBotonIzquierda;
int lecturaBotonDerecha;
int lecturaBotonEnter;
int ms;
int s;
bool estadoBuzzer;

#define PIN_BUZZER 33
#define PIN_COOLER 4
int estadoled = 0;
int modifHumedad;
int modifTemperatura;
int modifHoras;
int modifLdr;
float valorLdr = 0;
float valorHumedad;
float temp;
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);



Adafruit_BMP280 bmp; // use I2C interface

#define BOTtoken "6279852188:AAEM0Ya7gNj_960VzFXYhnueMvrXYuFrzus"  // cambiar por token
#define CHAT_ID "609914570"


String mensaje = "temperatura actual= ";
int delayRobot = 1000; /// intervalo
unsigned long lastTimeBotRan; /// ultimo tiempo



WiFiClientSecure client;
UniversalTelegramBot bot(BOTtoken, client);

Preferences preferencesTemp;
Preferences preferencesHum;
Preferences preferencesHoras;
Preferences preferencesLdr;
typedef struct
{
  long time;
  float T1;///tempe
  float H1;///humedad valor entre 0 y 100
  float luz;
  bool Alarma;
} estructura ;
/////////////////
const int valor_max_struct = 1000; ///valor vector de struct
estructura datos_struct [valor_max_struct];///Guardo valores hasta que lo pueda enviar
estructura aux2 ;


void setup() {

  Serial.begin(115200);
  /////declaro pines digitales
  setupmqtt();
  //Setup de time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  lcd.init();
  lcd.backlight();

  pinMode (BOTON_ARRIBA, INPUT_PULLUP);
  pinMode (BOTON_ABAJO, INPUT_PULLUP);
  pinMode (BOTON_IZQUIERDA, INPUT_PULLUP);
  pinMode (BOTON_DERECHA, INPUT_PULLUP);
  pinMode (BOTON_ENTER, INPUT_PULLUP);
  pinMode (LED_1, OUTPUT);
  pinMode (LED_2, OUTPUT);
  pinMode (LED_3, OUTPUT);
  pinMode (PIN_COOLER, OUTPUT);
  pinMode (PIN_BUZZER, OUTPUT);
  while ( !Serial ) delay(100);   // wait for native usb
  Serial.println(F("BMP280 Sensor event test"));

  unsigned status;
  status = bmp.begin(0x76);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                     "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  //  Default settings from datasheet.
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */




  //WiFi.mode(WIFI_STA);
 // WiFi.begin(ssid, password);
  client.setCACert(TELEGRAM_CERTIFICATE_ROOT);

  //while (WiFi.status() != WL_CONNECTED) {
  //  delay(500);
  //  Serial.println("Conectandose");
 // }

  //Serial.println(WiFi.localIP());






  preferencesTemp.begin("temperatura", modifTemperatura);// lo que quiero guardar
  preferencesHum.begin("humedad", modifHumedad);// lo que quiero guardar
  preferencesHoras.begin("horas", modifHoras);// lo que quiero guardar
  preferencesLdr.begin("ldr", modifLdr);// lo que quiero guardar

  modifTemperatura =  preferencesTemp.getInt("temperatura", 0);// lo que quiero guardar
  modifHumedad =  preferencesHum.getInt("humedad", 0);// lo que quiero guardar
  modifHoras =  preferencesHoras.getInt("horas", 0);// lo que quiero guardar
  modifLdr =  preferencesLdr.getInt("ldr", 0);// lo que quiero guardar

}


void loop() {
  Serial.println ("estado boton arriba =");
  Serial.println (estadoBotonArriba);
  Serial.println ("estado boton abajo =");
  Serial.println (estadoBotonAbajo);
  Serial.println ("estado boton derecha =");
  Serial.println (estadoBotonDerecha);
  Serial.println ("estado boton izquierda =");
  Serial.println (estadoBotonIzquierda);
  Serial.println ("estado boton enter =");
  Serial.println (estadoBotonEnter);
  //  temperatura = bmp.getTemperatureSensor();
  now = millis();
  if (now - lastMeasure1 > interval_envio) {    ////envio el doble de lectura por si falla algun envio
    lastMeasure1 = now;/// cargo el valor actual de millis
    fun_envio_mqtt();///envio los valores por mqtt
  }
  if (now - lastMeasure2 > interval_leeo) {
    lastMeasure2 = now;/// cargo el valor actual de millis
    fun_entra(); ///ingreso los valores a la cola struct
  }
  temp = bmp.readTemperature();
  lecturaTiempo ();

  Serial.println ("temperatura=");
  Serial.println (temp);

  switch  (estadoMaquina) {
    case PRINCIPAL:
      lecturaBotonArriba = digitalRead (BOTON_ARRIBA);
      lecturaBotonAbajo = digitalRead (BOTON_ABAJO);
      lecturaBotonIzquierda = digitalRead (BOTON_IZQUIERDA);
      lecturaBotonDerecha = digitalRead (BOTON_DERECHA);
      lecturaBotonEnter = digitalRead (BOTON_ENTER);
      valorLdr = analogRead (PIN_LDR);
      //Serial.println (map(valorLdr, 4095, 0, 0, 100));
      valorHumedad = analogRead(pinSensorHumedad);

      if (temp > modifTemperatura) {
        digitalWrite (PIN_COOLER, HIGH);
        bot.sendMessage(CHAT_ID, "se paso la temperatura");
      }

       if (valorHumedad > modifHumedad) {
         digitalWrite (LED_1, HIGH);
         digitalWrite (LED_2, HIGH);
        }

        if (valorLdr > modifLdr) {
         digitalWrite (LED_3, HIGH);
        }

        if (temp > modifTemperatura && valorHumedad > modifHumedad && valorLdr > modifLdr) {
         digitalWrite (PIN_BUZZER, HIGH);
         delay (500);
         digitalWrite (PIN_BUZZER, LOW);
         delay (500);
        }

      lcd.setCursor(0, 0);
      lcd.print ("temp:");
      lcd.print(temp);

      lcd.setCursor (9, 0);
      lcd.print ("mirki<3");


      lcd.setCursor(0, 1);
      lcd.print ("hum:");
      lcd.print(map(valorHumedad, 4095, 0, 0, 100));


      lcd.setCursor(6, 1);
      lcd.print ("ldr:");
      lcd.print(map(valorLdr, 4095, 0, 0, 100));


      /*lcd.setCursor(0, 3);
      lcd.print ("GMT:");
      lcd.print(GMT_Normal);*/


      if (lecturaBotonArriba == LOW && estadoBotonArriba == 0) {
        estadoBotonArriba = 1;
      }
      if (lecturaBotonArriba == HIGH && estadoBotonArriba == 1) {
        estadoBotonArriba = 0;
        estadoMaquina = PASAJE_SETTINGS;
        lcd.clear();

      }

      break;


    case PASAJE_SETTINGS: //raro
      lecturaBotonArriba = digitalRead (BOTON_ARRIBA);
      lecturaBotonAbajo = digitalRead (BOTON_ABAJO);
      lecturaBotonIzquierda = digitalRead (BOTON_IZQUIERDA);
      lecturaBotonDerecha = digitalRead (BOTON_DERECHA);
      lecturaBotonEnter = digitalRead (BOTON_ENTER);

      lcd.setCursor(0, 0);
      lcd.print("PASANDO A PANTALLA DE SETTINGS");
      if (lecturaBotonEnter == LOW && estadoBotonEnter == 0) {
        estadoBotonEnter = 1;
      }
      if (lecturaBotonEnter == HIGH && estadoBotonEnter == 1) {
        estadoBotonEnter = 0;
        estadoMaquina = SETTINGS;
        contadorSettings = 1;
        lcd.clear();
        pantallaSettings();
      }

      break;

    case SETTINGS:
      lecturaBotonArriba = digitalRead (BOTON_ARRIBA);
      lecturaBotonAbajo = digitalRead (BOTON_ABAJO);
      lecturaBotonIzquierda = digitalRead (BOTON_IZQUIERDA);
      lecturaBotonDerecha = digitalRead (BOTON_DERECHA);
      lecturaBotonEnter = digitalRead (BOTON_ENTER);

      if (  lecturaBotonAbajo == LOW ) {
        estadoMaquina = RESTA_CONTADOR;
      }

      if (  lecturaBotonArriba == LOW) {
        estadoMaquina = SUMA_CONTADOR;
      }



      ////////////////////////////////////////////////////////////////////////////////////////////////////
      if (lecturaBotonEnter == LOW && estadoBotonEnter == 0) {
        estadoBotonEnter = 1;
      }
      if (lecturaBotonEnter == HIGH && estadoBotonEnter == 1) {

        preferencesTemp.putInt("temperatura", modifTemperatura );
        preferencesHum.putInt("humedad", modifHumedad);
        preferencesHoras.putInt ("horas", modifHoras);
        preferencesLdr.putInt ("ldr", modifLdr);

        preferencesTemp.begin("temperatura", modifTemperatura);// lo que quiero guardar
        preferencesHum.begin("humedad", modifHumedad);// lo que quiero guardar
        preferencesHoras.begin("horas", modifHoras);// lo que quiero guardar
        preferencesLdr.begin("ldr", modifLdr);// lo que quiero guardar

        estadoBotonEnter = 0;
        lcd.clear ();
        estadoMaquina = PRINCIPAL;
      }
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      if (contadorSettings == 1 && lecturaBotonDerecha == LOW && estadoBotonDerecha == 0) {
        estadoBotonDerecha = 1;
      }
      if (contadorSettings == 1 && lecturaBotonDerecha == HIGH && estadoBotonDerecha == 1) {
        estadoBotonDerecha = 0;
        modifTemperatura ++;
      }
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      if (contadorSettings == 2 && lecturaBotonDerecha == LOW && estadoBotonDerecha == 0) {
        estadoBotonDerecha = 1;
      }
      if (contadorSettings == 2 && lecturaBotonDerecha == HIGH && estadoBotonDerecha == 1) {
        estadoBotonDerecha = 0;
        modifLdr ++;
      }
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      if (contadorSettings == 3 && lecturaBotonDerecha == LOW && estadoBotonDerecha == 0) {
        estadoBotonDerecha == 1;
      }
      if (contadorSettings == 3 && lecturaBotonDerecha == HIGH && estadoBotonDerecha == 1) {
        estadoBotonDerecha = 0;
        GMT ++;
      }
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      if (contadorSettings == 4 && lecturaBotonDerecha == LOW && estadoBotonDerecha == 0) {
        estadoBotonDerecha = 1;
      }
      if (contadorSettings == 4 && lecturaBotonDerecha == HIGH && estadoBotonDerecha == 1) {
        estadoBotonDerecha = 0;
        modifHumedad ++;
      }
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      if (contadorSettings == 1 && lecturaBotonIzquierda == LOW && estadoBotonIzquierda == 0) {
        estadoBotonIzquierda == 1;
      }
      if (contadorSettings == 1 && lecturaBotonIzquierda == HIGH && estadoBotonIzquierda == 1) {
        estadoBotonIzquierda == 0;
        modifTemperatura = modifTemperatura - 1;
      }
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      if (contadorSettings == 2 && lecturaBotonIzquierda == LOW && estadoBotonIzquierda == 0) {
        estadoBotonIzquierda = 1;
      }
      if (contadorSettings == 2 && lecturaBotonIzquierda == HIGH && estadoBotonIzquierda == 1) {
        estadoBotonIzquierda = 0;
        modifLdr = modifLdr - 1;
      }
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      if (contadorSettings == 3 && lecturaBotonIzquierda == LOW && estadoBotonIzquierda == 0) {
        estadoBotonIzquierda = 1;
      }
      if (contadorSettings == 3 && lecturaBotonIzquierda == HIGH && estadoBotonIzquierda == 1) {
        estadoBotonIzquierda == 0;
        modifHumedad = modifHumedad - 1;
      }
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      if (contadorSettings == 4 && lecturaBotonIzquierda == LOW && estadoBotonIzquierda == 0) {
        estadoBotonIzquierda = 1;
      }
      if (contadorSettings == 4 && lecturaBotonIzquierda == HIGH && estadoBotonIzquierda == 1) {
        estadoBotonIzquierda = 0;
        GMT =  GMT-1;
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      Serial.println (GMT);
     
      break;

    case RESTA_CONTADOR:
      lecturaBotonAbajo = digitalRead (BOTON_ABAJO);

      if ( lecturaBotonAbajo == HIGH) {
        estadoBotonAbajo = 0;
        contadorSettings -= 1;
        lcd.clear();
        pantallaSettings();
        estadoMaquina = SETTINGS;
      }
      break;

    case SUMA_CONTADOR:
      lecturaBotonArriba = digitalRead (BOTON_ARRIBA);

      if ( lecturaBotonArriba == HIGH) {
        estadoBotonArriba = 0;
        contadorSettings = contadorSettings + 1;
        lcd.clear();
        pantallaSettings();
        estadoMaquina = SETTINGS;
      }
      break;
  }
}


void pantallaSettings() {

  lcd.setCursor(0, 0);
  lcd.print(modifTemperatura);
  if ( contadorSettings == 1) {
    lcd.print("<---");
  }
  lcd.setCursor(0, 1);
  lcd.print(modifLdr);
  if ( contadorSettings == 2) {
    lcd.print ("<---");
  }
  lcd.setCursor(0, 2);
  lcd.print(modifHumedad);
  if ( contadorSettings == 3) {
    lcd.print ("<---");
  }
  lcd.setCursor(0, 3);
  lcd.print(modifHoras);
  if ( contadorSettings == 4) {
    lcd.print ("<---");
  }
  if (contadorSettings <= 0) {
    contadorSettings = 1;
    lcd.print("<---");
  }

  if (contadorSettings >= 5) {
    contadorSettings = 4;
    lcd.print ("<---");
  }

}

void lecturaTiempo () {

  if (millis() > lastTimeBotRan + delayRobot) {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    while (numNewMessages) {
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }
    lastTimeBotRan = millis();
  }
}
void handleNewMessages(int numNewMessages) {
  Serial.println("Mensaje nuevo");
  Serial.println(String(numNewMessages));

  for (int i = 0; i < numNewMessages; i++) {
    // inicio de verificacion
    String chat_id = String(bot.messages[i].chat_id);
    if (chat_id != CHAT_ID) {  ////si el id no corresponde da error . en caso de que no se quiera comprobar el id se debe sacar esta parte
      bot.sendMessage(chat_id, "Unauthorized user", "");
      continue;
    }
    String text = bot.messages[i].text;
    Serial.println(text);
    String from_name = bot.messages[i].from_name;
    if (text == "/temperatura_actual") {
      bot.sendMessage(chat_id, (String)temp, "");
    }
  }
}

void setupmqtt()
{
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
  WiFi.onEvent(WiFiEvent);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USERNAME, MQTT_PASSWORD);
  connectToWifi();
}

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
}

void fun_envio_mqtt (){
  fun_saca ();////veo si hay valores nuevos
  if (flag_vacio == 0) ////si hay los envio
  {
    Serial.print("enviando");
    ////genero el string a enviar
    snprintf (mqtt_payload, 150, "%u&%ld&%.2f&%.2f&%.2f&%u", name_device, aux2.time, aux2.T1, aux2.H1, aux2.luz, aux2.Alarma); //random(10,50)
    aux2.time = 0; ///limpio valores
    aux2.T1 = 0;
    aux2.H1 = 0;
    aux2.luz = 0;
    aux2.Alarma = 0;
    Serial.print("Publish message: ");
    Serial.println(mqtt_payload);
    // Publishes Temperature and Humidity values
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB, 1, true, mqtt_payload);
  }
  else
  {
    Serial.println("no hay valores nuevos");
  }
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}///////////////////////////////////////////////////

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}
void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void fun_saca () {
  if (indice_saca != indice_entra)
  {
    aux2.time = datos_struct[indice_saca].time;
    aux2.T1 = datos_struct[indice_saca].T1;
    aux2.H1 = datos_struct[indice_saca].H1;
    aux2.luz = datos_struct[indice_saca].luz;
    aux2.Alarma = datos_struct[indice_saca].Alarma;
    flag_vacio = 0;

    Serial.println(indice_saca);
    if (indice_saca >= (valor_max_struct - 1))
    {
      indice_saca = 0;
    }
    else
    {
      indice_saca++;
    }
    Serial.print("saco valores de la struct isaca:");
    Serial.println(indice_saca);
  }
  else
  {
    flag_vacio = 1; ///// no hay datos
  }
  return ;
}



void fun_entra (void)
{
  if (indice_entra >= valor_max_struct)
  {
    indice_entra = 0; ///si llego al maximo de la cola se vuelve a cero
  }
  //////////// timestamp/////// consigo la hora
  Serial.print("> NTP Time:");
  timestamp =  time(NULL);
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;  //// si no puede conseguir la hora
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  ///////////////////////// fin de consigo la hora
  datos_struct[indice_entra].time = timestamp;
  datos_struct[indice_entra].T1 = temp; /// leeo los datos //aca va la funcion de cada sensor /// temperaturs
  datos_struct[indice_entra].H1 = valorHumedad; //// se puede pasar por un parametro valor entre 0 y 100
  datos_struct[indice_entra].luz = valorLdr;
  datos_struct[indice_entra].Alarma = estadoBuzzer; //supera umbral y se pone en 1                          // valores de mis sensores.
  indice_entra++;
  Serial.print("saco valores de la struct ientra");
  Serial.println(indice_entra);
}
