#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>   // Universal Telegram Bot Library written by Brian Lough: https://github.com/witnessmenow/Universal-Arduino-Telegram-Bot
#include <ArduinoJson.h>

const char* ssid = "ORT-IoT";
const char* password = "OrtIOTnew22$2";

#define BOTtoken "6279852188:AAEMOYa7gNj_960VZFXYhnueMvrXYuFrzus"  // cambiar por token 
#define CHAT_ID "-609914570" ///

#define LED_ROJO
#define LED_AMARILLO
#define LED_VERDE

#define COMPROBACION 1
#define ALARMA 2
#define CHECKBOTON 3
#define AVISO 4

int umbral_vu = ;
int humedad_vu = ;

int estado_alarma = 1;
bool flagSubir;
bool flagBajar;

WiFiClientSecure client;
UniversalTelegramBot bot(BOTtoken, client);

int botRequestDelay = 1000; /// intervalo
unsigned long lastTimeBotRan; /// ultimo tiempo

UniversalTelegramBot bot(BOTtoken, client);

void setup() {
  
  Serial.begin(9600);
  pinMode(LED_ROJO, INPUT_PULLUP);
  pinMode(LED_AMARILLO, INPUT_PULLUP);
  pinMode (LED_VERDE, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  client.setCACert(TELEGRAM_CERTIFICATE_ROOT); 

    while (WiFi.status() != WL_CONNECTED) {   
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
  
  Serial.println(WiFi.localIP());
  bot.sendMessage(CHAT_ID, "Bot Hola mundo", "");

  
}

void loop() {

  switch (estado_alarma){
    
      case COMPROBACION:

            if (temp >      
           
  } 



}
