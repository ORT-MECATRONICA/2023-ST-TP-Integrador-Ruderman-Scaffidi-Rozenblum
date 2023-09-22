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
#define PIN_RELE
#define PIN_BUZZER
#define BOTON_ALARMA


#define COMPROBACION 1
#define ALARMA 2
#define CHECKBOTON 3
#define AVISO 4

int temp_vu = ;
int humedad_vu = ;

int estado_alarma = 1;
bool flagBotonAlarma;


WiFiClientSecure client;
UniversalTelegramBot bot(BOTtoken, client);

int botRequestDelay = 1000; /// intervalo
unsigned long lastTimeBotRan; /// ultimo tiempo

UniversalTelegramBot bot(BOTtoken, client);

void setup() {
  
  Serial.begin(9600);
  pinMode(LED_ROJO, OUTPUT);
  pinMode(LED_AMARILLO, INPUT_PULLUP);
  pinMode (LED_VERDE, OUTPUT);
  pinMode (PIN_RELE, OUTPUT);
  pinMode (PIN_BUZZER, OUTPUT);
  pinMode (BOTON_ALARMA, INPUT_PULLUP);

  digitalWrite (LED_VERDE, HIGH);
  
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

            if (temp > temp_vu || humedad < humedad_vu){
              estado_alarma = ALARMA;
              bot.SendMessage(chat_id, "revisar huerta, la humedad o la temperatura sufrieron un cambio", "");
              digitalWrite (LED_ROJO, HIGH);
              digitalWrite (LED_AMARILLO, LOW);
              digitalWrite (LED_VERDE, LOW);
              digitalWrite (PIN_RELE, HIGH); 
              digitalWrite (PIN_BUZZER, HIGH); 
              flagBotonAlarma = 0;   
            }
       break;

       case ALARMA:

            if (temp < temp_vu || humedad > humedad_vu){
              estado_alarma = AVISO;
              digitalWrite (LED_ROJO, LOW);
              digitalWrite (LED_AMARILLO, HIGH);
              digitalWrite (LED_VERDE, LOW);
              digitalWrite (PIN_RELE, LOW);  
              flagBotonAlarma = 1;               
            }

            if (digitalRead (BOTON_ALARMA)==LOW){
              estado_alarma = CHECKBOTON;
            }

       break;

       case AVISO: 
            
            if (temp > temp_vu || humedad < humedad_vu){
              estado_alarma = ALARMA;
              bot.SendMessage(chat_id, "revisar huerta, la humedad o la temperatura sufrieron un cambio", "");
              digitalWrite (LED_ROJO, HIGH);
              digitalWrite (LED_AMARILLO, LOW);
              digitalWrite (LED_VERDE, LOW);
              digitalWrite (PIN_RELE, HIGH); 
              digitalWrite (PIN_BUZZER, HIGH); 
              flagBotonAlarma = 0;              
            }     

            if (digitalRead (BOTON_ALARMA)==LOW){
              estado_alarma = CHECKBOTON;
              digitalWrite (LED_AMARILLO, LOW);
            }

            break;

        case CHECKBOTON:

            if (flagBotonAlarma == 0){
              if(digitalRead (BOTON_ALARMA) == HIGH){
                estado_alarma = ALARMA;
                digitalWrite (PIN_BUZZER; LOW);
              }
            }

            if (flagBotonAlarma == 1){
              if (digitalRead (BOTON_ALARMA) == HIGH){
                estado_alarma = COMPROBACION;
                digitalWrite (LED_AMARILLO, LOW);
                digitalWrite (LED_VERDE, HIGH);
              }
            }

         break;


}

void handleNewMessages(int numNewMessages) {
  Serial.println("handleNewMessages");
  Serial.println(String(numNewMessages));

  for (int i = 0; i < numNewMessages; i++) {
    // Chat id of the requester
    String chat_id = String(bot.messages[i].chat_id);
    if (chat_id != CHAT_ID) {
      bot.sendMessage(chat_id, "Unauthorized user", "");
      continue;
    }

    // Print the received message
    String text = bot.messages[i].text;
    Serial.println(text);

    String from_name = bot.messages[i].from_name;
