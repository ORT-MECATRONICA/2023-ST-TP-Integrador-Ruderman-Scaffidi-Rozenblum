

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>


// Detects whenever the door changed state
bool changeState = false;

// Holds reedswitch state (1=opened, 0=close)
bool state;
String doorState;

// Auxiliary variables (it will only detect changes that are 1500 milliseconds apart)
unsigned long previousMillis = 0; 
const long interval = 1500;

const char* ssid = "ORT-IoT";
const char* password = "OrtIOTnew22$2";

// Initialize Telegram BOT
#define BOTtoken "6279852188:AAEM0Ya7gNj_960VzFXYhnueMvrXYuFrzus"  // your Bot Token (Get from Botfather)

// Use @myidbot to find out the chat ID of an individual or a group
// Also note that you need to click "start" on a bot before it can
// message you
#define CHAT_ID "609914570"

WiFiClientSecure client;
UniversalTelegramBot bot(BOTtoken, client);


void setup() {
  // Serial port for debugging purposes
  Serial.begin(9600);  
  
  // Connect to Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  client.setCACert(TELEGRAM_CERTIFICATE_ROOT); // Add root certificate for api.telegram.org
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");  

  bot.sendMessage(CHAT_ID, "Bot started up", "");
}

void loop() {

      if (millis() > lastTimeBotRan + botRequestDelay) {
     lastTimeBotRan = millis();
  
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

    while(numNewMessages) {
      Serial.println("Veo los msj nuevos");
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }
   
  }
 
  }


void handleNewMessages(int numNewMessages) {
  Serial.println("Mensaje nuevo");
  Serial.println(String(numNewMessages));

  for (int i=0; i<numNewMessages; i++) {
    // inicio de verificacion
    String CHAT_ID = String(bot.messages[i].chat_id);
    
    ///fin de verificacion

    // imprime el msj recibido 
    String text = bot.messages[i].text;
    Serial.println(text);

    String from_name = bot.messages[i].from_name;

    if (text == "/nocapricho") {  
      
      bot.sendMessage(CHAT_ID, "voce quiere ensalada di fruta? ");

     
    }
