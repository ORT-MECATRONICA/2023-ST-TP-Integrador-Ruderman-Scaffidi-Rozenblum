
#define BUZZER_PIN 22
#define BUTTON1_PIN 23
#define BUTTON2_PIN 24
#define BUTTON3_PIN 25
#define BUTTON4_PIN 26
#define BUTTON5_PIN 27

void setup() {
  Serial.begin(9600);                // initialize serial
  pinMode(BUTTON1_PIN, INPUT_PULLUP); // set ESP32 pin to input pull-up mode
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(BUTTON3_PIN, INPUT_PULLUP);
  pinMode(BUTTON4_PIN, INPUT_PULLUP);
  pinMode(BUTTON5_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);       // set ESP32 pin to output mode
}

void loop() {
  int buttonState1 = digitalRead(BUTTON1_PIN); 

  if (buttonState1 == LOW) {
    Serial.println("El botón 1 está presionado");
    digitalWrite(BUZZER_PIN, HIGH); 
  }
  else
  if (buttonState1 == HIGH) {
    Serial.println("El botón 1 no está presionado");
    digitalWrite(BUZZER_PIN, LOW);  
  }
  if (digitalRead(BUTTON2_PIN)==LOW){
    Serial.println ("Botón 2 presionado");
  }
  if (digitalRead(BUTTON3_PIN)==LOW){
    Serial.println ("Botón 3 presionado");
  }
  if (digitalRead(BUTTON4_PIN)==LOW){
    Serial.println ("Botón 4 presionado");
  }
  if (digitalRead(BUTTON5_PIN)==LOW){
    Serial.println ("Botón 5 presionado");
  }
}
