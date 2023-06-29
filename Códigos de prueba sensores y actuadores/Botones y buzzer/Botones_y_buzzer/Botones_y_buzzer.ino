#define BUTTON_PIN 23
#define BUZZER_PIN 22

void setup() {
  Serial.begin(9600);                // initialize serial
  pinMode(BUTTON_PIN, INPUT_PULLUP); // set ESP32 pin to input pull-up mode
  pinMode(BUZZER_PIN, OUTPUT);       // set ESP32 pin to output mode
}

void loop() {
  int buttonState = digitalRead(BUTTON_PIN); 

  if (buttonState == LOW) {
    Serial.println("El botón está presionado");
    digitalWrite(BUZZER_PIN, HIGH); 
  }
  else
  if (buttonState == HIGH) {
    Serial.println("El botón no está presionado");
    digitalWrite(BUZZER_PIN, LOW);  
  }
}
