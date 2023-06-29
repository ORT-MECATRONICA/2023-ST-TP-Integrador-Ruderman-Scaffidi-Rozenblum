//Ruderman-Rozenblum-Scaffidi

#define LDR 17
#define LED1 18
#define LED2 19
#define LED3 20

int LDR_Val = 0;     /*Variable to store photoresistor value*/     /*Analogue Input for photoresistor*/
String lectura;


void setup() {
  Serial.begin(9600);
  pinMode(LED1, OUTPUT);
  pinMode (LED2, OUTPUT);
  pinMode (LED3, OUTPUT);
}
void loop() {
  LDR_Val = analogRead(LDR);
  Serial.print("LDR Output Value: ");
  Serial.println(LDR_Val);
  lectura = Serial.read ();
  if (LDR_Val > 100) {
    Serial.println(" High intensity ");
    ledOff ();
  }
  else {

    Serial.println("LOW Intensity ");
    ledOn ();
  }
  if (lectura == "on") {
    ledOn();
  }
  if (lectura == "off") {
    ledOff();
  }
}

void ledOn() {
  digitalWrite (LED1, HIGH);
  digitalWrite (LED2, HIGH);
  digitalWrite (LED3, HIGH);
}
void ledOff() {
  digitalWrite (LED1, LOW);
  digitalWrite (LED2, LOW);
  digitalWrite (LED3, LOW);
}
