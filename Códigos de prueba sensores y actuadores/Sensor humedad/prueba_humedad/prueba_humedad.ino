//Ruderman-Scaffidi-Rozenblum

#define PIN_HUMEDAD 12 //poner PIN correspondiente

void setup() {

      Serial.begin (9600);
      
}

void loop() {

    int valorSensor = analogRead(PIN_HUMEDAD);
    Serial.print ("humedad:");
    Serial.println (valorSensor);
    delay (450);

}
