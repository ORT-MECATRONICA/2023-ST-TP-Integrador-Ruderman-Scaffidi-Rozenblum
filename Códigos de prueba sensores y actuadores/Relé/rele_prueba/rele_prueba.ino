#define PIN_RELE 29

void setup() {

  pinMode (PIN_RELE, OUTPUT);

}

void loop() {

  digitalWrite (PIN_RELE, HIGH);
  delay(100);
  digitalWrite (PIN_RELE, LOW);

}
