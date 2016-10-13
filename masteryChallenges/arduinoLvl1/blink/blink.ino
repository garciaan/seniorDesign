int LEDPORT = 7;

void setup() {
  // put your setup code here, to run once:
  pinMode(LEDPORT,OUTPUT);
  

}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LEDPORT,HIGH);
  delay(500);
  digitalWrite(LEDPORT,LOW);
  delay(500);
}
