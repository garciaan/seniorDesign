int LEDPORT = 7;
int POTPORT = A0;
int InV = 0;
int OutV = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LEDPORT, OUTPUT);   // sets the pin as output
  

}

void loop() {
  // put your main code here, to run repeatedly:
  // read the analog in value:
  InV = analogRead(POTPORT);
  // map it to the range of the analog out:
  OutV = map(InV, 0, 1023, 0, 255);
  // change the analog out value:
  analogWrite(LEDPORT, OutV);

  // print the results to the serial monitor:
  Serial.print("sensor = ");
  Serial.print(InV);
  Serial.print("\t output = ");
  Serial.println(OutV);
  
  delay(2);
  
  
}
