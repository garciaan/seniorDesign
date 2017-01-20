const int Z = A10;
const int Y = A9;
const int X = A8;

const int ZLed = 7;
const int YLed = 6;
const int XLed = 5;

float xZero = 330.0;
float yZero = 325.0;
float zZero = 408.0;
float xMax = 264;
float yMax = 257;
float zMax = 275;

float xDeg;
float yDef;
float zDeg;



void setup() {
  // put your setup code here, to run once:
  //pinMode(Z,INPUT);
  //pinMode(Y,INPUT);
  //pinMode(X,INPUT);
  pinMode(ZLed, OUTPUT);
  pinMode(YLed, OUTPUT);
  pinMode(XLed, OUTPUT);
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  int xVal = analogRead(X);
  int yVal = analogRead(Y);
  int zVal = analogRead(Z);
  int ymap = map(yVal,yZero,yMax,0,90);
  ymap = map(ymap,0,90,0,256);
  if (ymap < 10){
    ymap = 0;
  }
  /*
  Serial.print("X Degrees: ");
  Serial.print(xVal);
  Serial.print("\t Y Degrees: ");
  Serial.print(yVal);
  Serial.print("\t Z Degrees: ");
  Serial.print(zVal);
  Serial.println("");
  */
  
  analogWrite(YLed,ymap);
  Serial.print("X Degrees: ");
  Serial.print(map(xVal,xZero,xMax,0,90));
  Serial.print("\t Y Degrees: ");
  Serial.print(map(yVal,yZero,yMax,0,90));
  Serial.print("\t Z Degrees: ");
  Serial.print(map(zVal,zZero,zMax,0,180));
  Serial.println("");
  delay(2);
  

}
