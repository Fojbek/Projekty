int triggerPin = 12;
int echoPin = 11;
float pingTravelTime;
float distance;
void setup() {
  // put your setup code here, to run once:
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(triggerPin, LOW);  //sending pin
  delayMicroseconds(10);          //it has to be really small
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);  //small as well
  digitalWrite(triggerPin, LOW);
  pingTravelTime = pulseIn(echoPin, HIGH);  //reads how long the ping goes
  delay(250);
  distance = (pingTravelTime * pow(10, -6) * 34300) / 2;
  Serial.print(0);
  Serial.print(",");
  Serial.print(distance);
  Serial.print(",");
  Serial.println(10); //open the serial plotter
}
