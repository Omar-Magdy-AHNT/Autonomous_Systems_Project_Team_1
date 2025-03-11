void setup() {
  // put your setup code here, to run once:
 Serial.begin(9600);

  pinMode(3, OUTPUT); //pwm1
  pinMode(4, OUTPUT); //direction1
  pinMode(5, OUTPUT); //pwm
  pinMode(6, OUTPUT); //direction0
  
}

void loop() {
  digitalWrite(4, HIGH);
  analogWrite(3, 0);
  digitalWrite(6, HIGH);
  analogWrite(5, 0);
  delay(1000);
  analogWrite(3,125);
  analogWrite(5,125);
  delay(3000);
  digitalWrite(4, LOW);
  analogWrite(3, 0);
  digitalWrite(6, LOW);
  analogWrite(5, 0);
  delay(1000);
  analogWrite(3,125);
  analogWrite(5,125);
  delay(3000);
}
