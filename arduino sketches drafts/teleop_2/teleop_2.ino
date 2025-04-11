
String incomingMessage;
float throttle = 0.0;
float steering = 0.0;

const int pwmPin1 = 9;   //PWM1
const int pwmPin2 = 10;  //PWM2

const int direc1 = 7; //Direction1
const int direc2 = 8; //Direction2

const int dirPin = 3; //Stepper Direction
const int stepPin = 2; //Step pin
const int maxSteps = 600;

float prevSteering = 0.0;
int prevSteps = 0;
int currSteps = 0;
int currStepPosition = 0;

void setup() {

Serial.begin(9600);

while (!Serial);

pinMode(pwmPin1, OUTPUT);
pinMode(direc1, OUTPUT);
pinMode(direc2, OUTPUT);

pinMode(stepPin, OUTPUT);
pinMode(dirPin, OUTPUT);

digitalWrite(dirPin, HIGH);

}

void loop() {

  if (Serial.available() > 0) {

    String incomingMessage = Serial.readStringUntil('\n');
    incomingMessage.trim();

    int t_index = incomingMessage.indexOf("T:");
    int s_index = incomingMessage.indexOf("S:");

    int comma_index = incomingMessage.indexOf(",");

    String throttleStr = incomingMessage.substring(t_index+2, comma_index);
    String steeringStr = incomingMessage.substring(s_index+2);

    throttle = throttleStr.toFloat();
    steering = steeringStr.toFloat();

    sendThrottle(throttle);
    sendSteering(steering);

  }
}

void sendThrottle(float value){
  int pwm = abs(value) * 255;

  if (value>0){
    digitalWrite(direc1, LOW);
    digitalWrite(direc2, LOW);
  }
  else if (value<0){
    digitalWrite(direc1, HIGH);
    digitalWrite(direc2, HIGH);

  }

  analogWrite(pwmPin1, pwm);
  analogWrite(pwmPin2, pwm);
}

void sendSteering(float value){

  int targetSteps = value * maxSteps;
  int stepDifference = targetSteps - currStepPosition;

  if (stepDifference == 0){
    return;
  }
  digitalWrite(dirPin, stepDifference > 0 ? HIGH : LOW);

  for (int i = 0; i < abs(stepDifference); i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500); // Adjust speed
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }

  currStepPosition = targetSteps;
  prevSteering = value;


  /*
  
  if(value == 0){
    return;
  }
  if (prevSteering == value){
    return;
  }
  
  if (prevSteering>0){

    if (prevSteering>value){
      currSteps = abs(prevSteps - (value*maxSteps));
      digitalWrite(dirPin, !digitalRead(dirPin));
    }
    if (prevSteering<value){
      currSteps = prevSteps - (value*maxSteps);
      digitalWrite(dirPin, digitalRead(dirPin));
    }
  }
  else if (prevSteering>0&&value>0){
    if (prevSteering<value){
      currSteps = abs(prevSteps - (value*maxSteps));
      digitalWrite(dirPin, !digitalRead(dirPin));
    }
    if (prevSteering>value){
      currSteps = prevSteps - (value*maxSteps);
      digitalWrite(dirPin, digitalRead(dirPin));
    }
  }

  for (int i = 0; i < currSteps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500); // Adjust speed here
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }



  prevSteering = value;
  prevSteps = currSteps;
  */


/*
  currSteps


  if (value == 0 || value == prevSteering){
    return;
  
  digitalWrite(dirPin, value > 0 ? HIGH : LOW);

  int stepsToMove = abs(value) * maxSteps;

  for (int i = 0; i < stepsToMove; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500); // Adjust speed here
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }
  prevSteering = value;
*/
}