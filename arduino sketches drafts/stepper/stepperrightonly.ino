const int dirPin = 3;   // Direction pin
const int stepPin = 2;  // Step pin
const int stepsPerRevolution = 1200; // For example, 200 steps per full revolution

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  digitalWrite(dirPin, HIGH); // Set direction (LOW or HIGH depending on your setup)

  moveSteps(stepsPerRevolution);  // Move one full revolution
}

void loop() {
  // Do nothing (or repeat movement if desired)
}

// Function to move a specific number of steps
void moveSteps(int steps) {
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500); // Control speed
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }
}