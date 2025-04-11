// Pin definitions
const int irPin = 2;  // IR sensor connected to digital pin 2

// Vardeviables to store state
int currentState = 0;
int previousState = 0;
unsigned long count = 0;

void setup() {
  Serial.begin(9600);     // Initialize serial communication
  pinMode(irPin, INPUT);  // Set IR pin as input
}

void loop() {
  // Read the current state of the IR sensor
  currentState = digitalRead(irPin);

  // Check if IR just changed from LOW to HIGH (rising edge)
  if (currentState == HIGH && previousState == LOW) {
    count++;  // Increment count
    Serial.print("Count: ");
    Serial.println(count);
  }

  // Update previous state for next loop iteration
  previousState = currentState;
}
