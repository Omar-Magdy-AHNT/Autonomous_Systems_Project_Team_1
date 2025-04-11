// Define the pin you want to read
const int inputPin = 7;  // Change this to your desired pin

void setup() {
  // Start the Serial communication
  Serial.begin(9600);

  // Set the pin mode
  pinMode(inputPin, INPUT);
}

void loop() {
  // Read the pin state
  int state = digitalRead(inputPin);

  // Print the result
  if (state == HIGH) {
    Serial.println("Pin is HIGH");
  } else {
    Serial.println("Pin is LOW");
  }

  // Add a small delay to avoid flooding the Serial Monitor
  delay(500);
}
