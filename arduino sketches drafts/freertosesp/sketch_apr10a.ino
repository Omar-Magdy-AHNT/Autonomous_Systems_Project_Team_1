#define LED_PIN 2  // Most ESP32 boards have an onboard LED on GPIO 2
#define irPin = 4;  // IR sensor connected to digital pin 4
SemaphoreHandle_t throttles;
SemaphoreHandle_t steerings;
// Variables to store state
int currentState = 0;
int previousState = 0;
unsigned long count = 0;
int throttle = 0;  // Shared integer 1
int steering = 0;  // Shared integer 2

// This is a normal function that toggles the LED
void blinkLED() {
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}
void setup() {
  Serial.begin(115200);           // Start Serial Monitor
  pinMode(LED_PIN, OUTPUT);       // Set the LED pin as output
  pinMode(irPin, INPUT);  // Set IR pin as input
  throttle = xSemaphoreCreateBinary();
  steering = xSemaphoreCreateBinary();
    if (throttle == NULL || steering == NULL) {
    Serial.println("Failed to create semaphores!");
    while (true);  // Halt execution if semaphore creation fails
  }
  xSemaphoreGive(throttle);
  xSemaphoreGive(steering);
  // Create a FreeRTOS task pinned to a specific cores
  xTaskCreatePinnedToCore(
    imu,        // ← Task function (the function that will run in background)
    "imu",      // ← Name of the task (optional, useful for debugging)
    10000,            // ← Stack size in words (not bytes) → 10000 words × 4 = ~40 KB
    NULL,             // ← Parameters passed to the task (not needed here, so NULL)
    1,                // ← Priority (1 = low, 2+ = higher priority)
    NULL,             // ← Task handle (used if you want to control the task later)
    1                 // ← Core number (0 or 1) → 0 = Core 0, 1 = Core 1
  );
  xTaskCreatePinnedToCore(
    encoder,       // The task function
    "encoder",     // Name of the task (useful for debugging)
    10000,              // Stack size (in words)
    NULL,               // Task parameters (none in this case)
    1,                  // Task priority (1 is low priority)
    NULL,               // Task handle (not needed)
    0                   // Pin the task to Core 0
  );
  xTaskCreatePinnedToCore(
    read, 
    "read", 
    10000, 
    NULL,
    1, 
    NULL, 
    0
  );  
  xTaskCreatePinnedToCore(
    motor, 
    "motor", 
    10000, 
    NULL, 
    1, 
    NULL, 
    1
  );
  xTaskCreatePinnedToCore(
    steering, 
    "steering", 
    10000, 
    NULL, 
    1, 
    NULL, 
    1
  );
}
void loop() {
  // Nothing here, the task handles everything
}

void read(void *pvParameters) {
  while (true) {
    // Read input from serial (assuming it's a string of two integers)
    if (Serial.available() > 0) {
      String input = Serial.readString();  // Read the input string from serial

      // Parse the string (assuming input format is "integer1 integer2")
      int tempInteger1 = 0, tempInteger2 = 0;
      int itemsRead = sscanf(input.c_str(), "%d %d", &tempInteger1, &tempInteger2);

      // If the string has both integers
      if (itemsRead == 2) {
        // Take semaphore1 to safely update integer1
        if (xSemaphoreTake(throttle, portMAX_DELAY)) {
          throttle = tempInteger1;  // Update integer1

          xSemaphoreGive(throttles);
        }

        // Take semaphore2 to safely update integer2
        if (xSemaphoreTake(steerings, portMAX_DELAY)) {
          steering = tempInteger2;  // Update integer2


          // Give semaphore2 back
          xSemaphoreGive(steerings);
        }
      }
    }

    // Optional: Add delay or other logic if needed
    vTaskDelay(100 / portTICK_PERIOD_MS);  // Adjust delay as needed
  }
}

// Task 2: This task will use both integers
void motor(void *pvParameters) {
  while (true) {
    // Take semaphore1 to safely read integer1
    if (xSemaphoreTake(throttles, portMAX_DELAY)) {
      // Use integer1 (do something with it)
      Serial.print("Task 2 is using integer1: ");
      Serial.println(throttle);

      // Give semaphore1 back
      xSemaphoreGive(throttles);
    }

    // Optional: Add delay or other logic if needed
    vTaskDelay(100 / portTICK_PERIOD_MS);  // Adjust delay as needed
  }
}
void steer(void *pvParameters){
  while(true){
    if(xSemaphoreTake(steerings, portMAX_DELAY)){

      xSemaphoreGive(steerings)
    }
  }
 // Optional: Add delay or other logic if needed
    vTaskDelay(100 / portTICK_PERIOD_MS);  // Adjust delay as needed
  }
}
// This is the FreeRTOS task function
void imu(void *pvParameters) {
  while (true) {
    blinkLED();
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay for 1000ms (1 second)
  }
}
// Task function that reads the IR sensor and counts
void encoder(void *pvParameters) {
  while (true) {
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

    vTaskDelay(50 / portTICK_PERIOD_MS);  // Small delay to debounce the IR sensor
  }
}