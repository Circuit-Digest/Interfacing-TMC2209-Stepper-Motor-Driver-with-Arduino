// Pin Definitions
#define EN_PIN 8    // LOW: Driver enabled, HIGH: Driver disabled
#define STEP_PIN 9  // Step on the rising edge
#define DIR_PIN 10  // Set stepping direction

int noOfSteps = 500;           // Number of steps to move in each direction
int microSecondsDelay = 1000;  // Delay in microseconds between each step

void setup() {
  // Configure pin modes
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  // Initialize pin states
  digitalWrite(EN_PIN, LOW);   // Enable the driver
  digitalWrite(DIR_PIN, LOW);  // Set initial direction
}

void loop() {
  // Move motor in one direction
  digitalWrite(DIR_PIN, LOW);  // Set direction to LOW
  moveSteps(noOfSteps);

  // Move motor in the opposite direction
  digitalWrite(DIR_PIN, HIGH); // Set direction to HIGH
  moveSteps(noOfSteps);
}

void moveSteps(int steps) {
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(microSecondsDelay);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(microSecondsDelay);
  }
}
