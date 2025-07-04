#include <Arduino.h>
#include <ESP32Servo.h>

// Servo objects
Servo servo1;
Servo servo2;

// Servo pins
const int SERVO1_PIN = 18;
const int SERVO2_PIN = 19;

// TB6612 Motor Driver pins
const int STBY_PIN = 5;

// Motor A (PUMP1)
const int PWMA_PIN = 21;
const int AIN1_PIN = 16;
const int AIN2_PIN = 17;

// Motor B (PUMP2)
const int PWMB_PIN = 22;
const int BIN1_PIN = 25;
const int BIN2_PIN = 26;

// Motor C (PUMP3) - Using second TB6612 or additional pins
const int PWMC_PIN = 27;
const int CIN1_PIN = 32;
const int CIN2_PIN = 33;

// Motor D (PUMP4)
const int PWMD_PIN = 23;
const int DIN1_PIN = 12;
const int DIN2_PIN = 13;

// PWM properties
const int PWM_FREQ = 1000;
const int PWM_RESOLUTION = 8;

void setupTB6612() {
  // Set TB6612 pins as outputs
  pinMode(STBY_PIN, OUTPUT);
  
  // Motor A pins
  pinMode(PWMA_PIN, OUTPUT);
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  
  // Motor B pins
  pinMode(PWMB_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  
  // Motor C pins
  pinMode(PWMC_PIN, OUTPUT);
  pinMode(CIN1_PIN, OUTPUT);
  pinMode(CIN2_PIN, OUTPUT);
  
  // Motor D pins
  pinMode(PWMD_PIN, OUTPUT);
  pinMode(DIN1_PIN, OUTPUT);
  pinMode(DIN2_PIN, OUTPUT);
  
  // Enable TB6612
  digitalWrite(STBY_PIN, HIGH);
  
  // Setup PWM channels
  ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(3, PWM_FREQ, PWM_RESOLUTION);
  
  ledcAttachPin(PWMA_PIN, 0);
  ledcAttachPin(PWMB_PIN, 1);
  ledcAttachPin(PWMC_PIN, 2);
  ledcAttachPin(PWMD_PIN, 3);
}

void controlPump(int pumpNumber, int speed, bool direction) {
  if (speed < 0) speed = 0;
  if (speed > 255) speed = 255;
  
  switch (pumpNumber) {
    case 1: // PUMP1
      digitalWrite(AIN1_PIN, direction);
      digitalWrite(AIN2_PIN, !direction);
      ledcWrite(0, speed);
      break;
      
    case 2: // PUMP2
      digitalWrite(BIN1_PIN, direction);
      digitalWrite(BIN2_PIN, !direction);
      ledcWrite(1, speed);
      break;
      
    case 3: // PUMP3
      digitalWrite(CIN1_PIN, direction);
      digitalWrite(CIN2_PIN, !direction);
      ledcWrite(2, speed);
      break;
      
    case 4: // PUMP4
      digitalWrite(DIN1_PIN, direction);
      digitalWrite(DIN2_PIN, !direction);
      ledcWrite(3, speed);
      break;
  }
}

void stopPump(int pumpNumber) {
  switch (pumpNumber) {
    case 1:
      digitalWrite(AIN1_PIN, LOW);
      digitalWrite(AIN2_PIN, LOW);
      ledcWrite(0, 0);
      break;
      
    case 2:
      digitalWrite(BIN1_PIN, LOW);
      digitalWrite(BIN2_PIN, LOW);
      ledcWrite(1, 0);
      break;
      
    case 3:
      digitalWrite(CIN1_PIN, LOW);
      digitalWrite(CIN2_PIN, LOW);
      ledcWrite(2, 0);
      break;
      
    case 4:
      digitalWrite(DIN1_PIN, LOW);
      digitalWrite(DIN2_PIN, LOW);
      ledcWrite(3, 0);
      break;
  }
}

void stopAllPumps() {
  for (int i = 1; i <= 4; i++) {
    stopPump(i);
  }
}

void setServoPosition(int servoNumber, int angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  
  if (servoNumber == 1) {
    servo1.write(angle);
  } else if (servoNumber == 2) {
    servo2.write(angle);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Water Marbling Robot - Starting...");
  
  // Initialize servos
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  
  // Set servos to center position
  servo1.write(90);
  servo2.write(90);
  
  // Initialize TB6612
  setupTB6612();
  
  // Stop all pumps initially
  stopAllPumps();
  
  Serial.println("Initialization complete!");
}

void loop() {
  // Example control sequence
  Serial.println("Starting demo sequence...");
  
  // Move servos to different positions
  setServoPosition(1, 45);
  setServoPosition(2, 135);
  delay(1000);
  
  // Start pump 1 forward
  controlPump(1, 150, true);
  delay(2000);
  
  // Start pump 2 reverse
  controlPump(2, 100, false);
  delay(2000);
  
  // Move servos back to center
  setServoPosition(1, 90);
  setServoPosition(2, 90);
  
  // Start pumps 3 and 4
  controlPump(3, 120, true);
  controlPump(4, 80, true);
  delay(3000);
  
  // Stop all pumps
  stopAllPumps();
  
  Serial.println("Demo sequence complete. Waiting...");
  delay(5000);
}
