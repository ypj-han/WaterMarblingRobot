#include <Arduino.h>
#include <ESP32Servo.h>
#include <math.h>

Servo servo1;  // Left arm servo
Servo servo2;  // Right arm servo  
Servo servo3;  // Pen lift servo

const int SERVO1_PIN = 18;
const int SERVO2_PIN = 19;
const int SERVO3_PIN = 21;

const float L1 = 35.0;  // Left arm length (mm)
const float L2 = 55.5;  // Right arm length (mm)
const float L3 = 13.2;  // Pen holder length (mm)

struct Point {
  float x, y;
};

struct Angles {
  float theta1, theta2, theta3;
};

Angles calculateAngles(float x, float y) {
  Angles angles;
  
  float d = sqrt(x*x + y*y);
  
  if (d > (L1 + L2) || d < abs(L1 - L2)) {
    angles.theta1 = 0;
    angles.theta2 = 0;
    angles.theta3 = 0;
    return angles;
  }
  
  float alpha = atan2(y, x);
  float beta = acos((L1*L1 + d*d - L2*L2) / (2*L1*d));
  float gamma = acos((L1*L1 + L2*L2 - d*d) / (2*L1*L2));
  
  angles.theta1 = alpha + beta;
  angles.theta2 = PI - gamma;
  angles.theta3 = 0;
  
  return angles;
}

void moveToPosition(float x, float y) {
  Angles angles = calculateAngles(x, y);
  
  int servo1_angle = constrain(degrees(angles.theta1), 0, 180);
  int servo2_angle = constrain(degrees(angles.theta2), 0, 180);
  
  servo1.write(servo1_angle);
  servo2.write(servo2_angle);
  
  delay(20);
}

void penUp() {
  servo3.write(90);
  delay(300);
}

void penDown() {
  servo3.write(0);
  delay(300);
}

void drawLine(float x1, float y1, float x2, float y2, int steps = 20) {
  penUp();
  moveToPosition(x1, y1);
  penDown();
  
  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;
    float x = x1 + t * (x2 - x1);
    float y = y1 + t * (y2 - y1);
    moveToPosition(x, y);
    delay(50);
  }
}

void drawCircle(float centerX, float centerY, float radius, int steps = 36) {
  penUp();
  moveToPosition(centerX + radius, centerY);
  penDown();
  
  for (int i = 0; i <= steps; i++) {
    float angle = 2 * PI * i / steps;
    float x = centerX + radius * cos(angle);
    float y = centerY + radius * sin(angle);
    moveToPosition(x, y);
    delay(50);
  }
}

void setup() {
  Serial.begin(115200);
  
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  
  penUp();
  moveToPosition(0, 60);
  
  Serial.println("PlotClock initialized");
}

void loop() {
  drawLine(-20, 40, 20, 40);
  delay(1000);
  
  drawLine(20, 40, 20, 80);
  delay(1000);
  
  drawLine(20, 80, -20, 80);
  delay(1000);
  
  drawLine(-20, 80, -20, 40);
  delay(1000);
  
  drawCircle(0, 60, 15);
  delay(2000);
}