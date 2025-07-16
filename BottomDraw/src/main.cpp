#include <Arduino.h>
#include <math.h>

// PWM 参数设置
const int freq = 50;  // 50Hz for servo
const int resolution = 16; // 16 bit resolution
// PWM channels
const int pwmChannel1 = 0;
const int pwmChannel2 = 1;
const int pwmChannel3 = 2;

const int SERVO1_PIN = 13;
const int SERVO2_PIN = 12;
const int SERVO3_PIN = 14;

const float L1 = 35.0;  // Left arm length (mm)
const float L2 = 55.5;  // Right arm length (mm)
const float L3 = 13.2;  // Pen holder length (mm)

struct Point {
  float x, y;
};

struct Angles {
  float theta1, theta2, theta3;
};

// 将角度转换为 PWM 值（16位分辨率）
uint32_t angleToPWM(int angle) {
  // 舵机通常需要 0.5ms 到 2.5ms 的脉冲宽度
  // 对于 50Hz，周期是 20ms (20000 微秒)
  // 16位分辨率下，65536 对应 20ms
  // 因此 0.5ms 对应 1638，2.5ms 对应 8192
  return map(angle, 0, 180, 1638, 8192);
}

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

  uint32_t pwm1 = angleToPWM(servo1_angle);
  uint32_t pwm2 = angleToPWM(servo2_angle);

  ledcWrite(pwmChannel1, pwm1);
  ledcWrite(pwmChannel2, pwm2);

  delay(20);
}

void penUp() {
  uint32_t pwm3 = angleToPWM(90);
  ledcWrite(pwmChannel3, pwm3);
  delay(300);
}

void penDown() {
  uint32_t pwm3 = angleToPWM(0);
  ledcWrite(pwmChannel3, pwm3);
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
  
  // 配置 LED PWM 功能
  ledcSetup(pwmChannel1, freq, resolution);
  ledcSetup(pwmChannel2, freq, resolution);
  ledcSetup(pwmChannel3, freq, resolution);
  
  // 将通道与对应的引脚连接
  ledcAttachPin(SERVO1_PIN, pwmChannel1);
  ledcAttachPin(SERVO2_PIN, pwmChannel2);
  ledcAttachPin(SERVO3_PIN, pwmChannel3);
  
  Serial.println("PWM Channels initialized");
  
  // 测试每个引脚是否能输出
  Serial.println("Testing PWM output...");
  
  delay(2000);  // Wait for system to initialize
  Serial.println("Setup complete");
}

void loop() {
  // 测试第一个舵机
  Serial.println("Testing PWM 1...");
  for (int angle = 0; angle <= 180; angle += 45) {
    uint32_t pwm = angleToPWM(angle);
    Serial.printf("Angle: %d, PWM: %d\n", angle, pwm);
    ledcWrite(pwmChannel1, pwm);
    delay(1000);
  }
  
  // 测试第二个舵机
  Serial.println("Testing PWM 2...");
  for (int angle = 0; angle <= 180; angle += 45) {
    uint32_t pwm = angleToPWM(angle);
    Serial.printf("Angle: %d, PWM: %d\n", angle, pwm);
    ledcWrite(pwmChannel2, pwm);
    delay(1000);
  }
  
  // 测试第三个舵机
  Serial.println("Testing PWM 3...");
  for (int angle = 0; angle <= 180; angle += 45) {
    uint32_t pwm = angleToPWM(angle);
    Serial.printf("Angle: %d, PWM: %d\n", angle, pwm);
    ledcWrite(pwmChannel3, pwm);
    delay(1000);
  }
  
  delay(3000);
}