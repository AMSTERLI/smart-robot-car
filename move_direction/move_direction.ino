/*
 * 2026-2-5 (Bingle Li)
 * Straight Line Driving Example (PID Control based on MPU6050)
 * Suitable for TB6612 Motor Driver Version
 * * Main Features:
 * - Get Yaw angle via MPU6050
 * - Use PID control algorithm to adjust left/right wheel speeds to maintain a straight line
 * * Hardware Connections:
 * - Motor AIN_1 -> Arduino Pin 7
 * - Motor BIN_1 -> Arduino Pin 8
 * - Motor PWMA  -> Arduino Pin 5 (PWM)
 * - Motor PWMB  -> Arduino Pin 6 (PWM)
 * - Motor STBY  -> Arduino Pin 3 (High level enable)
 * - MPU6050 SDA -> Arduino A4
 * - MPU6050 SCL -> Arduino A5
*/

#include <Wire.h>
#include <MPU6050.h> 

// --- Hardware Definitions (TB6612 Version) ---
#define PIN_Motor_PWMA 5
#define PIN_Motor_PWMB 6
#define PIN_Motor_AIN_1 7  // [Changed]
#define PIN_Motor_BIN_1 8  // [Changed]
#define PIN_Motor_STBY 3   // [New]

#define direction_just true
#define direction_back false

MPU6050 accelgyro;
int16_t gz;
float yaw = 0;
float yaw_target = 0;
unsigned long lastTime = 0;
float dt = 0;

// PID Parameters (Source: 1001-1002)
uint8_t Kp = 10;           
uint8_t UpperLimit = 255;  

void setup() {
  // 1. Initialize motor pins
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);
  
  // Enable TB6612
  digitalWrite(PIN_Motor_STBY, HIGH);

  // 2. Initialize MPU6050 (Source: 847-866)
  Wire.begin();
  Serial.begin(9600);
  accelgyro.initialize();
  
  // 3. Simple calibration (Source: 872-881)
  // Place the car still for a few seconds after power-up to calibrate zero offset
  delay(2000); 
}

// Get Yaw Angle (Source: 895-909)
void updateYaw() {
  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0;
  lastTime = now;
  
  gz = accelgyro.getRotationZ();
  // 131.0 is the sensitivity of MPU6050
  float gyroz = (gz) / 131.0; 
  
  // Simple integration
  if (abs(gyroz) > 0.5) { 
     yaw += gyroz * dt;
  }
}

/*
 * Straight Line PID Control Logic
 * Formula Source: Source 1035, 1044
 */
void moveStraight(uint8_t baseSpeed) {
  updateYaw(); // Update current angle
  
  int error = yaw - yaw_target; 
  
  // Calculate left/right wheel speeds based on Demo4 formula
  // R = (Yaw - yaw_So) * Kp + speed
  // L = (yaw_So - Yaw) * Kp + speed
  
  int speed_R = baseSpeed + (error * Kp);
  int speed_L = baseSpeed - (error * Kp);

  // Speed limiting (Clamping) (Source: 1036-1052)
  if (speed_R > UpperLimit) speed_R = UpperLimit;
  if (speed_R < 0) speed_R = 0; // Prevent reversal or too low speed
  if (speed_R < 10) speed_R = 10; // Minimum startup speed

  if (speed_L > UpperLimit) speed_L = UpperLimit;
  if (speed_L < 0) speed_L = 0;
  if (speed_L < 10) speed_L = 10;

  // Drive motors (STBY already set HIGH in setup)
  digitalWrite(PIN_Motor_AIN_1, direction_just);
  analogWrite(PIN_Motor_PWMA, speed_R);
  
  digitalWrite(PIN_Motor_BIN_1, direction_just);
  analogWrite(PIN_Motor_PWMB, speed_L);
}

void loop() {
  // Keep driving straight
  moveStraight(150);
  delay(10); 
}