/*
 * 2026-2-12
 * Advanced Smart Car: 
 * 1. PID Straight Line (MPU6050)
 * 2. Precise Turn by Angle (MPU6050)
 * 3. Ultrasonic Obstacle Avoidance
 */

#include <Wire.h>
#include <MPU6050.h>

// --- Hardware Pin Definitions (Unchanged) ---
#define PIN_Motor_PWMA 5
#define PIN_Motor_PWMB 6
#define PIN_Motor_AIN_1 7
#define PIN_Motor_BIN_1 8
#define PIN_Motor_STBY 3
#define TRIG_PIN 13
#define ECHO_PIN 12

// --- Parameters ---
MPU6050 accelgyro;
float yaw = 0;
float yaw_target = 0;
unsigned long lastTime = 0;

// PID Parameters
const uint8_t Kp = 10;           
const uint8_t BaseSpeed = 150;   
const uint8_t TurnSpeed = 180;   // Higher speed recommended for turning to overcome static friction
const uint8_t UpperLimit = 255;  

// Obstacle Avoidance Parameter
const int ObstacleDetection = 20; // Distance in cm

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize Motors
  pinMode(PIN_Motor_PWMA, OUTPUT); pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT); pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);
  digitalWrite(PIN_Motor_STBY, HIGH); // Enable the driver

  // Initialize Ultrasonic Sensor
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);

  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  accelgyro.initialize();
  
  // Simple Calibration: Keep still for 2 seconds after power-up to remove zero drift
  Serial.println("Calibrating...");
  delay(2000); 
  lastTime = millis();
}

// --- Core Functions ---

// 1. Read Ultrasonic Distance
unsigned int getDistance() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  return (unsigned int)pulseIn(ECHO_PIN, HIGH) / 58;
}

// 2. Update Current Yaw Angle (Core Algorithm)
void updateYaw() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;
  
  int16_t gz = accelgyro.getRotationZ();
  // MPU6050 sensitivity is default +/- 250 deg/s -> 131.0 LSB/deg/s
  float gyroz = gz / 131.0; 
  
  // Simple deadzone filter to remove tiny drift when stationary
  if (abs(gyroz) > 1.0) { 
     yaw += gyroz * dt;
  }
}

// 3. Straight Line PID Control
void moveStraightPID() {
  updateYaw();
  int error = yaw - yaw_target; 
  
  int speed_R = BaseSpeed + (error * Kp);
  int speed_L = BaseSpeed - (error * Kp);

  speed_R = constrain(speed_R, 0, UpperLimit);
  speed_L = constrain(speed_L, 0, UpperLimit);

  // Move Forward
  digitalWrite(PIN_Motor_AIN_1, HIGH); 
  analogWrite(PIN_Motor_PWMA, speed_R);
  digitalWrite(PIN_Motor_BIN_1, HIGH); 
  analogWrite(PIN_Motor_PWMB, speed_L);
}

// 4. Stop Car
void stopCar() {
  analogWrite(PIN_Motor_PWMA, 0);
  analogWrite(PIN_Motor_PWMB, 0);
}

// 5. Precise Rotation by Angle (Closed-loop Control)
// targetAngle: Positive for Right turn, Negative for Left turn (e.g., 90 or -90)
void rotateRelative(float targetAngle) {
  // Record starting angle
  updateYaw();
  float initialYaw = yaw;
  
  // Start Motor Rotation
  if (targetAngle > 0) {
    // Turn Right: Left Forward, Right Backward (Differential Spin)
    // Assuming: AIN1=HIGH is Forward, AIN1=LOW is Backward based on wiring
    digitalWrite(PIN_Motor_AIN_1, HIGH); // Left Forward
    analogWrite(PIN_Motor_PWMA, TurnSpeed);
    digitalWrite(PIN_Motor_BIN_1, LOW);  // Right Backward
    analogWrite(PIN_Motor_PWMB, TurnSpeed);
  } else {
    // Turn Left: Left Backward, Right Forward
    digitalWrite(PIN_Motor_AIN_1, LOW);  // Left Backward
    analogWrite(PIN_Motor_PWMA, TurnSpeed);
    digitalWrite(PIN_Motor_BIN_1, HIGH); // Right Forward
    analogWrite(PIN_Motor_PWMB, TurnSpeed);
  }

  // Loop until the turned angle reaches the target
  // Use abs() to handle both positive and negative directions
  float angleTurned = 0;
  while (abs(angleTurned) < abs(targetAngle)) {
    updateYaw(); // Must strictly update yaw inside the loop
    angleTurned = yaw - initialYaw;
    
    // Debug output to monitor real-time angle
    // Serial.print("Target: "); Serial.print(targetAngle);
    // Serial.print(" Current: "); Serial.println(angleTurned);
  }

  // Target reached, stop immediately
  stopCar();
  delay(200); // Wait for inertia to settle
  
  // Update straight-line target so PID doesn't force the car back
  updateYaw();
  yaw_target = yaw; 
}

// --- Obstacle Avoidance Logic ---
void executeAvoidance() {
  Serial.println("Obstacle! Executing precise avoidance...");
  
  stopCar();
  delay(200);

  // 1. Move Backwards (Time-based, as backward distance is hard to quantify with gyro)
  digitalWrite(PIN_Motor_AIN_1, LOW);
  analogWrite(PIN_Motor_PWMA, BaseSpeed);
  digitalWrite(PIN_Motor_BIN_1, LOW);
  analogWrite(PIN_Motor_PWMB, BaseSpeed);
  delay(400); 
  
  stopCar();
  delay(200);

  // 2. Precise Right Turn 90 Degrees
  rotateRelative(90.0);
  
  // Note: yaw_target was already updated at the end of rotateRelative
  // So when returning to loop, the car will drive straight in the new 90-degree direction
}

void loop() {
  unsigned int distance = getDistance();

  if (distance > 0 && distance < ObstacleDetection) {
    executeAvoidance();
  } else {
    moveStraightPID();
  }
  
  delay(10);
}