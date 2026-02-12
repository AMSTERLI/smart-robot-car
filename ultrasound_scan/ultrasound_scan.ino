#include <Servo.h> 

#define TRIG_PIN 13
#define ECHO_PIN 12
#define SERVO_PIN 10

Servo myServo; 

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  myServo.attach(SERVO_PIN);
  
  // 初始归位
  myServo.write(90);
  delay(500);
}

int getDistance() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // 【关键修改1】增加超时参数 30000微秒 (30ms)
  // 如果30ms内没有回波，函数强制结束，不再死等
  // 30ms 大约对应 5米距离，足够覆盖传感器量程
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  
  if (duration == 0) {
    return 400; // 如果超时（前面没东西），返回一个最大值，避免数据为0干扰绘图
  }
  
  return (int)(duration / 58);
}

void loop() {
  // 【关键修改2】步进改为 2 度，动作更细腻 (原为 5)
  for (int pos = 15; pos <= 165; pos += 2) { 
    myServo.write(pos);              
    
    // 【关键修改3】给舵机留出物理运动时间
    // 15ms 足以完成 2度的微小转动，配合 pulseIn 的时间，整体节奏刚好
    delay(15);                        
    
    int dist = getDistance();
    
    Serial.print(pos); 
    Serial.print(","); 
    Serial.println(dist); 
  }
  
  // 反向扫描
  for (int pos = 165; pos >= 15; pos -= 2) { 
    myServo.write(pos);
    delay(15);
    int dist = getDistance();
    
    Serial.print(pos);
    Serial.print(",");
    Serial.println(dist);
  }
}