#include <NewPing.h>
#include <Servo.h>


// UltraSonic Configurations
#define Echo A1
#define Trigger A0
#define MAX_DISTANCE 250   // Max Range
float duration;
int iterations = 5;
NewPing Sonar(Trigger,Echo,MAX_DISTANCE);

boolean Forward = false;

// Servo Configurations
#define SERVO_PIN 10
Servo myServo;

// Two Switches to handle the modes
#define Switch 7
#define Switch2 3
// Readings of the switches
int Mode;
int Mode2;

// L298 Driver
// Right Motor
#define IN1 11
#define IN2 6
// #define ENA 11
// Left Motor
#define IN3 9
#define IN4 5
// #define ENB 6


// Right Sensor
#define IR_Input_R 2
// Left Sensor
#define IR_Input_L 4

// Reading of the serial monitor
char dir;

// Turning all Motors in forward direction
void Forward_dir(){
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4,LOW);
}

// 1/3 speed in forward direction
void Forward1third_dir(){
    analogWrite(IN1,85);
    digitalWrite(IN2,LOW);
    analogWrite(IN3,85);
    digitalWrite(IN4,LOW);
}

// 2/3 speed in forward direction
void Forward2third_dir(){
    analogWrite(IN1,170);
    digitalWrite(IN2,LOW);
    analogWrite(IN3,170);
    digitalWrite(IN4,LOW);
}

// Turning all Motors in backward direction
void Backward_dir(){
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
}

// Stop all motors
void Stop_motors(){
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);
}

// Turning left
void Left_dir(){
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);
}
// Turning Right
void Right_dir(){
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
}

int readSensor(){
  delay(70);
  int readingCm = Sonar.ping_cm();
  if (readingCm == 0){
    readingCm = 250;
  }
  return readingCm;
}

int lookRight(){  
  myServo.write(50);
  delay(500);
  int distance = readSensor();
  delay(100);
  myServo.write(115);
  return distance;
}

int lookLeft(){
  myServo.write(170);
  delay(500);
  int distance = readSensor();
  delay(100);
  myServo.write(115);
  return distance;
  delay(100);
}

int distanceRight;
int distanceLeft;
int distance = 100;

void Collision_Avoidance(){
  delay(50);

  if(distance<=20){
    Stop_motors();
    delay(300);
    Backward_dir();
    delay(400);
    Stop_motors();
    delay(300);
    distanceRight = lookRight();
    delay(300);
    distanceLeft = lookLeft();
    delay(300);

    if (distance >= distanceLeft){
      Right_dir();
      Stop_motors();
    }
    else{
      Left_dir();
      Stop_motors();
    }
  }else{
    Forward_dir();
  }
  distance = readSensor();
}


void Manual_Mode(){
  // analogWrite(ENA,255);
  // analogWrite(ENB,255);
  while(Serial.available()){
    dir = Serial.read();
    Serial.println(dir);
    if(dir == 'F'){
      Forward_dir();        
    }else if(dir == 'B'){
      Backward_dir();
    }else if(dir == 'R'){
      Right_dir();
    }else if(dir == 'L'){
      Left_dir();
    }else if(dir == 'S'){
      Stop_motors();
    }else if(dir == 'V'){
      Forward1third_dir();
    }else if(dir == 'X'){
      Forward2third_dir();
    }      
  }
}


void LineFollowerMode(){
  // analogWrite(ENA,255);
  // analogWrite(ENB,255);
  // Reading the sensors value
  int sensorState_R = digitalRead(IR_Input_R);
  int sensorState_L = digitalRead(IR_Input_L);

  if(sensorState_L == LOW && sensorState_R == LOW){       // Both sensors are on white surface
    Forward_dir();
  }else if(sensorState_L == HIGH && sensorState_R == LOW){       // If the left sensor on the black line
    Left_dir();
  }else if(sensorState_L == LOW && sensorState_R == HIGH){       // If the right sensor on the black line
    Right_dir();
  }else if(sensorState_L == HIGH && sensorState_R == HIGH){       // If both sensors are on the black line
    Stop_motors();
  }
}


void setup() {

  myServo.attach(SERVO_PIN);

  pinMode(Switch,INPUT);
  pinMode(Switch2,INPUT);
  Serial.begin(9600);  // Baud rate

  // Motors Configurations
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  // pinMode(ENA,OUTPUT);
  // pinMode(ENB,OUTPUT);


  // IR Configurations
  pinMode(IR_Input_R,INPUT);
  pinMode(IR_Input_L,INPUT);

  // Collison Configurations
  myServo.write(115);
  delay(2000);
  distance = readSensor();
  delay(100);
  distance = readSensor();
  delay(100);
  distance = readSensor();
  delay(100);
  distance = readSensor();
  delay(100);

}

void loop() {
  Mode = digitalRead(Switch);
  Mode2 = digitalRead(Switch2);

  if(Mode == HIGH && Mode2 == LOW){
    LineFollowerMode();
  }else if(Mode == HIGH && Mode2 == HIGH){
    Manual_Mode();
  }else if(Mode == LOW && Mode2 == HIGH){
    // analogWrite(ENA,255);
    // analogWrite(ENB,255);
    Collision_Avoidance();
  }
}




