#include <NewPing.h>
#include <Servo.h>
#include <SoftwareSerial.h>
// Defining the bluetooth module 
SoftwareSerial mySerial(0,1);   // Rx and Tx

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
// Left Motor
#define IN3 9
#define IN4 5

// Right Sensor
#define IR_Input_R 2
// Left Sensor
#define IR_Input_L 4

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

// Turning Right
void Right_dir(){
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);
}
// Right direction but using the main wheels forward and the others backwad
void Man_Right_dir(){
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
}
// Turning Left
void Left_dir(){
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
}
// Left direction but using the main wheels forward and the others backwad
void Man_Left_dir(){
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
}

// A function to read the sensor values
int readSensor(){
  delay(70);
  int readingCm = Sonar.ping_cm();
  if (readingCm == 0){
    readingCm = 250;
  }
  return readingCm;
}

// To make the servo rotate right
int lookRight(){  
  myServo.write(50);
  delay(500);
  int distance = readSensor();
  delay(100);
  myServo.write(115);
  return distance;
}

// To make the servo rotate left
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

/* Collision avoidance mode based on the rotation of servo motor and the readings from ultrasonic sensor */

void Collision_Avoidance(){
  delay(50);

  if(distance<=17){
    Stop_motors();
    delay(300);
    Backward_dir();
    delay(400);
    Stop_motors();
    delay(300);
    // Calculate the distance on its right and left
    distanceRight = lookRight();
    delay(300);
    distanceLeft = lookLeft();
    delay(300);

    if (distance >= distanceLeft){
      Man_Right_dir();
      delay(1000);
    }
    else{
      Man_Left_dir();
      delay(1000);
    }
  }else{
    Forward_dir();
  }
  distance = readSensor();
}

/* Line follower mode based on the readings from two IR sensors */

void LineFollowerMode(){
  // Reading the sensors value
  int sensorState_R = digitalRead(IR_Input_R);
  int sensorState_L = digitalRead(IR_Input_L);

  if(sensorState_L == HIGH && sensorState_R == HIGH){            // Both sensors are on white surface
    Forward_dir();
  }else if(sensorState_L == LOW && sensorState_R == HIGH){       // If the left sensor on the black surface
    Left_dir();
  }else if(sensorState_L == HIGH && sensorState_R == LOW){       // If the right sensor on the black surface
    Right_dir();
  }else if(sensorState_L == LOW && sensorState_R == LOW){        // If both sensors are on the black line
    Stop_motors();
  }
}


// All configurations


void setup() {
  myServo.attach(SERVO_PIN);

  mySerial.begin(9600);  // For Bluetooth module
  // Serial.begin(9600);  // For Serial monitor in case of simulation

  // Motors Configurations
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);

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

// Reading from bluetooth module
char MODE;
// A flag to switch between the three modes
int dump;

void loop() {
  // In case of simulation
  // while(Serial.available()){
  //   // Reading a character from bluetooth module and then taking an action based on that character
  //   MODE = Serial.read();
  //   Serial.println(MODE);
  
  // In case of hardware
  while(mySerial.available()){
    // Reading a character from bluetooth module and then taking an action based on that character
    MODE = mySerial.read();
    //mySerial.println(MODE);
    
    if(MODE == 'F' || MODE == 'B' || MODE == 'R' || MODE == 'L' || MODE == 'V' || MODE == 'X'){
      dump = 0; 
    }else if(MODE == 'W'){
      dump = 1;
    }else if(MODE == 'U'){
      dump = 2;
    }
    if(dump == 0){
      // Manual Mode
      if(MODE == 'F'){
        Forward_dir();        
      }else if(MODE == 'B'){
        Backward_dir();
      }else if(MODE == 'R'){
        Man_Right_dir();
      }else if(MODE == 'L'){
        Man_Left_dir();
      }else if(MODE == 'S'){
        Stop_motors();
      }else if(MODE == 'V'){
        Forward1third_dir();        
      }else if(MODE == 'X'){
        Forward2third_dir();
      }      
    }else if(dump == 1){
      // Line follower mode
      LineFollowerMode();
    }else if(dump == 2){
      // Collision avoidance mode
      Collision_Avoidance();
    }
  }
}




