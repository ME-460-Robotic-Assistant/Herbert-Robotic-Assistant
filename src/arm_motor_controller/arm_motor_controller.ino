#define SERIAL_CLASS usb_serial_class

#include <ros.h>
#include <geometry_msgs/Vector3.h>

#include <Encoder.h>
#include <Servo.h>
#include "DCMotor.h"
#include "ServoMotor.h"


//ros::NodeHandle nh;

#define IR_BASE 18
#define IR_ARM  19
#define FSR_BOT 20
#define FSR_TOP 21
 
// Rotary Encoder 1 Pins
#define ROTENCINTPIN1 5
#define ROTENCDIRPIN1 0

// Rotary Encoder 2 Pins
#define ROTENCINTPIN2 6
#define ROTENCDIRPIN2 1

// Rotary Encoder 3 Pins
#define ROTENCINTPIN3 7
#define ROTENCDIRPIN3 2

// Rotary Encoder 4 Pins
#define ROTENCINTPIN4 8
#define ROTENCDIRPIN4 3

// DC Motor 1 Pins
#define ENA1  12
#define IN1   13
#define IN2   14

// DC Motor 2 Pins
#define ENA2  9
#define IN3   16
#define IN4   17

// Servo Motor 1 & 2 Pins
#define SERVOPIN1 4
#define SERVOPIN2 10

bool objectDetected = false;
bool collisionDetected = false;

// Instantiate a object for each encoder
// These objects use interrupts to update pulses, 2400 pulses/rev
Encoder Enc1(ROTENCINTPIN1, ROTENCDIRPIN1);
Encoder Enc2(ROTENCINTPIN2, ROTENCDIRPIN2);
Encoder Enc3(ROTENCINTPIN3, ROTENCDIRPIN3);
Encoder Enc4(ROTENCINTPIN4, ROTENCDIRPIN4);

// Instantiate a servo object for each servo
// Allows for servo speed control with PWM
Servo Servo1;
Servo Servo2;

// Instantiate ServoMotor objects to implement PID algorithm
//ServoMotorSpeed S1 = ServoMotorSpeed(&Servo1, &Enc3, 0.5, 0.3, 0.1, 1);
ServoMotor S1 = ServoMotor(&Servo1, &Enc3, 0.3, 0, 0.45, 1);
ServoMotor S2 = ServoMotor(&Servo2, &Enc4, 0.4, 0, 0, 1);

// Instantiate DCMotor objects to implement PID algorthm
DCMotor DC1 = DCMotor(IN1, IN2, ENA1, &Enc1, 1, 0.5, 3, 1);
DCMotor DC2 = DCMotor(IN3, IN4, ENA2, &Enc2, 1, 0.4, 3, -1);

// Initialize variables to determine if joints are in correct positions
// 1: Joint in incorrect position, motor is active
// 0: Joint in correct positon, motor is inactive
int DC1stopped = 1;
int DC2stopped = 1;
int Servo1stopped = 1;
int Servo2stopped = 1;

//void updateTarget( const geometry_msgs::Vector3& msg){
//  // ROS subscriber callback function, gets run each time a new message is recieved from the topic
//  DC1.setTarget((int) msg.x);
//  DC2.setTarget((int) msg.y);
//  S1.setTarget((int) msg.z);
//}
//
//ros::Subscriber<geometry_msgs::Vector3> sub("chatter", &updateTarget );

void stopAll(){
  DC1.stopDC();
  DC2.stopDC();
  S1.stopServo();
  S2.stopServo();
  collisionDetected = true;
}

void setup() {

  pinMode(IR_ARM, INPUT);
  pinMode(IR_BASE, INPUT);

  Servo1.attach(SERVOPIN1);
  Servo2.attach(SERVOPIN2);

  // Set intitial encoder pulse values assuming starting in neutral position
  Enc1.write(0);
  Enc2.write(920);
  Enc3.write(920);
  Enc4.write(0);

  DC1.setTarget(0);
  DC2.setTarget(-920);
  S1.setTarget(920);
  S2.setTarget(0);

//  nh.initNode();
//  nh.subscribe(sub);
  Serial.begin(9600);
}

int tar1 = 0, tar2 = -920, tar3 = 920;

void loop() {
 if (Serial.available() > 0) {
    // read the incoming byte:
    tar1 = Serial.parseInt();
    tar2 = Serial.parseInt();
    tar3 = Serial.parseInt();

    DC1.setTarget(tar1);
    DC2.setTarget(tar2);
    S1.setTarget(tar3);

    // say what you got:
    Serial.println("I received: " + String(tar1) + " " + String(tar2) + " " + String(tar3));
  }

  /*
  // Check sensors
  if (analogRead(FSR_TOP) > 512) objectDetected = true; else objectDetected = false; // FSR sensor in end effector pan
  if (analogRead(FSR_BOT) > 512) stopAll(); // FSR sensor on bottom of end effector
  else if(digitalRead(IR_BASE)) stopAll(); // IR sensor on mounting plate
  else if (digitalRead(IR_ARM)) stopAll(); // IR sensor on arm
  else collisionDetected = false;
  */
  
  if(!collisionDetected)
  {
    S2.setTarget(-(Enc1.read() - Enc2.read() + Enc3.read()));
    Servo2stopped = S2.servoUpdate();
    
    Servo1stopped = S1.servoUpdate();
    
    DC2stopped = DC2.dcUpdate();
    
    if(!DC2stopped) 
    {
      DC2.stopDC();
      DC1stopped = DC1.dcUpdate();
    }
  }
  
  // Check for new angular_position message
  // nh.spinOnce();
  delay(100);
}





//    // Start each loop assuming joints are in incorrect positions
//  DC1stopped = 1;
//  DC2stopped = 1;
//  Servo1stopped = 1;
//  Servo2stopped = 1;
//
//  // Update both servo motors but only run one DC motor at a time.
//  // This while loop runs DC Motor 2 and both servos
//  while(DC2stopped)
//  {
//    S2.setTarget(-(Enc1.read() + Enc2.read() + Enc3.read()));
//    Servo2stopped = S2.servoUpdate();
//    Servo1stopped = S1.servoUpdate();
//    DC2stopped = DC2.dcUpdate();
//    delay(100);
//  }
//
//  // This while runs DC Motor 1 and both servos
//  // Loop is terminated when all joints are in their target positions
//  while (Servo2stopped || Servo1stopped || DC1stopped)
//  {
//    S2.setTarget(-(Enc1.read() + Enc2.read() + Enc3.read()));
//    Servo2stopped = S2.servoUpdate();
//    Servo1stopped = S1.servoUpdate();
//    DC1stopped = DC1.dcUpdate();
//    delay(100);
//  }
