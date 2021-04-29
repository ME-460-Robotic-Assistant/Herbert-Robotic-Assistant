/*

Encoders have 2400 pulses/rev, so 0.15 degrees/pulse

The encoders are set to these initial values when the Teensy is powered on regardless of what position it is actully in,
so try to always end in this location. If you have to turn off the Teensy, you need to adjust the position relative to these initial values.

More negative values make the arms rotate in a clockwise motion.
Setting zero for all values makes the arm point straight out to the side.
A more detailed geometrical diagram is in the MATLAB section of the report.

INITIAL (NEUTRAL) POSITION: Enc1 = -100, Enc2 = -920, Enc3 = 920

LOWERED POSITION: Enc1 = -100, Enc2 = 200, Enc3 = 300

DONT DO THE RAISED POSITION FOR NOW
RAISED POSITION: Enc1 = -100, Enc2 = -920, Enc3 = 500

*/

#define SERIAL_CLASS usb_serial_class

#include <ros.h>
#include <geometry_msgs/Vector3.h>

#include <Encoder.h>
#include <Servo.h>
#include "DCMotor.h"
#include "ServoMotor.h"
#include "ServoMotorInc.h"

//++++++++++++++ UNCOMMENT FOR ROS ++++++++++++++++++++++
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
#define SERVOPIN3 15

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
ServoMotorInc S1 = ServoMotorInc(&Servo1, &Enc3, &Enc1, &Enc2, 0.3, 0.001, 0.45, 1);
ServoMotor S2 = ServoMotor(&Servo2, &Enc4, 0.3, 0, 0.2, 1);

// Instantiate DCMotor objects to implement PID algorthm
DCMotor DC1 = DCMotor(IN1, IN2, ENA1, &Enc1, 1, 0.5, 3, 1);
DCMotor DC2 = DCMotor(IN3, IN4, ENA2, &Enc2, 1, 0.4, 3, -1);

//++++++++++++++ UNCOMMENT FOR ROS ++++++++++++++++++++++
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
  Enc1.write(-100);
  Enc2.write(920);
  Enc3.write(920);
  Enc4.write(0);

  DC1.setTarget(-100);
  DC2.setTarget(-920);
  S1.setTarget(920);
  S2.setTarget(0);

//++++++++++++++ UNCOMMENT FOR ROS ++++++++++++++++++++++
//  nh.initNode();
//  nh.subscribe(sub);

//++++++++++++++ COMMENT OUT FOR ROS ++++++++++++++++++++++
  Serial.begin(9600);
}

int tar1 = -100, tar2 = -920, tar3 = 920;
// Initialize variables to determine if joints are in correct positions
// 1: Joint in incorrect position, motor is active
// 0: Joint in correct positon, motor is inactive
int DC1stopped = 0;
int DC2stopped = 0;
int Servo1stopped = 0;
int Servo2stopped = 0;

void loop() {

 // ++++++++++++++++ COMMENT OUT THIS WHOLE IF STATEMENT WHEN USING ROS ++++++++++++++++++++
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
    S2.setTarget(-(Enc1.read() - Enc2.read() + Enc3.read()) - 100);
    Servo2stopped = S2.servoUpdate();

    if (DC1.getTarget() < Enc1.read())
    {
      if(!DC2stopped && !Servo1stopped) 
      {
        DC2.stopDC();
        DC1stopped = DC1.dcUpdate();
        S1.stopServo();
      }
      
      if(!DC1stopped)
      {
        DC1.stopDC();
        DC2stopped = DC2.dcUpdate();
        Servo1stopped = S1.servoUpdate();
      }
    }
    else
    {
      if(!DC1stopped)
      {
        DC1.stopDC();
        DC2stopped = DC2.dcUpdate();
        Servo1stopped = S1.servoUpdate();
      }
      
      if(!DC2stopped && !Servo1stopped) 
      {
        DC2.stopDC();
        DC1stopped = DC1.dcUpdate();
        S1.stopServo();
      }
    }

    
   
  }
  
  // Check for new angular_position message

  //++++++++++++++ UNCOMMENT FOR ROS ++++++++++++++++++++++
  // nh.spinOnce();
  
  delay(100);
}
