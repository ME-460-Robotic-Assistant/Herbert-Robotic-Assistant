#ifndef SERVOMOTOR_H
#define SERVOMOTOR_H

#define ANTI_WIND_UP_LIMIT 10000
#define MAX_V 30

#include <Encoder.h>
#include <Servo.h>

class ServoMotor{
  private:
  int orientation;
  Encoder* enc;
  Servo* servo;

  int currentPos,
      targetPos,
      error = 0,
      prevError = 0;
      
  long integral = 0;

  double Kp, Ki, Kd, vSet, derivative = 0;
  int v;
          
  
  public:
  ServoMotor(Servo* servo, Encoder* enc, double Kp, double Ki, double Kd, int orientation);
  void setTarget(int tar)  { targetPos = orientation * tar; prevError = targetPos - enc->read(); }
  void stopServo() { servo->write(90); }
  int servoUpdate();
};

ServoMotor::ServoMotor(Servo* servo, Encoder* enc, double Kp, double Ki, double Kd, int orientation){
  this->servo = servo;
  this->enc = enc;
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->orientation = orientation;

  targetPos = enc->read();

  stopServo();
}

int ServoMotor::servoUpdate(){
  // Turn the motor a given number of steps in a given direction

  // Get the current position of the motor joint
  currentPos = enc->read();

  // Calculate the error between the the current and target position
  error = targetPos - currentPos;

  // Calculate the current speed of the arm movement (decreases as 
  derivative = error - prevError;
  prevError = error;
  

  if (abs(error) < 10 && abs(derivative) < 20)
  {
    //integral = 0;
    //stopServo();
    return 0;
  }

  // Update the integral (used to make sure that the motor does not stall before target)
  integral = integral + error;

  // limit the integral if growing too large
  if(integral > ANTI_WIND_UP_LIMIT) integral = ANTI_WIND_UP_LIMIT;
  else if(integral < -ANTI_WIND_UP_LIMIT) integral = -ANTI_WIND_UP_LIMIT;

  // update motor speed
  vSet = Kp * error + Ki * integral + Kd * derivative;
  //derivative = Kd * derivative;

  //if (abs(derivative) > abs(vSet) || abs(derivative) > MAX_V) vSet = 0;
  //else vSet = vSet + derivative;

  if(vSet > MAX_V) vSet = MAX_V;
  if(vSet < -MAX_V) vSet = -MAX_V;
  
  v = vSet;

  //Serial.println("E: " + String(error) + "\tI: " + String(integral) + "\tD: " + String(derivative) + "\tV: " + String(v));

  servo->write(-v + 90);

  return 1;

}

#endif
