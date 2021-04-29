#ifndef SERVOMOTORINC_H
#define SERVOMOTORINC_H

#define ANTI_WIND_UP_LIMIT 10000
#define MAX_V 90

#include <Encoder.h>
#include <Servo.h>

class ServoMotorInc{
  private:
  int orientation;
  Encoder* enc;
  Encoder* enc1;
  Encoder* enc2;
  Servo* servo;

  int currentPos,
      incTarget,
      targetPos,
      error = 0,
      prevError = 0;
      
  long integral = 0;

  double Kp, Ki, Kd, vSet, derivative = 0;
  int v;
          
  
  public:
  ServoMotorInc(Servo* servo, Encoder* enc, Encoder* enc1, Encoder* enc2, double Kp, double Ki, double Kd, int orientation);
  void setTarget(int tar)  { targetPos = orientation * tar; prevError = targetPos - enc->read(); incTarget = enc->read();}
  void stopServo() { servo->write(90); }
  int servoUpdate();
};

ServoMotorInc::ServoMotorInc(Servo* servo, Encoder* enc, Encoder* enc1, Encoder* enc2, double Kp, double Ki, double Kd, int orientation){
  this->servo = servo;
  this->enc = enc;
  this->enc1 = enc1;
  this->enc2 = enc2;  
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->orientation = orientation;

  targetPos = enc->read();

  stopServo();
}

int ServoMotorInc::servoUpdate(){
  // Turn the motor a given number of steps in a given direction

  if(targetPos - incTarget > 20) incTarget += 20;
  else if(targetPos - incTarget < -50) incTarget -= 50;
  else incTarget = targetPos;

  // Get the current position of the motor joint
  currentPos = enc->read();

  // Calculate the error between the the current and target position
  error = incTarget - currentPos;

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

  if (-(enc1->read() - enc2->read() + enc->read()) -100 < 600 && currentPos < targetPos) vSet = 0.25*vSet;
  else if (-(enc1->read() - enc2->read() + enc->read()) -100 > 600 && currentPos > targetPos) vSet = 0.25*vSet;
  
  v = vSet;

  //Serial.println("E: " + String(error) + "\tI: " + String(integral) + "\tD: " + String(derivative) + "\tV: " + String(v));

  servo->write(-v + 90);

  return 1;

}

#endif
