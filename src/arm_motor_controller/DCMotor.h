#ifndef DCMOTOR_H
#define DCMOTOR_H

#define ANTI_WIND_UP_LIMIT 250
#define MAX_V 255

#include <Encoder.h>

class DCMotor{
  private:
  byte in1;
  byte in2;
  byte ena;
  Encoder* enc;
  int orientation;

  int currentPos,
      targetPos,
      error = 0,
      prevError = 0,
      integral = 0;

  double Kp, Ki, Kd, vSet, derivative = 0;
  int v;
          
  
  public:
  DCMotor(byte in1, byte in2, byte ena, Encoder* enc, double Kp, double Ki, double Kd, int orientation);
  void setBrake()    { digitalWrite(in1, LOW);   digitalWrite(in2, LOW); analogWrite(ena, LOW); }
  void setFloating() { digitalWrite(in1, HIGH);  digitalWrite(in2, HIGH);  }
  void setForward()  { digitalWrite(in1, HIGH);  digitalWrite(in2, LOW);  }
  void setReverse()  { digitalWrite(in1, LOW);   digitalWrite(in2, HIGH); }
  void setVoltage(unsigned char v) { analogWrite(ena, v); }
  void setTarget(int tar)  { targetPos = orientation * tar; prevError = targetPos - enc->read(); }
  void stopDC()      { setVoltage(0); setBrake(); }
  int dcUpdate();
};

DCMotor::DCMotor(byte in1, byte in2, byte ena, Encoder* enc, double Kp, double Ki, double Kd, int orientation){
  this->in1 = in1;
  this->in2 = in2;
  this->ena = ena;
  this->enc = enc;
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->orientation = orientation;

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(ena, OUTPUT);

  targetPos = enc->read();
  
  setFloating();
  analogWrite(ena,0);
}




int DCMotor::dcUpdate(){
  // Turn the motor a given number of steps in a given direction

  // Get the current position of the motor joint
  currentPos = enc->read();

  // Calculate the error between the the current and target position
  error = targetPos - currentPos;

  // Calculate the current speed of the arm movement (decreases as 
  derivative = error - prevError;
  prevError = error;
  

  if (abs(error) < 5 && abs(derivative) < 20)
  {
    integral = 0;
    setBrake();
    return 0;
  }

  // Update the integral (used to make sure that the motor does not stall before target)
  integral = integral + error;

  // limit the integral if growing too large
  if(integral > ANTI_WIND_UP_LIMIT) integral = ANTI_WIND_UP_LIMIT;
  else if(integral < -ANTI_WIND_UP_LIMIT) integral = -ANTI_WIND_UP_LIMIT;

  // update motor speed
  vSet = Kp * error + Ki * integral + Kd * derivative;

  if(vSet > MAX_V) vSet = MAX_V;
  if(vSet < -MAX_V) vSet = -MAX_V;

  v = 0.5*v + 0.5*vSet;

  //Serial.println("E: " + String(error) + "\tI: " + String(integral) + "\tD: " + String(derivative) + "\tV: " + String(v));

  if(v > 0) 
  {
    setReverse();
    setVoltage((int) v);
  }
  if(v < 0) 
  {
    setForward();
    setVoltage((int) -v);
  }

  return 1;

}

#endif
