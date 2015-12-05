#include <Arduino.h>

#include "motors.h"
#include "pinmap.h"    

const int CTRL_SCALE = 1; // Scaling factor used to mix throttle with commands; use 5
  
ServoMotor::ServoMotor(const unsigned int pin, const int error, 
                       const int pitch, const int roll, const int yaw, const int throttle) :
   mPin(pin),
   mError(error),
   mPitch(pitch),
   mRoll(roll),
   mYaw(yaw),
   mThrottle(throttle)
{
   // Do nothing - servo attachment is performed by SetupMotors
}

void ServoMotor::SetSpeed(const int pwm)
{
   mServo.write(pwm);
}

void ServoMotor::SetupMotor()
{
   mServo.attach(this->mPin);
   mServo.setMinimumPulse(MIN_THROTTLE_US);
   mServo.setMaximumPulse(MAX_THROTTLE_US);
}

MotorSet::MotorSet() :
   mMotors(
   {
      // corresponds to motor inputs 1-4 in order
      // front right CCW
      // back left   CCW
      // front left  CW 
      // back right  CW
      //                    error, pitch, roll, yaw, throttle
      ServoMotor(MOTOR_1_PIN, 0,   1,  1,   1, 1),
      ServoMotor(MOTOR_2_PIN, 0,  -1, -1,   1, 1),
      ServoMotor(MOTOR_3_PIN, 0,   1, -1,  -1, 1),
      ServoMotor(MOTOR_4_PIN, 0,  -1,  1,  -1, 1)
   })
{
}

#if (CALIBRATE == 1)
void MotorSet::calibrateMotors()
{
   Serial.println("Calibrating motors...");
   // arm the speed controller, modify as necessary for your ESC  
   for (ServoMotor &motor : mMotors)
   {
      motor.SetSpeed(MAX_THROTTLE_DEG);
   }

   Serial.println("Enable power now...");
   delay(5000);

   // arm the speed controller, modify as necessary for your ESC  
   for (ServoMotor &motor : mMotors)
   {
      motor.SetSpeed(MIN_THROTTLE_DEG);
   }

   delay(7000);

   Serial.println("Calibration finished...");
}
#endif

void MotorSet::setupMotors()
{
   Serial.println("Initializing motors...");
   for (ServoMotor &motor : mMotors)
   {
      motor.SetupMotor();
   }

#if (CALIBRATE == 1)
   calibrateMotors();
#endif /* CALIBRATE */
}

void MotorSet::motorDebug()
{
   Serial.print("Enter pwm value: ");
   while(!Serial.available())
   {
      SoftwareServo::refresh();
   }
   int speed = Serial.parseInt();

   for (ServoMotor &motor : mMotors)
   {
      motor.SetSpeed(speed);
   }

   Serial.println(speed);
}

void MotorSet::controlMotors(const int yaw, const int pitch, const int roll, const int throttle)
{
   int controlOffset;
   
   for (ServoMotor &motor : mMotors)
   {
      controlOffset = (pitch    * motor.GetPitchMap()) + 
                      (roll     * motor.GetRollMap()) + 
                      (yaw      * motor.GetYawMap()); 
      controlOffset = controlOffset / CTRL_SCALE;
      motor.SetSpeed((throttle * motor.GetThrottleMap()) + controlOffset);
   }
}
