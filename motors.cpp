#include <Arduino.h>

#include "motors.h"
#include "pinmap.h"    

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
   mServo.write(map(pwm, MIN_THROTTLE, MAX_THROTTLE, 0, 179));
}

void ServoMotor::SetupMotor()
{
   mServo.attach(this->mPin);
   mServo.setMinimumPulse(MIN_THROTTLE);
   mServo.setMaximumPulse(MAX_THROTTLE);
}

MotorSet::MotorSet() :
   mMotors(
   {
      ServoMotor(MOTOR_1_PIN, 0,   1, -1,  1, 1),
      ServoMotor(MOTOR_2_PIN, 0,  -1,  1,  1, 1),
      ServoMotor(MOTOR_3_PIN, 0,   1,  1, -1, 1),
      ServoMotor(MOTOR_4_PIN, 0,  -1, -1, -1, 1)
   })
{
/*{{ 1, -1,  1, 1},  // pitch, roll, yaw, throttle
   {-1,  1,  1, 1},
   { 1,  1, -1, 1},
   {-1, -1, -1, 1}}; */
}

#if (CALIBRATE == 1)
void MotorSet::calibrateMotors()
{
   Serial.println("Calibrating motors...");
   // arm the speed controller, modify as necessary for your ESC  
   for (ServoMotor &motor : mMotors)
   {
      motor.SetSpeed(MAX_THROTTLE);
   }

   Serial.println("Enable power now...");
   delay(5000);

   // arm the speed controller, modify as necessary for your ESC  
   for (ServoMotor &motor : mMotors)
   {
      motor.SetSpeed(MIN_THROTTLE);
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
   while(!Serial.available());
   int speed = Serial.parseInt();

   for (ServoMotor &motor : mMotors)
   {
      motor.SetSpeed(speed);
   }

   Serial.println(speed);
}

void MotorSet::controlMotors(const int yaw, const int pitch, const int roll, const int throttle)
{
   for (ServoMotor &motor : mMotors)
   {
      motor.SetSpeed((pitch    * motor.GetPitchMap()) + 
                     (roll     * motor.GetRollMap()) + 
                     (yaw      * motor.GetYawMap()) + 
                     (throttle * motor.GetThrottleMap()));
   }
}
