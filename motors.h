#ifndef MOTORS_H
#define MOTORS_H

#include <SoftwareServo.h>

#define CALIBRATE    0 /* turn on to calibrate motors */

const int MOTORS_NUM    = 4;  // number of Servo motors

// min/max throttles for each motor
const int MAX_THROTTLE  = 1900;
const int MIN_THROTTLE  = 1200;

// unused
#if 0
const int MID_THROTTLE  = 1500;
/* mode 2 */
#define ROLL_CHANNEL     1
#define PITCH_CHANNEL    2
#define THROTTLE_CHANNEL 3
#define YAW_CHANNEL      4
#define ARM_CHANNEL      5
#endif

// Encapsulates a Servo motor
class ServoMotor
{
 public:
   ServoMotor(const unsigned int pin, const int error, 
              const int pitch, const int roll, const int yaw, const int throttle);
   
   void SetupMotor();               // Attach and bound Servo motor
   void SetSpeed(const int pwm);    // Set the speed of a motor 
   
   inline unsigned int GetPin()  const { return mPin; }
   inline int GetError()         const { return mError; }
   
   // motor mapping getters
   inline int GetPitchMap()      const { return mPitch; }
   inline int GetRollMap()       const { return mRoll; }
   inline int GetYawMap()        const { return mYaw; }
   inline int GetThrottleMap()   const { return mThrottle; }
   
 private:   
   unsigned int mPin;      // input pin associated with motor
   int mError;             // Correctional value to achieve neutral base command
   SoftwareServo mServo;   // Servo control class
   
   // motor mappings
   int mPitch;
   int mRoll;
   int mYaw;
   int mThrottle;
};

// Encapsulates the set up and control of all motors
class MotorSet
{
 public:
   MotorSet();
   
   void setupMotors();     // Set up the motors 
   void motorDebug();      // Used to set motor speed manually (for testing/debug) 
    
   // Control the motors pased on channel parameters
   // Yaw pitch and roll are PID values, throttle is % 
   void controlMotors(const int yaw, const int pitch, const int roll, const int throttle);
   
 private:
   void calibrateMotors(); // Calibrate all the motors 
   
   // corresponds to motor inputs 1-4 in order
   // front right CCW
   // back left   CCW
   // front left  CW 
   // back right  CW
   ServoMotor mMotors[MOTORS_NUM];
};

#endif /* MOTORS_H */
