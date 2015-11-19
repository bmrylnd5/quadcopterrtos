extern "C"
{
#include <Servo.h>

#include "motors.h"
#include "pid.h"
}

#include "IMU.h"

// Motor classes
Servo motors[MOTORS_NUM];

// IMU class
IMU imu;

// Initialize Quadcopter 
void setup()
{
   Serial.begin(115200);

   // Initialize IMU
   imu.SetupIMU();
}

// Outputs yaw, pitch, and roll via serial.
inline void printYPR(const float yaw, const float pitch, const float roll)
{
      Serial.print(yaw);
      Serial.print(",");
      Serial.print(pitch);
      Serial.print(",");
      Serial.println(roll);
}

// Quadcopter state machine (main loop)
void loop()
{
   /* commands */
   float arm         = 0.0;
   float yawCmd      = 0.0;
   float pitchCmd    = 0.0;
   float rollCmd     = 0.0;

   // float throttleCmd = 0.0; TODO not yet used
   float newYawCmd   = 0.0;
   float newPitchCmd = 0.0;
   float newRollCmd  = 0.0;


   /* IMU readings */
   float yawDeg   = 0.0;
   float pitchDeg = 0.0;
   float rollDeg  = 0.0;

  /* read receiver */
   Serial.print(F("YPR Rec CMD: "));
   printYPR(yawCmd, pitchCmd, rollCmd);

   /* quadcopter must be armed to fly */
   if (arm < MID_THROTTLE)
   {
      /* read IMU for each channel - in degrees */
      imu.ReadIMU(yawDeg, pitchDeg, rollDeg);
      Serial.print(F("YPR IMU Val: "));
      printYPR(yawDeg, pitchDeg, rollDeg);

      /* adjust command using PID - in degrees */
      newYawCmd   = pidYaw(yawCmd,     yawDeg);
      newPitchCmd = pidPitch(pitchCmd, pitchDeg);
      newRollCmd  = pidRoll(rollCmd,   rollDeg);

      Serial.print(F("YPR PID CMD: "));
      printYPR(newYawCmd, newPitchCmd, newRollCmd);

      /* convert degrees to PWM (us) */

      /* output to motors - in microseconds */
   }
   else
   {
      /* turn off motors */
   }
}
