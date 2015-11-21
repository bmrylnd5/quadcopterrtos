extern "C"
{
#include "pid.h"
}

#include "motors.h"
#include "IMU.h"
#include "Receiver.h"

// IMU class
IMU imu;

// Receiver class
Receiver receiver;

// Initialize Quadcopter 
void setup()
{
   Serial.begin(115200);

   // Initialize motors
   setupMotors();

   // Initialize IMU
   imu.SetupIMU();
   
   // Set up receiver PWM interrupts
   receiver.SetupReceiver();
}

// Outputs yaw, pitch, roll, and throttle via serial.
inline void printYPRT(const float yaw, const float pitch, const float roll, const int throttle)
{
      Serial.print(yaw);
      Serial.print(",");
      Serial.print(pitch);
      Serial.print(",");
      Serial.print(roll);
      Serial.print(",   ");
      Serial.println(throttle);
}

// Quadcopter state machine (main loop)
void loop()
{
   /* commands */
   int arm           = 0;
   int throttleCmd   = 0;
   int yawCmd        = 0;
   int pitchCmd      = 0;
   int rollCmd       = 0;

   // float throttleCmd = 0.0; TODO not yet used
   float newYawCmd   = 0.0;
   float newPitchCmd = 0.0;
   float newRollCmd  = 0.0;


   /* IMU readings */
   float yawDeg   = 0.0;
   float pitchDeg = 0.0;
   float rollDeg  = 0.0;

  /* read receiver */
   receiver.ReadReceiver(yawCmd, pitchCmd, rollCmd, throttleCmd, arm);
   Serial.print(F("YPRT Rec CMD: "));
   printYPRT(yawCmd, pitchCmd, rollCmd, throttleCmd);

   delay(500);
   return;

   /* quadcopter must be armed to fly */
   if (arm < MID_THROTTLE)
   {
      /* read IMU for each channel - in degrees */
      imu.ReadIMU(yawDeg, pitchDeg, rollDeg);
      Serial.print(F("YPRT IMU Val: "));
      printYPRT(yawDeg, pitchDeg, rollDeg, throttleCmd);

      /* adjust command using PID - in degrees */
      newYawCmd   = pidYaw(yawCmd,     yawDeg);
      newPitchCmd = pidPitch(pitchCmd, pitchDeg);
      newRollCmd  = pidRoll(rollCmd,   rollDeg);

      Serial.print(F("YPRT PID CMD: "));
      printYPRT(newYawCmd, newPitchCmd, newRollCmd, throttleCmd);

      /* convert degrees to PWM (us) */

      /* output to motors - in microseconds */
   }
   else
   {
      Serial.println(F("ARM command below threshold"));
      /* turn off motors */
   }
}
