extern "C"
{
#include "pid.h"
}

#include "motors.h"
#include "IMU.h"
#include "Receiver.h"

#define MOTOR_DEBUG 0

// IMU class
IMU imu;

// Receiver class
Receiver receiver;

// Initialize Quadcopter 
void setup()
{
   Serial.begin(115200);
   Serial2.begin(115200);

   // Initialize motors
   setupMotors();

   // Initialize IMU
   imu.SetupIMU();
   
   // Set up receiver PWM interrupts
   receiver.SetupReceiver();
}

// Outputs yaw, pitch, roll, and throttle via serial.
inline void printYPRT(const int port, const float yaw, const float pitch, const float roll, const int throttle)
{
	if(port == 2)
	{
	  Serial2.print(yaw);
      Serial2.print(",");
      Serial2.print(pitch);
      Serial2.print(",");
      Serial2.print(roll);
      Serial2.print(",   ");
      Serial2.println(throttle);
	}
	else
	{
      Serial.print(yaw);
      Serial.print(",");
      Serial.print(pitch);
      Serial.print(",");
      Serial.print(roll);
      Serial.print(",   ");
      Serial.println(throttle);
	}
}

// Quadcopter state machine (main loop)
void loop()
{
#if MOTOR_DEBUG == 0
   /* commands */
   int arm           = 0;
   static int throttleCmd   = 0;
   static int yawCmd        = 0;
   static int pitchCmd      = 0;
   static int rollCmd       = 0;

   static float newYawCmd   = 0.0;
   static float newPitchCmd = 0.0;
   static float newRollCmd  = 0.0;


   /* IMU readings */
   static float yawDeg   = 0.0;
   static float pitchDeg = 0.0;
   static float rollDeg  = 0.0;

  /* read receiver */
   receiver.ReadReceiver(yawCmd, pitchCmd, rollCmd, throttleCmd, arm);
   Serial.print(F("YPRT Rec CMD: "));
   printYPRT(1, yawCmd, pitchCmd, rollCmd, throttleCmd);

   /* quadcopter must be armed to fly */
   if (arm < ARM_PERCENT)
   {
      /* read IMU for each channel - in degrees */
      imu.ReadIMU(yawDeg, pitchDeg, rollDeg);
      Serial.print(F("YPRT IMU Val: "));
      printYPRT(2, yawDeg, pitchDeg, rollDeg, throttleCmd);

      /* adjust command using PID - in degrees */
      /* newYawCmd   = pidYaw(yawCmd,     yawDeg);
      newPitchCmd = pidPitch(pitchCmd, pitchDeg);
      newRollCmd  = pidRoll(rollCmd,   rollDeg);

      Serial.print(F("YPRT PID CMD: "));
      printYPRT(2, newYawCmd, newPitchCmd, newRollCmd, throttleCmd); */
	  
      /* output to motors - in microseconds */
      //controlMotors(newYawCmd, newPitchCmd, newRollCmd, throttleCmd);
	  controlMotors(0, 0, 0, throttleCmd);
	  delay(250);
   }
   else
   {
      /* turn off motors */
      for (int i = MOTOR_1; i < MOTORS_NUM; i++)
      {
         setSpeed((motorEnum)i, MIN_THROTTLE);
      }
   }
#else
   motorDebug();
#endif
}
