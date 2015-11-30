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

/* commands */
static int arm           = 0;
static int throttleCmd   = 0;
static int yawCmd        = 0;
static int pitchCmd      = 0;
static int rollCmd       = 0;

static float newYawCmd   = 0.0;
static float newPitchCmd = 0.0;
static float newRollCmd  = 0.0;

/* IMU readings */
static float yawDeg      = 0.0;
static float pitchDeg    = 0.0;
static float rollDeg     = 0.0;

void imuThread(void)
{
   /* read IMU for each channel - in degrees */
   imu.ReadIMU(yawDeg, pitchDeg, rollDeg);
   
   Serial.print(F("YPRT IMU Val: "));
   printYPRT(1, yawDeg, pitchDeg, rollDeg, throttleCmd);
}

// Quadcopter state machine (main loop)
void quadThread(void)
{
#if MOTOR_DEBUG == 0
   /* read receiver */
   receiver.ReadReceiver(yawCmd, pitchCmd, rollCmd, throttleCmd, arm);
   
   Serial.print(F("YPRT Rec CMD: "));
   printYPRT(1, yawCmd, pitchCmd, rollCmd, throttleCmd);

   /* quadcopter must be armed to fly */
   if (arm < ARM_PERCENT)
   {
      /* adjust command using PID - in degrees */
      newYawCmd   = pidYaw(yawCmd,     yawDeg);
      newPitchCmd = pidPitch(pitchCmd, pitchDeg);
      newRollCmd  = pidRoll(rollCmd,   rollDeg);

      Serial.print(F("YPRT PID CMD: "));
      printYPRT(1, newYawCmd, newPitchCmd, newRollCmd, throttleCmd);

      /* output to motors - in microseconds */
      controlMotors(newYawCmd, newPitchCmd, newRollCmd, throttleCmd);
   }
   else
   {
      /* turn off motors */
      Serial.println(F("Disarming motors"));
      controlMotors(0, 0, 0, MIN_THROTTLE);
   }
#else
   motorDebug();
#endif
}

// Initialize Quadcopter 
void setup()
{
   Serial.begin(115200);
   //Serial2.begin(115200);

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
   (void)port;
	/*if(port == 2)
	{
	   Serial2.print(yaw);
      Serial2.print(",");
      Serial2.print(pitch);
      Serial2.print(",");
      Serial2.print(roll);
      Serial2.print(",   ");
      Serial2.println(throttle);
	}
	else*/
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

void loop()
{
   quadThread();
   imuThread();  
}