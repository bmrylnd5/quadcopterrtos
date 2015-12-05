extern "C"
{
#include "pid.h"
}

#include "motors.h"
#include "IMU.h"
#include "Receiver.h"

#define PRINT_DEBUG 1
#define MOTOR_DEBUG 0
#define MOTOR_DELAY 100

const int ARM_PERCENT = 50; // Channel percent to arm quadcopter for flying. Error is 0.

// IMU class
IMU imu;

// Receiver class
Receiver receiver;

// Motors set class
MotorSet motors;

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
   
   printYPRT(1, "YPRT IMU Val: ", yawDeg, pitchDeg, rollDeg, throttleCmd);
}

// Quadcopter state machine (main loop)
void quadThread(void)
{
#if MOTOR_DEBUG == 0
   /* read receiver */
   receiver.ReadReceiver(yawCmd, pitchCmd, rollCmd, throttleCmd, arm);
   
   printYPRT(1, "YPRT Rec CMD: ", yawCmd, pitchCmd, rollCmd, throttleCmd);

   /* quadcopter must be armed to fly */
   if (arm > ARM_PERCENT)
   {
      /* adjust command using PID - in degrees */
      newYawCmd   = pidYaw(yawCmd,     yawDeg);
      newPitchCmd = pidPitch(pitchCmd, pitchDeg);
      newRollCmd  = pidRoll(rollCmd,   rollDeg);

      printYPRT(1, "YPRT PID CMD: ", newYawCmd, newPitchCmd, newRollCmd, throttleCmd);

      /* output to motors - in microseconds */
      motors.controlMotors(0, 0, 0, throttleCmd);
   }
   else
   {
      /* turn off motors */
      Serial.println(F("Disarming motors"));
      motors.controlMotors(BASE_VAL_DEG, BASE_VAL_DEG, BASE_VAL_DEG, MIN_THROTTLE);
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
   motors.setupMotors();

   // Initialize IMU
   imu.SetupIMU();
   
   // Set up receiver PWM interrupts
   receiver.SetupReceiver();
}

// Outputs yaw, pitch, roll, and throttle via serial.
void printYPRT(const int port, const char * const str, const float yaw, const float pitch, const float roll, const int throttle)
{
#if (PRINT_DEBUG == 1)
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
      Serial.print(str);
      Serial.print(yaw);
      Serial.print(",");
      Serial.print(pitch);
      Serial.print(",");
      Serial.print(roll);
      Serial.print(",   ");
      Serial.println(throttle);
   }
#else
   (void)str;
   (void)yaw;
   (void)pitch;
   (void)roll;
   (void)throttle;
#endif
   (void) port;
}

void loop()
{
   quadThread();
   imuThread();
   delay(MOTOR_DELAY);
}