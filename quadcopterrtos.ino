#include <PID_v1.h>
#include "motors.h"
#include "IMU.h"
#include "Receiver.h"

#define PRINT_DEBUG 1
#define MOTOR_DEBUG 0

#define LOW_MAX_CAP 5

const int ARM_PERCENT = 50; // Channel percent to arm quadcopter for flying. Error is 0.

// IMU class
IMU imu;

// Receiver class
Receiver receiver;

// Motors set class
MotorSet motors;

static double YAW_KP   = 1.0;
static double YAW_KI   = 0.03;
static double YAW_KD   = 0.05;

static double PITCH_KP = 0.5;
static double PITCH_KI = 0.03;
static double PITCH_KD = 0.05;

static double ROLL_KP  = 1.0;
static double ROLL_KI  = 0.03;
static double ROLL_KD  = 0.05;

/* commands */
static double arm           = 0.0;
static double throttleCmd   = 0.0;
static double yawCmd        = 0.0;
static double pitchCmd      = 0.0;
static double rollCmd       = 0.0;

static double newYawCmd   = 0.0;
static double newPitchCmd = 0.0;
static double newRollCmd  = 0.0;

/* IMU readings */
static double yawDeg      = 0.0;
static double pitchDeg    = 0.0;
static double rollDeg     = 0.0;

// PID
PID yawPID((double*)&yawDeg, &newYawCmd, (double*)&yawCmd, YAW_KP, YAW_KI, YAW_KD, DIRECT);
PID pitchPID((double*)&pitchDeg, &newPitchCmd, (double*)&pitchCmd, PITCH_KP, PITCH_KI, PITCH_KD, DIRECT);
PID rollPID((double*)&rollDeg, &newRollCmd, (double*)&rollCmd, ROLL_KP, ROLL_KI, ROLL_KD, DIRECT);

void imuThread(void)
{
   /* read IMU for each channel - in degrees */
   imu.ReadIMU(yawDeg, pitchDeg, rollDeg);
   
   printYPRT(1, "IMU: ", yawDeg, pitchDeg, rollDeg, throttleCmd);
}

// Quadcopter state machine (main loop)
void quadThread(void)
{
#if MOTOR_DEBUG == 0
   /* read receiver */
   receiver.ReadReceiver(yawCmd, pitchCmd, rollCmd, throttleCmd, arm);
   
   //printYPRT(1, "YPRT Rec CMD: ", yawCmd, pitchCmd, rollCmd, throttleCmd);

   /* quadcopter must be armed to fly */
   if (arm > ARM_PERCENT)
   {
      /* adjust command using PID - in degrees */
      yawPID.Compute();
      pitchPID.Compute();
      rollPID.Compute();

      //printYPRT(1, "PID: ", newYawCmd, newPitchCmd, newRollCmd, throttleCmd);
   }
   else
   {
      /* turn off motors */
      newYawCmd    = BASE_VAL_DEG;
      newPitchCmd  = BASE_VAL_DEG;
      newRollCmd   = BASE_VAL_DEG;
      throttleCmd  = MIN_THROTTLE_US;
   }

   /* output to motors */
   motors.controlMotors(0, newPitchCmd, 0, throttleCmd);
#else
   (void)arm;
   (void)yawCmd;
   (void)pitchCmd;
   (void)rollCmd;
   (void)newYawCmd;
   (void)newPitchCmd;
   (void)newRollCmd;
   motors.motorDebug();
#endif
}

// Initialize Quadcopter 
void setup()
{
   Serial.begin(115200);
   Serial.flush();

   pinMode(MOTOR_CALIBRATE_PIN, INPUT);
   pinMode(13, OUTPUT);
   digitalWrite(13, HIGH);

   //Serial2.begin(115200);

   // Initialize motors
   motors.setupMotors();

   // Initialize IMU
   imu.SetupIMU();
   
   // Set up receiver PWM interrupts
   receiver.SetupReceiver();

   // turn PID on
   yawPID.SetSampleTime(100);
   pitchPID.SetSampleTime(100);
   rollPID.SetSampleTime(100);

   yawPID.SetMode(AUTOMATIC);
   pitchPID.SetMode(AUTOMATIC);
   rollPID.SetMode(AUTOMATIC);

   yawPID.SetOutputLimits(YAW_LOWER_LIMIT+LOW_MAX_CAP, YAW_UPPER_LIMIT-LOW_MAX_CAP);
   pitchPID.SetOutputLimits(PITCH_LOWER_LIMIT+LOW_MAX_CAP, PITCH_UPPER_LIMIT-LOW_MAX_CAP);
   rollPID.SetOutputLimits(ROLL_LOWER_LIMIT+LOW_MAX_CAP, ROLL_UPPER_LIMIT-LOW_MAX_CAP);
}

// Outputs yaw, pitch, roll, and throttle via serial.
void printYPRT(const int port, const char * const str, const double yaw, const double pitch, const double roll, const int throttle)
{
#if (PRINT_DEBUG == 1)
      Serial.print(str);
      Serial.print(yaw);
      Serial.print(" ");
      Serial.print(pitch);
      Serial.print(" ");
      Serial.println(roll);
#else
   (void)str;
   (void)yaw;
   (void)pitch;
   (void)roll;
   (void)throttle;
#endif
   (void)throttle;
   (void) port;
}

void loop()
{
   imuThread();
   quadThread();
}