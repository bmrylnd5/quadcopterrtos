#include <Servo.h>

#include "motors.h"
#include "pid.h"

/* commands */
static float arm         = 0.0;
static float pitchCmd    = 0.0;
static float rollCmd     = 0.0;
static float yawCmd      = 0.0;
static float throttleCmd = 0.0;

/* IMU readings */
static float pitchDeg = 0.0;
static float rollDeg  = 0.0;
static float yawDeg   = 0.0;

/* Motor classes */
Servo motors[MOTORS_NUM];

void setup()
{
  Serial.begin(115200);
}

void loop() 
{
  /* read receiver */

  /* quadcopter must be armed to fly */
  if (arm < MID_THROTTLE)
  {
    /* read IMU for each channel - in degrees */
  
    /* adjust command using PID - in degrees */
    pitchCmd = pidPitch(pitchCmd, pitchDeg);
    rollCmd  = pidRoll(rollCmd,   rollDeg);
    yawCmd   = pidYaw(yawCmd,     yawDeg);
  
    /* convert degrees to PWM (us) */
    
    /* output to motors - in microseconds */
  }
  else
  {
    /* turn off motors */
  }
}
