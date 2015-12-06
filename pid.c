#include <math.h>
#include <stdlib.h>

#include "pid.h"

static float YAW_KP   = 0.0;
static float YAW_KI   = 0.0;
static float YAW_KD   = 0.0;

static float PITCH_KP = 0.5;
static float PITCH_KI = 2.5;
static float PITCH_KD = 0.0;

static float ROLL_KP  = 0.0;
static float ROLL_KI  = 0.0;
static float ROLL_KD  = 0.0;

float pidPitch(float cmd, 
               float actual)
{
  static float errorDerivative = 0.0;
  static float errorInt        = 0.0;
  float        derivative      = 0.0;
  float        error           = 0.0;
  float        output          = 0.0;

  /* calculate error */
  error = cmd - actual;

  /* don't integrate if error is small */
  if (abs(error) > epsilon)
  {
    /* integral */
    errorInt = errorInt + dt * error;
  }

  /* derivative */
  derivative = ((float)(error - errorDerivative))/dt;

  /* calculate output */
  output = PITCH_KP * error + PITCH_KI * errorInt - PITCH_KD * derivative;

  /* clamp the output */
  if (output > PITCH_UPPER_LIMIT)
  {
    output = PITCH_UPPER_LIMIT;

    if ((PITCH_KI * error) > 0)
    {
      errorInt = errorInt - dt * error;
    }
  }
  else if (output < PITCH_LOWER_LIMIT)
  {
    output = PITCH_LOWER_LIMIT;

    if ((PITCH_KI * error) < 0)
    {
      errorInt = errorInt - dt * error;
    }
  }

  errorDerivative = error;
  return output;
}

float pidRoll(float cmd, 
              float actual)
{
  static float errorDerivative = 0.0;
  static float errorInt        = 0.0;
  float        derivative      = 0.0;
  float        error           = 0.0;
  float        output          = 0.0;

  /* calculate error */
  error = cmd - actual;

  /* don't integrate if error is small */
  if (abs(error) > epsilon)
  {
    /* integral */
    errorInt = errorInt + dt * error;
  }

  /* derivative */
  derivative = ((float)(error - errorDerivative))/dt;

  /* calculate output */
  output = ROLL_KP * error + ROLL_KI * errorInt - ROLL_KD * derivative;

  /* clamp the output */
  if (output > ROLL_UPPER_LIMIT)
  {
    output = ROLL_UPPER_LIMIT;

    if ((ROLL_KI * error) > 0)
    {
      errorInt = errorInt - dt * error;
    }
  }
  else if (output < ROLL_LOWER_LIMIT)
  {
    output = ROLL_LOWER_LIMIT;

    if ((ROLL_KI * error) < 0)
    {
      errorInt = errorInt - dt * error;
    }
  }

  errorDerivative = error;
  return output;
}

float pidYaw(float cmd, 
             float actual)
{
  static float errorDerivative = 0.0;
  static float errorInt        = 0.0;
  float        derivative      = 0.0;
  float        error           = 0.0;
  float        output          = 0.0;

  /* calculate error */
  error = fmod((cmd - actual) + M_PI, 2 * M_PI) - M_PI;

  /* don't integrate if error is small */
  if (abs(error) > epsilon)
  {
    /* integral */
    errorInt = errorInt + dt * error;
  }

  /* derivative */
  derivative = ((float)(error - errorDerivative))/dt;

  /* calculate output */
  output = YAW_KP * error + YAW_KI * errorInt - YAW_KD * derivative;

  /* clamp the output */
  if (output > YAW_UPPER_LIMIT)
  {
    output = YAW_UPPER_LIMIT;

    if ((YAW_KI * error) > 0)
    {
      errorInt = errorInt - dt * error;
    }
  }
  else if (output < YAW_LOWER_LIMIT)
  {
    output = YAW_LOWER_LIMIT;

    if ((YAW_KI * error) < 0)
    {
      errorInt = errorInt - dt * error;
    }
  }

  errorDerivative = error;
  return output;
}

void setPidPitch(float p, float i, float d)
{
  PITCH_KP = p;
  PITCH_KI = i;
  PITCH_KD = d;
}

void setPidRoll(float p, float i, float d)
{
  ROLL_KP = p;
  ROLL_KI = i;
  ROLL_KD = d;
}

void setPidYaw(float p, float i, float d)
{
  YAW_KP = p;
  YAW_KI = i;
  YAW_KD = d;
}