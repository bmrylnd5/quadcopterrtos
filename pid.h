#ifndef PID_H
#define PID_H

#define epsilon 0.01 /* error limit */
#define dt      0.01 /* sampling time in seconds */

/* UNITS ARE IN RADIANS */
/* gains - TUNE ME */
#define PITCH_KP           0.1
#define PITCH_KI           0
#define PITCH_KD           0
#define PITCH_UPPER_LIMIT  45
#define PITCH_LOWER_LIMIT -45

#define ROLL_KP            0.1
#define ROLL_KI            0
#define ROLL_KD            0
#define ROLL_UPPER_LIMIT   45
#define ROLL_LOWER_LIMIT  -45

#define YAW_KP             0.1
#define YAW_KI             0
#define YAW_KD             0
#define YAW_UPPER_LIMIT    45
#define YAW_LOWER_LIMIT   -45

/**
 * Outputs the new adjusted pitch command based on current
 * command and IMU reading.
 */
float pidPitch(float cmd, 
               float actual);

/**
 * Outputs the new adjusted roll command based on current
 * command and IMU reading.
 */
float pidRoll(float cmd, 
              float actual);

/**
 * Outputs the new adjusted yaw command based on current
 * command and IMU reading.
 */
float pidYaw(float cmd, 
             float actual);

#endif /* PID_H */
