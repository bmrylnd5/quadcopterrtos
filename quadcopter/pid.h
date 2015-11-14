#ifndef PID_H
#define PID_H

#define epsilon 0.01 /* error limit */
#define dt      0.01 /* sampling time in seconds */

/* gains */
#define PITCH_KP           0.01
#define PITCH_KI           0.01
#define PITCH_KD           0.01
#define PITCH_UPPER_LIMIT  10
#define PITCH_LOWER_LIMIT -10

#define ROLL_KP            0.01
#define ROLL_KI            0.01
#define ROLL_KD            0.01
#define ROLL_UPPER_LIMIT   10
#define ROLL_LOWER_LIMIT  -10

#define YAW_KP             0.01
#define YAW_KI             0.01
#define YAW_KD             0.01
#define YAW_UPPER_LIMIT    10
#define YAW_LOWER_LIMIT   -10

float pidPitch(float cmd, 
               float actual);

float pidRoll(float cmd, 
              float actual);

float pidYaw(float cmd, 
             float actual);

#endif /* PID_H */
