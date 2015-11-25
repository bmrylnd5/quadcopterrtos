#ifndef MOTORS_H
#define MOTORS_H

#include <Servo.h>

#define CALIBRATE    1 /* turn on to calibrate motors */
#define MOTORS_NUM   4

/* Channel percent to arm quadcopter */
#define ARM_PERCENT  50

/*  min/max throttles for each motor */
#define MAX_THROTTLE 1850
#define MID_THROTTLE 1500
#define MIN_THROTTLE 1150

/* mode 2 */
#define ROLL_CHANNEL     1
#define PITCH_CHANNEL    2
#define THROTTLE_CHANNEL 3
#define YAW_CHANNEL      4
#define ARM_CHANNEL      5

/* mappings */
#define PITCH_MAPPING    0
#define ROLL_MAPPING     1
#define YAW_MAPPING      2
#define THROTTLE_MAPPING 3

typedef enum
{
	MOTOR_1 = 0, // front right CCW
	MOTOR_2 = 1, // back left   CCW
	MOTOR_3 = 2, // front left  CW 
	MOTOR_4 = 3  // back right  CW
} motorEnum;

extern int motorMappings[4][4];
extern Servo motors[MOTORS_NUM];

/*
 * Setups the motors 
 */
void setupMotors(void);

/* 
 * Set the speed of a motor 
 */
void setSpeed(motorEnum motor, int pwm);

/*
 * Control the motors pased on channel parameters
 *    Yaw pitch and roll are PID values, throttle is %
 */ 
void controlMotors(int yaw, int pitch, int roll, int throttle);

/*
 * Used to set motor speed manually (for testing/debug)
 */ 
void motorDebug(void);

#endif /* MOTORS_H */
