#ifndef MOTORS_H
#define MOTORS_H

#include <Servo.h>

#define CALIBRATE    0 /* turn on to calibrate motors */
#define MOTORS_NUM   4

/*  min/max throttles for each motor */
#define MAX_THROTTLE 1900
#define MID_THROTTLE 1500
#define MIN_THROTTLE 1100

/* mode 2 */
#define ROLL_CHANNEL     1
#define PITCH_CHANNEL    2
#define THROTTLE_CHANNEL 3
#define YAW_CHANNEL      4
#define ARM_CHANNEL      5

typedef enum
{
	MOTOR_1 = 0,
	MOTOR_2 = 1,
	MOTOR_3 = 2,
	MOTOR_4 = 3
} motorEnum;

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
 * Used to set motor speed manually (for testing/debug)
 */ 
void motorDebug(void);

#endif /* MOTORS_H */
