#include <Arduino.h>

#include "motors.h"
#include "pinmap.h"

int motorMappings[4][4] = {{ 1, -1,  1, 1},  // pitch, roll, yaw, throttle
                           {-1,  1,  1, 1},
                           { 1,  1, -1, 1},
                           {-1, -1, -1, 1}};

// Motor classes
Servo motors[MOTORS_NUM];

/*
 * Arm all the motors
 */
static void armMotors(void)
{
#if (CALIBRATE == 1)
	Serial.println("Calibrating motors...");
	// arm the speed controller, modify as necessary for your ESC  
	for (int i = MOTOR_1; i <= MOTOR_4; i++)
	{
		setSpeed((motorEnum)i, MAX_THROTTLE);
	}

	Serial.println("Enable power now...");
	delay(5000);

	// arm the speed controller, modify as necessary for your ESC  
	for (int i = MOTOR_1; i <= MOTOR_4; i++)
	{
		setSpeed((motorEnum)i, MIN_THROTTLE);
	}

	delay(2000);

	Serial.println("Calibration finished...");
#endif /* CALIBRATE */

	// arm the speed controller, modify as necessary for your ESC  
	for (int i = MOTOR_1; i <= MOTOR_4; i++)
	{
		setSpeed((motorEnum)i, MIN_THROTTLE + 100);
	}

	delay(2000);
}

void setupMotors(void)
{
	Serial.println("Initializing motors...");
	motors[0].attach(MOTOR_1_PIN);
	motors[1].attach(MOTOR_2_PIN);
	motors[2].attach(MOTOR_3_PIN);
	motors[3].attach(MOTOR_4_PIN);

	armMotors();
}

void setSpeed(motorEnum motor, int pwm)
{
	if (motor >= MOTOR_1 && motor <= MOTOR_4)
	{
		int angle = map(constrain(pwm, MIN_THROTTLE, MAX_THROTTLE), MIN_THROTTLE, MAX_THROTTLE, 0, 180);
		Serial.println(pwm);
		motors[motor].write(angle);    
	}
}

void motorDebug(void)
{
	Serial.print("Enter pwm value: ");
	while(!Serial.available());
	int speed = Serial.parseInt();

	setSpeed(MOTOR_1, speed);
	setSpeed(MOTOR_2, speed);
	setSpeed(MOTOR_3, speed);
	setSpeed(MOTOR_4, speed);

	Serial.println(speed);
}

void controlMotors(int yaw, int pitch, int roll, int throttle)
{
	for (int i = 0; i < MOTORS_NUM; i++)
	{
		setSpeed((motorEnum)i, (pitch       * motorMappings[i][PITCH_MAPPING]) + 
							   (roll        * motorMappings[i][ROLL_MAPPING]) + 
							   (yaw         * motorMappings[i][YAW_MAPPING]) + 
							   (throttle * motorMappings[i][THROTTLE_MAPPING]));
	}
}
