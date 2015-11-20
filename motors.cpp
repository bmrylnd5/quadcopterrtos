#include <Arduino.h>

#include "motors.h"
#include "pinmap.h"

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