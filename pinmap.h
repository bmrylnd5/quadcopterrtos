// Pin mapping used for Quadcopter implementation
// This is currently for UNO. If there are differences create pre-compiler directives.

#ifndef PINMAP_H
#define PINMAP_H

#define IMU_INT_PIN  3  // Pin used for IMU interrupt (PCINT 5)

// Mega using timers 2 and 4
#define MOTOR_1_PIN  6  // Pin used for NE motor PWM
#define MOTOR_2_PIN  7 // Pin used for NW motor PWM
#define MOTOR_3_PIN  8 // Pin used for SE motor PWM
#define MOTOR_4_PIN  9 // Pin used for SW motor PWM

// Mega receiver channel inputs 
// external interrupts (PCINT 2,3,4)
#define REC_CHAN_1_PIN  2  // Pin used for receiver channel yaw
#define REC_CHAN_2_PIN  18 // Pin used for receiver channel throttle
#define REC_CHAN_3_PIN  19 // Pin used for receiver channel pitch
// pin change interrupts (PCINT 9,10)
#define REC_CHAN_4_PIN  14 // Pin used for receiver channel roll
#define REC_CHAN_5_PIN  15 // Pin used for receiver channel arm

/*
 * The following pins/timers pairings are listed below for reference:
 *       UNO   Mega
 * SDA   A4    20
 * SCL   A5    21
 *
 * Mega Timers PWM Pins    Libraries
 * Timer 0     4, 13       8-bit system timer
 * Timer 1     11, 12      16-bit
 * Timer 2     9, 10       8-bit
 * Timer 3     2, 3, 5     16-bit
 * Timer 4     6, 7, 8     16-bit
 * Timer 5     44, 45, 46  16-bit Servo library (up to 12 servos)
 */
 
#endif