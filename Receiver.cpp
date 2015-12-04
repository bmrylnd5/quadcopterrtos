// Receiver implementation for Quadcopter. 

#include <Arduino.h>
#include <EnableInterrupt.h>

#include "motors.h"
#include "pid.h"
#include "pinmap.h"
#include "Receiver.h"

#define REC_DEBUG 0
#if (REC_DEBUG == 1)
#include <stdio.h>
#endif

// Structure holding data used to calculate a PWM duty cycle via timer ticks.
typedef struct
{
   unsigned long ticksStart;  // Number of timer ticks present at the last interrupt.
   unsigned long ticksHigh;   // Number of timer ticks while HIGH.
   unsigned long ticksLow;    // Number of timer ticks while LOW.
} pwmTickCount;

const unsigned int PWM_IN_NUM    = 5;     // Number of input PWM signals.
const unsigned int STALE_THRESH  = 60000; // Threshold in us for last reception
const int BASE_VAL_DUTY          = 50;    // Center command value (duty cycle)

// Protects critical section
static volatile bool mIsrDataInUse[PWM_IN_NUM];

// Tracks each timer data for each PWM input.
static volatile pwmTickCount mPwmLastCount[PWM_IN_NUM];

// Array offset for the specified PIN
static unsigned int PinIndex(const unsigned int pin)
{
   switch(pin)
   {
      case REC_CHAN_1_PIN:
         return 0;
      case REC_CHAN_2_PIN:
         return 1;
      case REC_CHAN_3_PIN:
         return 2;
      case REC_CHAN_4_PIN:
         return 3;
      case REC_CHAN_5_PIN:
         return 4;
      default:
         return 0;
   }
}

Channel::Channel(const unsigned int pin, const int error) :
   mPin(pin),
   mError(error)
{
}

Receiver::Receiver() :
   mYaw(REC_CHAN_4_PIN, 0),
   mPitch(REC_CHAN_2_PIN, 0),
   mRoll(REC_CHAN_1_PIN, 0),
   mThrottle(REC_CHAN_3_PIN, 0),
   mArm(REC_CHAN_5_PIN, 0)
{
   for (unsigned int i = 0; i < PWM_IN_NUM; i++)
   {
      mPwmLastCount[i].ticksStart = 0;
      mPwmLastCount[i].ticksHigh = 0;
      mPwmLastCount[i].ticksLow = 0;
      mIsrDataInUse[i] = false;
   }
}
   
void Receiver::PwmInIsr(const unsigned int pin)
{
   unsigned int pinIndex;
   unsigned long tickNow;
  
   pinIndex = PinIndex(pin);

    // Exit if in critical section as read count is not atomic.
   if(mIsrDataInUse[pinIndex])
   {
      return;
   }
   
   // capture current timer count
   tickNow = micros();

   // skip overflow 
   if (tickNow > mPwmLastCount[pinIndex].ticksStart)
   {
      // determine if rising or falling edge
      if (digitalRead(pin) == HIGH)
      {
         //get ticks while LOW
         mPwmLastCount[pinIndex].ticksLow = tickNow - mPwmLastCount[pinIndex].ticksStart;
      }
      else
      {  
         //get ticks while HIGH
         mPwmLastCount[pinIndex].ticksHigh = tickNow - mPwmLastCount[pinIndex].ticksStart;
      }
   }

   // update last tick count
   mPwmLastCount[pinIndex].ticksStart = tickNow;
}

void Receiver::PwmIn1Isr(void)
{
   PwmInIsr(REC_CHAN_1_PIN);
}

void Receiver::PwmIn2Isr(void)
{
   PwmInIsr(REC_CHAN_2_PIN);
}

void Receiver::PwmIn3Isr(void)
{
   PwmInIsr(REC_CHAN_3_PIN);
}

void Receiver::PwmIn4Isr(void)
{
   PwmInIsr(REC_CHAN_4_PIN);
}

void Receiver::PwmIn5Isr(void)
{
   PwmInIsr(REC_CHAN_5_PIN);
}

void Receiver::SetupReceiver() 
{   
   pinMode(REC_CHAN_1_PIN, INPUT_PULLUP);
   pinMode(REC_CHAN_2_PIN, INPUT_PULLUP);
   pinMode(REC_CHAN_3_PIN, INPUT_PULLUP);
   pinMode(REC_CHAN_4_PIN, INPUT_PULLUP);
   pinMode(REC_CHAN_5_PIN, INPUT_PULLUP);
   
   // external interrupts
   enableInterrupt(REC_CHAN_1_PIN, PwmIn1Isr, CHANGE);
   enableInterrupt(REC_CHAN_2_PIN, PwmIn2Isr, CHANGE);
   enableInterrupt(REC_CHAN_3_PIN, PwmIn3Isr, CHANGE);
   
   // pin change interrupts
   enableInterrupt(REC_CHAN_4_PIN, PwmIn4Isr, CHANGE);
   enableInterrupt(REC_CHAN_5_PIN, PwmIn5Isr, CHANGE);
}

#if (REC_DEBUG == 1)
void Receiver::PrintDebug(const unsigned int chanNum, 
                          const unsigned int &dutyCycle, 
                          const unsigned long &lastLow, 
                          const unsigned long &lastHigh)
{
   char buf[256];
   snprintf(buf, sizeof(buf), "  Chan %u: %u%% %luus period",
                              chanNum, dutyCycle, lastLow + lastHigh);
   Serial.println(buf);
}
#endif

void Receiver::ReadReceiver(int &yaw, int &pitch, int &roll, int &throttle, int &arm)
{
   unsigned long lastHigh[PWM_IN_NUM];
   unsigned long lastLow[PWM_IN_NUM];
   unsigned int dutyCycle[PWM_IN_NUM];

   for (unsigned int i = 0; i < PWM_IN_NUM; i++)
   {
      // Critical section
      mIsrDataInUse[i] = true;
      lastHigh[i] = mPwmLastCount[i].ticksHigh;
      lastLow[i] = mPwmLastCount[i].ticksLow;
      mIsrDataInUse[i] = false;
   }
   
   // check command for last update time to ensure we are actively receiving data
   // Note: must use an external interrupt pin
   if (abs(micros() - mPwmLastCount[PinIndex(REC_CHAN_1_PIN)].ticksStart) > STALE_THRESH)
   {
      // ERROR
      yaw      = BASE_VAL_DEG;
      pitch    = BASE_VAL_DEG;
      roll     = BASE_VAL_DEG;
      throttle = MIN_THROTTLE;
      arm      = 0;
      
      Serial.println(F("Receiver values stale, using default"));
   }
   else
   {
      for (unsigned int i = 0; i < PWM_IN_NUM; i++)
      {
         // calculate duty cycle based on last high count vs total number of ticks in period
         // note that this shifts value by a factor of 10 to normalize between 50% and 100%
         dutyCycle[i] = (lastHigh[i] * 1000) / (lastHigh[i] + lastLow [i]);
         
         // normalize duty cycle around 50%
         dutyCycle[i] = (dutyCycle[i] - BASE_VAL_DUTY) * 2;

#if(REC_DEBUG == 1)
         PrintDebug(i + 1, dutyCycle[i], lastLow[i], lastHigh[i]);
#endif
      }
      
      // account for error then convert to degrees (-45 to 45)
      yaw      = map(dutyCycle[PinIndex(mYaw.GetPin())]   + mYaw.GetError(),   0, 100, YAW_UPPER_LIMIT, YAW_LOWER_LIMIT);
      pitch    = map(dutyCycle[PinIndex(mPitch.GetPin())] + mPitch.GetError(), 0, 100, PITCH_UPPER_LIMIT, PITCH_LOWER_LIMIT);
      roll     = map(dutyCycle[PinIndex(mRoll.GetPin())]  + mRoll.GetError(),  0, 100, ROLL_UPPER_LIMIT, ROLL_LOWER_LIMIT);

      // do not convert to degrees
      throttle = map(dutyCycle[PinIndex(mThrottle.GetPin())] + mThrottle.GetError(), 0, 100, MIN_THROTTLE, MAX_THROTTLE);
      arm      = dutyCycle[PinIndex(mArm.GetPin())] + mArm.GetError();

      yaw      = constrain(yaw, YAW_LOWER_LIMIT, YAW_UPPER_LIMIT);
      pitch    = constrain(pitch, PITCH_LOWER_LIMIT, PITCH_UPPER_LIMIT);
      roll     = constrain(roll, ROLL_LOWER_LIMIT, ROLL_UPPER_LIMIT);
      throttle = constrain(throttle, MIN_THROTTLE, MAX_THROTTLE);
   }
}