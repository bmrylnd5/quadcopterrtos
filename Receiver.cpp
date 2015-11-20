// Receiver implementation for Quadcopter. 

#include <Arduino.h>
#include <stdio.h>

#include "pinmap.h"
#include "Receiver.h"

// Structure holding data used to calculate a PWM duty cycle via timer ticks.
typedef struct
{
   unsigned long ticksStart;  // Number of timer ticks present at the last interrupt.
   unsigned long ticksHigh;   // Number of timer ticks while HIGH.
   unsigned long ticksLow;    // Number of timer ticks while LOW.
} pwmTickCount;

static const unsigned int PWM_IN_NUM   = 3;     // Number of input PWM signals.
static const unsigned int STALE_THRESH = 20000; // Threshold in us for last reception
static const int BASE_VAL              = 50;    // Center command value

// Protects critical section
static volatile bool mIsrDataInUse[PWM_IN_NUM];

// Tracks each timer data for each PWM input.
static volatile pwmTickCount mPwmLastCount[PWM_IN_NUM];

Channel::Channel(unsigned int pin, int error) :
   mPin(pin),
   mError(error)
{
}

unsigned int Channel::GetPin() { return mPin; }
int Channel::GetError() { return mError; }

Receiver::Receiver() :
   mYaw(REC_CHAN_1_PIN, 8),
   mPitch(REC_CHAN_3_PIN, -1),
   mRoll(0, 0),
   mThrottle(REC_CHAN_2_PIN, 2),
   mArm(0, 0)
{
   for (unsigned int i = 0; i < PWM_IN_NUM; i++)
   {
      mPwmLastCount[i].ticksStart = 0;
      mPwmLastCount[i].ticksHigh = 0;
      mPwmLastCount[i].ticksLow = 0;
      mIsrDataInUse[i] = false;
   }
}

unsigned int Receiver::PinIndex(unsigned int pin)
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
   
void Receiver::PwmInIsr(unsigned int pin)
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

void Receiver::SetupReceiver() 
{   
   attachInterrupt(digitalPinToInterrupt(REC_CHAN_1_PIN), PwmIn1Isr, CHANGE);
   attachInterrupt(digitalPinToInterrupt(REC_CHAN_2_PIN), PwmIn2Isr, CHANGE);
   attachInterrupt(digitalPinToInterrupt(REC_CHAN_3_PIN), PwmIn3Isr, CHANGE);
}

void Receiver::PrintDebug(unsigned int chanNum, unsigned int &dutyCycle, unsigned long &lastLow, unsigned long &lastHigh)
{
   char buf[256];
   snprintf(buf, sizeof(buf), "  Chan %u: %u%% %luus period",
                              chanNum, dutyCycle, lastLow + lastHigh);
   Serial.println(buf);
}

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
   
   // check ARM command for last update time to ensure we are actively receiving data
   if (abs(micros() - mPwmLastCount[0].ticksStart) > STALE_THRESH)
   {
      // ERROR
      yaw = BASE_VAL;
      pitch = BASE_VAL;
      roll = BASE_VAL;
      throttle = BASE_VAL;
      arm = BASE_VAL;
   }
   else
   {
      for (unsigned int i = 0; i < PWM_IN_NUM; i++)
      {
         // calculate duty cycle based on last high count vs total number of ticks in period
         // note that this shifts value by a factor of 10 to normalize between 50% and 100%
         dutyCycle[i] = (lastHigh[i] * 1000) / (lastHigh[i] + lastLow [i]);
         
         // normalize duty cycle around 50%
         dutyCycle[i] = (dutyCycle[i] - BASE_VAL) * 2;

         //PrintDebug(i + 1, dutyCycle[i], lastLow[i], lastHigh[i]);
      }
      
      yaw      = dutyCycle[PinIndex(mYaw.GetPin())] + mYaw.GetError();
      throttle = dutyCycle[PinIndex(mThrottle.GetPin())] + mThrottle.GetError();
      pitch    = dutyCycle[PinIndex(mPitch.GetPin())] + mPitch.GetError();
   }
}