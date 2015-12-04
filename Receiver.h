#ifndef RECEIVER_H
#define RECEIVER_H

const int BASE_VAL_DEG = 0;     // Center command value (degrees)

// Encapsulates a receiver channel
class Channel
{
 public:
   Channel(const unsigned int pin, const int error);
   
   inline unsigned int GetPin()   const { return mPin; }
   inline int GetError()          const { return mError; }
   
 private:
   unsigned int mPin;   // input pin associated with channel
   int mError;          // Correctional value to achieve neutral base command
};

class Receiver
{
 public:
   Receiver();
   
   /*
    * Performs Receiver initialization.
    */
   void SetupReceiver();

   /*
    * Main Receiver loop reading yaw, pitch, roll, throttle, and arm commands.
    * YPR values range from -45 to 45
    * Throttle values range from MIN_THROTTLE to MAX_THROTTLE
    * ARM values range from 0 to 100
    */
   void ReadReceiver(int &yaw, int &pitch, int &roll, int &throttle, int &arm);

 private:
   // Main ISR for PWM calculation
   // Read timer and calculate the number of ticks while the PWM input is HIGH
   static void PwmInIsr(const unsigned int pin);

   // ISR for specific channels
   static void PwmIn1Isr();
   static void PwmIn2Isr();
   static void PwmIn3Isr();
   static void PwmIn4Isr();
   static void PwmIn5Isr();

   void PrintDebug(const unsigned int chanNum, 
                   const unsigned int &dutyCycle, 
                   const unsigned long &lastLow, 
                   const unsigned long &lastHigh);

   Channel mYaw;
   Channel mPitch;
   Channel mRoll;
   Channel mThrottle;
   Channel mArm;
};

#endif /* RECEIVER_H */