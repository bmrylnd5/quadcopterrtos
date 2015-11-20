#ifndef RECEIVER_H
#define RECEIVER_H

// Encapsulates a receiver channel
class Channel
{
 public:
   Channel(unsigned int pin, int error);
   unsigned int GetPin();
   int GetError();
   
 private:
   // input pin associated witch channel
   unsigned int mPin;
   
   // Correctional value to achive neutral base command
   int mError;
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
    * Values range from 0 to 100  with 50 being the base command
    */
   void ReadReceiver(int &yaw, int &pitch, int &roll, int &throttle, int &arm);

 private:
   // Main ISR for PWM calculation
   // Read timer and calculate the number of ticks while the PWM input is HIGH
   static inline void PwmInIsr(unsigned int pin);
   
   // Array offset for the specified PIN
   static inline unsigned int PinIndex(unsigned int pin);
   
   // ISR for specific channels
   static void PwmIn1Isr();
   static void PwmIn2Isr();
   static void PwmIn3Isr();
   static void PwmIn4Isr();
   static void PwmIn5Isr();

   void PrintDebug(unsigned int chanNum, unsigned int &dutyCycle, unsigned long &lastLow, unsigned long &lastHigh);

   Channel mYaw;
   Channel mPitch;
   Channel mRoll;
   Channel mThrottle;
   Channel mArm;
};

#endif /* RECEIVER_H */