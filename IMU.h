#ifndef IMU_H
#define IMU_H

#include "MPU6050.h"

class IMU
{
 public:
   IMU();
   
   /*
    * Performs IMU initialization.
    */
   void SetupIMU();

   /*
    * Main IMU loop reading yaw, pitch, and roll.
    */
   void ReadIMU(double &yaw, double &pitch, double &roll);

 private:
    // ISR for IMU feedback
   static void DmpDataReady();

   MPU6050 mpu;

   // Count of all bytes currently in FIFO. This persists across IMU loop to handle overflow.
   uint16_t mFifoCount;

   // set true if DMP init was successful
   bool mDmpReady;
   
   // expected DMP packet size (default is 42 bytes)
   uint16_t mPacketSize;
};

#endif /* IMU_H */