// IMU implementation for Quadcopter. 

//necessary to add before EnableInterrupt to avoid collision (all but 1 file)
#define LIBCALL_ENABLEINTERRUPT
#include <EnableInterrupt.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif


#include "pinmap.h"
#include "IMU.h"

// indicates whether MPU interrupt pin has gone high
static volatile bool mpuInterrupt = false;
   
IMU::IMU() :
   mDmpReady(false),
   mPacketSize(42)
{   
}

void IMU::DmpDataReady()
{
   mpuInterrupt = true;
}

void IMU::SetupIMU() 
{
   uint8_t devStatus;   // return status after device operation (0 = success, !0 = error)

   // Init I2C bus
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
   Wire.begin();
   TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
   Fastwire::setup(400, true);
#endif

   // Init IMU
   Serial.println(F("Initializing I2C devices..."));
   mpu.initialize();

   // Verify connection
   Serial.println(F("Testing device connections..."));
   Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

   // load and configure the DMP
   Serial.println(F("Initializing DMP..."));
   devStatus = mpu.dmpInitialize();

   // Ensure we are good to go
   if (devStatus == 0) 
   {
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);
	  
      // Enable low pass filtering of readings
      mpu.setDLPFMode(MPU6050_DLPF_BW_42);

      // enable Arduino interrupt detection
      Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
      enableInterrupt(IMU_INT_PIN, DmpDataReady, RISING);

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      mDmpReady = true;

      // get expected DMP packet size for later comparison
      mPacketSize = mpu.dmpGetFIFOPacketSize();
   } 
   else 
   {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
   }
}

void IMU::ReadIMU(float &yaw, float &pitch, float &roll) 
{
   uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
   uint8_t fifoBuffer[64]; // FIFO storage buffer
   Quaternion q;           // [w, x, y, z]         quaternion container
   VectorFloat gravity;    // [x, y, z]            gravity vector
   float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

    // if programming failed, don't try to do anything
    if (!mDmpReady) 
    {
      return;
    }

   // wait for MPU interrupt or extra packet(s) available
   if (!mpuInterrupt && (mFifoCount < mPacketSize)) 
   {
      // Do nothing
      return;
   }

   // reset interrupt flag and get INT_STATUS byte
   mpuInterrupt = false;
   mpuIntStatus = mpu.getIntStatus();

   // get current FIFO count
   mFifoCount = mpu.getFIFOCount();

   // check for overflow (this should never happen unless our code is too inefficient)
   if ((mpuIntStatus & 0x10) || (mFifoCount == 1024)) 
   {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));
   } 
   // otherwise, check for DMP data ready interrupt (this should happen frequently)
   else if (mpuIntStatus & 0x02) 
   {
      // wait for correct available data length, should be a VERY short wait
      while (mFifoCount < mPacketSize) 
      {
         mFifoCount = mpu.getFIFOCount();
      }

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, mPacketSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      mFifoCount -= mPacketSize;

      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      yaw = ypr[0] * 180/M_PI;
      pitch = ypr[2] * 180/M_PI;
      roll = ypr[1] * 180/M_PI;
   }
   else
   {
      Serial.print(F("unhandled mpu int status 0x"));
      Serial.print(mpuIntStatus, HEX);
   }
}