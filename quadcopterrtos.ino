extern "C" 
{
#include <Servo.h>

#include "motors.h"
#include "pid.h"
}
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

/* commands */
static float arm         = 0.0;
static float pitchCmd    = 0.0;
static float rollCmd     = 0.0;
static float yawCmd      = 0.0;
static float throttleCmd = 0.0;
static float newPitchCmd = 0.0;
static float newRollCmd  = 0.0;
static float newYawCmd   = 0.0;

/* IMU readings */
static float pitchDeg = 0.0;
static float rollDeg  = 0.0;
static float yawDeg   = 0.0;

/* Motor classes */
static Servo motors[MOTORS_NUM];

/*
* And this is the IMU stuff that I would put in its own class... IF ARDUINO WOULD LET ME COMPILE ONE"
*/
MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)

/*
 * ISR for IMU feedback
 */
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() 
{
    mpuInterrupt = true;
}

/*
 * Setup
 */
void setupIMU() {
	uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
	
    // Init I2C bus
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

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
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

/*
 * Main
 * Loops reading and printing pitch, roll, and yaw.
 */
void ReadIMU(float &pitch, float &roll, float &yaw) {
	uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
	uint16_t fifoCount;     // count of all bytes currently in FIFO
	uint8_t fifoBuffer[64]; // FIFO storage buffer
	Quaternion q;           // [w, x, y, z]         quaternion container
	VectorFloat gravity;    // [x, y, z]            gravity vector
	float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
	
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // Do nothing
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		
		yaw = ypr[0] * 180/M_PI;
		pitch = ypr[1] * 180/M_PI;
		roll = ypr[2] * 180/M_PI;
    }
}


/*
* Setup
*/
void setup()
{
  Serial.begin(115200);
  setupIMU();
}

/*
* Main Loop
*/
void loop() 
{
  /* read receiver */

  /* quadcopter must be armed to fly */
  if (arm < MID_THROTTLE)
  {
	/* read IMU for each channel - in degrees */
	ReadIMU(pitchDeg, rollDeg, yawDeg);
	Serial.print(yawDeg);
	Serial.print(",");
	Serial.print(pitchDeg);
	Serial.print(",");
	Serial.print(rollDeg);
	Serial.print(",");
  
    /* adjust command using PID - in degrees */
    newPitchCmd = pidPitch(pitchCmd, pitchDeg);
    newRollCmd  = pidRoll(rollCmd,   rollDeg);
    newYawCmd   = pidYaw(yawCmd,     yawDeg);
	Serial.print(newPitchCmd);
	Serial.print(",");
	Serial.print(newRollCmd);
	Serial.print(",");
	Serial.println(newYawCmd);
  
    /* convert degrees to PWM (us) */
    
    /* output to motors - in microseconds */
  }
  else
  {
    /* turn off motors */
  }
}
