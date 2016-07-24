/*
 * imu_mpu6050.cpp
 *
 * Created: 25/03/2015 23:00:38
 *  Author: rpantaleoni
 */ 
/*
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <helper_3dmath.h>
*/

#include <I2Cdev.h>
//#include <MPU6050.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include <Wire.h>
#endif

#include <MPU6050_6Axis_MotionApps20.h>
#include <helper_3dmath.h>

/*
 * Firmware Configuration session
 */
#undef USE_NILRTOS
#undef IRQ_DEBUG
#undef DEBUG
//#define __BOARD_YUN__

#ifdef ARDUINO_AVR_YUN //__BOARD_YUN__
#include <Bridge.h>
//#include <Console.h>
#endif

#ifdef USE_NILRTOS
#include <NilRTOS.h>
#endif

#include "board.h"
#include "imu_mpu6050.h"

#ifdef USE_NILRTOS
#define sleep(a)	nilThdSleep(a)
#else
#define sleep(a)	delay(a)
#endif

MPU6050 mpu;

bool dmpReady;
volatile bool mpuInterrupt = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q, lastQ(0,0,0,0);
float ypr[3];
int16_t gyro[3];
VectorFloat gravity;

//extern SEMAPHORE_DECL(dmpSem, 1);
bool blinkState;
bool isConnected;

#ifdef USE_NILRTOS
SEMAPHORE_DECL(dmpSem, 1);
#endif

void imu_init()
{
		
	uint8_t count = 10;
	
    // initialize device
#ifdef ARDUINO_AVR_YUN //__BOARD_YUN__
//	Console.println(F("Initializing I2C devices..."));
#else
    Serial.println(F("Initializing I2C devices..."));
#endif
    mpu.initialize();

    // verify connection
#ifdef ARDUINO_AVR_YUN //__BOARD_YUN__
    //Console.println(F("Testing device connections..."));
    //Console.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
#else
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
#endif
/*
    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
*/
    // load and configure the DMP
#ifdef ARDUINO_AVR_YUN //__BOARD_YUN__
    //Console.println(F("Initializing DMP..."));
#else
    Serial.println(F("Initializing DMP..."));
#endif    
	do {
	
		devStatus = mpu.dmpInitialize();
		// Set some offset to the MEMS
		mpu.setXGyroOffset(220);
		mpu.setYGyroOffset(76);
		mpu.setZGyroOffset(-85);
		mpu.setZAccelOffset(1788);
		// make sure it worked (returns 0 if so)
		if (devStatus == 0) 
		{
			count = 10;
			// turn on the DMP, now that it's ready
#ifdef ARDUINO_AVR_YUN //__BOARD_YUN__
			//Console.println(F("Enabling DMP..."));
#else			
			Serial.println(F("Enabling DMP..."));
#endif
			mpu.setDMPEnabled(true);

			mpuIntStatus = mpu.getIntStatus();

			// set our DMP Ready flag so the main loop() function knows it's okay to use it
#ifdef ARDUINO_AVR_YUN //__BOARD_YUN__
			//Console.println(F("DMP ready! Waiting for first interrupt..."));
#else
			Serial.println(F("DMP ready! Waiting for first interrupt..."));
#endif
			dmpReady = true;

			// get expected DMP packet size for later comparison
			packetSize = mpu.dmpGetFIFOPacketSize();
			return;
		} 
		else {
			// ERROR!
			// 1 = initial memory load failed
			// 2 = DMP configuration updates failed
			// (if it's going to break, usually the code will be 1)
#ifdef ARDUINO_AVR_YUN //__BOARD_YUN__
			//Console.print(F("DMP Initialization failed (code"));
			//Console.print(devStatus);
			//Console.println(F(")"));
			// New attempt message
			//Console.println(F("Trying again"));
#else
			Serial.print(F("DMP Initialization failed (code "));
			Serial.print(devStatus);
			Serial.println(F(")"));
			// New attempt message
			Serial.println(F("Trying again"));
#endif
		}
	}
	while (--count);
	
	// configure LED for output	
	pinMode(SOL_LED, OUTPUT);
	
	// Check if the configuration has failed
//	if (!count) 
	{	
#ifdef ARDUINO_AVR_YUN //__BOARD_YUN__
		//Console.println(F("DMP initializaion failed"));
#else
		Serial.println(F("DMP initialization failed"));
#endif
		while (true) 
		{
			// Locks in infinite loop
			digitalWrite(SOL_LED, HIGH);
			delay(300);
			digitalWrite(SOL_LED, LOW);
			delay(300);
		}
	}
	
}

void imu_read()
{
	uint8_t count = 3;
			
	// if programming failed, don't try to do anything
	if (!dmpReady) return;

#ifdef IRQ_DEBUG
	#ifdef ARDUINO_AVR_YUN //__BOARD_YUN__
	//Console.println(F("DMP is ready!\nAwaiting for IRQ ready flag"));
	#else	
	Serial.println(F("DMP is ready!\nAwaiting for IRQ ready flag"));
	#endif
#endif
	// wait for MPU interrupt or extra packet(s) available
/*	while (!mpuInterrupt && fifoCount < packetSize) {
//		Serial.println("Shouldn't get here...");
	}
	Serial.println("DMP flag OK");
*/
	while(!mpuInterrupt && fifoCount < packetSize) ;
	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		// reset so we can continue cleanly
		mpu.resetFIFO();
#ifdef ARDUINO_AVR_YUN //__BOARD_YUN__
		//Console.println(F("FIFO overflow!"));
#else
		Serial.println(F("FIFO overflow!"));
#endif

		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	} 
	else if (mpuIntStatus & 0x02) {
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize)  fifoCount = mpu.getFIFOCount();

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);
			
		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;

		// display Euler angles in degrees
		mpu.dmpGetQuaternion(&q, fifoBuffer);

#ifdef __USE_ANTILOCK__			

		if(lastQ == q) 
		{
			if(--count) 
			{
#ifdef ARDUINO_AVR_YUN //__BOARD_YUN__
				//Console.println(F("\nLOCKED\n"));
				dmpReady = false;
				//Console.println(F("DMP is stalled, reinitializing..."));
#else
				Serial.println(F("\nLOCKED\n"));
				dmpReady = false;
				Serial.println(F("DMP stalled, reinitializing..."));
#endif
				mpu.reset();
				if ((devStatus = mpu.dmpInitialize()) == 0)
				{
					mpu.setDMPEnabled(true);
					mpuIntStatus = mpu.getIntStatus();
					dmpReady = true;
					packetSize = mpu.dmpGetFIFOPacketSize();
				}
				else {
#ifdef ARDUINO_AVR_YUN //__BOARD_YUN__
					//Console.print(F("DMP reinitialization failed (code "));
					//Console.print(devStatus);
					//Console.println(F(")"));
#else
					Serial.print(F("DMP reinitialization failed (code "));
					Serial.print(devStatus);
					Serial.println(")");
#endif
					while (true) {
						//delay(300);
						//nilThdSleep(300);
						sleep(300);
						digitalWrite(SOL_LED, 0);
						//delay(300);
						//nilThdSleep(300);
						sleep(300);
						digitalWrite(SOL_LED, 1);
					}
				}
			}
		}
#endif

	}
	else {
#ifdef IRQ_DEBUG
		#ifdef __BOARD_YUN__
		//Console.print(F("OK"));
		#else
		Serial.print(F("OK "));
		#endif
#endif
		count = 3;
		lastQ = q;
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		mpu.dmpGetGyro(gyro, fifoBuffer);
#ifdef DEBUG
	#ifdef ARDUINO_AVR_YUN //__BOARD_YUN__
		//Console.print(F("ypr gyro\t"));
		//Console.print(ypr[0] * 180/M_PI);
		//Console.print(F("\t"));
		//Console.print(ypr[1] * 180/M_PI);
		//Console.print(F("\t"));
		//Console.print(ypr[2] * 180/M_PI);
		//Console.print(F("\t"));
		//Console.println(gyro);
	#else
		Serial.print(F("ypr gyro\t"));
		Serial.print(ypr[0] * 180/M_PI);
		Serial.print(F("\t"));
		Serial.print(ypr[1] * 180/M_PI);
		Serial.print(F("\t"));
		Serial.print(ypr[2] * 180/M_PI);
		Serial.print(F("\t"));
		Serial.println(gyro);
	#endif
#endif
	}

	// blink LED to indicate activity
	blinkState = !blinkState;
	digitalWrite(SOL_LED, blinkState);
			
	//delay(1);
	//nilThdSleep(1);
	sleep(1);
}

void imu_reset()
{
	mpu.reset();
	imu_init();
}

void imu_isr()
{
#ifdef IRQ_DEBUG
	#ifdef ARDUINO_AVR_YUN //__BOARD_YUN__
	//Console.print(F("ISR"));
	#else
	Serial.print(F("ISR"));
	#endif
#endif
#ifdef USE_NILRTOS
	NIL_IRQ_PROLOGUE();
#endif
	mpuInterrupt = true;
#ifdef USE_NILRTOS
	nilSysLockFromISR();
	nilSemSignalI(&dmpSem);
	nilSysUnlockFromISR();
	NIL_IRQ_EPILOGUE();
#endif
}

