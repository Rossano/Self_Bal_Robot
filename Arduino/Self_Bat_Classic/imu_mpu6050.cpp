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
#include <MPU6050_6Axis_MotionApps20.h>
#include <helper_3dmath.h>

// Flag to identify the use of NILRTOS
#define USE_NILRTOS				0	// 0 standard sketch, 1 NILRTOS

#if (USE_NILROTS == 1)
	#include <NilRTOS.h>
#endif

#include "board.h"
#include "imu_mpu6050.h"


MPU6050 mpu;

bool dmpReady;
bool mpuInterrupt = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q, lastQ(0,0,0,0);
float ypr[3];
VectorFloat gravity;

//extern SEMAPHORE_DECL(dmpSem, 1);
bool blinkState;
bool isConnected;

#if (USE_NILRTOS == 1)
	SEMAPHORE_DECL(dmpSem, 1);
#endif

void imu_init()
{

	uint8_t count = 10;

    // initialize device
    Serial.println("Initializing I2C devices...");
    mpu.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
/*
    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
*/
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
	do {

		devStatus = mpu.dmpInitialize();

		// make sure it worked (returns 0 if so)
		if (devStatus == 0)
		{
			count = 10;
			// turn on the DMP, now that it's ready
			Serial.println("Enabling DMP...");
			mpu.setDMPEnabled(true);

			mpuIntStatus = mpu.getIntStatus();

			// set our DMP Ready flag so the main loop() function knows it's okay to use it
			Serial.println("DMP ready! Waiting for first interrupt...");
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
			Serial.print("DMP Initialization failed (code ");
			Serial.print(devStatus);
			Serial.println(")");
			// New attempt message
			Serial.println("Trying again");
		}
	}
	while (--count);

	// configure LED for output
	pinMode(SOL_LED, OUTPUT);

	// Check if the configuration has failed
//	if (!count)
	{
		Serial.println("DMP initialization failed");
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

	Serial.println("DMP is ready!\nAwaiting for IRQ ready flag");
	// wait for MPU interrupt or extra packet(s) available
/*	while (!mpuInterrupt && fifoCount < packetSize) {
//		Serial.println("Shouldn't get here...");
	}
	Serial.println("DMP flag OK");
*/
	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		// reset so we can continue cleanly
		mpu.resetFIFO();
		Serial.println("FIFO overflow!");

		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	}
	else if (mpuIntStatus & 0x02) {
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);

		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;

		// display Euler angles in degrees
		mpu.dmpGetQuaternion(&q, fifoBuffer);

		if(lastQ == q)
		{
			if(--count)
			{
				Serial.println("*****************\nLOCKED\n************************");
				dmpReady = false;
				Serial.println("MPU stalled, reinitializing...");
				mpu.reset();
				if ((devStatus = mpu.dmpInitialize()) == 0)
				{
					mpu.setDMPEnabled(true);
					mpuIntStatus = mpu.getIntStatus();
					dmpReady = true;
					packetSize = mpu.dmpGetFIFOPacketSize();
				}
				else {
					Serial.print("DMP reinitialization failed (code ");
					Serial.print(devStatus);
					Serial.println(")");
					while (true) {
						//delay(300);
#if (USE_NILRTOS == 1)
						nilThdSleep(300);
#else
						delay(300);
#endif
						digitalWrite(SOL_LED, 0);
						//delay(300);
#if (USE_NILRTOS == 1)
						nilThdSleep(300);
#else
						delay(300);
#endif
						digitalWrite(SOL_LED, 1);
					}
				}
			}
		}
	}
	else {
		Serial.print("OK ");
		count = 3;
		lastQ = q;
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		Serial.print("ypr\t");
		Serial.print(ypr[0] * 180/M_PI);
		Serial.print("\t");
		Serial.print(ypr[1] * 180/M_PI);
		Serial.print("\t");
		Serial.println(ypr[2] * 180/M_PI);
	}

	// blink LED to indicate activity
	blinkState = !blinkState;
	digitalWrite(SOL_LED, blinkState);

#if (USE_NILRTOS == 1)
	nilThdSleep(1);
#else
	delay(1);
#endif

}

void imu_reset()
{
	mpu.reset();
	imu_init();
}

void imu_isr()
{
	Serial.print("ISR");
#if (USE_NILRTOS == 1)
	NIL_IRQ_PROLOGUE();
#endif
	mpuInterrupt = true;
#if (USE_NILRTOS == 1)
	nilSysLockFromISR();
	nilSemSignalI(&dmpSem);
	nilSysUnlockFromISR();
	NIL_IRQ_EPILOGUE();
#endif
}





