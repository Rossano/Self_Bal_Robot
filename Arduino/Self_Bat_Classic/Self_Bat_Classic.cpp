// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

//#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

#include "imu_mpu6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "board.h"

// Do not remove the include below
#include "Self_Bat_Classic.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
//MPU6050 mpu;
//The setup function is called once at startup of the sketch

#define PROMPT	"CALLOGERO> "

void initialize_robot();

void setup()
{
	// Add your initialization code here
	uint8_t count = 10;
	// join I2C bus (I2Cdev library doesn't do this automatically)
	Wire.begin();
	// initialize serial communication
	// (115200 chosen because it is required for Teapot Demo output, but it's
	// really up to you depending on your project)
	Serial.begin(115200);

//	while (!Serial); // wait for Leonardo enumeration, others continue immediately
//	// NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
//	// Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
//	// the baud timing being too misaligned with processor ticks. You must use
//	// 38400 or slower in these cases, or use some kind of external separate
//	// crystal solution for the UART timer.
//	pinMode(IRQ_PIN, INPUT);
//	digitalWrite(IRQ_PIN, HIGH);
//	// initialize device
//	Serial.println(F("Initializing I2C devices..."));
//	mpu.initialize();
//	// verify connection
//	Serial.println(F("Testing device connections..."));
//	Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
//	/*
//	// wait for ready
//	Serial.println(F("\nSend any character to begin DMP programming and demo: "));
//	while (Serial.available() && Serial.read()); // empty buffer
//	while (!Serial.available()); // wait for data
//	while (Serial.available() && Serial.read()); // empty buffer again
//	*/
//	// load and configure the DMP
//	Serial.println(F("Initializing DMP..."));
//	do {
//		devStatus = mpu.dmpInitialize();
//		// make sure it worked (returns 0 if so)
//		if (devStatus == 0) {
//			count = 10;
//			// turn on the DMP, now that it's ready
//			Serial.println(F("Enabling DMP..."));
//			mpu.setDMPEnabled(true);
//			// enable Arduino interrupt detection
//			Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
//			attachInterrupt(DMP_IRQ, dmpDataReady, RISING);
//			mpuIntStatus = mpu.getIntStatus();
//			// set our DMP Ready flag so the main loop() function knows it's okay to use it
//			Serial.println(F("DMP ready! Waiting for first interrupt..."));
//			dmpReady = true;
//			// get expected DMP packet size for later comparison
//			packetSize = mpu.dmpGetFIFOPacketSize();
//			// exit the loop since everything is configured
//			// configure LED for output
//			pinMode(LED_PIN, OUTPUT);
//			return;
//		} else {
//			// ERROR!
//			// 1 = initial memory load failed
//			// 2 = DMP configuration updates failed
//			// (if it's going to break, usually the code will be 1)
//			Serial.print(F("DMP Initialization failed (code "));
//			Serial.print(devStatus);
//			Serial.println(F(")"));
//			// New attempt message
//			Serial.println("Trying again");
//		}
//	}
//	while (--count);
//	// configure LED for output
//	pinMode(LED_PIN, OUTPUT);
//	// Check if the configuration has failed
//	// if (!count) {
//	Serial.println("DMP initialization failed");
//	while (true) {
//		// Locks in infinite loop
//		digitalWrite(LED_PIN, 0);
//		delay(300);
//		digitalWrite(LED_PIN, 1);
//		delay(300);
//	}
//	// }

	while (!Serial)
	{
		digitalWrite(13, HIGH);
		delay(300);
		digitalWrite(13, LOW);
		delay(300);
	}
	pinMode (IRQ_PORT, OUTPUT);
	digitalWrite(IRQ_PORT, LOW);
	initialize_robot();

	isConnected = true;

	Serial.print(PROMPT);
}

// The loop function is called in an endless loop
void loop()
{
	//Add your repeated code here
	// if programming failed, don't try to do anything

	imu_read();

//	if (!dmpReady) return;
//	// wait for MPU interrupt or extra packet(s) available
//	while (!mpuInterrupt && fifoCount < packetSize) {
//		// other program behavior stuff here
//		// .
//		// .
//		// .
//		// if you are really paranoid you can frequently test in between other
//		// stuff to see if mpuInterrupt is true, and if so, "break;" from the
//		// while() loop to immediately process the MPU data
//		// .
//		// .
//		// .
//	}
//	// reset interrupt flag and get INT_STATUS byte
//	mpuInterrupt = false;
//	mpuIntStatus = mpu.getIntStatus();
//	// get current FIFO count
//	fifoCount = mpu.getFIFOCount();
//	// check for overflow (this should never happen unless our code is too inefficient)
//	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
//		// reset so we can continue cleanly
//		mpu.resetFIFO();
//		Serial.println(F("FIFO overflow!"));
//		// otherwise, check for DMP data ready interrupt (this should happen frequently)
//	} else if (mpuIntStatus & 0x02) {
//		// wait for correct available data length, should be a VERY short wait
//		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
//		// read a packet from FIFO
//		mpu.getFIFOBytes(fifoBuffer, packetSize);
//		// track FIFO count here in case there is > 1 packet available
//		// (this lets us immediately read more without waiting for an interrupt)
//		fifoCount -= packetSize;
//		#ifdef OUTPUT_READABLE_QUATERNION
//		// display quaternion values in easy matrix form: w x y z
//		mpu.dmpGetQuaternion(&q, fifoBuffer);
//		Serial.print("quat\t");
//		Serial.print(q.w);
//		Serial.print("\t");
//		Serial.print(q.x);
//		Serial.print("\t");
//		Serial.print(q.y);
//		Serial.print("\t");
//		Serial.println(q.z);
//		#endif
//		#ifdef OUTPUT_READABLE_EULER
//		// display Euler angles in degrees
//		mpu.dmpGetQuaternion(&q, fifoBuffer);
//		mpu.dmpGetEuler(euler, &q);
//		Serial.print("euler\t");
//		Serial.print(euler[0] * 180/M_PI);
//		Serial.print("\t");
//		Serial.print(euler[1] * 180/M_PI);
//		Serial.print("\t");
//		Serial.println(euler[2] * 180/M_PI);
//		#endif
//		#ifdef OUTPUT_READABLE_YAWPITCHROLL
//		// display Euler angles in degrees
//		mpu.dmpGetQuaternion(&q, fifoBuffer);
//		if(lastQ == q) {
//			Serial.println("*****************\nLOCKED\n************************");
//			if(--count) {
//				dmpReady = false;
//				Serial.println("MPU stalled, reinitializing...");
//				mpu.reset();
//				if ((devStatus = mpu.dmpInitialize()) == 0)
//				{
//					mpu.setDMPEnabled(true);
//					mpuIntStatus = mpu.getIntStatus();
//					dmpReady = true;
//					packetSize = mpu.dmpGetFIFOPacketSize();
//				}
//				else {
//					Serial.print("DMP reinitialization failed (code ");
//					Serial.print(devStatus);
//					Serial.println(")");
//					while (true) {
//					delay(300);
//					digitalWrite(LED_PIN, 0);
//					delay(300);
//					digitalWrite(LED_PIN, 1);
//				}
//			}
//		}
//		}
//		else {
//			// Serial.print("OK ");
//			count = 3;
//			lastQ = q;
//			mpu.dmpGetGravity(&gravity, &q);
//			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//			//Serial.print("ypr\t");
//			Serial.print("DMP:");
//			Serial.print(ypr[0] * 180/M_PI);
//			//Serial.print("\t");
//			Serial.print(":");
//			Serial.print(ypr[1] * 180/M_PI);
//			//Serial.print("\t");
//			Serial.print(":");
//			Serial.println(ypr[2] * 180/M_PI);
//			delay(1);
//		}
//		#endif
//		#ifdef OUTPUT_READABLE_REALACCEL
//		// display real acceleration, adjusted to remove gravity
//		mpu.dmpGetQuaternion(&q, fifoBuffer);
//		mpu.dmpGetAccel(&aa, fifoBuffer);
//		mpu.dmpGetGravity(&gravity, &q);
//		mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//		Serial.print("areal\t");
//		Serial.print(aaReal.x);
//		Serial.print("\t");
//		Serial.print(aaReal.y);
//		Serial.print("\t");
//		Serial.println(aaReal.z);
//		#endif
//		#ifdef OUTPUT_READABLE_WORLDACCEL
//		// display initial world-frame acceleration, adjusted to remove gravity
//		// and rotated based on known orientation from quaternion
//		mpu.dmpGetQuaternion(&q, fifoBuffer);
//		mpu.dmpGetAccel(&aa, fifoBuffer);
//		mpu.dmpGetGravity(&gravity, &q);
//		mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//		mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
//		Serial.print("aworld\t");
//		Serial.print(aaWorld.x);
//		Serial.print("\t");
//		Serial.print(aaWorld.y);
//		Serial.print("\t");
//		Serial.println(aaWorld.z);
//		#endif
//		#ifdef OUTPUT_TEAPOT
//		// display quaternion values in InvenSense Teapot demo format:
//		teapotPacket[2] = fifoBuffer[0];
//		teapotPacket[3] = fifoBuffer[1];
//		teapotPacket[4] = fifoBuffer[4];
//		teapotPacket[5] = fifoBuffer[5];
//		teapotPacket[6] = fifoBuffer[8];
//		teapotPacket[7] = fifoBuffer[9];
//		teapotPacket[8] = fifoBuffer[12];
//		teapotPacket[9] = fifoBuffer[13];
//		Serial.write(teapotPacket, 14);
//		teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
//		#endif
//		// blink LED to indicate activity
//		blinkState = !blinkState;
//		digitalWrite(LED_PIN, blinkState);
//	}
}
// ================================================================
// === INTERRUPT DETECTION ROUTINE ===
// ================================================================
void dmpDataReady() {
	mpuInterrupt = true;
}

void initialize_robot(void)
{
	uint8_t count = 10;
	uint8_t devStatus;

	//Wire.begin();
	Wire.begin();

	/*pinMode(IRQ_PORT, INPUT);
	digitalWrite(IRQ_PORT, HIGH);
	attachInterrupt(DMP_IRQ, imu_isr, RISING); */

	imu_init();
	dmpReady = true;

	pinMode(SOL_LED, OUTPUT);

	pinMode(IRQ_PORT, INPUT);
	digitalWrite(IRQ_PORT, HIGH);
	attachInterrupt(DMP_IRQ, imu_isr, CHANGE);
	Serial.print("IRQ on pin: D");
	Serial.println(IRQ_PORT);
}
