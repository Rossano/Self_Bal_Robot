// Do not remove the include below
#include "Self_Balancing_Bot.h"

#include <Wire.h>

#include <I2Cdev.h>

#undef USE_NILRTOS

//#include <helper_3dmath.h>
//#include <MPU6050.h>
//#include <MPU6050_6Axis_MotionApps20.h>

//#include <MPU6050.h>
/*
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <helper_3dmath.h>
*/
#ifdef USE_NILRTOS
#include <NilRTOS.h>
#endif

#include "board.h"
#include "imu_mpu6050.h"

#ifdef USE_NILRTOS
#include <NilTimer1.h>

#define STACKSIZE	32
#endif

#define PROMPT		"CALLOGERO>"

uint32_t idleCount = 0;
uint32_t t0, t1;
extern float ypr[3];
//bool isConnected = false;
//bool blinkState = false;

#ifdef USE_NILRTOS
//MPU6050 mpu;
//extern SEMAPHORE_DECL(dmpSem, 1);
extern semaphore_t dmpSem;
//SEMAPHORE_DECL(dmpSem, 1);

NIL_WORKING_AREA(waSelf_Balancing_Thread, STACKSIZE);
NIL_WORKING_AREA(waControl_Thread, STACKSIZE);
#endif

extern bool mpuInterrupt;

#ifdef USE_NILRTOS

NIL_THREAD(self_balancing_thread, arg)
{
	t0 = millis();
	uint16_t i = 0;

	while (true)
	{
		Serial.println("Wait IRQ");
		//nilSemWait(&dmpSem);
		while (!mpuInterrupt || i == 32768) ;
		if (mpuInterrupt) Serial.println("IRQ!");
		mpuInterrupt = false;
		i = 0;

		imu_read();

		t1 = millis();

		Serial.print("DMP: dT= ");
		Serial.print(t1);// - t0);
		Serial.print(":");
		Serial.print(ypr[0] * 180/M_PI);
		Serial.print(":");
		Serial.print(ypr[1] * 180/M_PI);
		Serial.print(":");
		Serial.println(ypr[2] * 180/M_PI);
	}
}

NIL_THREAD(control_thread, arg)
{
	// Start Timer1
	nilTimer1Start(TIMER1_INTERVAL);

	while(true)
	{
		/*
		nilSemWait(&commandSem);
		// Execute command
		switch(command)
		{
			case FORWARD: break;
			case LEFT: break;
			case RIGHT: break;
			case BACKWARD: break;
			case STOP: break;
		}
		*/
		nilTimer1Wait();
	}
}

NIL_THREADS_TABLE_BEGIN()
	NIL_THREADS_TABLE_ENTRY("Self_Balancing", self_balancing_thread, NULL, waSelf_Balancing_Thread, sizeof(waSelf_Balancing_Thread))
	NIL_THREADS_TABLE_ENTRY("Control", control_thread, NULL, waControl_Thread, sizeof(waControl_Thread))
NIL_THREADS_TABLE_END()

#endif

//The setup function is called once at startup of the sketch
void setup()
{
// Add your initialization code here
	Serial.begin(BAUDRATE);
	while (!Serial)
	{
		digitalWrite(13, HIGH);
		delay(300);
		digitalWrite(13, LOW);
		delay(300);
	}

	// Configure IRQ port as output to avoid all issues
	pinMode (IRQ_PORT, OUTPUT);
	digitalWrite (IRQ_PORT, LOW);
	initialize_robot();
	isConnected = true;

	Serial.print(PROMPT);

#ifdef USE_NILRTOS
	nilSysBegin();
#endif
}

// The loop function is called in an endless loop
void loop()
{
//Add your repeated code here
#ifdef USE_NILRTOS
	noInterrupts();
	Serial.print("idle #: ");
	Serial.println(++idleCount);
	interrupts();
#else
	imu_read();
#endif
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
