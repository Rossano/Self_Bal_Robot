// Do not remove the include below
#include "Self_Balancing_Bot.h"
#include <Arduino.h>

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
#include "pid.h"
#include "controller.h"
#include "motor.h"
#include "shell.h"

#ifdef USE_NILRTOS
#include <NilTimer1.h>

#define STACKSIZE	32
#endif

//#define PROMPT		"CALLOGERO>"
#define CMD_STRING_LEN	16
#define DECIMATION		100
#define AXE_TO_USE		2
//
//	Global Data
//
String cmdString = "";				//	Line Command string buffer
boolean cmdReady = false;			//	Flag indicating that a command is ready
uint8_t inBufCount = 0;				//	Input buffer char counter
uint32_t idleCount = 0;
uint32_t t0, t1;
uint8_t count = 0;
//extern float ypr[3];
extern int16_t gyro[3];
int pwm = 0;
//Motor left_motor(MOTOR_SHIELD_DIRA, MOTOR_SHIELD_PWMA);
//Motor right_motor(MOTOR_SHIELD_DIRB, MOTOR_SHIELD_PWMB);

//bool isConnected = false;
//bool blinkState = false;
//_motor motor;
//_pid pid;
extern cShell shell;

//extern static ShellCommand_t ShellCommand[];

//
//	Function protorypes
//
void CDC_Task();
void initialize_robot();

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
	imu_init();
	isConnected = true;

	Serial.print(SHELL_PROMPT);

#ifdef USE_NILRTOS
	nilSysBegin();
#endif
}

// The loop function is called in an endless loop
void loop()
{
	float compensation;
//Add your repeated code here
#ifdef USE_NILRTOS
	noInterrupts();
	Serial.print(F("idle #: "));
	Serial.println(++idleCount);
	interrupts();
#else
	imu_read();
	double theta = ypr[AXE_TO_USE] + 0.06;
	double theta_dot = -((double)gyro[AXE_TO_USE] / 131.0);
	controller.set_state(theta, theta_dot, 0.0, 0.0);
	F = controller.calculate();

	//compensation = pid.calculate(ypr[1] - 3.1415 / 2);
	pwm = map(F, -1, 1, -255, 255);

	switch (eMotorMove) {
		case FORWARD: pwm += MOVE_OFFSET;
						break;
		case BACKWARD: pwm = -MOVE_OFFSET + pwm;
						break;
		default: break;
	};
	switch (eMotorTurn) {
		case LEFT: motor.move_A(pwm + TURN_OFFSET);
					motor.move_B(pwm - TURN_OFFSET);
					break;
		case RIGHT: motor.move_A(pwm - TURN_OFFSET);
					motor.move_B(pwm + TURN_OFFSET);
					break;
		default: motor.move_A(pwm);
			motor.move_B(pwm);
			break;
	};

	//motor.move_A(comp);
	//motor.move_B(comp);
#endif

	char ch;
	CDC_Task();
	if(cmdReady)
	{
		char buffer[CMD_STRING_LEN];
		char *buf = (char *)&buffer;
		cmdString.toCharArray(buf, CMD_STRING_LEN);
		shell.ShellTask((void *)ShellCommand, buf);
		Serial.print(SHELL_PROMPT);
		cmdString = "";
		cmdReady = false;
		inBufCount = 0;
	}
	if(++count == DECIMATION)
	{
//		Serial.print(millis());
//		Serial.print(ypr[1] - 3.1415 / 2);
//		Serial.print(pid.getError());
//		Serial.println(compensation);
		char **foo;
		vGetValues(0, foo);
		count = 0;
	}
}

//
//	USB-CDC Task
//
void CDC_Task()
{
	char ch;
	//
	//	Until data are available from the Serial Port read the data and store it into the input buffer cmdString
	//	Process ends if '\n' is received or the MAX input string length is reached
	//
	while(Serial.available())
	{
		//	If '\n" is received or MAX string lenght is reached set the cmdReady flag
		if (++inBufCount == CMD_STRING_LEN) ch = '\n';
		else
		{
			ch = (char)Serial.read();
			cmdString += ch;
		}
		if (ch == '\n') cmdReady = true;
	}
}

void initialize_robot(void)
{
	uint8_t count = 10;
	uint8_t devStatus;

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	TWBR = 24;							// 400KHz I2C
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	FastWire::Setup(400, true);
#else
#error "I2C implementation not found"
#endif
	//Wire.begin();
//	Wire.begin();
//	TWBR = 24;			//	400KHz I2C clock (200KHz if CPU is 8MHz)

	/*pinMode(IRQ_PORT, INPUT);
	digitalWrite(IRQ_PORT, HIGH);
	attachInterrupt(DMP_IRQ, imu_isr, RISING); */

	imu_init();
	dmpReady = true;

	pinMode(SOL_LED, OUTPUT);

	pinMode(IRQ_PORT, INPUT);
	digitalWrite(IRQ_PORT, HIGH);
	attachInterrupt(DMP_IRQ, imu_isr, CHANGE);
	Serial.print(F("IRQ on pin: D"));
	Serial.println(IRQ_PORT);

	for (int i = 0; i < 4; i++) {
#ifdef USE_STATE_VECTOR
		vState.set_element(0.0, i);
#else
		vState[i] = 0.0;
#endif
	}
}

void vGetValues(int argc, char *argv[]) { 		// Get the IMU and feedback values
	Serial.print(millis());
	Serial.print(F(" "));
#ifdef DEBUG
	Serial.print(ypr[0]/3.1415*180);
	Serial.print(F(" "));
	Serial.print(ypr[1]/3.1415*180);
	Serial.print(F(" "));
	Serial.print(ypr[2]/3.1415*180);
	Serial.print(F(" "));
	//Serial.print(pid.getError());
#else
	Serial.print(ypr[AXE_TO_USE]/3.1415*180);
	Serial.print(F(" "));
#endif
	Serial.print(gyro[AXE_TO_USE]);
	Serial.print(F(" "));
	Serial.println(F);
}
