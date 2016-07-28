/*
 * Self_Balancing_Bot.ino
 *
 *  Created on: 31 août 2015
 *      Author: Ross
 *
 *	Important: for Arduino Yun was mandatory to get rid of the USB module, else the sketch gets too big!
 *	To do so, go in arduino IDE folder @: <Arduino IDE>\hardware\tools\avr\avr\include\avr\main.cpp
 *
 *	Comment/uncomment the following line!
 *	#if defined(USBCON)
 *	//	USBDevice.attach();
 *	#endif
 *
 *	setup();
 *
 */


#include <MPU6050.h>
// Do not remove the include below
//#include "Self_Balancing_Bot.h"
#include <Arduino.h>

#include <Wire.h>

#include <I2Cdev.h>

//#define __BOARD_YUN__
#undef USE_NILRTOS

#include <MPU6050.h>
#ifdef USE_NILRTOS
#include <NilRTOS.h>
#endif // USE_NIL_RTOS

#include "board.h"
#include "imu_mpu6050.h"
#ifdef USE_PID_CONTROLLER
#include "pid.h"
#else #include "controller.h"
#endif
#include "controller.h"
#include "motor.h"

#ifdef ARDUINO_AVR_YUN
#include <Bridge.h>
#include <YunServer.h>
#include <YunClient.h>
#include "server.h"
//#else 
#elif defined (ARDUINO_AVR_LEONARDO)
#include "shell.h"
#else error "Unreconized ARDUINO BOARD"
#endif

/*
 *	NIL RTOS Section
 */
#ifdef USE_NILRTOS

#include <NilTimer1.h>

#define STACKSIZE	32
#endif // USE_NILRTOS

//#define PROMPT		"CALLOGERO>"
#define CMD_STRING_LEN	32
#define DECIMATION		100
#define AXE_TO_USE		2
#undef STAND_ALONE
#undef DEBUG
//
//	Global Data
//
String cmdString = "";				//	Line Command string buffer
boolean cmdReady = false;			//	Flag indicating that a command is ready
uint8_t inBufCount = 0;				//	Input buffer char counter
uint32_t idleCount = 0;
//uint32_t t0, t1;
uint8_t count = 0;
//extern float ypr[3];
extern int16_t gyro[3];
int pwm = 0;
extern bool mpuInterrupt;

#ifdef ARDUINO_AVR_YUN
extern YunServer server;
#elif defined ARDUINO_AVR_LEONARDO
extern cShell shell;
#endif

//
//	Function protorypes
//
#ifdef ARDUINO_AVR_YUN
void serverTask(YunClient);
#elif defined ARDUINO_ARCH_LEONARDO
void CDC_Task();
#endif

void initialize_robot();

/*
 *	NIL RTOS  Thread Definition
 */ 
#ifdef USE_NILRTOS
extern semaphore_t dmpSem;

NIL_WORKING_AREA(waSelf_Balancing_Thread, STACKSIZE);
NIL_WORKING_AREA(waControl_Thread, STACKSIZE);

NIL_THREAD(self_balancing_thread, arg)
{
	t0 = millis();
	uint16_t i = 0;

	while (true)
	{
		Serial.println(F("Wait IRQ");
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

#endif // USE_NILRTOS


//The setup function is called once at startup of the sketch
void setup()
{
// Add your initialization code here
#ifdef ARDUINO_AVR_YUN //__BOARD_YUN__
	Bridge.begin();
	// Listen for incoming connection only from localhost
	// noone from external network will connect
	server.listenOnLocalhost();
	server.begin();
	//Console.begin();
#else
	int count = 100;
	Serial.begin(BAUDRATE);
	while (!Serial && count--)
	{
		digitalWrite(13, HIGH);
		delay(300);
		digitalWrite(13, LOW);
		delay(300);
	}
#endif

	// Configure IRQ port as output to avoid all issues
	pinMode (IRQ_PORT, OUTPUT);
	digitalWrite (IRQ_PORT, LOW);
	initialize_robot();
	imu_init();
	isConnected = true;
	bEnableStateControl = true;
	controller.set_feedback_vector(K1_DEFAULT, K2_DEFAULT, K3_DEFAULT, K4_DEFAULT);
#ifdef ARDUINO_AVR_YUN //__BOARD_YUN__
//	Console.print(SHELL_PROMPT);
#else
	Serial.print(SHELL_PROMPT);
#endif

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
	double theta = ypr[AXE_TO_USE];// + 0.06;
	double theta_dot = -((double)gyro[AXE_TO_USE] / 131.0);
	controller.set_state(theta, theta_dot, 0.0, 0.0);
	F = controller.calculate();

	//compensation = pid.calculate(ypr[1] - 3.1415 / 2);
//	pwm = map(F, -1, 1, -255, 255);
	pwm = map_double(F, -100, 100, -255, 255);

	if (bEnableStateControl) {
		
		switch (eMotorMove) {
				case FORWARD: pwm += MOVE_OFFSET;
								break;
				case BACKWARD: pwm = -MOVE_OFFSET + pwm;
								break;
				default: break;
			};
			/*
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
			*/
			motor.move_A(pwm);
			motor.move_B(pwm);
#ifdef STAND_ALONE
			Serial.print(F("(PWM, F) = ")); 
			Serial.print(pwm);
			Serial.print(F(":"));
			Serial.print(F);
#endif
			//motor.move_A(comp);
			//motor.move_B(comp);
	}
	
#endif

#ifdef ARDUINO_AVR_YUN
	YunClient client = server.accept();
	if (client)
	{
		serverTask(client);
	}
	client.stop();
#else if defined ARDUINO_AVR_LEONARDO
	char ch;
	CDC_Task();
	if(cmdReady)
	{
		char buffer[CMD_STRING_LEN];
		char *buf = (char *)&buffer;
		cmdString.toCharArray(buf, CMD_STRING_LEN);
		shell.ShellTask((void *)ShellCommand, buf);
//#ifdef ARDUINO_AVR_YUN //__BOARD_YUN__
//		Console.print(SHELL_PROMPT);
//#endif
//#else
//		Serial.print(SHELL_PROMPT);
//#endif
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

//		char **foo;
//		vGetValues(0, foo);
		count = 0;
	}
#endif
}

#ifdef ARDUINO_AVR_YUN
//
//	YUN Server Task manager
//
void Server_Task()
{
	
}
#else if defined ARDUINO_AVR_LEONARDO
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
//#ifdef ARDUINO_AVR_YUN //__BOARD_YUN__
	while(Console.available()) {
		//	If '\n" is received or MAX string lenght is reached set the cmdReady flag
		if (++inBufCount == CMD_STRING_LEN) ch = '\n';
		else  {
			ch = (char)Console.read();
			cmdString += ch;
		}
		if (ch == '\n') cmdReady = true;
	}
//#else
	while(Serial.available()) {
		//	If '\n" is received or MAX string lenght is reached set the cmdReady flag
		if (++inBufCount == CMD_STRING_LEN) ch = '\n';
		else {
			ch = (char)Serial.read();
			cmdString += ch;
		}
		if (ch == '\n') cmdReady = true;
	}
//#endif
}
#endif

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
#ifdef ARDUINO_AVR_YUN //__BOARD_YUN__
	//Console.print(F("IRQ on pin: D"));
	//Console.println(IRQ_PORT);
#else
	Serial.print(F("IRQ on pin: D"));
	Serial.println(IRQ_PORT);
#endif

	for (int i = 0; i < 4; i++) {
#ifdef USE_STATE_VECTOR
		vState.set_element(0.0, i);
#else
		vState[i] = 0.0;
#endif
	controller.set_feedback_vector(K1_DEFAULT, K2_DEFAULT, K3_DEFAULT, K4_DEFAULT);
	}
}

void vGetValues(int argc, char *argv[]) { 		// Get the IMU and feedback values
#ifdef ARDUINO_AVR_YUN //__BOARD_YUN__
	Console.print(F(""));
	Console.print(millis());
	Console.print(F(" "));
#ifdef DEBUG
	Console.print(ypr[0]/3.1415*180);
	Console.print(F(" "));
	Console.print(ypr[1]/3.1415*180);
	Console.print(F(" "));
	Console.print(ypr[2]/3.1415*180);
	Console.print(F(" "));
	//Serial.print(pid.getError());
#else
	Console.print(ypr[AXE_TO_USE]/3.1415*180);
	Console.print(F(" "));
#endif
	Console.print(gyro[AXE_TO_USE]);
	Console.print(F(" "));
	Console.print(F);
	Console.print(F(" "));
	Console.println(pwm);
#else
	//
	//	Non Yun Output
	//
	Serial.print(F("MPU:"));
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
	//Serial.print(ypr[AXE_TO_USE]/3.1415*180);
	Serial.print(ypr[0]/3.1415*180);
	Serial.print(F(" "));
	Serial.print(ypr[1]/3.1415*180);
	Serial.print(F(" "));
	Serial.print(ypr[2]/3.1415*180);
	Serial.print(F(" "));
#endif
	Serial.print(gyro[AXE_TO_USE]);
	Serial.print(F(" "));
	Serial.print(F);
	Serial.print(F(" "));
	Serial.println(pwm);
#endif
}

int map_double(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
