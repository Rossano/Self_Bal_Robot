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

#undef USE_DMP
#define USE_KALMAN_LIB

#ifdef USE_DMP
#include <MPU6050.h>
#endif
#ifdef USE_KALMAN_LIB
//#include "I2C.c"
#include "Kalman.h"
#endif

// Do not remove the include below
//#include "Self_Balancing_Bot.h"
#include <Arduino.h>

#include <Wire.h>

//#include <I2Cdev.h>

//#define __BOARD_YUN__

#ifdef USE_DMP
#include <MPU6050.h>
#include "imu_mpu6050.h"
#endif
#include "board.h"
//#include "imu_mpu6050.h"
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

//#define PROMPT		"CALLOGERO>"
#define CMD_STRING_LEN	32
#define DECIMATION		100
#define AXE_TO_USE		2
#define GYRO_SCALING	131.0	//	+/- 250 °/s //	16.4	// +/- 2000°/s
//#define RESTRICT_PITCH
#undef STAND_ALONE
#undef DEBUG
//#define DEBUG

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
bool blinkState;
bool isConnected;

#ifdef USE_KALMAN_LIB
Kalman kalmanX;
Kalman kalmanY;

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
uint16_t tempRaw;

double kalAngleX, kalAngleY;

uint32_t timer;
uint8_t i2cData[14];
#endif

double angle, angle_dot;

//
//	Function protorypes
//
#ifdef ARDUINO_AVR_YUN
void serverTask(YunClient);
#elif defined ARDUINO_AVR_LEONARDO
void CDC_Task();
#endif

void initialize_robot();
void readIMUData();

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
#ifdef USE_DMP
	imu_init();
#else
	// Configure DMP 6050
	i2cData[0] = 7;
	i2cData[1] = 0x00;
	i2cData[2] = 0x00;
	i2cData[3] = 0x00; 
	while (i2cWrite(0x19, i2cData, 4, false)) ;		//	Set datarate divider to 8, datatate 1Ks/s
	while (i2cWrite(0x1B, 0x00, 1, false)) ;		//	Set Gyro Range to 2000 rad/s
	while (i2cWrite(0x1C, 0x00, 1, false)) ;		//	Set Accelleration range to +/- 2g
	while (i2cWrite(0x6B, 0x01, true)) ;			//	Set clock source to gyro X axes (accurate)
	while (i2cRead(0x75, i2cData, 1)) ;
	if (i2cData[0] != 0x68) 
	{
		//	Read WhoAmI
		Serial.print(F("Error reading sensor!"));
		while (true) ;
	}
	delay (100);	// Wait for sensor to stabilize
	
	//	Set Kalman and gyro starting angles
	//while (i2cRead(0x33, i2cData, 6)) ;
	while (i2cRead(0x3B, i2cData, 6)) ;		// Read ACC values
	accX = (i2cData[0] << 8) | i2cData[1];
	accY = (i2cData[2] << 8) | i2cData[3];
	accZ = (i2cData[4] << 8) | i2cData[5];
	
#ifdef RESTRICT_PITCH
	#error "Should not get here!!!!"
	//double roll = atan2(accY, accZ);
	double roll = atan2(-accZ, accX);
	//double pitch = atan(-accX / sqrt(accY*accY + accZ*accZ));
#else
	double roll = atan2(-accZ, accX); //atan(accY / sqrt(accX*accX + accZ*accZ));
	//double pitch = atan2(-accX, accZ);
#endif	

	kalmanX.setAngle(roll);
	//kalmanY.setAngle(pitch);
	
	timer = micros();
#endif
	
	isConnected = true;
	bEnableStateControl = true;
	controller.set_feedback_vector(K1_DEFAULT, K2_DEFAULT, K3_DEFAULT, K4_DEFAULT);
#ifdef ARDUINO_AVR_YUN //__BOARD_YUN__
//	Console.print(SHELL_PROMPT);
#else
	Serial.print(SHELL_PROMPT);
#endif

	#if 0
Serial.println("Motor control test, neverending loop!");
	while (true)
	{
		Serial.println("FORWARD....");
		for(int val = 0; val <= 255; val += 5)
		{
			motor.move_A(val);
			motor.move_B(val);
			delay (30);
		}
		for(int val = 255; val; val -= 5)
		{
			motor.move_A(val);
			motor.move_B(val);
			delay(30);
		}
		Serial.println("BACKWARD....");
		for(int val = 0; val >= -255; val -= 5)
		{
			motor.move_A(val);
			motor.move_B(val);
			delay (30);
		}
		for(int val = -255; val <= 0; val += 5)
		{
			motor.move_A(val);
			motor.move_B(val);
			delay(30);
		}
	}
#endif
Serial.print("dT"); Serial.print(": ");
Serial.print("level"); Serial.print(",");
Serial.print("Motor1%"); Serial.print(",");
Serial.print("Motor2%"); Serial.print(",");
Serial.print("torque"); Serial.print(",");
Serial.print("zAngle"); Serial.print(",");
Serial.println("gyroAngle_dt");
}
#define USE_PID

#ifdef USE_PID

#define ACCEL_GAIN	18
#define GYRO_GAIN	5
#define ANGLE_GAIN	1.20

#define MOTOR1_FULL_FORWARD	255
#define MOTOR1_FULL_REVERSE	-255
#define MOTOR2_FULL_FORWARD 255
#define MOTOR2_FULL_REVERSE	-255

unsigned int STD_LOOP_TIME	= 9;	// 10ms
unsigned int lastLoopTime = STD_LOOP_TIME;
unsigned int lastLoopUsefulTime = STD_LOOP_TIME;
unsigned long loopStartTime = 0;
float cycle_time = 0.01;
float steer = 0.0;

float overallGain;
float SG_filter_result;
float z_acc;
float z_acc_deg;
float gAngle_deg;
float gAnglerate_deg;
float gAngle_rad;
float gAnglerate_rad;
float gyroAngle_dt;
float zAngle;

float balance_torque;
float level = 0.0;
float cur_speed = 0.0;
float balanceTrim = 0.0;
float set_point;

float ti_constant = 3;
float aa_constant = 0.005;

signed char Motor1percent;
signed char Motor2percent;

void pid_calculation()
{
	gAngle_deg = (float)angle/3.14159*180;
	// Angle gain
	SG_filter_result = (float)SG_filter_result * ANGLE_GAIN;
	z_acc_deg = (float)((SG_filter_result - (80 + balanceTrim)) * (1.0));
	
	if (z_acc_deg < -72) z_acc_deg = -72;
	if (z_acc_deg > 72) z_acc_deg = 72;
	
	gAngle_deg = (float)(angle_dot);
	if (gAngle_deg < -110) gAngle_deg = -110;
	if (gAngle_deg > 110) gAngle_deg = 110;
	
	gyroAngle_dt = (float) ti_constant * cycle_time * gAngle_deg;
	gAngle_rad = (float)gAngle_deg * DEG_TO_RAD;
	
	// Complementary filter
	zAngle = (float)((1-aa_constant)*zAngle + gyroAngle_dt) + (aa_constant * z_acc_deg);
	angle = (float)zAngle * DEG_TO_RAD;
	
	balance_torque = (float)(ACCEL_GAIN * angle + GYRO_GAIN * gAngle_rad);
	cur_speed = (float)(cur_speed + (angle * 6 * cycle_time)) * 0.999;
	level = (float)(balance_torque + cur_speed) * overallGain;
}

void set_motor()
{
	uint8_t cSpeedVal_Motor1;
	uint8_t cSpeedVal_Motor2;
	
	level *= 20;
	if (level < -100) level = -100;
	if (level > 100) level = 100;
	
	steer = (float) 0 * 0.09;
	
	Motor1percent = (signed char)level + steer;
	Motor2percent = (signed char)level - steer;
	if (Motor1percent < -100) Motor1percent = -100;
	if (Motor1percent > 100) Motor1percent = 100;
	if (Motor2percent < -100) Motor2percent = -100;
	if (Motor2percent > 100) Motor2percent = 100;
	
	cSpeedVal_Motor1 = map(Motor1percent, -100, 100, MOTOR1_FULL_REVERSE, MOTOR1_FULL_FORWARD);
	cSpeedVal_Motor2 = map(Motor2percent, -100, 100, MOTOR2_FULL_REVERSE, MOTOR2_FULL_FORWARD);
	
	motor.move_A(cSpeedVal_Motor1);
	motor.move_B(cSpeedVal_Motor2);
}

void plot_data()
{
	Serial.print(lastLoopTime); Serial.print(": ");
	Serial.print(level); Serial.print(",");
	Serial.print(Motor1percent); Serial.print(",");
	Serial.print(Motor2percent); Serial.print(",");
	Serial.print(balance_torque); Serial.print(",");
	Serial.print(zAngle); Serial.print(",");
	Serial.println(gyroAngle_dt);	
}

#endif

// The loop function is called in an endless loop
void loop()
{
	float compensation;
//Add your repeated code here
	
#ifdef USE_DMP
	imu_read();
	double theta = ypr[AXE_TO_USE];// + 0.06;
	double theta_dot = -((double)gyro[AXE_TO_USE] / GYRO_SCALING);
#endif
	
	
#ifdef USE_PID
	while(true)
	{
		readIMUData();
		pid_calculation();
		set_motor();
		// Sleep T = 10ms
		lastLoopUsefulTime = millis() - loopStartTime;
		if (lastLoopUsefulTime < STD_LOOP_TIME) delay(STD_LOOP_TIME - lastLoopUsefulTime);
		lastLoopTime = millis() - loopStartTime;
		loopStartTime = millis();
		// end timing loop
		
		// soft start
		if (overallGain < 0.5) overallGain = (float)overallGain + 0.005;
		if (overallGain > 0.5) overallGain = 0.5;
		plot_data();
	}
#endif
	readIMUData();
	controller.set_state(angle, angle_dot, 0.0, 0.0);
	F = controller.calculate();

	//compensation = pid.calculate(ypr[1] - 3.1415 / 2);
	//->pwm = map(F, -100, 100, -255, 255);
	pwm = map_double(F, -50, 50, -255, 255);
	//pwm = F / 5;
//	pwm = constrain(F, -255, 255);
	if (pwm < -255) pwm = -255;
	else if (pwm > 255) pwm = 255;
	
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
		char **foo;
		vGetValues(0, foo);
		Serial.print(SHELL_PROMPT);
		cmdString = "";
		cmdReady = false;
		inBufCount = 0;
	}
	char **foo;
	vGetValues(0, foo);
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
	//
	//	Place a delay to let the sensors to stabilize
	//
	//delay(100);
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
#ifdef ARDUINO_AVR_YUN //__BOARD_YUN__
	while(Console.available()) {
		//	If '\n" is received or MAX string lenght is reached set the cmdReady flag
		if (++inBufCount == CMD_STRING_LEN) ch = '\n';
		else  {
			ch = (char)Console.read();
			cmdString += ch;
		}
		if (ch == '\n') cmdReady = true;
	}
#endif
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

#ifdef USE_DMP
	imu_init();
	dmpReady = true;
	
	pinMode(IRQ_PORT, INPUT);
	digitalWrite(IRQ_PORT, HIGH);
	attachInterrupt(DMP_IRQ, imu_isr, CHANGE);
#endif

	pinMode(SOL_LED, OUTPUT);

	//pinMode(IRQ_PORT, INPUT);
	//digitalWrite(IRQ_PORT, HIGH);
	//attachInterrupt(DMP_IRQ, imu_isr, CHANGE);
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
	Console.print(F(","));
	#ifdef DEBUG
		Console.print(ypr[0]/3.1415*180);
		Console.print(F(","));
		Console.print(ypr[1]/3.1415*180);
		Console.print(F(","));
		Console.print(ypr[2]/3.1415*180);
		Console.print(F(","));
		//Serial.print(pid.getError());
	#else
		Console.print(ypr[AXE_TO_USE]/3.1415*180);
		Console.print(F(","));
	#endif
	Console.print(gyro[AXE_TO_USE]);
	Console.print(F(","));
	Console.print(F);
	Console.print(F(","));
	Console.println(pwm);
#else
	//
	//	Non Yun Output
	//
	Serial.print(F("MPU:"));
	Serial.print(millis());
	Serial.print(F(","));
	#ifdef DEBUG
		Serial.print(ypr[0]/3.1415*180);
		Serial.print(F(","));
		Serial.print(ypr[1]/3.1415*180);
		Serial.print(F(","));
		Serial.print(ypr[2]/3.1415*180);
		Serial.print(F(","));
		//Serial.print(pid.getError());
	#else
		//Serial.print(ypr[AXE_TO_USE]/3.1415*180);
		#ifdef USE_DMP
			Serial.print(ypr[0]/3.1415*180);
			Serial.print(F(","));
			Serial.print(ypr[1]/3.1415*180);
			Serial.print(F(","));
			Serial.print(ypr[2]/3.1415*180);
			Serial.print(F(","));
			Serial.print(gyro[AXE_TO_USE]);
		#endif
		#ifdef USE_KALMAN_LIB
	/*		Serial.print(kalAngleX/3.1415*180);
			Serial.print(F(","));
			Serial.print(kalAngleY/3.1415*180);
			Serial.print(F(","));
			Serial.print(gyroX / GYRO_SCALING);
			Serial.print(F(","));
			Serial.print(gyroY / GYRO_SCALING);
			Serial.print(F(","));
			Serial.print(gyroZ / GYRO_SCALING);
			Serial.print(F(","));*/
		#endif
	#endif
	//Serial.print(gyro[AXE_TO_USE]);
	Serial.print(angle*180/3.14159);
	Serial.print(F(","));
	Serial.print(F);
	Serial.print(F(","));
	Serial.println(pwm);
#endif
}

int map_double(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void readIMUData()
{
#ifdef USE_DMP
	imu_read();
	//double theta = ypr[AXE_TO_USE];// + 0.06;
	//double theta_dot = -((double)gyro[AXE_TO_USE] / GYRO_SCALING);
	angle = ypr[AXE_TO_USE];
	angle_dot = -((double)gyro[AXE_TO_USE] / GYRO_SCALING);
#endif
#ifdef USE_KALMAN_LIB
	//	Update values
	//while (i2cRead(0x38, i2cData, 14)) ;
	while (i2cRead(0x3B, i2cData, 14)) ;				// Read ACC, temperature and Gyro
	accX = ((i2cData[0] << 8) | i2cData[1]);
	accY = ((i2cData[2] << 8) | i2cData[3]);
	accZ = ((i2cData[4] << 8) | i2cData[5]);
	tempRaw = ((i2cData[6] << 8) | i2cData[7]);
	gyroX = ((i2cData[8] << 8) | i2cData[9]);
	gyroY = ((i2cData[10] << 8) | i2cData[11]);
	gyroZ = ((i2cData[12] << 8) | i2cData[13]);
	
	double dt = (double)(micros() - timer) / 1000000;
	timer = micros();
	
	#ifdef RESTRICT_PITCH
		#error "Should not get here!!!!"
		//double roll = atan2(accY, accZ);
		double roll = atan2(-accZ, accX);
		//double pitch = atan(-accX / sqrt(accY*accY + accZ*accZ));
	#else
		double roll = atan ( sqrt(accX*accX + accY*accY) / (1.0 * accZ)) ;//- 3.14159/2;
		//if (accZ < 0) roll = -roll;
		//double roll = atan(accY / sqrt(accX*accX + accZ*accZ));
		//double pitch = atan2(-accX, accZ);
	#endif
	
	//double gyroXrate = gyroX / GYRO_SCALING;
	//double gyroYrate = gyroY / GYRO_SCALING;
	double gyroZrate = gyroZ / GYRO_SCALING;
	
	#ifdef RESTRICT_PITCH
	#error "Should not get here!!!!"
		if((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90))
		{
			kalmanX.setAngle(roll);
			kalAngleX = roll;
		}
		else
			kalAngleX = kalmanX.getAngle(roll, gyroZrate, dt);
			
			if(abs(kalAngleX) > 90) gyroXrate = - gyroXrate;
			//kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
	#else
		/*
		if((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90))
		{
			kalmanY.setAngle(pitch);
			kalAngleY = pitch;
		}
		else
			kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);*/
		//if((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90))
		if((roll < -3.14159/2 && kalAngleX > 3.14159/2) || (roll > 3.14159/2 && kalAngleX < -3.14159/2))
		{
			kalmanX.setAngle(roll);
			kalAngleX = roll;
		}
		else
			kalAngleX = kalmanX.getAngle(roll, gyroZrate, dt);
	#endif
	// Output the result
	angle = roll;
	//angle = kalAngleX;
	angle_dot = gyroZrate;
	/*
	// Plot result
	Serial.print("debug: ");
	Serial.print(accX); Serial.print(F(","));
	Serial.print(accY); Serial.print(F(","));
	Serial.print(accZ); Serial.print(F(","));	
	Serial.print(gyroX); Serial.print(F(","));
	Serial.print(gyroY); Serial.print(F(","));
	Serial.print(gyroZ); Serial.print(F(","));
	Serial.print(angle/3.1415*180);
	//Serial.print(F(","));
	//Serial.print(kalAngleY/3.1415*180);
	Serial.print(F(","));
	Serial.print(gyroX / GYRO_SCALING);
	Serial.print(F(","));
	Serial.print(gyroY / GYRO_SCALING);
	Serial.print(F(","));
	Serial.print(gyroZ / GYRO_SCALING);
	Serial.print(F(","));
	*/
#endif

}