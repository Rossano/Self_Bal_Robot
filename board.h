

//#include <Wire.h>
//#include <I2Cdev.h>


/*
 * board.h
 *
 * Created: 25/03/2015 22:58:32
 *  Author: rpantaleoni
 */ 


#ifndef BOARD_H_
#define BOARD_H_

//#include <Wire.h>

//#include "imu_mpu6050.h"

//
//	Motor Control Pin
//

#define __USE_PWM__

#ifdef __USE_PWM__
#define MOTOR_E1	5
#define MOTOR_M1	4
#define MOTOR_E2	6
#define MOTOR_M2	7
#elif defined(__USE_PLL__)
#define MOTOR_E1	4
#define MOTOR_M1	5
#define MOTOR_E2	7
#define MOTOR_M2	6
#else
#error "L298 Control mode not defined!"
#endif
//
//	IMU Pin
//
#define DMP_IRQ		2
#define IRQ_PORT	0
//
//	Sign of Life Pin
//
#define SOL_LED		13


#define BAUDRATE	115200U

//bool isConnected = false;
//bool blinkState = false;

#ifdef USE_DMP
extern bool dmpReady;
#endif	// USE_DMP

/*void initialize_robot(void)
{
	uint8_t count = 10;
	uint8_t devStatus;
	
	//Wire.begin();
	Wire.begin();
	
	pinMode(IRQ_PORT, INPUT);
	digitalWrite(IRQ_PORT, HIGH);
	attachInterrupt(DMP_IRQ, imu_isr, RISING);
	
	imu_init();
	dmpReady = true;
	
	pinMode(SOL_LED, OUTPUT);
}*/

#endif /* BOARD_H_ */

