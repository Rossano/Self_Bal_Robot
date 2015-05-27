

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


#define DMP_IRQ		2
#define IRQ_PORT	0
#define SOL_LED		13

#define BAUDRATE	115200U

//bool isConnected = false;
//bool blinkState = false;
extern bool dmpReady;

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
