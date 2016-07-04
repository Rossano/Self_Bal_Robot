/*
 * imu_mpu6050.h
 *
 * Created: 25/03/2015 23:00:08
 *  Author: rpantaleoni
 */ 


#ifndef IMU_MPU6050_H_
#define IMU_MPU6050_H_

/*
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

SEMAPHORE_DECL(dmpSem, 0);
*/
extern bool blinkState;
extern bool isConnected;
extern bool dmpReady;
extern float ypr[3];

void imu_isr();

void imu_init();

void imu_read();

void imu_reset();

#endif /* IMU_MPU6050_H_ */

