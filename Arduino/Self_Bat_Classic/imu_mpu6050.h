/*
 * imu_mpu6050.h
 *
 *  Created on: 3 juin 2015
 *      Author: rpantale
 */

#ifndef IMU_MPU6050_H_
#define IMU_MPU6050_H_

//
// Function prototypes
//
void imu_isr();

void imu_init();

void imu_read();

void imu_reset();


#endif /* IMU_MPU6050_H_ */
