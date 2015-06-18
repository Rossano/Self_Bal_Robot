/*
 * motor.h
 *
 *  Created on: 17 juin 2015
 *      Author: rpantaleoni
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#define MOTOR_SHIELD_DIRA		7
#define MOTOR_SHIELD_DIRB		4
#define MOTOR_SHIELD_PWMA		6
#define MOTOR_SHIELD_PWMB		5

class Motor {
private:
	int _dir;
	int _pwm;
	int _speed;
public:
	Motor(int dir, int pwm);
	virtual ~Motor();
	void move(int speed);
	void stop();
};

#endif /* MOTOR_H_ */
