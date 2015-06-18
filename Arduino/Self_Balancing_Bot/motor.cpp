/*
 * motor.cpp
 *
 *  Created on: 17 juin 2015
 *      Author: rpantaleoni
 */

#include "motor.h"
#include "Arduino.h"

Motor::Motor(int dir, int pwm) {
	_dir = dir;
	_pwm = pwm;
	_speed = 0;

	// Configure Arduino pin
	pinMode(_dir, OUTPUT);
	pinMode(_pwm, OUTPUT);
}

Motor::~Motor() {

}

void Motor::move(int speed) {
	_speed = speed;

	if(speed > 0) { // FORWARD
		// Saturation
		if (speed > 255) speed = 255;
		digitalWrite(_dir, HIGH);
		analogWrite(_pwm, speed);
	}
	else {			// BACKWARD
		// Saturation
		map(speed, 0, -255, 0, 255);
		if (speed < -255) speed = -255;
		digitalWrite(_dir, HIGH);
		analogWrite(_pwm, speed);
	}
}

void Motor::stop() {
	_speed = 0;
	analogWrite(_pwm, 0);
	digitalWrite(_dir, LOW);
}
