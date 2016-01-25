/*
 * Pid.cpp
 *
 *  Created on: 17 juin 2015
 *      Author: rpantaleoni
 */

#include "Pid.h"

#define round(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))

Pid::Pid() {
	_Kp = 100;
	_Ki = 2;
	_Kd = 0.5;
	_set_point = 0.0;
	out_max = 255,
	out_min = -255;
	Iterm = 0.0;
	last_input = 0.0;
}

Pid::~Pid() {

}

Pid::Pid(double Kp, double Ki, double Kd) {
	set_constants(Kp, Ki, Kd);
	out_max = 255,
	out_min = -255;
	Iterm = 0.0;
	last_input = 0.0;
}

void Pid::set_constants(double Kp, double Ki, double Kd) {
	_Kp = Kp;
	_Ki = Ki;
	_Kd = Kd;
}

void Pid::set_set_point(double set_point) {
	_set_point = set_point;
}

double Pid::get_set_point() {
	return _set_point;
}

int Pid::compute(double input) {
	double error = _set_point - input;

	// Integral term
	Iterm += (_Ki * error);

	// Clipping
	if (Iterm > out_max) Iterm = out_max;
	else if (Iterm < out_min) Iterm = out_min;

	double d_input = input - last_input;

	// Carry out PID output
	double output = _Kp*error + Iterm + _Kd*input;

	// Clipping
	if(output > out_max) output = out_max;
	else if (output < out_min) output = out_min;

	// Store data for next iteration
	last_input = input;

	return round(output);
}
