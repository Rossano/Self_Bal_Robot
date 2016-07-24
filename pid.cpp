/*
 * pid.cpp
 *
 *  Created on: 5 sept. 2015
 *      Author: Ross
 */

//#define __BOARD_YUN__

#include "Arduino.h"
#include "shell.h"
#include "pid.h"

#ifdef __BOARD_YUN__
#include <Bridge.h>
//#include <Console.h>
#endif

_pid pid;

bool bEnableControl;

extern void vUsage(char *str);


_pid::_pid() {
	Kp = 10;
	Kd = 0.1;
	Ki = 1000;
	last_error = 0;
	cumulated_error = 0;
	tn_1 = 0;
	bEnableControl = false;
}

_pid::~_pid() { }

void _pid::set_coefficients(float _kp, float _kd, float _ki)
{
	Kp = _kp;
	Kd = _kd;
	Ki = _ki;
}

void _pid::get_coefficients(float& _kp, float& _kd, float& _ki)
{
	_kp = Kp;
	_kd = Kd;
	_ki = Ki;
}

float _pid::calculate(float error)
{
	float out;
	float in = error - ANGLE_OFFSET;
	float tn = millis();
	float dt = tn - tn_1;

	out = Kp * in + Kd * (in - last_error)/dt + Ki * (cumulated_error + in) * dt;
	last_error = error;
	cumulated_error += in;
	tn_1 = tn;

	return out;
}

float _pid::getError()
{
	return this->last_error;
}

void vpidToggle(int argc, char *argv[]) 		// Toggle the PID ON & OFF
{
	if(argc != 1)
	{
		vUsage("pid <0,1>");
	}
	else {
		uint8_t val = atoi(argv[0]);
		if(val) bEnableControl = true;
		else bEnableControl = false;
	}
}

void vpidSet(int argc, char *argv[]) 			// Set the PID coeff
{
	if(argc != 3)
	{
		vUsage("pid_set <Kd> <Kd> <Ki>");
	}
	else {
		float val1 = atof(argv[0]);
		float val2 = atof(argv[1]);
		float val3 = atof(argv[2]);

		pid.set_coefficients(val1, val2, val3);
		// Send an ACK();
#ifdef __BOARD_YUN__
		//Console.println("\r\nOK");
#else
		Serial.println("\r\nOK");
#endif
	}
}

void vpidGet(int argc, char *argv[]) 			// Get the PID coeff
{
	float Kp, Kd, Ki;

	pid.get_coefficients(Kp, Kd, Ki);

#ifdef __BOARD_YUN__
	/*Console.print("PID coefficients [Kp, Kd, Ki]: ");
	Console.print(Kp);
	Console.print("\t");
	Console.print(Kd);
	Console.print("\t");
	Console.println(Ki);

	// Send an ACK();
	Console.println("\r\nOK");*/
#else
	Serial.print("PID coefficients [Kp, Kd, Ki]: ");
	Serial.print(Kp);
	Serial.print("\t");
	Serial.print(Kd);
	Serial.print("\t");
	Serial.println(Ki);

	// Send an ACK();
	Serial.println("\r\nOK");
#endif
}

void vpidGetError(int argc, char *argv[]) 		// Get the PID error
{
	float val = pid.getError();

#ifdef __BOARD_YUN__
	/*Console.print("PID Error: ");
	Console.println(val);

	// Send an ACK();
	Console.println("\r\nOK");*/
#else
	Serial.print("PID Error: ");
	Serial.println(val);

	// Send an ACK();
	Serial.println("\r\nOK");
#endif
}

