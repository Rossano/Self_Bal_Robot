/*
 * pid.h
 *
 *  Created on: 5 sept. 2015
 *      Author: Ross
 */

#ifndef PID_H_
#define PID_H_

#define ANGLE_OFFSET	0	// This defines the offsets to apply to IMU lecture depending
							// on how it is mounted on the bot
class _pid {
private:
	float Kp;
	float Kd;
	float Ki;
	float last_error;
	float cumulated_error;
	float tn_1;
public:
	_pid();
	virtual ~_pid();
	void set_coefficients(float, float, float);
	void get_coefficients(float&, float&, float&);
	float calculate(float);
	float getError();
};

extern _pid pid;

extern bool bEnableControl;

//
//	Function Prototype
//
#if BOARD==ARDUINO_AVR_LEONARDO
void vpidToggle(int argc, char *argv[]); 		// Toggle the PID ON & OFF
void vpidSet(int argc, char *argv[]); 			// Set the PID coeff
void vpidGet(int argc, char *argv[]); 			// Get the PID coeff
void vpidGetError(int argc, char *argv[]); 		// Get the PID error
#endif

#endif /* PID_H_ */

