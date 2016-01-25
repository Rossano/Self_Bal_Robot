/*
 * Pid.h
 *
 *  Created on: 17 juin 2015
 *      Author: rpantaleoni
 */

#ifndef PID_H_
#define PID_H_

class Pid {
private:
	int out_max;
	int out_min;
	float last_input;
	double Iterm;
	double _Kp;
	double _Ki;
	double _Kd;
	double _set_point;
public:
	Pid();
	Pid(double Kp, double Ki, double Kd);
	virtual ~Pid();
	void set_constants(double Kp, double Ki, double Kd);
	void set_set_point(double set_point);
	double get_set_point();
	int compute(double input);
};

#endif /* PID_H_ */
