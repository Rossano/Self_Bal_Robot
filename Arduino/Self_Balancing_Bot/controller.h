/*
 * controller.h
 *
 *  Created on: 23 déc. 2015
 *      Author: rpantale
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "math_comp.h"
//
//	Define Section
//
#define USE_STATE_VECTOR
#undef USE_STATE_MATRIX
#define DEFINE_CONTROLLER_IN_LIB

//
//	Constant Section
//
#define VECTOR_SIZE		4	// Size of the state space vector or state Matrix

#define K1_DEFAULT		10000
#define K2_DEFAULT		10000
#define K3_DEFAULT		10000
#define K4_DEFAULT		10000
//namespace controller {

//
//	Controller Class
//
class _controller {
private:
#ifdef USE_STATE_VECTOR
	math_comp::_vector<double,VECTOR_SIZE> K;
	math_comp::_vector<double,VECTOR_SIZE> state;
#else
	double K[VECTOR_SIZE];
	double state[VECTOR_SIZE];
#endif
#ifdef USE_STATE_MATRIX
	_matrix<double> AK(VECTOR_SIZE, VECTOR_SIZE);
	_vector<double> BK(VECToR_SIZE);
	math_comp::_vector<double, VECTOR_SIZE> state;
#endif
//	math_comp::_vector<double, VECTOR_SIZE> state;
public:
#ifdef USE_STATE_VECTOR
	_controller(double, double, double, double);
	_controller(math_comp::_vector<double,VECTOR_SIZE> );
	math_comp::_vector<double, VECTOR_SIZE>& get_feedback_vector();
	double * get_feedback_elements();
	math_comp::_vector<double, VECTOR_SIZE> get_state();
	void set_feedback_vector(math_comp::_vector<double,VECTOR_SIZE> );
	void set_feedback_vector(double, double, double, double);
	void set_state(double, double, double, double);
	//double calculate(math_comp::_vector<double,VECTOR_SIZE> &);
	double calculate();
#else
	_controller(double, double, double, double);
	_controller(double *);
	double * get_feedback_vector();
	double * get_feedback_elements();
	double * get_state();
	void set_feedback_vactor(double *);
	void set_feedback_vector(double, double, double, double);
	void set_feedback_vector(double *);
	void set_state(double, double, double, double);
	double calculate();
#endif
#ifdef USE_STATE_MATRIX
	_controller(_matrix<double> &, _vector<double> &);
	_matrix<double> get_state_matrix();
	_vector<double> get_state_vector();
	_vector<double> calculate();
#endif
//	math_comp::_vector<double, VECTOR_SIZE> get_state();
	virtual ~_controller();
};

//
//	Global Variables
//
#ifdef DEFINE_CONTROLLER_IN_LIB
extern _controller controller;
#endif
// Enable/disable controller
extern bool bEnableStateControl;
//	Compensation Force to apply
extern double F;
//	State vector
#ifdef USE_STATE_VECTOR
extern math_comp::_vector<double,VECTOR_SIZE> vState;
#else
extern double vState[VECTOR_SIZE];
#endif
#ifdef USE_STATE_MATRIX
math_comp::_matrix<double, VECTOR_SIZE, VECTOR_SIZE>;
#endif

//
//	Function prototypes
//
void vControllerToggle(int argc, char *argv[]); 		// Controller Toggle ON/OFF
void vControllerSet(int argc, char *argv[]);			// Sets feedback coefficients
void vControllerGet(int argc, char *argv[]);			// Gets feedback coefficient on terminal
void vControllerState(int argc, char *argv[]);			// Gets the controller state on the terminal
//void vControllerError(int argc, char *argv[]);			// Gets the controller error on terminal
//void vControllerLastState(int argc, char *argv[]);		// Gets the controller last state on terminal

//} /* namespace controller */

#endif /* CONTROLLER_H_ */

