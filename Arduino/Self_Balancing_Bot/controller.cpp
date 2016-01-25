/*
 * controller.cpp
 *
 *  Created on: 23 déc. 2015
 *      Author: rpantale
 */

//
//	Include Section
//
#include "math_comp.h"
#include "controller.h"
#include "shell.h"

//#define DEFINE_CONTROLLER_IN_LIB

//using namespace math_comp;

//namespace controller {

#ifdef DEFINE_CONTROLLER_IN_LIB
_controller controller(0,0,0,0);
#endif
// Enable/disable controller
bool bEnableStateControl;
//	Compensation Force to apply
double F;
//	State vector
#ifdef USE_STATE_VECTOR
math_comp::_vector<double,VECTOR_SIZE> vState;
#endif

/*
//
//	Function prototypes
//
void vControllerToggle(int argc, char *argv[]); 		// Controller Toggle ON/OFF
void vControllerSet(int argc, char *argv[]);			// Sets feedback coefficients
void vControllerGet(int argc, char *argv[]);			// Gets feedback coefficient on terminal
void vControllerState(int argc, char *argv[]);			// Gets the controller state on the terminal
//void vControllerError(int argc, char *argv[]);			// Gets the controller error on terminal
//void vControllerLastState(int argc, char *argv[]);		// Gets the controller last state on terminal
*/

//	Constructors
//
_controller::_controller(double k1, double k2, double k3, double k4) {
#ifdef USE_STATE_VECTOR
	set_feedback_vector(0.0, 0.0, 0.0, 0.0);
#endif
	set_state(0,0,0,0);
}

#ifdef USE_STATE_VECTOR
_controller::_controller(math_comp::_vector<double,VECTOR_SIZE> k) {
#endif
#ifdef USE_STATE_MATRIX
_controller::_controller(math_comp::_matrix<double,N,M> Ai, _vector<double,VECTOR_SIZE> Bi) {
#endif
	//int i,j;

	K = k;
	set_state(0,0,0,0);
}

//
//	Destructor
//
_controller::~_controller() {
	// TODO Auto-generated destructor stub
}

//
//	Method implementation Section
//

math_comp::_vector<double, VECTOR_SIZE>& _controller::get_feedback_vector() {
	math_comp::_vector<double, VECTOR_SIZE> foo;
	for(size_t i; i < VECTOR_SIZE; i++) foo.set_element(K.get_element(i),i);
	return foo;
}

double * _controller::get_feedback_elements() {
	double *k = new double[VECTOR_SIZE];
	for (size_t i; i < VECTOR_SIZE; i++) *(k + i) = K.get_element(i);
	return k;
}

void _controller::set_feedback_vector(math_comp::_vector<double,VECTOR_SIZE> v) {
	//K = v;
	double k1, k2, k3, k4;
	k1 = v.get_element(0);
	k2 = v.get_element(1);
	k3 = v.get_element(2);
	k4 = v.get_element(3);

	K.set_element(v.get_element(0), 0);
	K.set_element(v.get_element(1), 1);
	K.set_element(v.get_element(2), 2);
	K.set_element(v.get_element(3), 3);
}

void _controller::set_feedback_vector(double k1, double k2, double k3, double k4) {
	K.set_element(k1, 0);
	K.set_element(k2, 1);
	K.set_element(k3, 2);
	K.set_element(k4, 3);
}

void _controller::set_state(double k1, double k2, double k3, double k4) {

	/*K[1] = k1;
	K[2] = k2;
	K[3] = k3;
	K[4] = k4; */
	K.set_element(k1, 0);
	K.set_element(k2, 1);
	K.set_element(k3, 2);
	K.set_element(k4, 3);
}

//double _controller::calculate(math_comp::_vector<double,VECTOR_SIZE> & state) {
double _controller::calculate() {
	double Fout;

	Fout = state * K;

	return Fout;
}

math_comp::_vector<double, VECTOR_SIZE> _controller::get_state() {
	return state;
}

// Controller Toggle ON/OFF
void vControllerToggle(int argc, char *argv[]) {
	if (argc != 1) {
		vUsage("controller <0,1>");
	}
	else {
		uint8_t val = atoi(argv[0]);
		if (val) bEnableStateControl = true;
		else bEnableStateControl = false;
	}
}

// Sets feedback coefficients
void vControllerSet(int argc, char *argv[]) {
	if(argc != 4) {
		vUsage("contr_set <k1> <k2> <k3> <k4>");
	}
	else {
		double k[VECTOR_SIZE];
		k[0] = atof(argv[0]);
		k[1] = atof(argv[1]);
		k[2] = atof(argv[2]);
		k[3] = atof(argv[3]);

		controller.set_feedback_vector(math_comp::_vector<double, VECTOR_SIZE>(k));
	}
}

// Gets feedback coefficient on terminal
void vControllerGet(int argc, char *argv[]) {
	math_comp::_vector<double,VECTOR_SIZE> _K;

	_K = controller.get_feedback_vector();

	Serial.print(_K[0]); Serial.print("\t");
	Serial.print(_K[1]); Serial.print("\t");
	Serial.print(_K[2]); Serial.print("\t");
	Serial.println(_K[3]);
}

// Gets the controller state on the terminal
void vControllerState(int argc, char *argv[]) {
	math_comp::_vector<double,VECTOR_SIZE> _K;

	_K = controller.get_state();

	Serial.print(_K[0]); Serial.print("\t");
	Serial.print(_K[1]); Serial.print("\t");
	Serial.print(_K[2]); Serial.print("\t");
	Serial.println(_K[3]);
}
//}

//void vControllerError(int argc, char *argv[]);			// Gets the controller error on terminal
//void vControllerLastState(int argc, char *argv[]);		// Gets the controller last state on terminal

//} /* namespace controller */
