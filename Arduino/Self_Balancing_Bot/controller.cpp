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

//#define __BOARD_YUN__
#ifdef __BOARD_YUN__
#include <Bridge.h>
#include <Console.h>
#endif
 
//#define DEFINE_CONTROLLER_IN_LIB

//using namespace math_comp;

//namespace controller {

#ifdef DEFINE_CONTROLLER_IN_LIB
_controller controller(K1_DEFAULT, K2_DEFAULT, K3_DEFAULT, K4_DEFAULT);
#endif
// Enable/disable controller
bool bEnableStateControl;
//	Compensation Force to apply
double F;
//	State vector
#ifdef USE_STATE_VECTOR
math_comp::_vector<double,VECTOR_SIZE> vState;
#else
double vState[VECTOR_SIZE];
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
//#ifdef USE_STATE_VECTOR
	set_feedback_vector(0.0, 0.0, 0.0, 0.0);
//#endif
	set_state(0,0,0,0);
}

#ifdef USE_STATE_VECTOR
_controller::_controller(math_comp::_vector<double,VECTOR_SIZE> k) {
#ifdef USE_STATE_MATRIX
_controller::_controller(math_comp::_matrix<double,N,M> Ai, _vector<double,VECTOR_SIZE> Bi) {
#endif
	//int i,j;

	K = k;
	set_state(0,0,0,0);
}
#else
_controller::_controller(double * k) {
	for(size_t i; i < VECTOR_SIZE; i++) K[i] = *(k + i);
}
#endif


//
//	Destructor
//
_controller::~_controller() {
	// TODO Auto-generated destructor stub
}

//
//	Method implementation Section
//
#ifdef USE_STATE_VECTOR
math_comp::_vector<double, VECTOR_SIZE>& _controller::get_feedback_vector() {
	math_comp::_vector<double, VECTOR_SIZE> foo;
	for(size_t i = 0; i < VECTOR_SIZE; i++) foo.set_element(K.get_element(i),i);
	return foo;
}
#else
double * _controller::get_feedback_vector() {
	double *foo = new double(VECTOR_SIZE);
	for (size_t i = 0; i < VECTOR_SIZE; i++) *(foo + i) = K[i];
}
#endif

double * _controller::get_feedback_elements() {
	double *k = new double[VECTOR_SIZE];
#ifdef USE_STATE_VECTOR
	for (size_t i = 0; i < VECTOR_SIZE; i++) *(k + i) = K.get_element(i);
#else
	for (size_t i = 0; i < VECTOR_SIZE; i++) *(k + i) = K[i];
#endif
	return k;
}

#ifdef USE_STATE_VECTOR
void _controller::set_feedback_vector(math_comp::_vector<double,VECTOR_SIZE> v) {
	//K = v;
/*	double k1, k2, k3, k4;
	k1 = v.get_element(0);
	k2 = v.get_element(1);
	k3 = v.get_element(2);
	k4 = v.get_element(3);*/

	K.set_element(v.get_element(0), 0);
	K.set_element(v.get_element(1), 1);
	K.set_element(v.get_element(2), 2);
	K.set_element(v.get_element(3), 3);
}
#else
void _controller::set_feedback_vector(double *v) {
	for (size_t i; i<VECTOR_SIZE; i++) {
		K[i] = *(v + i);
	}
}
#endif

void _controller::set_feedback_vector(double k1, double k2, double k3, double k4) {
#ifdef USE_STATE_VECTOR
	K.set_element(k1, 0);
	K.set_element(k2, 1);
	K.set_element(k3, 2);
	K.set_element(k4, 3);
#else
	K[0] = k1;
	K[1] = k2;
	K[2] = k3;
	K[3] = k4;
#endif
}

void _controller::set_state(double k1, double k2, double k3, double k4) {

	/*K[1] = k1;
	K[2] = k2;
	K[3] = k3;
	K[4] = k4; */
#ifdef USE_STATE_VECTOR
	state.set_element(k1, 0);
	state.set_element(k2, 1);
	state.set_element(k3, 2);
	state.set_element(k4, 3);
#else
	state[0] = k1;
	state[1] = k2;
	state[2] = k3;
	state[3] = k4;
#endif
}

//double _controller::calculate(math_comp::_vector<double,VECTOR_SIZE> & state) {
double _controller::calculate() {
	double Fout;

#ifdef USE_STATE_VECTOR
	Fout = state * K;
	//for (size_t i; i<VECTOR_SIZE; i++) Fout += state[i]*K[i];
#else
	for (size_t i; i<VECTOR_SIZE; i++) Fout += state[i]*K[i];
#endif

	return Fout;
}

#ifdef USE_STATE_VECTOR
math_comp::_vector<double, VECTOR_SIZE> _controller::get_state() {
	return state;
}
#else
double * _controller::get_state() {
	double *foo = new double(VECTOR_SIZE);
	for (size_t i = 0; i < VECTOR_SIZE; i++) *(foo + i) = state[i];
	return foo;
}
#endif

// Controller Toggle ON/OFF
void vControllerToggle(int argc, char *argv[]) {
	if (argc != 1) {
		vUsage("cont_toggle <0,1>");
		Serial.print(F("State: "));
		Serial.println(bEnableStateControl);		
	}
	else {
		uint8_t val = atoi(argv[0]);
		if (val) bEnableStateControl = true;
		else bEnableStateControl = false;
		//Serial.print(F("arg: "));
		//Serial.println(val);
	}
}

// Sets feedback coefficients
void vControllerSet(int argc, char *argv[]) {
	if(argc != 4) {
		vUsage("contr_set <k1> <k2> <k3> <k4>");
		Serial.println(argc);
	}
	else {
		double k[VECTOR_SIZE];
		k[0] = atof(argv[0]);
		k[1] = atof(argv[1]);
		k[2] = atof(argv[2]);
		k[3] = atof(argv[3]);

#ifdef USE_STATE_VECTOR
		controller.set_feedback_vector(math_comp::_vector<double, VECTOR_SIZE>(k));
#else
		double *foo = new double(VECTOR_SIZE);
		for(size_t i = 0; i < VECTOR_SIZE; i++) *(foo + i) = k[i];
		controller.set_feedback_vector(k);
#endif
	}
}

// Gets feedback coefficient on terminal
void vControllerGet(int argc, char *argv[]) {
#ifdef USE_STATE_VECTOR
	math_comp::_vector<double,VECTOR_SIZE> _K;

	_K = controller.get_feedback_vector();
#else
	double *_K = new double(VECTOR_SIZE);
	_K = controller.get_feedback_vector();
#endif

#ifdef __BOARD_YUN__
	Console.print(_K[0]); Console.print(F("\t"));
	Console.print(_K[1]); Console.print(F("\t"));
	Console.print(_K[2]); Console.print(F("\t"));
	Console.println(_K[3]);
#else
	Serial.print("cont: ");
	Serial.print(_K[0]); Serial.print(F("\t"));
	Serial.print(_K[1]); Serial.print(F("\t"));
	Serial.print(_K[2]); Serial.print(F("\t"));
	Serial.println(_K[3]);
#endif
}

// Gets the controller state on the terminal
void vControllerState(int argc, char *argv[]) {
#ifdef USE_STATE_VECTOR
	math_comp::_vector<double,VECTOR_SIZE> _K;
#else
	double *_K = new double(VECTOR_SIZE);
#endif

	_K = controller.get_state();
#ifdef USE_STATE_VECTOR
	#ifdef __BOARD_YUN__
	Console.print(_K[0]); Console.print(F("\t"));
	Console.print(_K[1]); Console.print(F("\t"));
	Console.print(_K[2]); Console.print(F("\t"));
	Console.println(_K[3]);
	#else
	Serial.print("state ");
	Serial.print(_K[0]); Serial.print(F("\t"));
	Serial.print(_K[1]); Serial.print(F("\t"));
	Serial.print(_K[2]); Serial.print(F("\t"));
	Serial.println(_K[3]);
	#endif
#else
	#ifdef __BOARD_YUN__
	Console.print(*_K); Console.print(F("\t"));
	Console.print(*(_K+1)); Console.print(F("\t"));
	Console.print(*(_K+2)); Console.print(F("\t"));
	Console.print(*(_K+3));
	#else
	Serial.print(*_K); Serial.print(F("\t"));
	Serial.print(*(_K+1)); Serial.print(F("\t"));
	Serial.print(*(_K+2)); Serial.print(F("\t"));
	Serial.print(*(_K+3));
	#endif
#endif
}
//}

//void vControllerError(int argc, char *argv[]);			// Gets the controller error on terminal
//void vControllerLastState(int argc, char *argv[]);		// Gets the controller last state on terminal

//} /* namespace controller */

