/*
 * motor.h
 *
 *  Created on: 31 ao�t 2015
 *      Author: Ross
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#define MOTOR_OFFSET 30
#define TURN_OFFSET	30
#define MOVE_OFFSET 50

typedef enum {
	STOP,
	FORWARD,
	BACKWARD
} eMotorMove_t;

typedef enum {
	NO_TURN,
	LEFT,
	RIGHT
} eMotorTurn_t;

extern eMotorTurn_t eMotorTurn;
extern eMotorMove_t eMotorMove;

class _motor {
public:
	eMotorMove_t moveStatus;
	eMotorTurn_t turnStatus;
	uint8_t uiMotorA_Offset;
	uint8_t uiMotorB_Offset;
	_motor();
	virtual ~_motor();
	void move_A(int);
	void move_B(int);
	void stop_A();
	void stop_B();
	void stop();
	int getPWM_A();
	int getPWM_B();
private:
	int pwmA;
	int pwmB;
};

extern _motor motor;

//
//	Function Prototypes
//
void vMotorTurn(int argc, char *argv[]); 		// Turn the Bot
void vMotorMove(int argc, char *argv[]); 		// Move the Bot

#endif /* MOTOR_H_ */
