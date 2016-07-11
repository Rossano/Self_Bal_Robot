/*
 * server.h
 *
 * Created: 10/07/2016 21:39:21
 *  Author: Ross
 */ 


#ifndef SERVER_H_
#define SERVER_H_

#define MOTOR_CMD		"motor"
#define CONTROLLER_CMD	"controller"
#define HELP_CMD		"help"
#define MOVE_CMD		"move"
#define TURN_CMD		"turn"
#define MOVE_FWD_CMD	"forward"
#define MOVE_BCK_CMD	"backward"
#define TURN_LEFT_CMD	"left"
#define TURN_RIGHT_CMD	"right"
#define STOP_CMD		"stop"
#define NO_TURN_CMD		"no_turn"

#define COMMAND_NOT_FOUND "error: command unknow"
#define COMMAND_OK		"OK"

extern YunServer server;

void serverTask(String);


#endif /* SERVER_H_ */