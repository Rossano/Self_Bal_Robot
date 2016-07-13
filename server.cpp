/*
 * server.cpp
 *
 * Created: 10/07/2016 21:39:47
 *  Author: Ross
 */ 

#include <Bridge.h>
#include <YunServer.h>
#include <YunClient.h>
#include "server.h"
#include "controller.h"
#include "motor.h"
#include <stdlib.h>

YunServer server;

void serverTask(YunClient client)
{
	String cmd = client.readStringUntil('/');
	if (cmd == CONTROLLER_CMD)
	{
		String foo = client.readStringUntil('/');
		if (foo == "get")
		{
			double * k = controller.get_feedback_elements();
			client.print(*k++ + ',' + *k++ + ',' + *k++ + ',' + *k);
		}
		else if (foo == "on")
		{
			bEnableStateControl = true;
		}
		else if (foo == "off")
		{
			bEnableStateControl = false;
		}
		else if (foo == "set")
		{
			double k[VECTOR_SIZE];
			for (int i=0; i<VECTOR_SIZE; i++)
			{
				String str = client.readStringUntil(',');							
				int len = str.length() + 1;
				char ciccio [len + 1];
								
				str.toCharArray(ciccio, len);
		/*		k[i] = atof(ciccio); //parseFloat(str);*/
			}
			controller.set_feedback_vector(math_comp::_vector<double, VECTOR_SIZE>(k));
		}
		else 
		{
			client.print(COMMAND_NOT_FOUND);
			return;
		}
		client.print(COMMAND_OK);
		return;
	}
	else if (cmd == MOTOR_CMD)
	{
		String foo = client.readStringUntil('/');
		if (foo == MOVE_CMD)
		{
			foo = client.readStringUntil('/');
			if (foo == MOVE_FWD_CMD)
			{
				motor.moveStatus = FORWARD;
				motor.uiMotorA_Offset = MOTOR_OFFSET;
				motor.uiMotorB_Offset = MOTOR_OFFSET;
			}
			else if (foo == MOVE_BCK_CMD)
			{
				motor.moveStatus = BACKWARD;
				motor.uiMotorA_Offset = -MOTOR_OFFSET;
				motor.uiMotorB_Offset = -MOTOR_OFFSET; 
			}
			else 
			{
				client.print(COMMAND_NOT_FOUND);
				return;
			}
			client.print(COMMAND_OK);
			return;
		}
		else if (foo == STOP_CMD)
		{
			motor.moveStatus = STOP;
			motor.uiMotorB_Offset = motor.uiMotorA_Offset = 0;	
		}
		else if (foo == TURN_CMD)
		{
			foo = client.readStringUntil('/');
			if (foo == TURN_LEFT_CMD)
			{
				motor.turnStatus = LEFT;
				motor.uiMotorA_Offset += TURN_OFFSET;
			}
			else if (foo == TURN_RIGHT_CMD)
			{
				motor.turnStatus = RIGHT;
				motor.uiMotorB_Offset += TURN_OFFSET;
			}
			else if (foo == NO_TURN_CMD)
			{
				motor.turnStatus = NO_TURN;
				motor.uiMotorB_Offset = motor.uiMotorA_Offset = 0;
			}
			else
			{
				client.print(COMMAND_NOT_FOUND);
				return;
			}
			client.print(COMMAND_OK);
			return;
		}
	}
	else
	{
		client.print(COMMAND_NOT_FOUND);
	}
}