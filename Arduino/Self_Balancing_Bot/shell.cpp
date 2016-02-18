/*
 * shell.cpp
 *
 *  Created on: 6 sept. 2015
 *      Author: Ross
 */

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <string.h>
//#include <stdio.h>

//	My included file and headers
#include <stddef.h>

//#include "controller.h"
#include "motor.h"
#if USE_PID
#include "pid.h"
#else
#include "controller.h"
#endif
#include "Self_Balancing_Bot.h"
//	Shell own inclusion header
#include "shell.h"

cShell shell;

/*
//
//	Definition of the shell commands:
//	This data structure links the command to the corrisponding action function
//
static ShellCommand_t ShellCommand[]  =
{
	{
		"get_ACK",	vSendACK
	},
	{
		"pid", vpidToggle
	},
	{
		"pid_set", vpidSet
	},
	{
		"pid_get", vpidGet
	},
	{
		"pid_error", vpidGetError
	},
	{
		"turn", vMotorTurn,
	},
	{
		"move", vMotorMove,
	},
	{
		NULL, NULL
		//(char *)0, (void **)0
	}
} ;
*/

cShell::cShell() {
	// TODO Auto-generated constructor stub

}

cShell::~cShell() {
	// TODO Auto-generated destructor stub
}


//	???
static boolean bEnd = false;

//
//	Shell Local command definition data structure
//
static ShellCommand_t LocalCommands[] =
{
	{
		"info", vCmdInfo
	},
	{
		"systime", vCmdSystime
	},
	{
		NULL, NULL
	}
};

/// <summary>
/// Shells  task code.
/// </summary>
/// <param name="p">Pointer to a data structure storing all implemented commands</param>
/// <param name="line">The commanbd line.</param>
void cShell::ShellTask(void *p, char *line)
{
	int n;
	//	Initialize the shell command data structure
	const ShellCommand_t *scp=((ShellConfig_t *)p)->sc_command;
	char *lp, *cmd, *tokp;
	char *args[SHELL_MAX_ARGUMENTS + 1];

	//	Get all the tokens from the input string and stores them on lp and tokp pointer
	lp = vStrtok(line, " \r\n", &tokp);

	#ifdef DEBUG
	avrPrintf("lp -> ");
	avrPrintf(lp);
	avrPrintf("\ntokp -> ");
	avrPrintf(tokp);
	avrPrintf(CR);
	#endif // DEBUG

	//	The command to execute is stored into lp
	cmd = lp;
	n = 0;

	//
	//	Until there are valid tokens fill the arguments array args with them
	//
	while ((lp = vStrtok(NULL, " \r\n", &tokp)) != NULL)
	{
		//	If there are too many arguments display an error
		if (n > SHELL_MAX_ARGUMENTS)
		{
			Serial.println (F("Too many arguments"));
			cmd = NULL;
			break;
		}
		// else fill arguments array
		args[n++] = lp;
	}
	//	End the args array with a NULL argument
	args[n] = NULL;
	// do we really need it ????
	if (n == 0)
	{
		#ifdef DEBUG
		avrPrintf("Forcing end of string\r\n");
		#endif // DEBUG
		int len = strlen(cmd);
		cmd[len] = '\0';
	}

	#ifdef DEBUG
	avrPrintf("Cmd -> ");
	avrPrintf(cmd);
	avrPrintf(CR);
	char numArgv[2];
	numArgv[0] = '0' + n;
	numArgv[1] = '\0';
	avrPrintf("n -> ");
	avrPrintf(numArgv);
	avrPrintf(CR);
	#endif // DEBUG

	//
	//	If there is a valid command to execute (not NULL),parse it and execute the corresponding action
	//
	if (cmd != NULL)
	{
		//	Exit the Shell
		if (strcasecmp(cmd, "exit") == 0)
		{
			//	Exit has no arguments
			if (n > 0)
			{
				vUsage("exit");
			}
			// Set the shell end flag
			bEnd = true;
			return;
		}
		//	Display the list of supported commands
		else if (strcasecmp(cmd, "help") == 0)
		{
			//	Help has no arguments
			Serial.println(F("Entering help"));
			if (n > 1)
			{
				vUsage("help");
			}
			Serial.println(F("Commands:"));
			//	Display the Local Commands
			vListCommands(LocalCommands);
			//	Display the Shell Commands
			vListCommands(ShellCommand);
			Serial.println();
		}
		//	Try to Execute the other command, if it exits an error the command is not recognized
		else if (vCmdExec(LocalCommands, cmd, n, args) && ((scp == NULL) || vCmdExec(/*scp*/ ShellCommand, cmd, n, args)))
		{
			Serial.print(F("Error: Command not recognized -> "));
			Serial.print (cmd);
			//Serial.println (" ???");
		}
	}
}

/// <summary>
/// Shell thread loop.
/// this function read from stdin and calls the real thread code
/// </summary>
/// <param name="p">A pointer to the implemented commands.</param>
void cShell::vShellThread(void *p)
{
	char line[SHELL_MAX_LINE_LENGTH];
	// chRegSetThreadName("shell");

	Serial.println("\r\nAVR Shell;");

	while (!bEnd)
	{
		// Display the prompt
		Serial.print(SHELL_PROMPT);
		// Get the command line from stdin
		gets(line);
		// Calls the shell task
		ShellTask(p, line);
	}
	return;
}

/// <summary>
/// Command Execution.
/// Execute a command and return the result.
/// </summary>
/// <param name="scp">A pointer to the implemented commands.</param>
/// <param name="name">Name of the command to execute</param>
/// <param name="argv">A pointer to the Argument list.</param>
int cShell::vCmdExec(const ShellCommand_t *scp, char *name, int argc, char *argv[])
{
	while (scp->sc_name != NULL)
	{
		if(strcmp(scp->sc_name, name) == 0)
		{
			scp->sc_function(argc, argv);
			return 0;
		}
		scp++;
	}
	return 1;
}

/// <summary>
/// System Time Command.
/// Not implemented.
/// </summary>
/// <param name="argc">Number of parameters.</param>
/// <param name="argv">A pointer to the Argument list.</param>
void vCmdSystime(int argc, char *argv[])
{
	(void) argv;
	//	If there are arguments display and error message
	if(argc > 0)
	{
		vUsage("systime");
		return;
	}
	//	Else display a string stating that it is not implemented
	Serial.println(F("Sys Time: Not implemented yet\r\nOK"));
}

/// <summary>
/// Info Command.
/// Display firmware information.
/// </summary>
/// <param name="argc">Number of parameters.</param>
/// <param name="argv">A pointer to the Argument list.</param>

void vCmdInfo(int argc, char *argv[])
{
	(void)argv;
	//	If there are arguments plot an error message
	if(argc > 1)
	{
		vUsage("info");
		return;
	}
	//	Else display Firmware and OS versions
	Serial.print(F("Firmware: "));
	Serial.print(FW_VERSION);
	Serial.print(F("\r\nOS Version: "));
	Serial.print(OS_VERSION);
	//Serial.println(F("\r\nOK\r\n"));
}

/// <summary>
/// List the Commands in the scp data structure.
/// </summary>
/// <param name="scp">Command data structure.</param>
void vListCommands(ShellCommand_t *scp)
{
	//	Until the commands data structure has valid elements display the command name
	while (scp->sc_name != NULL)
	{
		Serial.println((char *)scp->sc_name);
		scp++;
	}
}

/// <summary>
/// Command Usage Function.
/// Display information how to use the command.
/// </summary>
/// <param name="strc">Command usage string.</param>
void vUsage(char *str)
{
	Serial.print(F("Error: Usage-> "));
	Serial.println(str);
}

/// <summary>
/// Substring token extraction.
/// </summary>
/// <param name="strc">Input string.</param>
/// <param name="delim">String containing a list of delimiters.</param>
/// <param name="saveptr">Vector of String containing all the substrings.</param>
char * cShell::vStrtok(char *str, const char *delim, char **saveptr)
{
	char *token;
	if (str) *saveptr = str;
	token = *saveptr;

	if (!token) return NULL;

	token += strspn(token, delim);
	*saveptr = strpbrk(token, delim);

	if (*saveptr) *(*saveptr)++ = '\0';

	return *token ? token : NULL;
}

/// <summary>
/// Send the ACK
/// </summary>
/// <param name="argc">Number of parameters.</param>
/// <param name="argv">A pointer to the Argument list.</param>
void vSendACK(int argc, char *argv[])
{
	//	If there are arguments send an error message
	if (argc > 0)
	{
		vUsage("get_ACK");
	}
	else
	{
		//	Send the ACK
		Serial.println(F("ACK\r\nOK"));
	}
}

