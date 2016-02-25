// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef _Sel_Bal_Bot_FW_H_
#define _Sel_Bal_Bot_FW_H_
#include "Arduino.h"
//add your includes for the project Sel_Bal_Bot_FW here


//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

//add your function definitions for the project Sel_Bal_Bot_FW here

#define TIMER1_INTERVAL		1000000 // 1 sec

extern bool isConnected;
extern bool blinkState;

void initialize_robot(void);



//Do not add code below this line
#endif /* _Sel_Bal_Bot_FW_H_ */

