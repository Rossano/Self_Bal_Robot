// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef _nav6_H_
#define _nav6_H_
#include "Arduino.h"
//add your includes for the project nav6 here


//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

//add your function definitions for the project nav6 here
boolean initialize_mpu();
void enable_mpu();
void getEuler(float *data, Quaternion *q);
void getGravity(struct FloatVectorStruct *v, Quaternion *q);
void dmpGetYawPitchRoll(float *data, Quaternion *q, struct FloatVectorStruct *gravity);
int freeMemory();
void sendQuaternion(Quaternion & q);
void sendYawPitchRoll(float y, float p, float r);

//Do not add code below this line
#endif /* _nav6_H_ */
