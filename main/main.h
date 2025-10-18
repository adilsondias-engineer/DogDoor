#ifndef _MAIN_H_
#define _MAIN_H_
// declaration in the .h file:
#ifdef __cplusplus
extern "C" {
#endif

void openDoor_http_handle();
void closeDoor_http_handle();
const char* getDoorStatus();
bool isMovementDetected();
char* getSensorTriggered();
float getDetectedDistance();

#ifdef __cplusplus
}
#endif
#endif
