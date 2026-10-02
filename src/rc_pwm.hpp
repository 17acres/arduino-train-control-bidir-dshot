#pragma once
#include "Arduino.h"
void rcPwmSetup();

//Throw away pulse count if placed in critical section by dshot tx
void disableRcPwm();
void reEnableRcPwm();

bool checkRcMissing();

//returns throttle from 0 to 1000, with deadzone and direction stickiness (only updated if out of dz)
void getThrottle(uint16_t *throttle, bool *direction);
void getManSw(uint16_t *magnitude, bool *direction);