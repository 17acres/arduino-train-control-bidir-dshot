#pragma once
#include "Arduino.h"
void rcPwmSetup();

//Throw away pulse count if placed in critical section by dshot tx
void disableRcPwm();
void reEnableRcPwm();

bool checkRcMissing();

extern volatile uint16_t v_THR_VAL;