#pragma once
#include "Arduino.h"
#define SET_BIT(reg,bit) (reg |= _BV(bit))
#define CLR_BIT(reg,bit) (reg &= ~_BV(bit))

#define RPM_SHIFT 2
#define RAW_2_RPM(raw) (raw>>2)
#define RPM_2_RAW(rpm) (rpm<<2)