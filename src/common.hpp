#pragma once
#include "Arduino.h"
#define SET_BIT(reg,bit) (reg |= _BV(bit))
#define CLR_BIT(reg,bit) (reg &= ~_BV(bit))