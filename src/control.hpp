#pragma once
#include "Arduino.h"

void run_control(uint16_t commutation_period, uint16_t speed_tgt, uint16_t *throttle, bool *is_fwd);