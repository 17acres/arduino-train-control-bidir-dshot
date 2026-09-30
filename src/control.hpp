#pragma once
#include "Arduino.h"

void run_control(uint16_t commutation_period, uint16_t speed_tgt, bool dir_tgt, uint16_t *throttle_ptr, bool *motor_dir);