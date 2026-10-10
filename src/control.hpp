#pragma once
#include "Arduino.h"
typedef struct{
    int32_t error;
    int32_t iacc;
    int32_t vff_term;
    int32_t p_term;
    int32_t i_term;
    int32_t pid_rq;
} ControlLogStruct;
void run_control(uint16_t filtered_rpm, uint16_t speed_tgt, bool dir_tgt, uint16_t *throttle_ptr, bool *motor_dir, ControlLogStruct *logStruct);

