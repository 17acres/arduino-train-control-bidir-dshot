#pragma once
extern void dshotSetup();

#define THROTTLE_MOTOR_STOP 0xDEAD //will cause value of actual 0 to be sent over dshot, not 48
extern bool doDshotTransaction(uint16_t throttle, bool is_fwd, uint16_t *p_commutation_period);

