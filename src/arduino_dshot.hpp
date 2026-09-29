#pragma once
extern void requestThrottle(uint16_t throttle, bool is_fwd);
extern void stopMotor();
extern bool processTelemetryResponse(uint16_t *commutation_period);
extern void dshotSetup();
extern void doDshotTransaction();