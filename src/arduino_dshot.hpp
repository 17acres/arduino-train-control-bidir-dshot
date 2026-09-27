extern void requestThrottle(uint16_t throttle, bool is_fwd);
extern void stopMotor();
extern bool processTelemetryResponse(uint16_t *commutation_period);
extern void dshotSetup();
extern bool v_FRAME_COMPLETE;