#include "control.hpp"
#include "common.hpp"

uint16_t filter_rpm(uint16_t rpm);

#define Vff

void run_control(uint16_t commutation_period, uint16_t speed_tgt, uint16_t *throttle_ptr, bool *is_fwd_ptr)
{
    static uint16_t last_throttle;
    uint16_t throttle;
    uint16_t unfilt_rpm = RPM_2_RAW(8571428) / commutation_period; // 60,000,000/(14/2) max rpm is about 10000 unladen
    uint16_t filtered_rpm = filter_rpm(unfilt_rpm);

    if (filtered_rpm >= speed_tgt)
    {
        throttle = 0;
    }
    else
    {
        throttle = (speed_tgt - filtered_rpm) >> 2; // TODO velocity feed forward
    }

    // Serial.print(commutation_period);
    // Serial.print(",");
    // Serial.print(RAW_2_RPM(unfilt_rpm));
    // Serial.print(",");
    // Serial.print(RAW_2_RPM(filtered_rpm));
    // Serial.print(",");
    // Serial.println(throttle);

    last_throttle = throttle;
    *throttle_ptr = throttle;
    *is_fwd_ptr = true;
}

uint16_t filter_rpm(uint16_t rpm)
{
    static int16_t filtered_rpm = 0;
    filtered_rpm += ((int16_t)((int32_t)rpm - filtered_rpm)) >> 4;
    return filtered_rpm;
}