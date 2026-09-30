#include "control.hpp"
#include "common.hpp"

uint16_t filter_rpm(uint16_t rpm);

#define Vff

void run_control(uint16_t commutation_period, uint16_t speed_tgt, bool dir_tgt, uint16_t *throttle_ptr, bool *motor_dir)
{
    static uint16_t last_throttle;
    static bool current_dir;
    uint16_t throttle;
    uint16_t unfilt_rpm = RPM_2_RAW(8571428) / commutation_period; // 60,000,000/(14/2) max rpm is about 10000 unladen
    uint16_t filtered_rpm = filter_rpm(unfilt_rpm);

    if (filtered_rpm < RPM_2_RAW(200)) //stops reporting at low numbers
    {
        *motor_dir = dir_tgt;
        current_dir = dir_tgt;
    }
    else //not changing direction
    {
        *motor_dir = current_dir;
        if (dir_tgt != current_dir) //slow down for direction change
        {
            speed_tgt = 0;
        }
    }

    if (filtered_rpm >= speed_tgt)
    {
        if (last_throttle > 0)
            throttle = last_throttle - 1;
        else
            throttle = 0;
    }
    else
    {
        throttle = (speed_tgt - filtered_rpm) >> 4; // TODO velocity feed forward
    }

    Serial.print(RAW_2_RPM(unfilt_rpm));
    Serial.print(",");
    Serial.print(RAW_2_RPM(filtered_rpm));
    Serial.print(",");
    Serial.println(throttle);

    last_throttle = throttle;
    *throttle_ptr = throttle;
}

uint16_t filter_rpm(uint16_t rpm)
{
    static uint16_t filtered_rpm = 0;
    filtered_rpm += ((int16_t)((int32_t)rpm - filtered_rpm)) >> 4;
    return filtered_rpm;
}