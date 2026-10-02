#include "control.hpp"
#include "common.hpp"

uint16_t filter_rpm(uint16_t rpm);

#define Vff

void run_control(uint16_t filtered_rpm, uint16_t speed_tgt, bool dir_tgt, uint16_t *throttle_ptr, bool *motor_dir)
{
    static uint16_t last_throttle;
    static bool current_dir;
    uint16_t throttle;


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

    last_throttle = throttle;
    *throttle_ptr = throttle;
}

