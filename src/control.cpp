#include "control.hpp"
#include "common.hpp"
#include <math.h>

uint16_t filter_rpm(uint16_t rpm);

// to maximize resolution, want to shift left as much as possible before converting to int so that coeff * shift is 15 bits or less

#define VFF_COEFF_SHIFT 15 // max of about 16 for p of 1 (16-log2(Vff))
#define Vff 0.05            // throttle counts per rpm

#define P_COEFF_SHIFT 15
#define Kp 0.2 // throttle counts per rpm error

#define I_COEFF_SHIFT 20 // max of like 22 for i of 1 (because of extra scaling for loop time)
#define Ki 0.4           // throttle counts per rpm*second of iacc

#define Vff_int FLOAT_2_INT16(Vff, VFF_COEFF_SHIFT - RPM_SHIFT)
#define Kp_int FLOAT_2_INT16(Kp, P_COEFF_SHIFT - RPM_SHIFT)
#define Ki_int FLOAT_2_INT16(Ki *main_loop_halfmicros / 2000000, I_COEFF_SHIFT)

#define IACC_MAX (1l<<17)

void run_control(uint16_t filtered_rpm, uint16_t speed_tgt, bool dir_tgt, uint16_t *throttle_ptr, bool *motor_dir, ControlLogStruct *logStruct)
{
    static bool current_dir;
    static int32_t iacc; // LSB of 1 rpm*loop
    uint16_t throttle;

    if (filtered_rpm == 0) // clipped to 0 at low speeds
    {
        *motor_dir = dir_tgt;
        current_dir = dir_tgt;
    }
    else // not changing direction
    {
        *motor_dir = current_dir;
        if (dir_tgt != current_dir) // slow down for direction change
        {
            speed_tgt = 0;
            iacc = 0;
        }
    }

    int32_t error = (int32_t)speed_tgt - (int32_t)filtered_rpm; // have headroom of 15 bits until overflow

    if (((iacc > -IACC_MAX) && (iacc < IACC_MAX)) || ((iacc > IACC_MAX) && (error < 0)) || ((iacc < -IACC_MAX) && (error > 0)))
    {
        iacc += RAW_2_RPM(error);//convert lsb
    }

    int32_t vff_term = ((int32_t)speed_tgt * Vff_int) >> VFF_COEFF_SHIFT;
    int32_t p_term = (error * Kp_int) >> P_COEFF_SHIFT;
    int32_t i_term = (iacc * Ki_int) >> I_COEFF_SHIFT;
    int32_t pid_rq = vff_term + p_term + i_term;

    if (pid_rq > 0)
    {
        if (pid_rq <= 1000)
        {
            throttle = pid_rq;
        }
        else
        {
            throttle = 1000;
        }
    }
    else
    {
        throttle = 0; // could cause problems to command negative duty cycle to active brake because the rpm feedback is unsigned
        // if(pid_rq>=-1000)
        // {
        //     throttle = -pid_rq;
        // }
    }

    logStruct->error = error;
    logStruct->iacc = iacc;
    logStruct->vff_term = vff_term;
    logStruct->p_term = p_term;
    logStruct->i_term = i_term;
    logStruct->pid_rq = pid_rq;

    *throttle_ptr = throttle;
}
