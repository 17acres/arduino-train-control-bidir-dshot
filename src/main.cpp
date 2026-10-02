#include "Arduino.h"
#include "arduino_dshot.hpp"
#include "common.hpp"
#include "control.hpp"
#include "rc_pwm.hpp"
#include "voltage.hpp"
#include <util/atomic.h>


#define ESC_TIMEOUT_LOOPS 100
#define INIT_LOOPS 2500 // 5 second of command 0
#define pinMainLoop 5
#define portPinMainLoop PC6 // green led
#define main_loop_micros 2000

void setup()
{
    Serial.begin(115200);
    // while(!Serial); actually waits for port to be open on the host!
    dshotSetup();
    rcPwmSetup();
    initVoltage();
    pinMode(pinMainLoop, OUTPUT);
}

uint16_t filter_rpm(uint16_t rpm)
{
    static uint16_t filtered_rpm = 0;
    filtered_rpm += ((int16_t)((int32_t)rpm - filtered_rpm)) >> 4;
    return filtered_rpm;
}

void loop()
{
    static uint8_t esc_missing_ctr;
    static uint16_t init_loop_ctr;
    static uint16_t commutation_period = INT16_MAX;
    static bool crc_ok = false;
    static uint32_t last_micros = 0;
    uint16_t thr_req = 0;
    bool thr_direction = 0;
    uint16_t man_magnitude = 0;
    bool man_direction = 0;
    uint16_t unfilt_rpm = 0;
    uint16_t filtered_rpm = 0;
    uint16_t voltage = 0;

    uint16_t throttle = 0;
    bool is_fwd = false;

    uint32_t current_micros = micros();
    //spin until main_loop_micros has elapsed and allowed to run
    if(((current_micros-last_micros)<main_loop_micros))
    {
        return;
    }
    
    SET_BIT(PORTC, portPinMainLoop);


    if (crc_ok)
    {
        esc_missing_ctr = 0;
    }
    else if (esc_missing_ctr < ESC_TIMEOUT_LOOPS)
    {
        esc_missing_ctr++;
    }

    if (init_loop_ctr < INIT_LOOPS)
    {
        init_loop_ctr++;
    }

    if (init_loop_ctr < INIT_LOOPS)
    {
        Serial.println("Init Delay");
        throttle = THROTTLE_MOTOR_STOP;
    }
    else if(esc_missing_ctr >= ESC_TIMEOUT_LOOPS)
    {
        Serial.println("ESC Missing Timeout");
        throttle = THROTTLE_MOTOR_STOP;
    }
    else if(checkRcMissing())
    {
        Serial.println("R/C Signal Missing Timeout");
        throttle = THROTTLE_MOTOR_STOP;
    }
    else
    {
        unfilt_rpm = RPM_2_RAW(8571428) / commutation_period; // 60,000,000/(14/2) max rpm is about 10000 unladen
        filtered_rpm = filter_rpm(unfilt_rpm);

        getThrottle(&thr_req,&thr_direction);
        getManSw(&man_magnitude, &man_direction);
        voltage = readVoltage();
        
        if(man_direction)
        {
            throttle = thr_req;
            is_fwd = thr_direction;
        }
        else
        {
            run_control(filtered_rpm, RPM_2_RAW(thr_req<<4), thr_direction, &throttle, &is_fwd);
        }
    }
    crc_ok = doDshotTransaction(throttle, is_fwd, &commutation_period); /* will not write to commutation_period if crc is faulty */

    //don't want to change performance with/without print by doing this before the dshot transaction
    Serial.print(thr_req);
    Serial.print(",");
    Serial.print(thr_direction);
    Serial.print(",");
    // Serial.print(man_magnitude);
    // Serial.print(",");
    // Serial.print(man_direction);
    //Serial.print(",");
    Serial.print(voltage);
    Serial.print(",");
    Serial.print(RAW_2_RPM(unfilt_rpm));
    Serial.print(",");
    Serial.print(RAW_2_RPM(filtered_rpm));
    Serial.print(",");
    Serial.print(thr_req<<4); //rpm target
    Serial.print(",");
    Serial.println(throttle);
    last_micros = current_micros;
    CLR_BIT(PORTC, portPinMainLoop);

}