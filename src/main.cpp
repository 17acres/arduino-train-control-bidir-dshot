#include "Arduino.h"
#include "arduino_dshot.hpp"
#include "common.hpp"
#include "control.hpp"

#define ESC_TIMEOUT_LOOPS 100
#define INIT_LOOPS 2500 // 5 second of command 0
#define pinMainLoop 5
#define portPinMainLoop PC6 // green led
#define main_loop_freq 500


void setup()
{
    Serial.begin(115200);
    // while(!Serial); actually waits for port to be open on the host!
    dshotSetup();
    pinMode(pinMainLoop, OUTPUT);
    stopMotor();
}

void loop()
{
    doDshotTransaction();
    SET_BIT(PORTC, portPinMainLoop);
    static uint8_t esc_missing_ctr;
    static uint16_t init_loop_ctr;
    static uint16_t commutation_period = INT16_MAX;

    bool crc_ok;

    v_FRAME_COMPLETE = false;

    crc_ok = processTelemetryResponse(&commutation_period); /* will not write to commutation_period if crc is faulty */

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

    if ((init_loop_ctr < INIT_LOOPS) || (esc_missing_ctr >= ESC_TIMEOUT_LOOPS))
    {
        Serial.println("ESC Missing Timeout");
        stopMotor();
    }
    else
    {
        uint16_t throttle;
        bool is_fwd;

        run_control(commutation_period, RPM_2_RAW(1000), &throttle, &is_fwd);
        requestThrottle(throttle, is_fwd);
    }
    CLR_BIT(PORTC, portPinMainLoop);
}