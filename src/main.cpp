#include "Arduino.h"
#include "arduino_dshot.hpp"
#include "common.hpp"
#include "control.hpp"

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
    pinMode(pinMainLoop, OUTPUT);
}

void loop()
{
    static uint8_t esc_missing_ctr;
    static uint16_t init_loop_ctr;
    static uint16_t commutation_period = INT16_MAX;
    static bool crc_ok = false;
    static uint32_t last_micros = 0;

    uint16_t throttle;
    bool is_fwd = false;

    uint32_t current_micros = micros();
    //spin until main_loop_micros has elapsed
    if((current_micros-last_micros)<main_loop_micros)
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

    if ((init_loop_ctr < INIT_LOOPS) || (esc_missing_ctr >= ESC_TIMEOUT_LOOPS))
    {
        Serial.println("ESC Missing Timeout");
        throttle = THROTTLE_MOTOR_STOP;
    }
    else
    {
        run_control(commutation_period, RPM_2_RAW(1000), &throttle, &is_fwd);
    }
    crc_ok = doDshotTransaction(throttle, is_fwd, &commutation_period); /* will not write to commutation_period if crc is faulty */
    last_micros = current_micros;
    CLR_BIT(PORTC, portPinMainLoop);

}