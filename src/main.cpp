#include "Arduino.h"
#include "arduino_dshot.hpp"
#include "common.hpp"

#define ESC_TIMEOUT_LOOPS 10
#define pinMainLoop 5
#define portPinMainLoop PD5 //green led

void setup() {
    Serial.begin(115200);
    while(!Serial);
    dshotSetup();
    pinMode(pinMainLoop, OUTPUT);
    stopMotor();
}

void loop() {  
    if(v_FRAME_COMPLETE)/* Timer ISR for DSHOT just finished - previous command sent and RPM feedback hopefully received - run main program loop synchronously with this */
    {
        SET_BIT(PORTD, portPinMainLoop);
        static uint8_t esc_missing_ctr;
        static uint16_t commutation_period = INT16_MAX;

        bool crc_ok;


        v_FRAME_COMPLETE = false;

        crc_ok = processTelemetryResponse(&commutation_period); /* will not write to commutation_period if crc is faulty */

        if(crc_ok)
        {
            esc_missing_ctr = 0;
        }
        else if (esc_missing_ctr < ESC_TIMEOUT_LOOPS)
        {
            esc_missing_ctr++;
        }

        if(esc_missing_ctr >= ESC_TIMEOUT_LOOPS)
        {
            Serial.println("ESC Missing Timeout");
            stopMotor();
            return;
        }

        requestThrottle(((millis())>4)%1000,true);
        CLR_BIT(PORTD, portPinMainLoop);
    }
}