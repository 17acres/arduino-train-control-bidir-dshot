#include "Arduino.h"
#include "arduino_dshot.hpp"

#define ESC_TIMEOUT_LOOPS 10

void setup() {
    Serial.begin(115200);
    while(!Serial);
    dshotSetup();

    stopMotor();
}

void loop() {  
    if(v_FRAME_COMPLETE)/* Timer ISR for DSHOT just finished - previous command sent and RPM feedback hopefully received - run main program loop synchronously with this */
    {
        static uint8_t esc_missing_ctr;
        static uint16_t commutation_period = INT16_MAX;

        bool crc_ok;


        v_FRAME_COMPLETE = false;

        crc_ok = processTelemetryResponse(&commutation_period); /* will not write to commutation_period if crc is faulty */

        if(crc_ok)
        {
            esc_missing_ctr = 0;
        }

        if(esc_missing_ctr > ESC_TIMEOUT_LOOPS)
        {
            Serial.println("ESC Missing Timeout");
            stopMotor();
            return;
        }

        requestThrottle(((millis())>4)%1000,true);

    }
}