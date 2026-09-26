#include "Arduino.h"
#include "arduino_dshot.hpp"

void setup() {
    dshotSetup();

    /*
    if(enableEdt) {
      frame = dshot.buildFrame(13, 1);
    }
    */
}

void loop() {
    if(FRAME_COMPLETE)/* Timer ISR for DSHOT just finished - previous command sent and RPM feedback hopefully received - run main program loop synchronously with this */
    {
        FRAME_COMPLETE = false;
        processTelemetryResponse(); /* Updates */

    }
}