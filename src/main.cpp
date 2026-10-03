#include "Arduino.h"
#include "arduino_dshot.hpp"
#include "common.hpp"
#include "control.hpp"
#include "rc_pwm.hpp"
#include "voltage.hpp"
#include <util/atomic.h>

#define FOURTH_BYTE(var) ((uint8_t)((var >> 24) &0xFF))
#define THIRD_BYTE(var) ((uint8_t)((var >> 16) & 0xFF))
#define SECOND_BYTE(var) ((uint8_t)((var >> 8) & 0xFF))
#define LOWER_BYTE(var) ((uint8_t)(var & 0xFF))
#define DUMP_U16(var, res) SECOND_BYTE(var), LOWER_BYTE(var), ','
#define DUMP_U32(var, res) FOURTH_BYTE(var), THIRD_BYTE(var),SECOND_BYTE(var), LOWER_BYTE(var), ','
#define DUMP_U8(var, res) (var), ','

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

//throttle is using 10 bits, so is voltage
uint16_t voltage_comp(uint16_t throttle, uint16_t voltage)
{
    if(voltage<400) //only vcomp over like 10V
    {
        voltage = 400;
    }    
    uint32_t tmp = (((uint32_t)throttle) << 10) / (uint32_t)voltage; //this cost 30us more than shifting voltage right by 4
    if (tmp>1000)
        return 1000;
    return tmp;
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
    // spin until main_loop_micros has elapsed and allowed to run
    if (((current_micros - last_micros) < main_loop_micros))
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
    else if (esc_missing_ctr >= ESC_TIMEOUT_LOOPS)
    {
        Serial.println("ESC Missing Timeout");
        throttle = THROTTLE_MOTOR_STOP;
    }
    else if (checkRcMissing())
    {
        Serial.println("R/C Signal Missing Timeout");
        throttle = THROTTLE_MOTOR_STOP;
    }
    else
    {
        unfilt_rpm = RPM_2_RAW(8571428) / commutation_period; // 60,000,000/(14/2) max rpm is about 10000 unladen
        filtered_rpm = filter_rpm(unfilt_rpm);

        getThrottle(&thr_req, &thr_direction);
        getManSw(&man_magnitude, &man_direction);
        voltage = readVoltage();
        CLR_BIT(PORTC, portPinMainLoop);

        if (man_direction)
        {
            throttle = thr_req;
            is_fwd = thr_direction;
        }
        else
        {
            run_control(filtered_rpm, RPM_2_RAW(thr_req << 4), thr_direction, &throttle, &is_fwd);
        }
        throttle = voltage_comp(throttle,voltage);
    }
    crc_ok = doDshotTransaction(throttle, is_fwd, &commutation_period); /* will not write to commutation_period if crc is faulty */
    SET_BIT(PORTC, portPinMainLoop);
#define Timestamp current_micros
    uint8_t dataPacket[] = {
        DUMP_U32(Timestamp, 0.000001), // name for advantagescope
        DUMP_U16(thr_req, 1),
        DUMP_U8(thr_direction, 1),
        DUMP_U8(man_direction, 1),
        DUMP_U16(voltage, 1),
        DUMP_U16(RAW_2_RPM(unfilt_rpm), 1),
        DUMP_U16(RAW_2_RPM(filtered_rpm), 1),
        DUMP_U16(RAW_2_RPM(thr_req << 4), 1), // rpm target
        DUMP_U16(throttle, 1),
        0, 0, 0, 0 ,0};
    Serial.write(dataPacket, sizeof(dataPacket));
    // don't want to change performance with/without print by doing this before the dshot transaction
    last_micros = current_micros;
    CLR_BIT(PORTC, portPinMainLoop);
}