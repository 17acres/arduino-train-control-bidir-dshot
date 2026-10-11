#include "Arduino.h"
#include "arduino_dshot.hpp"
#include "common.hpp"
#include "control.hpp"
#include "rc_pwm.hpp"
#include "voltage.hpp"
#include <util/atomic.h>

#define FOURTH_BYTE(var) ((uint8_t)((var >> 24) & 0xFF))
#define THIRD_BYTE(var) ((uint8_t)((var >> 16) & 0xFF))
#define SECOND_BYTE(var) ((uint8_t)((var >> 8) & 0xFF))
#define LOWER_BYTE(var) ((uint8_t)(var & 0xFF))
#define DUMP_U16(var, res) SECOND_BYTE(var), LOWER_BYTE(var), ','
#define DUMP_U32(var, res) FOURTH_BYTE(var), THIRD_BYTE(var), SECOND_BYTE(var), LOWER_BYTE(var), ','
#define DUMP_S32(var, res) DUMP_U32(var,res)
#define DUMP_U8(var, res) (var), ','

#define ESC_TIMEOUT_LOOPS 100
#define INIT_LOOPS 2500 // 5 second of command 0
#define pinMainLoop 5
#define portPinMainLoop PC6

#define portPinRxLed PB0

void setup()
{
    Serial.begin(115200);
    // while(!Serial); actually waits for port to be open on the host!
    dshotSetup();
    rcPwmSetup();
    initVoltage();
    pinMode(pinMainLoop, OUTPUT);
    pinMode(LED_BUILTIN_RX,OUTPUT);

    TIMSK0 = 0; // disable timer0
    // Still get some lag spikes affecting RC value when plugged in with USB because of random USB stuff (even when not doing serial)
}

uint16_t filter_rpm(uint16_t rpm)
{
    static uint16_t filtered_rpm = 0;
    filtered_rpm += ((int16_t)((int32_t)rpm - filtered_rpm)) >> 4;
    return filtered_rpm;
}

// throttle is using 10 bits, so is voltage
uint16_t voltage_comp(uint16_t throttle, uint16_t voltage)
{
    if (voltage < 400) // only vcomp over like 10V
    {
        voltage = 400;
    }
    uint32_t tmp = (((uint32_t)throttle) << 10) / (uint32_t)voltage; // this cost 30us more than shifting voltage right by 4
    if (tmp > 1000)
        return 1000;
    return tmp;
}

#define HZ_TO_OVERFLOWS(hz) ((uint16_t)((1000/(32.767*2*hz)+1)))
void run_flash_pattern(uint16_t toggle_overflows,uint16_t overflows)
{
    static uint16_t last_change_overflows = 0;
    static bool state= false;
    if((overflows-last_change_overflows) > toggle_overflows)
    {
        state^=true;
        last_change_overflows = overflows;
    }

    if(state) //need to write every loop to win against serial code
    {
        SET_BIT(PORTB,portPinRxLed);
    }
    else
    {
        CLR_BIT(PORTB,portPinRxLed);
    }
}

void loop()
{
    static uint8_t esc_missing_ctr;
    static uint16_t init_loop_ctr;
    static uint16_t commutation_period = INT16_MAX;
    static bool crc_ok = false;
    static uint16_t last_halfmicros = 0;
    static uint16_t halfmicros_overflows = 0;
    static uint16_t filt_rpm_rq = 0;
    uint16_t filt_rpm_rq_4control = 0;
    uint16_t thr_req = 0;
    bool thr_direction = 0;
    uint16_t man_magnitude = 0;
    bool man_direction = 0;
    uint16_t unfilt_rpm = 0;
    uint16_t filtered_rpm = 0;
    uint16_t voltage = 0;
    ControlLogStruct logStruct = {0, 0, 0, 0, 0, 0};

    uint16_t throttle = 0;
    bool is_fwd = false;
    uint16_t current_halfmicros;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        current_halfmicros = TCNT3;
    }
    // spin until main_loop_halfmicros has elapsed and allowed to run
    if (((current_halfmicros - last_halfmicros) < main_loop_halfmicros))
    {
        return;
    }
    if (current_halfmicros < last_halfmicros) //every 32.768ms
    {
        halfmicros_overflows += 1;
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
        CLR_BIT(PORTB,portPinRxLed); //solid on
    }
    else if (esc_missing_ctr >= ESC_TIMEOUT_LOOPS)
    {
        Serial.println("ESC Missing Timeout");
        run_flash_pattern(1,halfmicros_overflows);
        throttle = THROTTLE_MOTOR_STOP;
    }
    else if (checkRcMissing())
    {
        Serial.println("R/C Signal Missing Timeout");
        throttle = THROTTLE_MOTOR_STOP;
        run_flash_pattern(HZ_TO_OVERFLOWS(5),halfmicros_overflows);
    }
    else
    {
        unfilt_rpm = RPM_2_RAW(8571428) / commutation_period; // 60,000,000/(14/2) max rpm is about 10000 unladen
        filtered_rpm = filter_rpm(unfilt_rpm);
        if(filtered_rpm < RPM_2_RAW(200))
        {
            filtered_rpm = 0;
        }

        getThrottle(&thr_req, &thr_direction);
        thr_direction = !thr_direction; //invert direction
        filt_rpm_rq += (int16_t)((int32_t)RPM_2_RAW(thr_req << 4) - filt_rpm_rq) >> 4;
        getManSw(&man_magnitude, &man_direction);
        voltage = readVoltage();
        CLR_BIT(PORTC, portPinMainLoop);

        if (man_direction)
        {
            throttle = RAW_2_RPM(filt_rpm_rq >> 4);
            is_fwd = thr_direction;
            run_flash_pattern(HZ_TO_OVERFLOWS(1),halfmicros_overflows);
        }
        else
        {
            if(filt_rpm_rq > RPM_2_RAW(400))  //deadzone because the rpm feedback has a deadzone
            {
                filt_rpm_rq_4control = filt_rpm_rq;
            }
            else
            {
                filt_rpm_rq_4control = 0;
            }
            run_control(filtered_rpm, filt_rpm_rq, thr_direction, &throttle, &is_fwd, &logStruct);
            run_flash_pattern(HZ_TO_OVERFLOWS(0.5),halfmicros_overflows);
        }
        throttle = voltage_comp(throttle, voltage);
    }
    crc_ok = doDshotTransaction(throttle, is_fwd, &commutation_period); /* will not write to commutation_period if crc is faulty */
    SET_BIT(PORTC, portPinMainLoop);
    #define Timestamp ((((uint32_t)halfmicros_overflows) << 16) + current_halfmicros)
    uint8_t dataPacket[] = {
        DUMP_U32(Timestamp, 0.0000005), // name for advantagescope
        DUMP_U16(thr_req, 1),
        DUMP_U16(filt_rpm_rq_4control, 0.25), // RAW_2_RPM is >>2
        DUMP_U8(thr_direction, 1),
        DUMP_U8(man_direction, 1),
        DUMP_U16(voltage, 1),
        DUMP_U16(unfilt_rpm, 0.25),
        DUMP_U16(filtered_rpm, 0.25),
        DUMP_U16(throttle, 1),
        DUMP_S32(logStruct.error, 0.25),
        DUMP_S32(logStruct.iacc, 0.002),
        DUMP_S32(logStruct.vff_term, 1),
        DUMP_S32(logStruct.p_term, 1),
        DUMP_S32(logStruct.i_term, 1),
        DUMP_S32(logStruct.pid_rq, 1),
        0, 0, 0, 0, 0};
    Serial.write(dataPacket, sizeof(dataPacket));
    // don't want to change performance with/without print by doing this before the dshot transaction

    last_halfmicros = current_halfmicros;
    CLR_BIT(PORTC, portPinMainLoop);
}