#include "Arduino.h"
#include "common.hpp"
#include "rc_pwm.hpp"
#include <util/atomic.h>

#define pinThrottle 0       // RX
#define portPinThrottle PD2 // INT2
#define pinManual 1         // TX on arduino , mapped to 'gear' on receiver
#define portPinManual PD3   // INT3

#define RC_TIMEOUT_LOOPS 250 // 500hz loop rate, 50hz rc rate, allow 1/2 second

static volatile uint16_t v_THR_START; // treats 0 as cleared state so technically loses 1/65535 rc signals
static volatile uint16_t v_THR_ACCUM;
static volatile uint8_t v_THR_ACCUM_CNT;
volatile uint16_t v_THR_VAL;

void rcPwmSetup()
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        pinMode(pinThrottle, INPUT);
        pinMode(pinManual, INPUT);

        // disable external interrupt before configuring
        EIMSK = 0;

        // interrupt on either edge of either pin
        EICRA = _BV(ISC31) | _BV(ISC30) | _BV(ISC21) | _BV(ISC20);

        // clear ifr
        EIFR = _BV(INTF3) | _BV(INTF2);

        // re-enable interrupts
        EIMSK = /*_BV(INT3) | */ _BV(INT2);

        TCCR3B = 0;
        TCCR3A = 0;
        TIMSK3 = 0;
        TCNT3 = 0;

        TCCR3B = _BV(CS31); // straight count, prescaler 8
    }
}

void disableRcPwm()
{
}

// called at end of dshot critical section (inside it), throw away any pin changes that happened during the critical section
void reEnableRcPwm()
{
    if (EIFR & _BV(INTF2)) //restart timing routine if pin change arrive at some point during the dshot critical section
    {
        v_THR_START = 0;
    }

    // clear ifr, if the interrupt happens betwen the check and here it doesn't matter because if there is a missed rising edge, the time delta from the last rising edge is so big it will be filtered out, missed falling edge obviously doesn't matter
    EIFR = _BV(INTF3) | _BV(INTF2);
}

ISR(INT2_vect)
{
    uint16_t cnt = TCNT3;
    if (PORTD & _BV(portPinThrottle)) // rising edge
    {
        v_THR_START = cnt;
    }
    else
    {
        uint16_t timer_delta = (cnt - v_THR_START);
        if ((v_THR_START == 0) || (timer_delta > (4200)) || (timer_delta < (1800))) //only allow between 900us and 2100us
        {
            v_THR_START = 0;
            return; // discard invalid value
        }
        v_THR_START = 0;

        // use accumulator to average over four samples
        v_THR_ACCUM += timer_delta;
        v_THR_ACCUM_CNT++;
        if (v_THR_ACCUM_CNT == 4)
        {
            v_THR_ACCUM_CNT = 0;
            v_THR_VAL = v_THR_ACCUM >> 2; // divide by 4
            v_THR_ACCUM = 0;
        }
    }
}

bool checkIsMissing(uint16_t v_val, uint16_t v_accum, uint16_t *ptr_last_accum, uint8_t *ptr_unchanging_loops)
{
    // corruption from nonatomic read during write also counts as a non-missing signal... so don't care

    if (v_val == 0) // return missing until filter accumulator first emptied
    {
        return true;
    }

    if (v_accum != *ptr_last_accum)
    {
        *ptr_unchanging_loops = 0;
        *ptr_last_accum = v_accum;
        return false;
    }

    if (*ptr_unchanging_loops < RC_TIMEOUT_LOOPS)
    {
        (*ptr_unchanging_loops)++;
        return false;
    }

    return true;
}

bool checkRcMissing()
{
    static uint16_t last_throttle_accum;
    static uint8_t unchanging_throttle_loops;

    return checkIsMissing(v_THR_VAL, v_THR_ACCUM, &last_throttle_accum, &unchanging_throttle_loops);
}