#include "Arduino.h"
#include "common.hpp"
#include "rc_pwm.hpp"
#include <util/atomic.h>

#define pinPwmIsr 3
#define portPinPwmIsr PD0
#define pinThrottle 0        // RX
#define portPinThrottle PD2  // INT2
#define pinManual 1          // TX on arduino , mapped to 'gear' on receiver
#define portPinManual PD3    // INT3

#define RC_TIMEOUT_LOOPS 250 // 500hz loop rate, 50hz rc rate, allow 1/2 second

#define DZ_HALFUS 30 // this much either side of midpoint

static volatile uint16_t v_THR_START; // treats 0 as cleared state so technically loses 1/65535 rc signals
static volatile uint16_t v_THR_SAMPLES[3];
static volatile uint8_t v_THR_SAMPLE_COUNT;
static volatile uint16_t v_THR_SAMPLE_SEQ;
static volatile uint16_t v_THR_VAL; /* LSB 0.5us */

static volatile uint16_t v_MAN_START; // treats 0 as cleared state so technically loses 1/65535 rc signals
static volatile uint16_t v_MAN_SAMPLES[3];
static volatile uint8_t v_MAN_SAMPLE_COUNT;
static volatile uint16_t v_MAN_VAL; /* LSB 0.5us */

static inline uint16_t medianOfThree(uint16_t first, uint16_t second, uint16_t third)
{
    uint16_t swap;
    if (first > second)
    {
        swap = first;
        first = second;
        second = swap;
    }
    if (second > third)
    {
        swap = second;
        second = third;
        third = swap;
    }
    if (first > second)
    {
        second = first;
    }
    return second;
}

static inline uint16_t updateMedian(volatile uint16_t *samples, volatile uint8_t *sample_count, uint16_t sample)
{
    if (*sample_count < 3)
    {
        samples[*sample_count] = sample;
        ++*sample_count;
        if (*sample_count < 3)
        {
            return sample;
        }
    }
    else
    {
        samples[0] = samples[1];
        samples[1] = samples[2];
        samples[2] = sample;
    }

    return medianOfThree(samples[0], samples[1], samples[2]);
}

void rcPwmSetup()
{
    pinMode(pinPwmIsr, OUTPUT);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        pinMode(pinThrottle, INPUT_PULLUP);
        pinMode(pinManual, INPUT_PULLUP);

        // disable external interrupt before configuring
        EIMSK = 0;

        // interrupt on either edge of either pin
        EICRA = _BV(ISC30) | _BV(ISC20);

        // clear ifr
        EIFR = _BV(INTF3) | _BV(INTF2);

        // re-enable interrupts
        EIMSK = _BV(INT3) | _BV(INT2);

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

// called at end of dshot critical section (inside it), throw away any pin changes that happened during the critical section (prioritize maintaining main pid loop consistency)
void reEnableRcPwm()
{
    if (EIFR & _BV(INTF2)) // restart timing routine if pin change arrive at some point during the dshot critical section
    {
        v_THR_START = 0;
    }

    if (EIFR & _BV(INTF3)) // restart timing routine if pin change arrive at some point during the dshot critical section
    {
        v_MAN_START = 0;
    }

    // clear ifr, if the interrupt happens betwen the check and here it doesn't matter because if there is a missed rising edge, the time delta from the last rising edge is so big it will be filtered out, missed falling edge obviously doesn't matter
    EIFR = _BV(INTF3) | _BV(INTF2);
}

ISR(INT2_vect)
{
    uint16_t cnt = TCNT3;
    SET_BIT(PORTD, portPinPwmIsr);
    if (PIND & _BV(portPinThrottle)) // rising edge
    {
        v_THR_START = cnt;
    }
    else
    {

        uint16_t timer_delta = (cnt - v_THR_START);
        if ((v_THR_START == 0) || (timer_delta > (4200)) || (timer_delta < (1800))) // only allow between 900us and 2100us
        {
            v_THR_START = 0;
            CLR_BIT(PORTD, portPinPwmIsr);
            return; // discard invalid value
        }
        v_THR_START = 0;

        v_THR_VAL = updateMedian(v_THR_SAMPLES, &v_THR_SAMPLE_COUNT, timer_delta);
        ++v_THR_SAMPLE_SEQ;
    }
    CLR_BIT(PORTD, portPinPwmIsr);
}

ISR(INT3_vect)
{
    uint16_t cnt = TCNT3;
    SET_BIT(PORTD, portPinPwmIsr);
    if (PIND & _BV(portPinManual)) // rising edge
    {
        v_MAN_START = cnt;
    }
    else
    {

        uint16_t timer_delta = (cnt - v_MAN_START);
        if ((v_MAN_START == 0) || (timer_delta > (4200)) || (timer_delta < (1800))) // only allow between 900us and 2100us
        {
            v_MAN_START = 0;
            CLR_BIT(PORTD, portPinPwmIsr);
            return; // discard invalid value
        }
        v_MAN_START = 0;

        v_MAN_VAL = updateMedian(v_MAN_SAMPLES, &v_MAN_SAMPLE_COUNT, timer_delta);
    }
    CLR_BIT(PORTD, portPinPwmIsr);
}

bool checkIsMissing(uint16_t v_val, uint16_t v_update_count, uint16_t *ptr_last_update_count, uint8_t *ptr_unchanging_loops)
{
    // corruption from nonatomic read during write also counts as a non-missing signal... so don't care

    if (v_val == 0) // return missing until filter accumulator first emptied
    {
        return true;
    }

    if (v_update_count != *ptr_last_update_count)
    {
        *ptr_unchanging_loops = 0;
        *ptr_last_update_count = v_update_count;
        return false;
    }

    if (*ptr_unchanging_loops < RC_TIMEOUT_LOOPS)
    {
        (*ptr_unchanging_loops)++;
        return false;
    }

    return true;
}

bool checkRcMissing() // not checking dir sw cuz it will be a pretty fixed value
{
    static uint16_t last_throttle_update_count;
    static uint8_t unchanging_throttle_loops;

    return checkIsMissing(v_THR_VAL, v_THR_SAMPLE_SEQ, &last_throttle_update_count, &unchanging_throttle_loops);
}

static void getAxis(volatile uint16_t *p_V_AXIS_VAL, bool *p_last_direction, uint16_t *p_magnitude, bool *p_new_direction)
{
    uint16_t axis_val;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        axis_val = *p_V_AXIS_VAL;
    }
    if (axis_val > 4000 + DZ_HALFUS)
    {
        *p_magnitude = 1000;
        *p_new_direction = 1;
    }
    else if (axis_val < 2000 - DZ_HALFUS)
    {
        *p_magnitude = 1000;
        *p_new_direction = 0;
    }
    else if (axis_val > (3000 + DZ_HALFUS))
    {
        *p_magnitude = axis_val - (3000 + DZ_HALFUS);
        *p_new_direction = 1;
    }
    else if (axis_val < (3000 - DZ_HALFUS))
    {
        *p_magnitude = (3000 - DZ_HALFUS) - axis_val;
        *p_new_direction = 0;
    }
    else
    {
        *p_magnitude = 0;
        *p_new_direction = *p_last_direction;
    }
    *p_last_direction = *p_new_direction;
}

void getThrottle(uint16_t *p_throttle, bool *p_direction)
{
    static bool last_direction;
    getAxis(&v_THR_VAL, &last_direction, p_throttle, p_direction);
}

void getManSw(uint16_t *p_magnitude, bool *p_direction)
{
    static bool last_direction;
    getAxis(&v_MAN_VAL, &last_direction, p_magnitude, p_direction);
}