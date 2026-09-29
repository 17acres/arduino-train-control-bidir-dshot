#include "Arduino.h"
#include "Dshot.h"
#include "arduino_dshot.hpp"
#include <util/atomic.h>
#include "common.hpp"
/**
 * Update frequencies from 2kHz onwards tend to cause issues in regards
 * to processing the DShot response and will result in actual 3kHz instead.
 *
 * At this point the serial port will not be able to print anything anymore since
 * interrupts are "queing" up, rendering the main loop basically non-functional.
 *
 * 8kHz can ONLY be achieved when sending (uninverted) Dshot und not processing
 * responses.
 *
 * For real world use you should thus not go over 1kHz, better stay at 500Hz, this
 * will give you some headroom in order to serial print some more data.
 *
 * The limit of sending at 3kHz shows that the time difference is the actual time
 * that is needed to process the DShot response - so around 400us - 400kns or 6400
 * clock cycles.
 *
 * Even if response processing can be sped up, at the higher frequencies we would
 * still struggle to serial print the results.
 *
 * NOT USING TIMER FOR THIS SO IT CAN BE USED FOR HIGH ACCURACY RC PWM MEASUREMENT
 */

// Always inverted (bidir dshot)

// DSHOT Output pin
#define pinDshot 4 // PD4 is timer1 input capture
#define portPinDshot PD4
#define pinCriticalSection 6 // PD7 is timer2 ISR-running-indicator
#define portPinCriticalSection PD7

// Timer 1 for input capture of BDSHOT
// Timer 3 for loop timing
// Port D for interface

#define MIN_THR 48u
#define NUM_VALUES_PER_DIR 1000u // reverse commands are this much above fwd commands

#define buffSize 20 // for state_durations array

Dshot dshot = new Dshot(true);

#define SIZE_LUT 23
// Duration LUT - considerably faster than division
const uint8_t state_duration_to_bits_lut[SIZE_LUT] = {
    0,
    0,
    0,
    0,
    1, // 4
    1, // 5 <
    1, // 6
    2, // 7
    2,
    2,
    2, // 10 <
    2,
    2, // 12
    3, // 13
    3,
    3, // 15 <
    3,
    3, // 17

    // There should not be more than 3 bits with the same state after each other
    4, // 18
    4,
    4, // 20 <
    4,
    4, // 22
};

#define DELAY_CYCLES(n) __builtin_avr_delay_cycles(n)

static void sendDshot300Frame(uint16_t frame);
static void sendInvertedDshot300Bit(uint8_t bit);
static void readTelemetryResponse(uint8_t state_durations[buffSize]);
static bool processTelemetryResponse(const uint8_t state_durations[], uint16_t *commutation_period);
static uint16_t buildFrame(uint16_t throttle, bool is_fwd);

static void readTelemetryResponse(uint8_t state_durations[buffSize])
{
    // Set to Input in order to process the response - this will be at 3.3V level
    // I suppose we need to wait long enough anyway so why not use the slow Arduino version
    pinMode(pinDshot, INPUT_PULLUP);

    // Delay around 26us
    DELAY_CYCLES(300);

    register uint8_t ices1High = 0b01000000;
    register uint16_t prevVal = 0;
    register uint8_t tifr;
    volatile register uint8_t *p_state_duration;

    TCCR1A = 0b00000001; // Toggle OC1A on compare match
    TCCR1B = 0b00000010; // trigger on falling edge, prescaler 8, filter off

    // Limit to 70us - that should be enough to fetch the whole response
    // at 2MHz - scale factor 8 - 150 ticks seems to be a sweetspot.
    OCR1A = 150;
    TCNT1 = 0x00;

    TIFR1 = (1 << ICF1) | (1 << OCF1A) | (1 << TOV1); // clear all timer flags
    for (p_state_duration = state_durations; p_state_duration <= &state_durations[buffSize - 1];)
    {
        // wait for edge or overflow (output compare match)
        while (!(tifr = (TIFR1 & ((1 << ICF1) | (1 << OCF1A)))))
        {
        }

        uint16_t val = ICR1;

        // Break if counter overflows
        if (tifr & (1 << OCF1A))
        {
            // Ignore overflow at the beginning of capture, puts garbage in first element?
            if (p_state_duration != &state_durations[0])
            {
                *p_state_duration = 0; // so it doesn't read past the end of the array
                break;
            }
        }

        TCCR1B ^= ices1High;                // toggle the trigger edge
        TIFR1 = (1 << ICF1) | (1 << OCF1A); // clear input capture and output compare flag bit

        *p_state_duration = ICR1 - prevVal;

        prevVal = val;
        p_state_duration++;
    }

    pinMode(pinDshot, OUTPUT);
}

/**
 * Frames are sent MSB first.
 *
 * Unfortunately we can't  rotate through carry on an ATMega.
 * Thus we fetch MSB, left shift the result and then right shift the frame.
 *
 * IMPROVEMENT: Since this part is actually time critical, any improvement that can be
 *              made is a good improvment. Thus when the frame is initially generated
 *              it might make sense to arange it in a way that is benefitial for
 *              transmission.
 */
static void sendDshot300Frame(uint16_t frame)
{
    uint8_t offset = 0;
    do
    {
        sendInvertedDshot300Bit((frame & 0x8000) >> 15);
        frame <<= 1;
    } while (++offset < 0x10);
}

/**
 * digitalWrite takes about 3.4us to execute, that's why we switch ports directly.
 * Switching ports directly will allow a transition in 0.19us or 190ns.
 *
 * In an optimal case, without any lag for sending a "1" we would switch high, stay high for 2500 ns (40 ticks) and then switch back to low.
 * Since a transition takes some time too, we need to adjust the waiting period accordingly. Ther resulting values have been set using an
 * oscilloscope to validate the delay cycles.
 *
 * Duration for a single byte should be 1/300kHz = 3333.33ns = 3.3us or 53.3 ticks
 *
 * The delays after switching back to low are to account for the overhead of going through the loop ins sendBitsDshot*
 */
static void sendInvertedDshot300Bit(uint8_t bit)
{
    if (bit)
    {
        CLR_BIT(PORTD, portPinDshot);
        // DELAY_CYCLES(40);
        DELAY_CYCLES(36);
        SET_BIT(PORTD, portPinDshot);
        // DELAY_CYCLES(13);
        DELAY_CYCLES(6);
    }
    else
    {
        CLR_BIT(PORTD, portPinDshot);
        // DELAY_CYCLES(20);
        DELAY_CYCLES(15);
        SET_BIT(PORTD, portPinDshot);
        // DELAY_CYCLES(33);
        DELAY_CYCLES(24);
    }
}

bool doDshotTransaction(uint16_t throttle, bool is_fwd, uint16_t *p_commutation_period)
{
    // Buffer for counting duration between falling and rising edges
    static uint8_t state_durations[buffSize];

    uint16_t frame = buildFrame(throttle, is_fwd);

    // This is a critical section since it is actually doing the communication
    SET_BIT(PORTD, portPinCriticalSection);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        sendDshot300Frame(frame);
        readTelemetryResponse(state_durations);
    }
    CLR_BIT(PORTD, portPinCriticalSection);

    return processTelemetryResponse(state_durations, p_commutation_period);
}

void dshotSetup()
{
    pinMode(pinDshot, OUTPUT);
    pinMode(pinCriticalSection, OUTPUT);

    // Set the default signal Level
    SET_BIT(PORTD, portPinDshot);
    CLR_BIT(PORTD, portPinCriticalSection);
}

static uint16_t buildFrame(uint16_t throttle, bool is_fwd)
{
    if (throttle == THROTTLE_MOTOR_STOP)
    {
        throttle = 0;
    }
    else
    {
        if (throttle >= NUM_VALUES_PER_DIR)
        {
            throttle = NUM_VALUES_PER_DIR - 1;
        }

        if (is_fwd)
        {
            throttle += MIN_THR;
        }
        else
        {
            throttle += (MIN_THR + NUM_VALUES_PER_DIR);
        }
    }

    return dshot.buildFrame(throttle, 0);
}

static bool processTelemetryResponse(const uint8_t state_durations[], uint16_t *commutation_period)
{
    // Statistics for success rate
    static uint16_t ls_receivedPackets = 0;
    static uint16_t ls_successPackets = 0;

    uint32_t dshotResponse;

    ls_receivedPackets++;

    // Set all 21 possible bits to one and flip the once that should be zero
    dshotResponse = 0x001FFFFF;
    unsigned long bitValue = 0x00;
    uint8_t bitCount = 0;
    for (uint8_t i = 1; i < buffSize; i += 1)
    {
        // We are done once the first intereval has a 0 value or the duration is too long (will cause crc failure in that case).
        if ((state_durations[i] == 0) || (state_durations[i] >= SIZE_LUT))
        {
            break;
        }

        bitValue ^= 0x01; // Toggle bit value - always start with 0
        uint8_t num_constant_bits = state_duration_to_bits_lut[state_durations[i]];
        for (uint8_t j = 0; j < num_constant_bits; j += 1)
        {
            dshotResponse ^= (bitValue << (20 - bitCount++));
        }
    }

    // Decode GCR 21 -> 20 bit (since the 21st bit is definetly a 0)
    dshotResponse ^= (dshotResponse >> 1);

    uint16_t mapped = dshot.mapTo16Bit(dshotResponse);
    uint8_t crc = mapped & 0x0F;
    uint16_t value = mapped >> 4;
    uint8_t crcExpected = dshot.calculateCrc(value);

    // Serial.println(mapped, BIN);
    //  Serial.print(value, BIN);
    //  Serial.print(" ");
    //  Serial.println(crc, BIN);

    if (crc != crcExpected)
    {
        return false;
    }
    ls_successPackets++;

    // Reset packet count if overflows
    if (!ls_receivedPackets)
    {
        ls_successPackets = 0;
    }

    // DShot Frame: EEEMMMMMMMMM
    uint32_t periodBase = value & 0b0000000111111111;
    uint8_t periodShift = value >> 9 & 0b00000111;
    *commutation_period = periodBase << periodShift;

    return true;

    //   #if debug
    //     float successPercent = (ls_successPackets * 1.0 / ls_receivedPackets * 1.0) * 100;
    //   #endif

    //   Serial.print(periodTime);
    //   #if debug
    //     Serial.print("us ");
    //     Serial.print(round(successPercent));
    //     Serial.print("%");
    //   #endif
    //   Serial.println();
}
