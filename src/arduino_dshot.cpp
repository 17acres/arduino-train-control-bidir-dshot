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
 */
#define main_loop_freq 500

// Always inverted (bidir dshot)

// DSHOT Output pin
#define pinDshot 4 // PD4 is timer1 input capture
#define portPinDshot PD4
#define pinIsrTimer 6 //PD7 is timer2 ISR-running-indicator
#define portPinIsrTimer PD7

//Timer 1 for input capture of BDSHOT
//Timer 3 for loop timing
//Port D for interface

#define MIN_THR 48u
#define NUM_VALUES_PER_DIR 1000u //reverse commands are this much above fwd commands


Dshot dshot = new Dshot(true);


/* ISR Variables */
uint32_t v_DSHOT_RESPONSE = 0;
bool v_FRAME_COMPLETE = false;
volatile uint16_t v_FRAME = dshot.buildFrame(0, 0);

// Duration LUT - considerably faster than division
const uint8_t state_duration_to_bits_lut[] = {
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

void sendDshot300Frame();
void sendInvertedDshot300Bit(uint8_t bit);
void readTelemetryResponse();

//CALLED FROM ISR
void readTelemetryResponse() {
  // Buffer for counting duration between falling and rising edges
  // Do the calculation of DSHOT_RESPONSE here so it is easier to read atomically
  #define buffSize 20
  uint16_t counter[buffSize];
  
  // Set to Input in order to process the response - this will be at 3.3V level
  //I suppose we need to wait long enough anyway so why not use the slow Arduino version
  pinMode(pinDshot, INPUT_PULLUP);

  // Delay around 26us
  DELAY_CYCLES(300);

  register uint8_t ices1High = 0b01000000;
  register uint16_t prevVal = 0;
  register uint8_t tifr;
  register uint16_t *pCapDat;

  TCCR1A = 0b00000001; // Toggle OC1A on compare match
  TCCR1B = 0b00000010; // trigger on falling edge, prescaler 8, filter off

  // Limit to 70us - that should be enough to fetch the whole response
  // at 2MHz - scale factor 8 - 150 ticks seems to be a sweetspot.
  OCR1A = 150;
  TCNT1 = 0x00;

  TIFR1 = (1 << ICF1) | (1 << OCF1A) | (1 << TOV1); // clear all timer flags
  for(pCapDat = counter; pCapDat <= &counter[buffSize - 1];) {
    // wait for edge or overflow (output compare match)
    while(!(tifr = (TIFR1 & ((1 << ICF1) | (1 << OCF1A))))) {}

    uint16_t val = ICR1;

    // Break if counter overflows
    if(tifr & (1 << OCF1A)) {
      // Ignore overflow at the beginning of capture
      if(pCapDat != counter) {
        break;
      }
    }

    TCCR1B ^= ices1High; // toggle the trigger edge
    TIFR1 = (1 << ICF1) | (1 << OCF1A); // clear input capture and output compare flag bit

    *pCapDat = val - prevVal;

    prevVal = val;
    pCapDat++;
  }

  pinMode(pinDshot, OUTPUT);
  CLR_BIT(PORTD,portPinIsrTimer); //TODO move this to application code

  uint32_t temp_dshot_response;
  // Set all 21 possible bits to one and flip the once that should be zero
  temp_dshot_response = 0x001FFFFF;
  unsigned long bitValue = 0x00;
  uint8_t bitCount = 0;
  for(uint8_t i = 1; i < buffSize; i += 1) {
    // We are done once the first intereval has a 0 value.
    if(counter[i] == 0) {
      break;
    }

    bitValue ^= 0x01; // Toggle bit value - always start with 0
    counter[i] = state_duration_to_bits_lut[counter[i]];
    for(uint8_t j = 0; j < counter[i]; j += 1) {
      temp_dshot_response ^= (bitValue << (20 - bitCount++));
    }
  }

  // Decode GCR 21 -> 20 bit (since the 21st bit is definetly a 0)
  temp_dshot_response ^= (temp_dshot_response >> 1);

  v_DSHOT_RESPONSE = temp_dshot_response;
}

/**
 * CALLED FROM ISR
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
void sendDshot300Frame() {
  uint16_t temp = v_FRAME;
  uint8_t offset = 0;
  do {
    sendInvertedDshot300Bit((temp & 0x8000) >> 15);
    temp <<= 1;
  } while(++offset < 0x10);
}

/**
 * CALLED FROM ISR
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
void sendInvertedDshot300Bit(uint8_t bit) {
  if(bit) {
    CLR_BIT(PORTD,portPinDshot);
    //DELAY_CYCLES(40);
    DELAY_CYCLES(36);
    SET_BIT(PORTD,portPinDshot);
    //DELAY_CYCLES(13);
    DELAY_CYCLES(6);
  } else {
    CLR_BIT(PORTD,portPinDshot);
    //DELAY_CYCLES(20);
    DELAY_CYCLES(15);
    SET_BIT(PORTD,portPinDshot);
    //DELAY_CYCLES(33);
    DELAY_CYCLES(24);
  }
}

void setupTimer() {
  cli();

  TCCR3B = 0;
  TCCR3A = 0;
  TIMSK3 = 0;
  TCNT3 = 0;

  OCR3A = F_CPU/(8*main_loop_freq)-1;
  TIFR3 = _BV(OCF3A); //clear flag
  TIMSK3 = _BV(OCIE3A);
  TCCR3B = _BV(WGM32) | _BV(CS31); // CTC mode, prescaler 8

  sei();
}

ISR(TIMER3_COMPA_vect) {
    SET_BIT(PORTD,portPinIsrTimer);
    sendDshot300Frame();
    readTelemetryResponse();
    v_FRAME_COMPLETE = true;
    CLR_BIT(PORTD,portPinIsrTimer);
}

void dshotSetup() {
  pinMode(pinDshot, OUTPUT);
  pinMode(pinIsrTimer, OUTPUT);

  // Set the default signal Level
  SET_BIT(PORTD,portPinDshot);
  CLR_BIT(PORTD,portPinIsrTimer);

  setupTimer();
}

void stopMotor()
{
    uint16_t tmp_frame = dshot.buildFrame(0, 0);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
        v_FRAME = tmp_frame;
    }
}

/* Has atomic protections. Input range of 0 to 999, will saturate */
void requestThrottle(uint16_t throttle, bool is_fwd)
{
    if(throttle >= NUM_VALUES_PER_DIR) {
      throttle = NUM_VALUES_PER_DIR-1;
    }
    throttle+=MIN_THR;

    if(!is_fwd){
      throttle+=NUM_VALUES_PER_DIR;
    }
    uint16_t tmp_frame = dshot.buildFrame(throttle, 0);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
        v_FRAME = tmp_frame;
    }
}

bool processTelemetryResponse(uint16_t *commutation_period) {
    // Statistics for success rate
    static uint16_t ls_receivedPackets = 0;
    static uint16_t ls_successPackets = 0;

    uint32_t dshotResponse_local;

    v_FRAME_COMPLETE  = false;
    ls_receivedPackets++;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
        dshotResponse_local = v_DSHOT_RESPONSE;
    }

    uint16_t mapped = dshot.mapTo16Bit(dshotResponse_local);
    uint8_t crc = mapped & 0x0F;
    uint16_t value = mapped >> 4;
    uint8_t crcExpected = dshot.calculateCrc(value);

    //Serial.println(mapped, BIN);
    // Serial.print(value, BIN);
    // Serial.print(" ");
    // Serial.println(crc, BIN);

    if(crc != crcExpected){
      return false;
    }

    ls_successPackets++;

    // Reset packet count if overflows
    if(!ls_receivedPackets) {
      ls_successPackets = 0;
    }



    // DShot Frame: EEEMMMMMMMMM
    uint32_t periodBase = value & 0b0000000111111111;
    uint8_t periodShift = value >> 9 & 0b00000111;
    *commutation_period =  periodBase << periodShift;

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
