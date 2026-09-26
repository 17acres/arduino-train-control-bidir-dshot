#include "Arduino.h"
#include "Dshot.h"
#include "arduino_dshot.hpp"
#include <util/atomic.h>
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
const FREQUENCY frequency = F500;

// Always inverted (bidir dshot)

// DSHOT Output pin
const uint8_t pinDshot = 8;

/**
 * If debug mode is enabled, more information is printed to the serial console:
 * - Percentage of packages successfully received (CRC checksums match)
 * - Information on startup
 *
 * With debug disabled output will look like so:
 * --: 65408
 * OK: 65408
 *
 * With debug enabled output will look like so:
 * OK: 13696us 96.52%
 * --: 65408us 96.43%
 * OK: 13696us 96.43%
 * OK: 22400us 96.44%
 */
#define debug true

#define MIN_THR 48u
#define NUM_VALUES_PER_DIR 1000u //reverse commands are this much above fwd commands

/* Initialization */
uint32_t DSHOT_RESPONSE = 0;
uint32_t dshotResponseLast = 0;
uint16_t mappedLast = 0;

// Buffer for counting duration between falling and rising edges
const uint8_t buffSize = 20;
uint16_t counter[buffSize];

// Statistics for success rate
uint16_t receivedPackets = 0;
uint16_t successPackets = 0;

bool FRAME_COMPLETE = false;
bool hasEsc = false;


// Duration LUT - considerably faster than division
const uint8_t duration[] = {
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

Dshot dshot = new Dshot(true);
volatile uint16_t frame = dshot.buildFrame(0, 0);

uint32_t lastPeriodTime = 0;

#define DELAY_CYCLES(n) __builtin_avr_delay_cycles(n)

void sendDshot300Frame();
void sendInvertedDshot300Bit(uint8_t bit);
void readTelemetryResponse();
void printResponse();

void readTelemetryResponse() {
  // Set to Input in order to process the response - this will be at 3.3V level
  //I suppose we need to wait long enough anyway so why not use the slow Arduino version
  pinMode(pinDshot, INPUT_PULLUP);

  // Delay around 26us
  DELAY_CYCLES(410);

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

  // Set all 21 possible bits to one and flip the once that should be zero
  DSHOT_RESPONSE = 0x001FFFFF;
  unsigned long bitValue = 0x00;
  uint8_t bitCount = 0;
  for(uint8_t i = 1; i < buffSize; i += 1) {
    // We are done once the first intereval has a 0 value.
    if(counter[i] == 0) {
      break;
    }

    bitValue ^= 0x01; // Toggle bit value - always start with 0
    counter[i] = duration[counter[i]];
    for(uint8_t j = 0; j < counter[i]; j += 1) {
      DSHOT_RESPONSE ^= (bitValue << (20 - bitCount++));
    }
  }

  // Decode GCR 21 -> 20 bit (since the 21st bit is definetly a 0)
  DSHOT_RESPONSE ^= (DSHOT_RESPONSE >> 1);
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
void sendDshot300Frame() {
  uint16_t temp = frame;
  uint8_t offset = 0;
  do {
    sendInvertedDshot300Bit((temp & 0x8000) >> 15);
    temp <<= 1;
  } while(++offset < 0x10);
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
void sendInvertedDshot300Bit(uint8_t bit) {
  if(bit) {
    PORTB = B00000000;
    //DELAY_CYCLES(40);
    DELAY_CYCLES(37);
    PORTB = B00000001;
    //DELAY_CYCLES(13);
    DELAY_CYCLES(7);
  } else {
    PORTB = B00000000;
    //DELAY_CYCLES(20);
    DELAY_CYCLES(16);
    PORTB = B00000001;
    //DELAY_CYCLES(33);
    DELAY_CYCLES(25);
  }
}

void setupTimer() {
  cli();

  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;

  switch(frequency) {
    case F500: {
      // 500 Hz (16000000/((124 + 1) * 256))
      OCR2A = 124;
      TCCR2B |= 0b00000110; // Prescaler 256
    } break;

    case F1k: {
      // 1000 Hz (16000000/((124 + 1) * 128))
      OCR2A = 124;
      TCCR2B |= 0b00000101; // Prescaler 128
    } break;

    case F2k: {
      // 2000 Hz (16000000/((124 + 1) * 64))
      OCR2A = 124;
      TCCR2B |= 0b00000100; // Prescaler 64
    } break;

    case F4k: {
      // 4000 Hz (16000000/( (124 + 1) * 32))
      OCR2A = 124;
      TCCR2B |= 0b00000011; // Prescaler 32
    } break;

    default: {
      // 8000 Hz (16000000/( (249 + 1) * 8))
      OCR2A = 249;
      TCCR2B |= 0b00000010; // Prescaler 8
    } break;
  }

  TCCR2A |= 0b00001010; // CTC mode - count to OCR2A
  TIMSK2 = 0b00000010; // Enable INT on compare match A

  sei();
}

ISR(TIMER2_COMPA_vect) {
    sendDshot300Frame();
    readTelemetryResponse();
    FRAME_COMPLETE = true;
}

void dshotSetup() {
  pinMode(pinDshot, OUTPUT);

  // Set the default signal Level
  PORTB = B00000001;

  setupTimer();
}

void stopMotor()
{
    uint16_t tmp_frame = dshot.buildFrame(0, 0);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
        frame = tmp_frame;
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
        frame = tmp_frame;
    }
}

void processTelemetryResponse() {
  if(FRAME_COMPLETE) {
    uint32_t dshotResponse_local;

    FRAME_COMPLETE  = false;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
        dshotResponse_local = DSHOT_RESPONSE;
    }

    uint16_t mapped = dshot.mapTo16Bit(dshotResponse_local);
    uint8_t crc = mapped & 0x0F;
    uint16_t value = mapped >> 4;
    uint8_t crcExpected = dshot.calculateCrc(value);

    //Serial.println(mapped, BIN);
    Serial.print(value, BIN);
    Serial.print(" ");
    Serial.println(crc, BIN);

    // Wait for a first valid response
    if(!hasEsc) {
      if(crc == crcExpected) {
        hasEsc = true;
      }

      return;
    }

    // Calculate success rate - percentage of packeges on which CRC matched the value
    receivedPackets++;
    if(crc == crcExpected) {
      successPackets++;
    }

    // Reset packet count if overflows
    if(!receivedPackets) {
      successPackets = 0;
    }

    if((DSHOT_RESPONSE != dshotResponseLast) || !debug) {
      dshotResponseLast = DSHOT_RESPONSE;

      // DShot Frame: EEEMMMMMMMMM
      uint32_t periodBase = value & 0b0000000111111111;
      uint8_t periodShift = value >> 9 & 0b00000111;
      uint32_t periodTime =  periodBase << periodShift;

      if(crc == crcExpected) {
        Serial.print("OK: ");
      } else {
        Serial.print("--: ");
      }

      #if debug
        float successPercent = (successPackets * 1.0 / receivedPackets * 1.0) * 100;
      #endif

      Serial.print(periodTime);
      #if debug
        Serial.print("us ");
        Serial.print(round(successPercent));
        Serial.print("%");
      #endif
      Serial.println();
    }
  }
}

void dshotLoop() {
  printResponse();
}
