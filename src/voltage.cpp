#include "voltage.hpp"
#include <util/atomic.h>

#define VOLTAGE_PIN A0

#define averagingLoopsLog2 1

void initVoltage()
{
    ADMUX = _BV(REFS1) | _BV(REFS0) |_BV(MUX2) | _BV(MUX1) | _BV(MUX0); //2.56V reference, A0 (ADC7)
    ADCSRB = 0; //free-running mode
    SET_BIT(DIDR0,ADC7D);//disable digital input on this pin
    ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADATE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); //enable adc, start capture, divide clock by 128
}

uint16_t readVoltage(){
    static uint16_t adc_accum;
    static uint8_t accum_cnt;
    static uint16_t filt_val;
    uint16_t tmp_adc;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
        tmp_adc = ADC;
    }
    adc_accum += tmp_adc;
    if ((++accum_cnt) == 1 << averagingLoopsLog2)
    {
        accum_cnt = 0;
        filt_val = adc_accum >> averagingLoopsLog2;
        adc_accum = 0;
    }
    return filt_val;
}