void setupTimer()
{
    cli();

    TCCR3B = 0;
    TCCR3A = 0;
    TIMSK3 = 0;
    TCNT3 = 0;

    OCR3A = F_CPU / (8 * main_loop_freq) - 1;
    TIFR3 = _BV(OCF3A); // clear flag
    TIMSK3 = _BV(OCIE3A);
    TCCR3B = _BV(WGM32) | _BV(CS31); // CTC mode, prescaler 8

    sei();
}