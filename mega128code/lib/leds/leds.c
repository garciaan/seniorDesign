#include "leds.h"

void init_leds(){
    DDRE |= 0xFF;
    PORTE = 0;
    set_16bitPWM3();
    set_rgb(WHITE);
}

void set_rgb(int red, int green, int blue){
    float temp = 0;
    if (red < 0)
        red = 0;
    if (red > 255)
        red = 255;
    if (green < 0)
        green = 0;
    if (green > 255)
        green = 255;
    if (blue < 0)
        blue = 0;
    if (blue > 255)
        blue = 255;
    temp = (255 - red)/(float)255;
    //OCR3A = temp*ICR3;
    OCR3A = 32000;
    temp = (255 - green)/(float)255;
    //OCR3B = temp*ICR3;
    OCR3B = 32000;
    temp = (255 - blue)/(float)255;
    //OCR3C = temp*ICR3;
    OCR3C = 32000;

}

void set_16bitPWM3(){

    TCCR3A |= (1 << COM3A1); //non-inverting
    TCCR3A |= (1 << COM3B1); //non-inverting
    TCCR3A |= (1 << COM3C1); //non-inverting

    //Fast PWM w/ TOP ICR1
    TCCR3A |= (1 << WGM31); 
    TCCR3B |= (1 << WGM33) | (1 << WGM32);
    
    switch (LED_PRESCALER){
        case 1:
            TCCR3B |= (1 << CS30); //244.140625 Hz
            break;
        case 8:
            TCCR3B |= (1 << CS31); //30.517578 Hz
            break;
        default:                    //prescaler 1
            TCCR3B |= (1 << CS30); //244.140625 Hz
            break;
    }
    

    TIM16_WriteTCNT3(1);
    ICR3 = (unsigned int) 65535;
    _delay_ms(100);
}

void TIM16_WriteTCNT3( unsigned int i ) {
    unsigned char sreg;
    /* Save global interrupt flag */ 
    sreg = SREG;
    /* Disable interrupts */ 
    cli();
    /* Set TCNTn to i */
    TCNT3 = i;
    sei();
    /* Restore global interrupt flag */ 
    SREG = sreg;
}
