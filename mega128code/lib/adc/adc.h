#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>



#define VREF        2.56    //Vref voltage for ADC

void enable_adc(int pin);
int read_adc(int pin);
double get_voltage(int adc);

