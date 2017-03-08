#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>

/*************************
*   Distance Sensing Functions
*   To be used with the laser and photoresistor
**************************/
double get_distance(unsigned int pin);
double print_distance(unsigned int pin);