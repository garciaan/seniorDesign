#ifndef F_CPU
#define F_CPU 16000000UL
#endif


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define LED_PRESCALER 1

#define RED     255,0,0
#define GREEN   0,255,0
#define BLUE    0,0,255
#define PURPLE  255,0,255
#define TEAL    0,255,255
#define YELLOW  255,255,0
#define WHITE   255,255,255
#define OFF     0,0,0

/*************************
*   PWM and LED Strip Functions
**************************/
void TIM16_WriteTCNT3( unsigned int i );
void set_rgb(int red, int green, int blue);
void set_16bitPWM3();
void init_leds();
