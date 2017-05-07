#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "../../lib/uart/ascii_special.h"
#include "../../lib/adc/adc.h"
#include "../../lib/motors/motors.h"
#include "../../lib/uart/uart.h"
#include "../../lib/magnometer/magnometer.h"
#include "../../lib/lasersensor/lasersensor.h" //NOT YET IMPLEMENTED
#include "../../lib/lcd/lcd.h"
#include "../../lib/pressuresensor/pressuresensor.h"
#include "../../lib/leds/leds.h"


//Some easy to read defines
#define PATH1 1
#define PATH2 2
#define PATH3 3
#define MOVE_SPEED 50
#define TURN_SPEED 15
//#define STABLE_Z 50
volatile int STABLE_Z = 50;
#define MAX_DEPTH 12 //in feet, used for LED response


volatile int data_timer_counter;
#define TIMER0_DIVIDER 15
/************************
*   High level movement functions
*************************/
void path1();
void path2();
void path3();
void turn(int degrees);
void forward();
void reverse();
void dive(float depth);
void depth_to_leds();

void init_data_timer();
void enable_bumpers();

char buffer[10];
volatile int object_detected = 0;


int main(){
    DDRB = 0xFF;
    PORTB = 0;
    STABLE_Z = 50;

    enable_bumpers();

   
    
    PORTB = 0b01010101;
    _delay_ms(1000);
    PORTB = 0;
    sei();
    while (1){
        PORTB = 0b10101010;
        _delay_ms(1000);
        PORTB = 0;
        _delay_ms(1000);

    }

    return 0;
}

ISR(INT6_vect){  //Left bumper on PE6
    cli();
    PORTB = 255;
    _delay_ms(500);
    sei();


}

ISR(INT7_vect){  //Right bumper on PE7
    
    cli();
    PORTB = 0b00001111;
    _delay_ms(500);
    sei();
}


/*********
* Need to do rising edge
**********/
void enable_bumpers(){
    //Set pins as inputs
    DDRE &= ~(1 << 6);
    DDRE &= ~(1 << 7);
    
    //Enable internal pullups
    PORTE |= (1 << 6);
    PORTE |= (1 << 7);
    //PORTE = 0;

    //Set both interrupt 6 and 7 to rising edge
    EICRB |= (1 << ISC61) | (1 << ISC60);
    EICRB |= (1 << ISC71) | (1 << ISC70);

    //enable the interrupts
    EIMSK |= (1 << INT6) | (1 << INT7);
}

