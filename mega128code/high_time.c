
#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#define FOSC 1843200// Clock Speed 
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1
#define P1 0x01
#define P2 0x02
#define P3 0x04
#define P4 0x08
#define P5 0x10
#define P6 0x20
#define P7 0x40
#define P8 0x80
#define TIMER3_FREQ F_CPU   //change this per the scaler equation

#define TRIG 0x01
#define ECHO 0x02
#define SONAR1 (1<<0)
#define SONAR2 (1<<2)
#define NUM_SONARS  1
#define SOUND_SPEED 340	//Meters per second

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>

#include "./I2C-master-lib/i2c_master.h"

void USART_Init( unsigned int ubrr );
void USART_Transmit( unsigned char data );
void home_line2(void);
void string2lcd(char *lcd_str);
void strobe_lcd(void);
void clear_display(void);
void home_line2(void);
void char2lcd(char a_char);
void string2lcd(char *lcd_str);
void spi_init(void);
void lcd_init(void);
void trigger(unsigned int pin);
double get_distance(unsigned int pin);
double print_distance(unsigned int pin);
void blink(int led, int speed);
void enable_input_capture();
int is_rising();
void set_to_rising();
void set_to_falling();
void enable_input_capture();
void disable_input_capture();
void clear_input_flag();
void setup_input_capture();


uint8_t temp, read_byte;

//Global Variables for Input Capture
volatile unsigned int start_time = 0;
volatile unsigned int end_time = 0;
volatile unsigned int high_time = 0;
volatile unsigned int capture_complete = 0;


int main(void){
	DDRB = 0xFF;
    PORTB = 0x00;
    DDRD = 0x00;
    DDRE = 0b01010100;    //1,3,5 is output, the rest input
    PORTE = 0x00;   //set low
	i2c_init();

	int count = 0;
    int direction = 0;
	char buffer[16];
    double distance = 0;
    int sonars[NUM_SONARS];
    int i;
    for (i = 0; i < NUM_SONARS; ++i){
        sonars[i] = (1<<(i * 2));
    }
    int limit = 20;
    int step = 5;
    spi_init();
    lcd_init();
    
    double temp;


    //8 bit phase correct pwm
    TCCR0 |= (1 << WGM00);  
    TCCR0 &= ~(1 << WGM01);
    TCCR0 |= (1 << COM01); //Clear OC0 on compare match when up-counting. Set OC0 on compare match when downcounting.
    //Prescaler 256
    TCCR0 |= (1 << CS02) | (1 << CS01);
    TCCR0 &= ~(1 << CS00);

    OCR0 = 0;

    clear_display();
    string2lcd("Starting Program");


    _delay_ms(500);
    while(1){
        clear_display();
        
        for (i = 0; i < NUM_SONARS; ++i){
            distance = print_distance(sonars[i]);
            if (distance <= limit){
                PORTB |= (1 << 6);
            }
            else {
                PORTB &= ~(1 << 6);
            }
            if (distance > limit && distance <= limit + 4 * step){

                temp = distance - limit;
                if (temp < 0){
                    temp = 0;
                }
                temp *= 255.0/(4 * step);
                temp = 255.0 - temp;
                OCR0 = temp;
                home_line2();
                dtostrf(temp,1,6,buffer);
                //string2lcd(buffer);
            }
            else {
                OCR0 = 0;
                home_line2();
                dtostrf(0.0,1,6,buffer);
                //string2lcd(buffer);

            }

        }

        _delay_ms(60);
        
	}

	return 0;
}

//Input capture interrupt
ISR(TIMER3_CAPT_vect){
    if (is_rising()){
        set_to_falling();
        start_time = ICR3;
        clear_input_flag();
    }
    else {
        end_time = ICR3;
        disable_input_capture();
        set_to_rising();
        high_time = end_time - start_time;
        capture_complete = 1;
    }
}
int is_rising(){
    if (TCCR3B & (1 << ICES3)){
        return 1;
    }
    return 0;
}
void set_to_rising(){
    TCCR3B |= (1 << ICES3);
}
void set_to_falling(){
    TCCR3B &= ~(1 << ICES3);
}
void enable_input_capture(){
    set_to_rising();
    ETIMSK |= (1 << TICIE3);
}
void disable_input_capture(){
    ETIMSK &= ~(1 << TICIE3);
}
void clear_input_flag(){
    ETIFR |= (1 << ICF3);
}
void setup_input_capture(){
    disable_input_capture();
    //reset the registers
    TCCR3A = 0;
    TCCR3B = 0;
    //Normal Port Operation, OC3B Disconnected
    TCCR3A &= ~((1 << COM3B1) | (1 << COM3B0));
    //clear the reserved bit
    TCCR3B &= ~(1 << 5);
    //enable input capture noise canceller
    TCCR3B |= (1 << ICNC3); 
    //Set ICR3 to TOP, noted as Fast PWM Mode?
    TCCR3A &= ~(1 << WGM30);
    TCCR3A |= (1 << WGM31);
    TCCR3B |= ((1 << WGM32) | (1 << WGM33));
    //CLKio/1 (no prescaling)
    TCCR3B |= (1 << CS30);
    TCCR3B &= ~(1 << CS31);
    TCCR3B &= ~(1 << CS32);
    set_to_rising();

}

void blink(int led, int speed){
    if (speed < 0){
        speed = 0;
    }
    if (speed > 100){
        speed = 100;
    }
    int i;
    PORTB |= (1 << led);
    for (i = 0; i < (121-speed); ++i){
        _delay_ms(1);
    }
    PORTB &= ~(1 << led);

}
double print_distance(unsigned int pin){
    char str[16];
    double distance = 0;
    distance = get_distance(pin);
    dtostrf(distance,1,6,str);
    string2lcd(str);
    return distance;
}

void trigger(unsigned int pin){
    PORTE |= pin;
    _delay_us(15);
    PORTE &= ~pin;
}

double get_distance(unsigned int pin){
    double distance = 0;
	unsigned long count = 0;
    double time_sec;
	trigger(pin);
    
    while ((PINE & (pin << 1)) == 0); //while pinc port 1 is low, aka wait for echo to raise
    while (1){
        if ((PINE & (pin << 1)) != (pin << 1)){ //wait for pinc port 1 to go back to low
            break;
        }
        if (count == 255){
            break;
        }
        _delay_us(50);
        ++count;
    }
	//time = count * (0.001); //convert to seconds 	
    //distance = time * SOUND_SPEED / 2;
    distance = (double)count * 40;
    distance /= 58;

    
    /*
    enable_input_capture();
    while (!capture_complete);
    time_sec = high_time / (double)TIMER3_FREQ;
    distance = time_sec * SOUND_SPEED / 2; // meters
    */
    return distance;
}


void USART_Init( unsigned int ubrr ) {
    /* Set baud rate */
    UBRR1H = (unsigned char)(ubrr>>8);
    UBRR1L = (unsigned char)ubrr;
    /* Enable receiver and transmitter */ 
    UCSR1B = (1<<RXEN1)|(1<<TXEN1);
    /* Set frame format: 8data, 2stop bit */ 
    UCSR1C = (1<<USBS1)|(3<<UCSZ01);
}
void USART_Transmit( unsigned char data ) {
    /* Wait for empty transmit buffer */ 
    while ( !( UCSR1A & (1<<UDRE1)) );
    /* Put data into buffer, sends the data */ 
    UDR1 = data;
}
//twiddles bit 3, PORTF creating the enable signal for the LCD
void strobe_lcd(void){
    PORTF |= 0x08;
    PORTF &= ~0x08;
}

void clear_display(void){
    while (!(SPSR & 0x80)) {}   // Wait for SPI transfer to complete
    SPDR = 0x00;    //command, not data
    while (!(SPSR & 0x80)) {}   // Wait for SPI transfer to complete
    SPDR = 0x01;    //clear display command
    while (!(SPSR & 0x80)) {}   // Wait for SPI transfer to complete
    strobe_lcd();   //strobe the LCD enable pin
    _delay_ms(1.6);   //obligatory waiting for slow LCD
}

void home_line2(void){
    SPDR = 0x00;    //command, not data
    while (!(SPSR & 0x80)) {}   // Wait for SPI transfer to complete
    SPDR = 0xC0;   // cursor go home on line 2
    while (!(SPSR & 0x80)) {}   // Wait for SPI transfer to complete
    strobe_lcd(); 
    _delay_us(37);
}

//sends a char to the LCD
void char2lcd(char a_char){
    //sends a char to the LCD
    //usage: char2lcd('H');  // send an H to the LCD
    SPDR = 0x01;   //set SR for data xfer with LSB=1
    while (!(SPSR & 0x80)) {}   // Wait for SPI transfer to complete
    SPDR = a_char; //send the char to the SPI port
    while (!(SPSR & 0x80)) {}   // Wait for SPI transfer to complete
    strobe_lcd();  //toggle the enable bit
    _delay_us(37);
}

//sends a string in FLASH to LCD
void string2lcd(char *lcd_str){
    int count;
    for (count=0; count<=(strlen(lcd_str)-1); count++){
        while (!(SPSR & 0x80)) {}   // Wait for SPI transfer to complete
        SPDR = 0x01; //set SR for data
        while (!(SPSR & 0x80)) {}   // Wait for SPI transfer to complete
        SPDR = lcd_str[count]; 
        while (!(SPSR & 0x80)) {}   // Wait for SPI transfer to complete
        strobe_lcd();
        _delay_us(37);  // Max delay for this function is 48us
    }
}   

/* Run this code before attempting to write to the LCD.*/
void spi_init(void){
    DDRF |= 0x08;  //port F bit 3 is enable for LCD
    PORTB |= 0x00; //port B initalization for SPI
    DDRB |= 0x07;  //Turn on SS, MOSI, SCLK 
    //Master mode, Clock=clk/2, Cycle half phase, Low polarity, MSB first  
    SPCR = 0x50;
    SPSR = 0x01;
}

//initialize the LCD to receive data
void lcd_init(void){
    int i;
    //initalize the LCD to receive data
    _delay_ms(15);   
    for(i=0; i<=2; i++){ //do funky initalize sequence 3 times
        SPDR = 0x00;
        while (!(SPSR & 0x80)) {}   // Wait for SPI transfer to complete
        SPDR = 0x30;
        while (!(SPSR & 0x80)) {}   // Wait for SPI transfer to complete
        strobe_lcd();
        _delay_us(37);
    }

    SPDR = 0x00;
    while (!(SPSR & 0x80)) {}   // Wait for SPI transfer to complete
    SPDR = 0x38;
    while (!(SPSR & 0x80)) {}   // Wait for SPI transfer to complete
    strobe_lcd();
    _delay_us(37);

    SPDR = 0x00;
    while (!(SPSR & 0x80)) {}   // Wait for SPI transfer to complete
    SPDR = 0x08;
    while (!(SPSR & 0x80)) {}   // Wait for SPI transfer to complete
    strobe_lcd();
    _delay_us(37);

    SPDR = 0x00;
    while (!(SPSR & 0x80)) {}   // Wait for SPI transfer to complete
    SPDR = 0x01;
    while (!(SPSR & 0x80)) {}   // Wait for SPI transfer to complete
    strobe_lcd();
    _delay_ms(1.6);

    SPDR = 0x00;
    while (!(SPSR & 0x80)) {}   // Wait for SPI transfer to complete
    SPDR = 0x06;
    while (!(SPSR & 0x80)) {}   // Wait for SPI transfer to complete
    strobe_lcd();
    _delay_us(37);

    SPDR = 0x00;
    while (!(SPSR & 0x80)) {}   // Wait for SPI transfer to complete
    SPDR = 0x0E;
    while (!(SPSR & 0x80)) {}   // Wait for SPI transfer to complete
    strobe_lcd();
    _delay_us(37);
}
