#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
#define P1 0x01
#define P2 0x02
#define P3 0x04
#define P4 0x08
#define P5 0x10
#define P6 0x20
#define P7 0x40
#define P8 0x80

#define PRESCALER 8


#define STOP 1500   //1500 usec pulse for calibration

#define MAX_SPEED   3800  //in timer steps -- 1900usec @ 30.517578Hz
#define MIN_SPEED   2200  //in timer steps -- 1100usec @ 30.517578Hz
#define SPEED_RANGE MAX_SPEED - MIN_SPEED
#define STOP_SPEED  SPEED_RANGE/2 + MIN_SPEED

#define MIN_INPUT   0
#define MAX_INPUT   100
#define SATURATE_DIFFERENCE 30 //takes this value away from the max and min

#define STEP        (double)(MAX_INPUT - MIN_INPUT)/((double)(MAX_SPEED - MIN_SPEED))

#define MAX_STRING_SIZE 4

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "ascii_special.h"



void USART_Init( unsigned int ubrr );
void USART_Transmit(unsigned char data );
void USART_send_string(unsigned char *data);
unsigned char USART_Receive(void);
void USART_Receive_String(unsigned char *str);
void USART0_Init( unsigned int ubrr );
void USART0_Transmit(unsigned char data );
void USART0_send_string(unsigned char *data);
unsigned char USART0_Receive(void);
void USART0_Receive_String(unsigned char *str);
void home_line2(void);
void string2lcd(unsigned char *lcd_str);
void strobe_lcd(void);
void clear_display(void);
void home_line2(void);
void char2lcd(unsigned char a_char);
void spi_init(void);
void lcd_init(void);
void trigger(unsigned int pin);
double get_distance(unsigned int pin);
double print_distance(unsigned int pin);
void blink(int led, int speed);
void forward(int speed);
void reverse(int speed);
void stop();
void TIM16_WriteTCNT1( unsigned int i );
void move(float left, float right, float z);
void set_16bitPWM1();
void init_esc();

uint8_t temp, read_byte;


int main(void){
    DDRB = 0xFF;
    PORTB = 0x00;
    DDRD = 0x00 | (1 << 2);

    /*
    [0] == left motor power
    [1] == right motor power
    [2] == z motor power
    [3] == reserved for string terminator
    */
    unsigned char buffer[MAX_STRING_SIZE]; 
    int i;
    for (i = 0; i < 4; ++i){
        buffer[i] = ' ';
    }

    spi_init();
    lcd_init();

    //Wait for S1 to be pressed before doing anything, remove this eventually
    string2lcd((unsigned char *)"Press S1");
	while (((PIND) & (1 << 0)));
	clear_display();

	init_esc();
    set_16bitPWM1();
   
    USART_Init(MYUBRR); 
    USART0_Init(MYUBRR);
    _delay_ms(100);
    unsigned char c;
    USART_send_string((unsigned char*)"Connected\n");
    while(1){

        c = USART_Receive();
        clear_display();
        char2lcd(c);
        //USART_Transmit(c);

        _delay_ms(20);
	}

    return 0;
}

void init_esc(){
    PORTB |= (1 << 4);
    _delay_us(STOP);
    PORTB &= ~(1 << 4);
    _delay_ms(1000);
}

void set_16bitPWM1(){
    //16-bit fast pwm non-inverting on PB5
    TCCR1A |= (1 << COM1A1); //inverting

    //16-bit fast pwm non-inverting on PB6
    TCCR1A |= (1 << COM1B1); //non-inverting

    //16-bit fast pwm non-inverting on PB7
    TCCR1A |= (1 << COM1C1); //non-inverting

    //Fast PWM w/ TOP ICR1
    TCCR1A |= (1 << WGM11); 
    TCCR1B |= (1 << WGM13) | (1 << WGM12);
    
    switch (PRESCALER){
        case 1:
            TCCR1B |= (1 << CS10); //244.140625 Hz
            break;
        case 8:
            TCCR1B |= (1 << CS11); //30.517578 Hz
            break;
        default:                    //prescaler 1
            TCCR1B |= (1 << CS10); //244.140625 Hz
            break;
    }
    

    TIM16_WriteTCNT1(1);
    ICR1 = (unsigned int) 65535;
    _delay_ms(100);
}

void TIM16_WriteTCNT1( unsigned int i ) {
    unsigned char sreg;
    /* Save global interrupt flag */ 
    sreg = SREG;
    /* Disable interrupts */ 
    cli();
    /* Set TCNTn to i */
    TCNT1 = i;
    sei();
    /* Restore global interrupt flag */ 
    SREG = sreg;
}

/*
    OCR1A = Left motor PB5
    OCR1B = Right motor PB6
    maybe OCR1C = Z motor? PB7

    NOTE: Does not activate AfroESC with Simonk firmware given less than 6
*/
void move(float left, float right, float z){
    
    if (left < (MIN_INPUT + SATURATE_DIFFERENCE)){
        left = MIN_INPUT + SATURATE_DIFFERENCE;
    }
    if (left > (MAX_INPUT - SATURATE_DIFFERENCE)){
        left = MAX_INPUT - SATURATE_DIFFERENCE;
    }
    if (right < (MIN_INPUT + SATURATE_DIFFERENCE)){
        right = MIN_INPUT + SATURATE_DIFFERENCE;
    }
    if (right > (MAX_INPUT - SATURATE_DIFFERENCE)){
        right = MAX_INPUT - SATURATE_DIFFERENCE;
    }
    if (z < (MIN_INPUT + SATURATE_DIFFERENCE)){
        z = MIN_INPUT + SATURATE_DIFFERENCE;
    }
    if (z > (MAX_INPUT - SATURATE_DIFFERENCE)){
        z = MAX_INPUT - SATURATE_DIFFERENCE;
    }

    unsigned int left_speed, right_speed, z_speed;
    left_speed = (unsigned int)((left - MIN_INPUT)/((double)STEP) + MIN_SPEED);
    right_speed = (unsigned int)((right - MIN_INPUT)/((double)STEP) + MIN_SPEED);
    //This needs to be redone to account for a different controller
    z_speed = (unsigned int)(ICR1 * (z/((double)MAX_INPUT))); 
    OCR1A = left_speed;
    OCR1B = right_speed;
    OCR1C = z_speed;
    
	unsigned char buffer[16];
    clear_display();
	string2lcd((unsigned char *)utoa(left_speed,(char *)buffer,10));
	home_line2();
	string2lcd((unsigned char *)utoa(right_speed,(char *)buffer,10));
    

}


void USART_Init( unsigned int ubrr ) {
    /* Set baud rate */
    UBRR1H = (unsigned char)(ubrr>>8);
    UBRR1L = (unsigned char)ubrr;
    /* Enable receiver and transmitter */ 
    UCSR1B = (1<<RXEN1)|(1<<TXEN1);
    /* Set frame format: 8data, 1stop bit */ 
    UCSR1C = (1 << UCSZ10) | (1 << UCSZ11);
    //UCSR1B |= (1 << UCSZ12);

    _delay_ms(100);
}
void USART_Transmit(unsigned char data ) {
    /* Wait for empty transmit buffer */ 
    while ( !( UCSR1A & (1<<UDRE1)) );
    /* Put data into buffer, sends the data */ 
    UDR1 = data;
}

void USART_send_string(unsigned char *data){
    int i = 0;
    while (data[i] != '\0'){
        USART_Transmit(data[i]);
        ++i;
    }
}

unsigned char USART_Receive(void){
    uint16_t timeout = 50000;
    //unsigned char buffer[16];
    /* Wait for data to be received or for timeout*/ 
    while (timeout) {
        if((UCSR1A & (1<<RXC1))){
            /* Get and return received data from buffer */ 
            return UDR1;
       }
        //clear_display();
        //string2lcd((unsigned char *)utoa((unsigned int)timeout,buffer,10));
        _delay_us(30);
        --timeout;
    }
    //while (!(UCSR1A & (1 << RXC1))){
    //}

    return 255;
}

void USART_Receive_String(unsigned char *str){
    int i = 0;
    unsigned char c;

    while ((c = (unsigned char)USART_Receive()) != END_STRING){ //END_STRING == ~ or 0x7E
        if (c == 255){
            str[0] = 50;
            str[1] = 50;
            str[2] = 50;
            str[3] = '\0';
            return;
        }
        str[i] = c;
        //char2lcd(c);
        //string2lcd(str);
        ++i;
        if (i >= MAX_STRING_SIZE){
            str[MAX_STRING_SIZE - 1] = '\0';

            return;
        }
    }
    str[i] = '\0';
    //string2lcd(str);
}


void USART0_Init( unsigned int ubrr ) {
    /* Set baud rate */
    UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)ubrr;
    /* Enable receiver and transmitter */ 
    UCSR0B = (1<<RXEN0)|(1<<TXEN0);
    /* Set frame format: 8data, 1stop bit */ 
    UCSR0C = (3<<UCSZ00);
    _delay_ms(100);
}
void USART0_Transmit(unsigned char data ) {
    /* Wait for empty transmit buffer */ 
    while ( !( UCSR0A & (1<<UDRE0)) );
    /* Put data into buffer, sends the data */ 
    UDR0 = data;
}

void USART0_send_string(unsigned char *data){
    int i = 0;
    while (data[i] != '\0'){
        USART0_Transmit(data[i]);
        ++i;
    }
}

unsigned char USART0_Receive(void){
    uint16_t timeout = 50000;
    //unsigned char buffer[16];
    /* Wait for data to be received or for timeout*/ 
    while (1) {
        if((UCSR0A & (1<<RXC0))){
            /* Get and return received data from buffer */ 
            return UDR0;
        }
        //clear_display();
        //string2lcd((unsigned char *)utoa((unsigned int)timeout,buffer,10));
        //_delay_us(10);
        //--timeout;
    }
    return 255;
}

void USART0_Receive_String(unsigned char *str){
    int i = 0;
    unsigned char c;

    while ((c = (unsigned char)USART0_Receive()) != END_STRING){ //END_STRING == ~ or 0x7E
        if (c == 255){
            str[0] = 50;
            str[1] = 50;
            str[2] = 50;
            str[3] = '\0';
            return;
        }
        str[i] = c;
        //char2lcd(c);
        //string2lcd(str);
        ++i;
        if (i >= MAX_STRING_SIZE){
            str[MAX_STRING_SIZE - 1] = '\0';

            return;
        }
    }
    str[i] = '\0';
    //string2lcd(str);

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
    _delay_ms(2.6);   //obligatory waiting for slow LCD
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
void char2lcd(unsigned char a_char){
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
void string2lcd(unsigned char *lcd_str){
    int count;
    for (count=0; count<=(strlen((char*)lcd_str)-1); count++){
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
