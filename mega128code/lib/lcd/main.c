#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "ascii_special.h"

//Prescaler for PWM signals from the atmega128 manual
#define PRESCALER 8

/**********************
*   T100 Thruster Information
*   Left Motor Signal:  PB5
*   Right Motor Signal: PB6
*   Z Motor Signal:     PB7
***********************/
#define STOP 1500   //1500 usec pulse for calibration
#define MAX_SPEED   3800  //in timer steps -- 1900usec @ 30.517578Hz
#define MIN_SPEED   2200  //in timer steps -- 1100usec @ 30.517578Hz
#define SPEED_RANGE MAX_SPEED - MIN_SPEED
#define STOP_SPEED  SPEED_RANGE/2 + MIN_SPEED

/***********************
*   Remote Controller Information
*   These are the saturation values. 
*   Saturation difference will make it so the values calculated cannot 
*       go below MIN_INPUT + SATURATION_DIFFERENCE
*       and above MAX_INPUT - SATURATION_DIFFERENCE
*   This will essentially limit motor power by SATURATION_DIFFERENCE*2 percent
*************************/
#define MIN_INPUT   0
#define MAX_INPUT   100
#define SATURATE_DIFFERENCE 30 //takes this value away from the max and min

//Number of OCR1x steps per remote control step
#define STEP        (double)(MAX_INPUT - MIN_INPUT)/((double)(MAX_SPEED - MIN_SPEED))

//Maximum size of string expected from buoy
#define MAX_STRING_SIZE 4


/************************
*   Magnometer information
*   SDA: PD1
*   SCL: PD0
*************************/
#include "./I2C-master-lib/i2c_master.h"
#define HMC5883L_WRITE 0x3C
#define HMC5883L_READ 0x3D 
int16_t raw_x = 0;
int16_t raw_y = 0;
int16_t raw_z = 0;
float headingDegrees = 0;


//Some easy to read defines
#define PATH1 1
#define PATH2 2
#define PATH3 3
#define MOVE_SPEED 50
#define TURN_SPEED 15

/************************
*   BOTH UART FUNCTIONS
*************************/
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

/************************
*   LCD Functions
*************************/
void home_line2(void);
void string2lcd(unsigned char *lcd_str);
void strobe_lcd(void);
void clear_display(void);
void home_line2(void);
void char2lcd(unsigned char a_char);
void spi_init(void);
void lcd_init(void);

/*************************
*   Distance Sensing Functions
*   To be used with the laser and photoresistor
**************************/
double get_distance(unsigned int pin);
double print_distance(unsigned int pin);

/*************************
*   PWM and Motor Control Functions
**************************/
void TIM16_WriteTCNT1( unsigned int i );
void move(float left, float right, float z);
void set_16bitPWM1();
void init_esc();

/************************
*   High level movement functions
*************************/
void path1();
void path2();
void path3();
void turn(int degrees);

/**************************
*   Magnometer Functions
***************************/
void init_HMC5883L(void);
float getHeading(void);


int main(){


    while (1){


        
    }




    return 0;
}



void path1(){
    //Forward for 2 seconds (about 6 feet)
    //Down 3 seconds (aim for about 4 feet)
    //spin left 90 degrees
    //Forward for 2 seconds (about 6 feet)
    //Spin left 90 degrees
    //Forward for 2 seconds (about 6 feet)
    //Up 3 seconds (resurface)
    //Spin left 90 degrees
    //Forward for 2 seconds (about 6 feet)
    //Spin left 90 degrees
    //Complete (back in some position as start)
}
void path2(){
    //Implement if necessary
}
void path3(){
    //Implement if necessary
}

void turn(int degrees){
    char lcd[16];
    //Get the new heading to aim for
    int new_heading = (int)(headingDegrees + degrees)%360;
    getHeading();
    if (degrees < 0){
        //Spin left until new heading
        while ((int)headingDegrees != new_heading){
            move(-TURN_SPEED,TURN_SPEED,50);
            getHeading();
            itoa(OCR1A/2,lcd,10);
            clear_display();
            string2lcd((unsigned char *)lcd);
            char2lcd((unsigned char)' ');
            itoa(OCR1B/2,lcd,10);
            string2lcd((unsigned char *)lcd);
            home_line2();
            getHeading();
            dtostrf((double)raw_y,3,6,lcd);
            _delay_ms(100);
        }
    }
    else if (degrees > 0){
        //Spin right until new heading
        while ((int)headingDegrees != (int)new_heading){
            move(TURN_SPEED,-TURN_SPEED,50);
            getHeading();
            itoa(OCR1A/2,lcd,10);
            clear_display();
            string2lcd((unsigned char *)lcd);
            char2lcd((unsigned char)' ');
            itoa(OCR1B/2,lcd,10);
            string2lcd((unsigned char *)lcd);
            home_line2();
            getHeading();
            dtostrf((double)raw_y,3,6,lcd);
            _delay_ms(100);
        }
    }
}
void init_HMC5883L(void){

    i2c_start(HMC5883L_WRITE);
    i2c_write(0x00); // set pointer to CRA
    i2c_write(0x70); // write 0x70 to CRA
    i2c_stop();

    i2c_start(HMC5883L_WRITE);
    i2c_write(0x01); // set pointer to CRB
    i2c_write(0xA0);
    i2c_stop();

    i2c_start(HMC5883L_WRITE);
    i2c_write(0x02); // set pointer to measurement mode
    i2c_write(0x00); // continous measurement
    i2c_stop();
}
float getHeading(void){

    i2c_start(HMC5883L_WRITE);
    i2c_write(0x03); // set pointer to X axis MSB
    i2c_stop();

    i2c_start(HMC5883L_READ);

    raw_x = ((uint8_t)i2c_read_ack())<<8;
    raw_x |= i2c_read_ack();

    raw_z = ((uint8_t)i2c_read_ack())<<8;
    raw_z |= i2c_read_ack();

    raw_y = ((uint8_t)i2c_read_ack())<<8;
    raw_y |= i2c_read_nack();

    i2c_stop();

    headingDegrees = atan2((double)raw_y,(double)raw_x) * 180 / 3.141592654 + 180;

    return headingDegrees;
}

void init_esc(){
    int i;
    for (i = 0; i < 3; ++i){
        PORTB |= (1 << 4);
        _delay_us(STOP);
        PORTB &= ~(1 << 4);
        _delay_ms(1000);
    }
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
    //clear_display();
    //string2lcd((unsigned char *)utoa(left_speed,(char *)buffer,10));
    //home_line2();
    //string2lcd((unsigned char *)utoa(right_speed,(char *)buffer,10));
    

}


void USART_Init( unsigned int ubrr ) {
    /* Set baud rate */
    UBRR1H = (unsigned char)(ubrr>>8);
    UBRR1L = (unsigned char)ubrr;
    /* Enable receiver and transmitter */ 
    UCSR1B = (1<<RXEN1)|(1<<TXEN1);
    /* Set frame format: 8data, 2stop bit */ 
    UCSR1C = (1<<USBS1)|(3<<UCSZ01);
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
    uint16_t timeout = 20000;
    /* Wait for data to be received or for timeout*/ 
    do {
        if((UCSR1A & (1<<RXC1))){
            /* Get and return received data from buffer */ 
            return UDR1;
        }
    } while (--timeout);
    return -1;
}

void USART_Receive_String(unsigned char *str){
    int i = 0;
    char c;

    while ((c = USART_Receive()) != END_STRING){ //END_STRING == ~ or 0x7E
        if (c == -1){
            str[0] = 50;
            str[1] = 50;
            str[2] = 50;
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
    //str[i] = '\0';
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
    while (timeout > 0) {
        if((UCSR0A & (1<<RXC0))){
            /* Get and return received data from buffer */ 
            return UDR0;
        }
        //clear_display();
        //string2lcd((unsigned char *)utoa((unsigned int)timeout,buffer,10));
        _delay_us(10);
        --timeout;
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

