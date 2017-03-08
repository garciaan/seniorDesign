
#ifndef F_CPU
#define F_CPU 16000000UL
#endif


#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include <avr/interrupt.h>


#include "../I2C-master-lib/i2c_master.h"
//#define HMC5883L_WRITE 0x3C
//#define HMC5883L_READ 0x3D 
#define HMC5883L_READ 0x1E
#define HMC5883L_WRITE 0x1E  

char buffer[16];
int16_t raw_x = 0;
int16_t raw_y = 0;
int16_t raw_z = 0;
float headingDegrees = 0;

void home_line2(void);
void string2lcd(char *lcd_str);
void strobe_lcd(void);
void clear_display(void);
void home_line2(void);
void char2lcd(char a_char);
void string2lcd(char *lcd_str);
void spi_init(void);
void lcd_init(void);
void init_HMC5883L(void);
float getHeading(void);


int main(void){
	DDRB = 0xFF;
    PORTB = 0x00;
    DDRD = 0x00;

    spi_init();
    lcd_init();

	i2c_init();
    init_HMC5883L();


    clear_display();
    string2lcd("Starting Program");


    _delay_ms(500);
    while(1){
        clear_display();
        getHeading();
        itoa(raw_y,buffer,10);
        string2lcd(buffer);
        /*
        char2lcd(' ');
        itoa(raw_y,buffer,10);
        string2lcd(buffer);
        home_line2();
        itoa(raw_z,buffer,10);
        string2lcd(buffer);*/
        //dtostrf((double)raw_x,3,6,buffer);

        _delay_ms(500);
        
	}

	return 0;
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
