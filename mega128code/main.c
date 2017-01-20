
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

#define TRIG 0x01
#define ECHO 0x02
#define SONAR1 (1<<0)
#define SONAR2 (1<<2)
#define NUM_SONARS  1


#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>



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


uint8_t temp, read_byte;


int main(void){
	DDRB = 0xFF;
    PORTB = 0x00;
    DDRD = 0x00;
    DDRE = 0b01010101;    //1,3,5,7 is output, the rest input
    PORTE = 0x00;   //set low


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
    // 8-bit fast pwm asynchronous
    //TCCR0|=(1<<WGM00)|(1<<WGM01)|(1<<COM01)|(1<<CS00);

    //8 bit phase correct pwm
    TCCR0 |= (1 << WGM00);  
    TCCR0 &= ~(1 << WGM01);
    TCCR0 |= (1 << COM01); //Clear OC0 on compare match when up-counting. Set OC0 on compare match when downcounting.
    //Prescaler 256
    TCCR0 |= (1 << CS02) | (1 << CS01);
    TCCR0 &= ~(1 << CS00);

    OCR0 = 0;
    
    //PORTB |= (1<<7);

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
                string2lcd(buffer);
            }
            else {
                OCR0 = 0;
                home_line2();
                dtostrf(0.0,1,6,buffer);
                string2lcd(buffer);

            }
            /*
            if (distance <= limit){
                PORTB |= (1 << (7 - i));
                //switch motor direction
            }
            else if (distance <= limit + step){
                blink(7-i,90);
                //very slow motor
            }
            else if (distance <= limit + 2 * step){
                blink(7 - i, 75);
                //slower motor
            }
            else if (distance <= limit + 3 * step){
                blink(7 - i, 50);
                //slow motor
            }
            else if (distance <= limit + 4 * step){
                blink(7 - i, 25);
                //start slowing motor
            }
            else {
                PORTB &= ~(1 << (7 - i));
            }*/
        }
        
        /*

        if (direction){
            count += 1;
        }
        else {
            count -= 1;
        }
        if (count >= 255){
            count = 255;
            direction = !direction;
        }
        else if (count <= 0){
            count = 0;
            direction = !direction;
        }
        
        OCR0 = count;
        */
        _delay_ms(60);
        
	}

	return 0;
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
    /*
    int wait = 0;
    int i;
    if (speed <= 25){
        wait = 100;
    }
    else if (speed > 25 && speed <= 50){
        wait = 60;
    }
    else if (speed > 50 && speed <= 75){
        wait = 30;
    }
    else {
        wait = 10;
    }
    PORTB |= (1 << led);
    for (i = 0; i < wait; ++i){
        _delay_ms(1);
    }
    PORTB &= ~(1 << led);
    */
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
    int count = 0;
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

    distance = (double)count * 40;
    distance /= 58;


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
