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
#define STOP 1500

#define MAX_SPEED   3800  //in timer steps -- 1900usec @ 30.517578Hz
#define MIN_SPEED   2200  //in timer steps -- 1100usec @ 30.517578Hz
#define SPEED_RANGE MAX_SPEED - MIN_SPEED
#define STOP_SPEED  SPEED_RANGE/2 + MIN_SPEED

#define MIN_INPUT   -100
#define MAX_INPUT   100

#define STEP        (double)(MAX_INPUT - MIN_INPUT)/((double)(MAX_SPEED - MIN_SPEED))


#include <avr/io.h>
#include <avr/interrupt.h>
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
void forward(int speed);
void reverse(int speed);
void stop();
void TIM16_WriteTCNT1( unsigned int i );
void move(float left, float right, float z);

uint8_t temp, read_byte;


int main(void){
    DDRB = 0xFF;
    PORTB = 0x00;
    DDRD = 0x00;


    char buffer[16];
    spi_init();
    lcd_init();

    double temp;

    clear_display();
    string2lcd("Press S1");
	while (((PIND) & (1 << 0)));
	
	PORTB |= (1 << 4);
	_delay_us(STOP);
	PORTB &= ~(1 << 4);
	_delay_ms(1000);

    /*
    //8 bit phase correct pwm
    TCCR0 = (1 << WGM00) | (1 << WGM01);

    //Non inverting
    TCCR0 |= (1 << COM01);
    
    //Prescaler 1024
    TCCR0 |= (1 << CS02) | (1 << CS01) | (1 << CS00);

    OCR0 = 0;
    */

    
    //16-bit fast pwm non-inverting on PB5
    TCCR1A |= (1 << COM1A1); //inverting

    //16-bit fast pwm non-inverting on PB6
    TCCR1A |= (1 << COM1B1); //non-inverting

    //16-bit fast pwm non-inverting on PB7
    TCCR1A |= (1 << COM1C1); //non-inverting

    //Fast PWM w/ TOP ICR1
    TCCR1A |= (1 << WGM11); 
    TCCR1B |= (1 << WGM13) | (1 << WGM12);
    
    //Prescaler 8 which is 30.517578 Hz
    TCCR1B |= (1 << CS11);

    //Prescaler 1 which is 244.140625 Hz --> actually 800Hz?
    //TCCR1B |= (1 << CS10);

    TIM16_WriteTCNT1(1);
    ICR1 = (unsigned int) 65535;
    _delay_ms(100);
	
    while(1){
        clear_display();
		if (!((PIND) & (1 << 7))){
			string2lcd("Left 6%");
            move(6,0,0);
            OCR1C = 65000;
		}
		else if (!((PIND) & (1 << 6))){
			string2lcd("Left 7%");
            move(7,0,0);
            OCR1C = 0;
		}
		else if (!((PIND) & (1 << 5))){
			string2lcd("Left 8%");
            move(8,0,0);
            OCR1C = 10000;
		}
		else if (!((PIND) & (1 << 4))){
            string2lcd("Left 9%");
			move(9,0,0);
		}
		else if (!((PIND) & (1 << 3))){
            string2lcd("Left 10%");
            move(10,0,0);
		}
        else if (!((PIND) & (1 << 1))){
            string2lcd("Left 50%");
            move(50,0,0);
        }
		else {
			string2lcd("Stop");
			//stop();
            move(0,0,0);
		}
		_delay_ms(20);
	}

    return 0;
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
    OCR1A = Left motor
    OCR1B = Right motor
    maybe OCR1C = Z motor?

    NOTE: Does not activate AfroESC with Simonk firmware given less than 6
*/
void move(float left, float right, float z){
    if (left < MIN_INPUT){
        left = MIN_INPUT;
    }
    if (left > MAX_INPUT){
        left = MAX_INPUT;
    }
    if (right < MIN_INPUT){
        right = MIN_INPUT;
    }
    if (right > MAX_INPUT){
        right = MAX_INPUT;
    }
    if (z < MIN_INPUT){
        z = MIN_INPUT;
    }
    if (z > MAX_INPUT){
        z = MAX_INPUT;
    }
    unsigned int left_speed, right_speed, z_speed;
    left_speed = (unsigned int)((100 + left)/((double)STEP) + MIN_SPEED);
    right_speed = (unsigned int)((100 + right)/((double)STEP) + MIN_SPEED);
    //This needs to be redone to account for a different controller
    z_speed = (unsigned int)((100 + z)/((double)STEP) + MIN_SPEED); 
    OCR1A = left_speed;
    OCR1B = right_speed;
    /*
	char buffer[16];
	string2lcd(utoa(left_speed,buffer,10));
	home_line2();
	string2lcd(utoa(right_speed,buffer,10));
    */

}

void forward(int speed){
	if (speed > 100){
		speed = 100;
	}
	if (speed < 0){
		speed = 0;
	}
	speed *= 4;

	PORTB |= (1 << 4);
	int i;
	for (i = 0; i < STOP + speed; ++i){
		_delay_us(1);
	}
	//_delay_us(1600);
	PORTB &= ~(1 << 4);
	home_line2();
	char buffer[16];
	string2lcd(itoa(STOP + speed,buffer,10));
}
void reverse(int speed){
	if (speed > 100){
		speed = 100;
	}
	if (speed < 0){
		speed = 0;
	}
	speed *= 4;
	home_line2();
	char buffer[16];
	string2lcd(itoa(STOP - speed,buffer,10));
	PORTB |= (1 << 4);
	int i;
	for (i = 0; i < STOP - speed; ++i){
		_delay_us(1);
	}
	
	//_delay_us(1400);
	PORTB &= ~(1 << 4);
}
void stop(){
	PORTB |= (1 << 4);
	_delay_us(STOP);
	PORTB &= ~(1 << 4);
	home_line2();
	char buffer[16];
	string2lcd(itoa(STOP,buffer,10));
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
