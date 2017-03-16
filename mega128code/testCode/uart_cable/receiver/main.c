#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include "../../../lib/uart/uart.h"
#include "../../../lib/lcd/lcd.h"

#define size 5

int main(){
    DDRB = 0xFF;
    PORTB = 0x00;
    USART0_Init(MYUBRR);
    USART0_send_string((unsigned char *)"USART0 (RS232) Initialized \r");

    spi_init();
    lcd_init();
    string2lcd((unsigned char*)"LCD Initialized");
    _delay_ms(1000);

    unsigned char data[size];
    int i;
    for (i = 0; i < size; ++i){
        data[i] = '\0';
    }
    data[size-2] = '~';

    /*****************
    *   THIS IS THE TEST FILE FOR TESTING BOTH UARTS
    *******************/
    while (1){
        
        clear_display();
        string2lcd((unsigned char *)"Waiting...");
        USART0_Receive_String(data);
        home_line2();
        string2lcd(data);
        USART0_send_string(data);
        _delay_ms(1000);
    }

    return 0;
}
