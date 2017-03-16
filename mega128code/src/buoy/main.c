#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include "../../lib/uart/ascii_special.h"
#include "../../lib/uart/uart.h"
#include "../../lib/lcd/lcd.h"







int main(){
    
    USART0_Init(MYUBRR);
    USART1_Init(MYUBRR);
	spi_init();
	lcd_init();
    unsigned char data[MAX_STRING_SIZE];

    while (1){
        USART1_Receive_String(data);
		clear_display();
		string2lcd(data);
        USART0_send_string(data);
		USART0_Receive_String(data);
		USART1_send_string(data);
		home_line2();
		string2lcd(data);

        _delay_ms(100);
    }

    return 0;
}



