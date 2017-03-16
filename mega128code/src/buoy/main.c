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
    USART1_send_string((unsigned char *)"Bluetooth Initialized\r");

    unsigned char data[MAX_STRING_SIZE];
    int i;
    for (i = 0; i < MAX_STRING_SIZE; ++i){
        data[i] = '\0';
    }
    data[MAX_STRING_SIZE - 2] = '~';

    unsigned char rec_data[MAX_STRING_SIZE];
    for (i = 0; i < MAX_STRING_SIZE; ++i){
        rec_data[i] = '\0';
    }
    rec_data[MAX_STRING_SIZE - 2] = '~';

    spi_init();
    lcd_init();

    clear_display();
    string2lcd((unsigned char *)"Ready");
    while (1){

        USART1_Receive_String(data);
        USART1_send_string((unsigned char *)"Data received: ");
        USART1_send_string(data);
        USART1_send_string((unsigned char *)"\r");
        USART0_send_string(data);
        USART0_Receive_String(rec_data);
        USART1_send_string(rec_data);
        USART1_send_string((unsigned char *)"\r");

        _delay_ms(10);
    }

    return 0;
}



