#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include "../../lib/uart/ascii_special.h"
#include "../../lib/uart/uart.h"








int main(){
    
    USART0_Init(MYUBRR);
    USART1_Init(MYUBRR);

    unsigned char data[MAX_STRING_SIZE];

    while (1){
        USART1_Receive_String(data);
        USART0_send_string(data);

        _delay_ms(10);
    }

    return 0;
}



