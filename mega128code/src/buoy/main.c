#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "../../lib/uart/ascii_special.h"
#include "../../lib/uart/uart.h"
#include "../../lib/lcd/lcd.h"

unsigned char buffer[100];




void enable_rx0_int(){
    UCSR0B |= (1 << RXCIE0);
}
void enable_rx1_int(){
    UCSR1B |= (1 << RXCIE1);
}


ISR(USART0_RX_vect){
    unsigned char c;
    USART1_send_string((unsigned char *)"Drone Data: ");
    USART0_Receive_String(buffer);
    USART1_send_string(buffer);
    USART1_send_string((unsigned char *)"\r");   
}
ISR(USART1_RX_vect){
    unsigned char c;
    USART1_send_string((unsigned char *)"GUI Data: "); 
}


int main(){
    
    USART0_Init(MYUBRR);
    USART1_Init(MYUBRR);
    USART1_send_string((unsigned char *)"Bluetooth Initialized\r");
    sei();
    enable_rx0_int();
    //enable_rx1_int();
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
        USART1_send_string((unsigned char *)"GUI Data: "); 
        USART1_send_string(data);
        USART1_send_string("\r");
        clear_display();
        //string2lcd(data);
        if (data[0] == 100 && data[1] == 100){
            string2lcd((unsigned char *)"forward");
        }
        else if (data[0] == 1 && data[1] == 1){
            string2lcd((unsigned char *)"reverse");
        }
        else if (data[0] == 100 && data[1] == 1){
            string2lcd((unsigned char *)"right");
        }
        else if (data[0] == 1 && data[1] == 100){
            string2lcd((unsigned char *)"left");
        }
        else if (data[0] == 50 && data[1] == 50){
            string2lcd((unsigned char *)"stop");
        }
        else {
            string2lcd((unsigned char *)"Moving");
            home_line2();
            string2lcd((unsigned char *)itoa(data[0],rec_data,10));
            string2lcd((unsigned char *)itoa(data[1],rec_data,10));
            string2lcd((unsigned char *)itoa(data[2],rec_data,10));
            string2lcd((unsigned char *)itoa(data[3],rec_data,10));
        }
        USART0_send_string(data);

        _delay_ms(10);
    }

    return 0;
}



