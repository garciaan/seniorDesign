#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include "../../lib/uart/ascii_special.h"
#include "../../lib/uart/uart.h"
#include "../../lib/lcd/lcd.h"

#define size 4

int main(){
    DDRB = 0xFF;
    PORTB = 0x00;
    USART0_Init(MYUBRR);
    USART0_send_string((unsigned char *)"USART0 (RS232) Initialized \r");
    //USART0_Transmit('A');
    USART1_Init(MYUBRR);
    USART1_send_string((unsigned char *)"USART1 (Bluetooth) Initialized \r");

    spi_init();
    lcd_init();
    string2lcd((unsigned char*)"LCD Initialized");
    _delay_ms(1000);

    unsigned char data[size];
    int i;
    for (i = 0; i < size; ++i){
        data[i] = '\0';
    }

    /*****************
    *   THIS IS THE TEST FILE FOR TESTING BOTH UARTS
    *******************/
    while (1){

        // clear_display();
        // string2lcd((unsigned char *)"RS232 Test");
        // USART0_send_string((unsigned char *)"Please send string \"123\" over RS232\r");
        // USART0_Receive_String(data);
        // home_line2();
        // string2lcd(data);
        // if (strcmp((char *)data,"123")){
        //     USART0_send_string((unsigned char *)"Error: ");
        //     USART0_send_string(data);
        //     USART0_send_string((unsigned char *)"\r");
        // }
        // else {
        //     USART0_send_string((unsigned char *)"Received Data: ");
        //     USART0_send_string(data);
        //     USART0_send_string((unsigned char *)"\r");
        // }
        // USART0_send_string((unsigned char *)"RS232 Test Completed\r");

        // clear_display();
        // string2lcd((unsigned char *)"Bluetooth Test");
        // USART1_send_string((unsigned char *)"Please send string \"123\" over bluetooth\r");
        // USART1_Receive_String(data);
        // home_line2();
        // string2lcd(data);
        // if (strcmp((char *)data,"123")){
        //     USART1_send_string((unsigned char *)"Error");
        //     USART1_send_string(data);
        //     USART1_send_string((unsigned char *)"\r");
        // }
        // else {
        //     USART1_send_string((unsigned char *)"Received Data: ");
        //     USART1_send_string(data);
        //     USART1_send_string((unsigned char *)"\r");
        // }
        // USART1_send_string((unsigned char *)"Bluetooth Test Completed\r");

        // clear_display();
        // string2lcd((unsigned char *)"UART Test Completed");
        // home_line2();
        // string2lcd((unsigned char *)"Resetting...");
    }

    return 0;
}
