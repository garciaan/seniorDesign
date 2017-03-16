#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include "../../../lib/uart/uart.h"
#include "../../../lib/lcd/lcd.h"

#define size 5


/**************************
* This will test the male to male RS232 connection and will simulate the buoy to drone
*   connection, along with returning signals

    PC (Bluetooth) ----> (Bluetooth) Relay (RS232) ----> (RS232) Receiver |
  ^                                                                       |
  |                                                                       |
  |                                                                       v
  | <------------------- (Bluetooth) Relay (RS232) <---- (RS232) Receiver 

Relay Setup:
* HC-05:    Connect the TX to PIND2(RX)
            Connect the RX to PIND3(TX)
* Connect out of box end of the RS232 cable to RS232 port 
* Load the hex file contained in the relay folder to the mega128

Receiver Setup:
* Connect the other end of the RS232 port to the RS232 port
* Load the hex file contained in the receiver folder to the mega128

PC Setup:
* Open Cool Term
* Set the terminal to connect to the HC-05 bluetooth module with 8 data bits and 1 stop bit
* Press connect

To run:
* Reset board and follow instructions
* Verify terminal does not report an error
**************************/

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
    data[size-2] = '~';

    /*****************
    *   THIS IS THE TEST FILE FOR TESTING BOTH UARTS
    *******************/
    while (1){
        
        clear_display();
        string2lcd((unsigned char *)"Waiting Rec...");
        USART1_send_string((unsigned char *)"Please send string \"123~\" over bluetooth\r");
        USART1_Receive_String(data);
        home_line2();
        string2lcd(data);
        if (strcmp((char *)data,"123~") != 0){ //Strings are not equal
            USART1_send_string((unsigned char *)"Error");
            USART1_send_string(data);
            USART1_send_string((unsigned char *)"\r");
            continue;
        }

        USART0_send_string(data);
        USART0_Receive_String(data);
        if (strcmp((char *)data,"123~") != 0){ //Strings are not equal
            USART1_send_string((unsigned char *)"Error Receiving from drone: ");
            USART1_send_string(data);
            USART1_send_string((unsigned char *)"\r");
            continue;
        }
        else {
            USART1_send_string((unsigned char *)"SUCCESS: ");
            USART1_send_string(data);
            USART1_send_string((unsigned char *)"\r");
        }

        USART1_send_string((unsigned char *)"TEST COMPLETE\r-------------------------------\r");
        clear_display();
        string2lcd((unsigned char *)"Tests Complete");
        home_line2();
        string2lcd((unsigned char *)"Resetting...");
        _delay_ms(2000);
    }

    return 0;
}
