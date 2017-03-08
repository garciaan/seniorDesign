#include "uart.h"


void USART1_Init( unsigned int ubrr ) {
    /* Set baud rate */
    UBRR1H = (unsigned char)(ubrr>>8);
    UBRR1L = (unsigned char)ubrr;
    /* Enable receiver and transmitter */ 
    UCSR1B = (1<<RXEN1)|(1<<TXEN1);
    /* Set frame format: 8data, 2stop bit */ 
    UCSR1C = (1<<USBS1)|(3<<UCSZ01);
    _delay_ms(100);
}
void USART1_Transmit(unsigned char data ) {
    /* Wait for empty transmit buffer */ 
    while ( !( UCSR1A & (1<<UDRE1)) );
    /* Put data into buffer, sends the data */ 
    UDR1 = data;
}

void USART1_send_string(unsigned char *data){
    int i = 0;
    while (data[i] != '\0'){
        USART1_Transmit(data[i]);
        ++i;
    }
}

unsigned char USART1_Receive(void){
    uint16_t timeout = 20000;
    /* Wait for data to be received or for timeout*/ 
    do {
        if((UCSR1A & (1<<RXC1))){
            /* Get and return received data from buffer */ 
            return UDR1;
        }
    } while (--timeout);
    return -1;
}

void USART1_Receive_String(unsigned char *str){
    int i = 0;
    char c;

    while ((c = USART1_Receive()) != END_STRING){ //END_STRING == ~ or 0x7E
        if (c == -1){
            str[0] = 50;
            str[1] = 50;
            str[2] = 50;
        }
        str[i] = c;
        ++i;
        if (i >= MAX_STRING_SIZE){
            str[MAX_STRING_SIZE - 1] = '\0';

            return;
        }
    }
}


void USART0_Init( unsigned int ubrr ) {
    /* Set baud rate */
    UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)ubrr;
    /* Enable receiver and transmitter */ 
    UCSR0B = (1<<RXEN0)|(1<<TXEN0);
    /* Set frame format: 8data, 1stop bit */ 
    UCSR0C = (3<<UCSZ00);
    _delay_ms(100);
}
void USART0_Transmit(unsigned char data ) {
    /* Wait for empty transmit buffer */ 
    while ( !( UCSR0A & (1<<UDRE0)) );
    /* Put data into buffer, sends the data */ 
    UDR0 = data;
}

void USART0_send_string(unsigned char *data){
    int i = 0;
    while (data[i] != '\0'){
        USART0_Transmit(data[i]);
        ++i;
    }
}

unsigned char USART0_Receive(void){
    uint16_t timeout = 50000;
    //unsigned char buffer[16];
    /* Wait for data to be received or for timeout*/ 
    while (timeout > 0) {
        if((UCSR0A & (1<<RXC0))){
            /* Get and return received data from buffer */ 
            return UDR0;
        }
        //clear_display();
        //string2lcd((unsigned char *)utoa((unsigned int)timeout,buffer,10));
        _delay_us(10);
        --timeout;
    }
    return 255;
}

void USART0_Receive_String(unsigned char *str){
    int i = 0;
    unsigned char c;

    while ((c = (unsigned char)USART0_Receive()) != END_STRING){ //END_STRING == ~ or 0x7E
        if (c == 255){
            str[0] = 50;
            str[1] = 50;
            str[2] = 50;
            str[3] = '\0';
            return;
        }
        str[i] = c;
        //char2lcd(c);
        //string2lcd(str);
        ++i;
        if (i >= MAX_STRING_SIZE){
            str[MAX_STRING_SIZE - 1] = '\0';

            return;
        }
    }
    str[i] = '\0';
    //string2lcd(str);

}