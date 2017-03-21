#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "../../lib/uart/ascii_special.h"
#include "../../lib/adc/adc.h"
#include "../../lib/motors/motors.h"
#include "../../lib/uart/uart.h"
#include "../../lib/magnometer/magnometer.h"
#include "../../lib/lasersensor/lasersensor.h" //NOT YET IMPLEMENTED
#include "../../lib/lcd/lcd.h"
#include "../../lib/pressuresensor/pressuresensor.h"


//Some easy to read defines
#define PATH1 1
#define PATH2 2
#define PATH3 3
#define MOVE_SPEED 50
#define TURN_SPEED 15
//#define STABLE_Z 50
volatile int STABLE_Z = 50;


/************************
*   High level movement functions
*************************/
void path1();
void path2();
void path3();
void turn(int degrees);
void forward();
void reverse();
void dive(float depth);

void init_data_timer();
void enable_bumpers();

char buffer[10];
volatile int object_detected = 0;


int main(){
    DDRB = 0xFF;
    PORTB = 0;
    USART0_Init(MYUBRR);
    USART0_send_string((unsigned char *)"USART0 (RS232) Initialized\r\n");
    STABLE_Z = 50;
    enable_adc();
    calibrate_pressure_sensor();
    //init_HMC5883L();
    init_motors();
    init_data_timer();
    enable_bumpers();
    sei();
    

    unsigned char data[MAX_STRING_SIZE];
    int i;
    for (i = 0; i < MAX_STRING_SIZE; ++i){
        data[i] = '\0';
    }
    data[MAX_STRING_SIZE - 2] = '~';

    move(50,50,50);
    while (1){
        USART0_Receive_String(data);
        // USART0_send_string(data);

        if (strcmp((char *)data,"eee~") == 0){
            USART0_send_string(data);
            //disable RX0
            UCSR0B &= ~(1<<RXEN0);
            path1();
            //enable RX0
            UCSR0B |= (1<<RXEN0);
        }
        else if (data[0] == 103 && data[1] == 103 && data[3] == 126){ //0x67 0x67 0xdd 0x7e or 103 103 ddd 126
            USART0_send_string(data);
            USART0_send_string((unsigned char *)"Stable Z Set\r\n");
            STABLE_Z = data[2]; 
        }
        else if (strcmp((char *)data,"hhh~") == 0){ //0x68 0x68 0x68 0x7e or 104 104 104 126
            dive(10);
            USART0_send_string((unsigned char *)"Diving to 10 feet\r\n");
            USART0_send_string(data);
            USART0_send_string((unsigned char*)"\r\n");
            _delay_ms(10000);
        }
        else if (strcmp((char *)data,"fff~") == 0){ //0x66 0x66 0x66 0x7e or 102 102 102 126
            calibrate_pressure_sensor();
            USART0_send_string(data);
            USART0_send_string((unsigned char*)"\r\n");
        }
        else if (strcmp((char *)data,"222~") == 0){
            move(50,50,50);
            USART0_send_string(data);
            USART0_send_string((unsigned char*)"\r\n");
        }
        else{
            USART0_send_string((unsigned char *)"Moving: ");
            USART0_send_string(data);
            USART0_send_string((unsigned char*)"\r\n");
            move((float)data[0],(float)data[1],(float)data[2]);
        }

        //_delay_ms(10);
    }

    return 0;
}

ISR(INT4_vect){  //Left bumper on PE4
    object_detected = 1;
    USART0_send_string((unsigned char *)"Left Bumper Hit\r\n");

    //reverse
    USART0_send_string((unsigned char *)"Reversing\r\n");
    move(0,0,STABLE_Z);
    _delay_ms(1000);
    

    move (50,50,STABLE_Z);
    _delay_ms(500);

    //turn left
    USART0_send_string((unsigned char *)"Turning Right\r\n");
    move(50 - TURN_SPEED,50 + TURN_SPEED,STABLE_Z);
    _delay_ms(1000);
    USART0_send_string((unsigned char*)"Resuming\r\n");
}

ISR(INT5_vect){  //Right bumper on PE5
    object_detected = 1;
    USART0_send_string((unsigned char *)"Right Bumper Hit\r\n");

    //reverse
    USART0_send_string((unsigned char *)"Reversing\r\n");
    move(0,0,STABLE_Z);
    _delay_ms(1000);
    

    move (50,50,STABLE_Z);
    _delay_ms(500);

    //turn left
    USART0_send_string((unsigned char *)"Turning Left\r\n");
    move(50 + TURN_SPEED,50 - TURN_SPEED,STABLE_Z);
    _delay_ms(1000);
    USART0_send_string((unsigned char*)"Resuming\r\n");
}

/******************
*   Sends the drone info once ever .5 seconds
    Format:
        Depth: ddd.dddddd
        Object: (NO | YES)
        Heading: ddd.dddddd
        Water Level: (OK | WARNING | ERROR) : dd.dd
*******************/
ISR(TIMER3_COMPA_vect){ 
    USART0_send_string((unsigned char *)"Depth: ");
    USART0_send_string((unsigned char *)dtostrf(get_depth_feet(),3,7,buffer));
    USART0_send_string((unsigned char*)"\r\n");
    USART0_send_string((unsigned char*)"Object: ");
    if (object_detected){
        USART0_send_string((unsigned char*)"YES");
        object_detected = 0;
    }
    else {
        USART0_send_string((unsigned char*)"NO");
    }
    USART0_send_string((unsigned char*)"\r\n");
    USART0_send_string((unsigned char*)"Heading: ");
    USART0_send_string((unsigned char*)"Not yet implemented");
    USART0_send_string((unsigned char*)"\r\n");
    USART0_send_string((unsigned char*)"Water Level: ");
    USART0_send_string((unsigned char*)"Not yet implemented");
    USART0_send_string((unsigned char*)"\r\n");
    USART0_send_string((unsigned char*)"\r\n");
}

void dive(float depth){
    float current_depth = 0;
    while (current_depth < depth){
        current_depth = get_depth_feet();
        move(50,50,STABLE_Z + 20);
        _delay_ms(100);
    }
    move(50,50,STABLE_Z);
}

void enable_bumpers(){
    //Set pins as inputs
    DDRE &= ~(1 << 4);
    DDRE &= ~(1 << 5);
    
    //Enable internal pullups
    PORTE |= (1 << 4);
    PORTE |= (1 << 5);

    //Set both interrupt 4 and 5 to falling edge
    EICRB |= (1 << ISC41);
    EICRB |= (1 << ISC51);

    //enable the interrupts
    EIMSK |= (1 << INT4) | (1 << INT5);
}
void init_data_timer(){
    //CTC Mode
    TCCR3A |= (1 << COM3A1);
    TCCR3A &= ~(1 << COM3A0);

    //Prescalar 256
    TCCR3B |= (1 << CS32);

    unsigned char sreg;
    /* Save global interrupt flag */ 
    sreg = SREG;
    /* Disable interrupts */ 
    cli();
    /* Set TCNTn to 1 */
    TCNT3 = 1;
    sei();
    /* Restore global interrupt flag */ 
    SREG = sreg;
    ICR3 = 65535;
    OCR3A = 65535;

    //Enable timer3a interrupt
    ETIMSK = (1 << OCIE3A);

}

void path1(){
    //Forward for 2 seconds (about 6 feet)
    // clear_display();
    // string2lcd((unsigned char *)"Forward");
    USART0_send_string((unsigned char*)"Move Forward\r\n");
    move(50 + (MOVE_SPEED/2),50 + (MOVE_SPEED/2),STABLE_Z);
    _delay_ms(2000);
    //Down 3 seconds (aim for about 4 feet)
    // clear_display();
    // string2lcd((unsigned char *)"Down");
    USART0_send_string((unsigned char*)"Move Down\r\n");
    move (50,50,0);
    _delay_ms(3000);
    //spin left 90 degrees
    // clear_display();
    // string2lcd((unsigned char *)"Turn Left");
    USART0_send_string((unsigned char*)"Turn Left\r\n");
    move(50 - MOVE_SPEED/2, 50 + MOVE_SPEED/2, STABLE_Z);
    _delay_ms(2000);
    //turn(-90);
    //Forward for 2 seconds (about 6 feet)
    // clear_display();
    // string2lcd((unsigned char *)"Forward");
    USART0_send_string((unsigned char*)"Move Forward\r\n");
    move(50 + MOVE_SPEED/2, 50 + MOVE_SPEED/2, STABLE_Z);
    _delay_ms(2000);
    //Spin left 90 degrees
    // clear_display();
    // string2lcd((unsigned char *)"Turn Left");
    USART0_send_string((unsigned char*)"Turn Left\r\n");
    move(50 - MOVE_SPEED/2, 50 + MOVE_SPEED/2, STABLE_Z);
    _delay_ms(2000);
    //turn(-90);
    //Forward for 2 seconds (about 6 feet)
    // clear_display();
    // string2lcd((unsigned char *)"Forward");
    USART0_send_string((unsigned char*)"Move Forward\r\n");
    move(50 + MOVE_SPEED/2, 50 + MOVE_SPEED/2, STABLE_Z);
    _delay_ms(2000);
    //Up 3 seconds (resurface)
    // clear_display();
    // string2lcd((unsigned char *)"Up");
    USART0_send_string((unsigned char*)"Move Up\r\n");
    move(50,50,100);
    //Spin left 90 degrees
    //turn(-90);
    //Forward for 2 seconds (about 6 feet)
    // clear_display();
    // string2lcd((unsigned char *)"Forward");
    USART0_send_string((unsigned char*)"Move Forward\r\n");
    move(50 + MOVE_SPEED/2, 50 + MOVE_SPEED/2, STABLE_Z);
    //Spin left 90 degrees
    // clear_display();
    // string2lcd((unsigned char *)"Turn Left");
    USART0_send_string((unsigned char*)"Turn Left\r\n");
    move(50 - MOVE_SPEED/2, 50 + MOVE_SPEED/2, STABLE_Z);
    _delay_ms(2000);
    //turn(-90);
    //Complete (back in some position as start)
}
void path2(){
    //Implement if necessary
}
void path3(){
    //Implement if necessary
}

void turn(int degrees){
    int *x = 0;
    int *y = 0;
    int *z = 0;
    float headingDegrees = getHeading(x,y,z);
    //Get the new heading to aim for
    int new_heading = (int)(headingDegrees + degrees)%360;
    if (degrees < 0){
        //Spin left until new heading
        while ((int)headingDegrees != new_heading){
            move(-TURN_SPEED,TURN_SPEED,50);
            getHeading(x,y,z);
        }
    }
    else if (degrees > 0){
        //Spin right until new heading
        while ((int)headingDegrees != (int)new_heading){
            move(TURN_SPEED,-TURN_SPEED,50);
            getHeading(x,y,z);
        }
    }
}



