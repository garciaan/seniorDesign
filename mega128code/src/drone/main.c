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
#include "../../lib/leds/leds.h"


//Some easy to read defines
#define PATH1 1
#define PATH2 2
#define PATH3 3
#define MOVE_SPEED 50
#define TURN_SPEED 15
//#define STABLE_Z 50
volatile int STABLE_Z = 50;
#define MAX_DEPTH 12 //in feet, used for LED response
#define AUTO_DIVE_DEPTH 5

volatile int data_timer_counter;
#define TIMER0_DIVIDER 15
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
void depth_to_leds();

void init_data_timer();
void enable_bumpers();
void bumper_response();
volatile int bumper_hit;
void auto_delay(int ms);

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
    //init_HMC5883L(); //magnometer
    init_motors();
    data_timer_counter = 0;
    init_data_timer();
    enable_bumpers();
    //init_leds();
    //set_rgb(RED);
    sei();
    

    unsigned char data[MAX_STRING_SIZE];
    int i;
    for (i = 0; i < MAX_STRING_SIZE; ++i){
        data[i] = '\0';
    }
    data[MAX_STRING_SIZE - 2] = '~';

    move(50,50,50);
    while (1){
        //PORTB &= ((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3));
        if (bumper_hit) {
            bumper_hit = 0;
            bumper_response();
            bumper_hit = 0;
        }
        USART0_Receive_String(data);
        USART0_send_string((unsigned char *)"Data received: ");
        USART0_send_string(data);
        USART0_send_string((unsigned char *)"\r\n");
        //depth_to_leds();
        if (strcmp((char *)data,"eee~") == 0){
            USART0_send_string((unsigned char *)"Initiating path 1\r\n");
            //disable RX0
            UCSR0B &= ~(1<<RXEN0);
            path1();
            //enable RX0
            UCSR0B |= (1<<RXEN0);
        }
        else if (data[0] == 103 && data[1] == 103 && data[3] == 126){ //0x67 0x67 0xdd 0x7e or 103 103 ddd 126
            USART0_send_string((unsigned char *)"Stable Z Set\r\n");
            STABLE_Z = data[2]; 
        }
        else if (strcmp((char *)data,"hhh~") == 0){ //0x68 0x68 0x68 0x7e or 104 104 104 126
            USART0_send_string((unsigned char *)"Diving to 10 feet\r\n");
            dive(10);
            move(50,50,STABLE_Z);
            USART0_send_string((unsigned char *)"Depth reached. Waiting for 10 seconds.\r\n");
            auto_delay(10000);
            move(50,50,STABLE_Z);
            USART0_send_string((unsigned char *)"Returning to surface\r\n");
        }
        else if (strcmp((char *)data,"fff~") == 0){ //0x66 0x66 0x66 0x7e or 102 102 102 126
            USART0_send_string((unsigned char *)"Calibrating\r\n");
            calibrate_pressure_sensor();
            USART0_send_string((unsigned char *)"Calibrate complete\r\n");
        }
        else if (strcmp((char *)data,"222~") == 0){
            USART0_send_string((unsigned char *)"Stopping \r\n");
            move(50,50,50);
        }
        else{
            USART0_send_string((unsigned char *)"Moving: ");
            USART0_send_string(data);
            USART0_send_string((unsigned char*)"\r\n");
            move((float)data[0],(float)data[1],(float)data[2]);
        }

        auto_delay(10);
    }

    return 0;
}

ISR(INT6_vect){  //Left bumper on PE6
    bumper_hit = 1;
    EIFR |= (1 << INTF6);
}

ISR(INT7_vect){  //Right bumper on PE7
    bumper_hit = 1;
    EIFR |= (1 << INTF7);
}

/******************
*   Sends the drone info 30 times per sec
    Format:
        Depth: ddd.dddddd
        Object: (NO | YES)
        Heading: ddd.dddddd
        Water Level: (OK | WARNING | ERROR) : dd.dd
*******************/
ISR(TIMER0_COMP_vect){
    if (data_timer_counter >= TIMER0_DIVIDER){ 
        data_timer_counter = 0;
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
    else {
        ++data_timer_counter;
    }
}

void dive(float depth){
    float current_depth = 0;
    while (current_depth < depth){
        current_depth = get_depth_feet();
        move(50,50,STABLE_Z + 20);
        auto_delay(100);
    }
    move(50,50,STABLE_Z);
}

/*********
* Need to do rising edge
**********/
void enable_bumpers(){
    //Set pins as inputs
    DDRE &= ~(1 << 6);
    DDRE &= ~(1 << 7);
    
    //Enable internal pullups
    PORTE |= (1 << 6);
    PORTE |= (1 << 7);

    //Set both interrupt 4 and 5 to rising edge
    EICRB |= (1 << ISC61) | (1 << ISC60);
    EICRB |= (1 << ISC71) | (1 << ISC70);

    //enable the interrupts
    EIMSK |= (1 << INT7);
}

void bumper_response(){
    set_rgb(RED);
    object_detected = 1;
    USART0_send_string((unsigned char *)"--------------------------------\r\n");
    USART0_send_string((unsigned char *)"Bumper Hit\r\n");
    USART0_send_string((unsigned char *)"Object: YES\r\n");

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
    move(50,50,STABLE_Z);
    USART0_send_string((unsigned char*)"Resuming\r\n");
    USART0_send_string((unsigned char *)"--------------------------------\r\n");
    EIFR |= (1 << INTF7);
    bumper_hit = 0;
}
/**********
*    Timer 0
**********/
void init_data_timer(){
    //CTC Mode
    TCCR0 |= (1 << WGM01);
    TCCR0 &= ~(1 << WGM00);

    //Prescalar 1024
    TCCR0 |= (1 << CS02);
    TCCR0 |= (1 << CS01);
    TCCR0 |= (1 << CS00);

    unsigned char sreg;
    /* Save global interrupt flag */ 
    sreg = SREG;
    /* Disable interrupts */ 
    cli();
    /* Set TCNTn to 1 */
    TCNT0 = 1;
    sei();
    /* Restore global interrupt flag */ 
    SREG = sreg;
    OCR0 = 255;

    //Enable timer0 interrupt
    ETIMSK = (1 << OCIE0);

}

void path1(){
    //Forward for 2 seconds (about 6 feet)
    // clear_display();
    // string2lcd((unsigned char *)"Forward");
    USART0_send_string((unsigned char*)"Move Forward\r\n");
    move(50 + (MOVE_SPEED/2),50 + (MOVE_SPEED/2),0);
    auto_delay(2000);
    //Down 3 seconds (aim for about 4 feet)
    // clear_display();
    // string2lcd((unsigned char *)"Down");
    USART0_send_string((unsigned char*)"Move Down\r\n");
    move (50,50,0);
    while (get_depth_feet() < AUTO_DIVE_DEPTH);
    //spin left 90 degrees
    // clear_display();
    // string2lcd((unsigned char *)"Turn Left");
    USART0_send_string((unsigned char*)"Turn Left\r\n");
    move(50 - MOVE_SPEED/2, 50 + MOVE_SPEED/2, STABLE_Z);
    auto_delay(500);
    //turn(-90);
    //Forward for 2 seconds (about 6 feet)
    // clear_display();
    // string2lcd((unsigned char *)"Forward");
    USART0_send_string((unsigned char*)"Move Forward\r\n");
    move(50 + MOVE_SPEED/2, 50 + MOVE_SPEED/2, STABLE_Z);
    auto_delay(2000);
    //Spin left 90 degrees
    // clear_display();
    // string2lcd((unsigned char *)"Turn Left");
    USART0_send_string((unsigned char*)"Turn Left\r\n");
    move(50 - MOVE_SPEED/2, 50 + MOVE_SPEED/2, STABLE_Z);
    auto_delay(500);
    //turn(-90);
    //Forward for 2 seconds (about 6 feet)
    // clear_display();
    // string2lcd((unsigned char *)"Forward");
    USART0_send_string((unsigned char*)"Move Forward\r\n");
    move(50 + MOVE_SPEED/2, 50 + MOVE_SPEED/2, STABLE_Z);
    auto_delay(2000);
    //Up 3 seconds (resurface)
    // clear_display();
    // string2lcd((unsigned char *)"Up");
    USART0_send_string((unsigned char*)"Move Up\r\n");
    move(50,50,100);
    while (get_depth_feet() > 1);
    
    //Spin left 90 degrees
    //turn(-90);
    //Forward for 2 seconds (about 6 feet)
    // clear_display();
    // string2lcd((unsigned char *)"Forward");
    USART0_send_string((unsigned char*)"Move Forward\r\n");
    move(50 + MOVE_SPEED/2, 50 + MOVE_SPEED/2, 0);
    auto_delay(2000);
    //Spin left 90 degrees
    // clear_display();
    // string2lcd((unsigned char *)"Turn Left");
    USART0_send_string((unsigned char*)"Turn Left\r\n");
    move(50 - MOVE_SPEED/2, 50 + MOVE_SPEED/2, 0);
    auto_delay(500);
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

void depth_to_leds(){
    float depth = get_depth_feet();
    int red = 0;
    int green = 0;
    int blue = 0;

    red = (int)((depth)/((float)MAX_DEPTH) * 255);
    green = (int)((MAX_DEPTH - depth)/((float)MAX_DEPTH) * 255);

    set_rgb(red,green,blue);
}

void auto_delay(int ms){
    _delay_ms(1);
    if (bumper_hit){
        bumper_response();
    }
}
