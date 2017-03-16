#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include "../../lib/uart/ascii_special.h"
#include "../../lib/adc/adc.h"
#include "../../lib/motors/motors.h"
#include "../../lib/uart/uart.h"
#include "../../lib/magnometer/magnometer.h"
#include "../../lib/lasersensor/lasersensor.h" //NOT YET IMPLEMENTED
#include "../../lib/lcd/lcd.h"


//Some easy to read defines
#define PATH1 1
#define PATH2 2
#define PATH3 3
#define MOVE_SPEED 50
#define TURN_SPEED 15
#define STABLE_Z 50



/************************
*   High level movement functions
*************************/
void path1();
void path2();
void path3();
void turn(int degrees);
void forward();
void reverse();



int main(){
    
    USART0_Init(MYUBRR);
    //enable_adc();
    //init_HMC5883L();
    //init_motors();
    spi_init();
    lcd_init();
    clear_display();

    unsigned char data[MAX_STRING_SIZE];
    int i;
    for (i = 0; i < MAX_STRING_SIZE; ++i){
        data[i] = '\0';
    }
    data[MAX_STRING_SIZE - 2] = '~';

    string2lcd((unsigned char *)"Ready");
    while (1){
        USART0_Receive_String(data);
        //USART0_send_string(data);
        clear_display();
        string2lcd(data);
        if (strcmp((char *)data,"eee~") == 0){
            clear_display();
            string2lcd((unsigned char *)"Path 1");
            USART0_send_string((unsigned char*)"Confirm Path1. Executing.");
            path1();
        }
        else{
            //USART0_send_string((unsigned char *)"Moving: ");
            USART0_send_string(data);
            clear_display();
            string2lcd((unsigned char*)"Moving ");
            string2lcd(data);
            //move((float)data[0],(float)data[1],(float)data[2]);
        }

        _delay_ms(10);
    }

    return 0;
}

void path1(){
    //Forward for 2 seconds (about 6 feet)
    clear_display();
    string2lcd((unsigned char *)"Forward");
    move(50 + (MOVE_SPEED/2),50 + (MOVE_SPEED/2),STABLE_Z);
    _delay_ms(2000);
    //Down 3 seconds (aim for about 4 feet)
    clear_display();
    string2lcd((unsigned char *)"Down");
    move (50,50,0);
    _delay_ms(3000);
    //spin left 90 degrees
    clear_display();
    string2lcd((unsigned char *)"Turn Left");
    move(50 - MOVE_SPEED/2, 50 + MOVE_SPEED/2, STABLE_Z);
    _delay_ms(2000);
    //turn(-90);
    //Forward for 2 seconds (about 6 feet)
    clear_display();
    string2lcd((unsigned char *)"Forward");
    move(50 + MOVE_SPEED/2, 50 + MOVE_SPEED/2, STABLE_Z);
    _delay_ms(2000);
    //Spin left 90 degrees
    clear_display();
    string2lcd((unsigned char *)"Turn Left");
    move(50 - MOVE_SPEED/2, 50 + MOVE_SPEED/2, STABLE_Z);
    _delay_ms(2000);
    //turn(-90);
    //Forward for 2 seconds (about 6 feet)
    clear_display();
    string2lcd((unsigned char *)"Forward");
    move(50 + MOVE_SPEED/2, 50 + MOVE_SPEED/2, STABLE_Z);
    _delay_ms(2000);
    //Up 3 seconds (resurface)
    clear_display();
    string2lcd((unsigned char *)"Up");
    move(50,50,100);
    //Spin left 90 degrees
    //turn(-90);
    //Forward for 2 seconds (about 6 feet)
    clear_display();
    string2lcd((unsigned char *)"Forward");
    move(50 + MOVE_SPEED/2, 50 + MOVE_SPEED/2, STABLE_Z);
    //Spin left 90 degrees
    clear_display();
    string2lcd((unsigned char *)"Turn Left");
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



