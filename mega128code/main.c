#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include "./lib/uart/ascii_special.h"
#include "./lib/adc/adc.h"
#include "./lib/lcd/lcd.h"
#include "./lib/motors/motors.h"
#include "./lib/uart/uart.h"
#include "./lib/magnometer/magnometer.h"
#include "./lib/lasersensor/lasersensor.h" //NOT YET IMPLEMENTED


//Some easy to read defines
#define PATH1 1
#define PATH2 2
#define PATH3 3
#define MOVE_SPEED 50
#define TURN_SPEED 15



/************************
*   High level movement functions
*************************/
void path1();
void path2();
void path3();
void turn(int degrees);


int main(){


    while (1){


        _delay_ms(10);
    }

    return 0;
}



void path1(){
    //Forward for 2 seconds (about 6 feet)
    //Down 3 seconds (aim for about 4 feet)
    //spin left 90 degrees
    //Forward for 2 seconds (about 6 feet)
    //Spin left 90 degrees
    //Forward for 2 seconds (about 6 feet)
    //Up 3 seconds (resurface)
    //Spin left 90 degrees
    //Forward for 2 seconds (about 6 feet)
    //Spin left 90 degrees
    //Complete (back in some position as start)
}
void path2(){
    //Implement if necessary
}
void path3(){
    //Implement if necessary
}

void turn(int degrees){
    char lcd[16];
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
            itoa(OCR1A/2,lcd,10);
            clear_display();
            string2lcd((unsigned char *)lcd);
            char2lcd((unsigned char)' ');
            itoa(OCR1B/2,lcd,10);
            string2lcd((unsigned char *)lcd);
            home_line2();
            getHeading(x,y,z);
            dtostrf((double)*y,3,6,lcd);
            _delay_ms(100);
        }
    }
    else if (degrees > 0){
        //Spin right until new heading
        while ((int)headingDegrees != (int)new_heading){
            move(TURN_SPEED,-TURN_SPEED,50);
            getHeading(x,y,z);
            itoa(OCR1A/2,lcd,10);
            clear_display();
            string2lcd((unsigned char *)lcd);
            char2lcd((unsigned char)' ');
            itoa(OCR1B/2,lcd,10);
            string2lcd((unsigned char *)lcd);
            home_line2();
            getHeading(x,y,z);
            dtostrf((double)*y,3,6,lcd);
            _delay_ms(100);
        }
    }
}



