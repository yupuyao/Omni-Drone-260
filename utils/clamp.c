#include "wiringPi.h"
#include <stdio.h>
#define PWM_pin 0

int main()
{
    wiringPiSetup();
    
    int div = 384;                    
    int arr = 1000;                
    int minDuty = 50;                                   

    pinMode(PWM_pin, PWM_OUTPUT);          
   
    pwmSetClock(PWM_pin, div);
    pwmSetRange(PWM_pin, arr);
    pwmWrite(PWM_pin, minDuty);
}

