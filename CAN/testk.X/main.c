#include <stdio.h>
#include <stdlib.h>
#include <delays.h>
#include <p18f25k80.h>
#include <timers.h>
#include <pwm.h>
#include "../libcan/can18xx8.h"


/////*CONFIGURATION*/////
#pragma config RETEN = OFF
#pragma config INTOSCSEL = HIGH
#pragma config SOSCSEL = HIGH
#pragma config FOSC = HS2
#pragma config PLLCFG = OFF
#pragma config FCMEN = OFF
#pragma config IESO = OFF
#pragma config BOREN = OFF
#pragma config BORV = 0
#pragma config WDTEN = OFF
#pragma config CANMX = PORTB
#pragma config MSSPMSK = MSK5 /// A voir
#pragma config MCLRE = ON
#pragma config STVREN = ON
#pragma config BBSIZ = BB2K



/////*CONSTANTES*/////
#define led PORTCbits.RC2


/////*VARIABLES GLOBALES*/////



/////*PROGRAMME PRINCIPAL*/////
void main (void)
{
    /* Initialisations. */
    ADCON1 = 0x0F ;
    ADCON0 = 0b00000000;
    WDTCON = 0 ;


    /* Configurations. */
    TRISA   = 0b11000011 ;
    TRISB   = 0b01111111 ;
    TRISC   = 0b11111000 ;
    PORTC = 0xFF;

    /* Configuration du PWM1 */
    OpenTimer2(TIMER_INT_OFF & T2_PS_1_1 & T2_POST_1_1);
    

   // OpenEPWM(0xF0);
    //OpenPWM2(0xF0);
   // SetDCPWM1(0);
    //SetDCPWM2(0);

    /* Boucle principale. */
     while(1)
    {
         Delay10KTCYx(25);
         Delay10KTCYx(25);
         Delay10KTCYx(25);
         Delay10KTCYx(25);
         Delay10KTCYx(25);
         led = led^1;
    }
}

