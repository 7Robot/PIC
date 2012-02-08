/*
* Template PIC18F
* Compiler : Microchip C18
* µC : 18f2680
* Nov.10 2011
*    ____________      _           _
*   |___  /| ___ \    | |         | |
*      / / | |_/ /___ | |__   ___ | |_
*     / /  |    // _ \| '_ \ / _ \| __|
*    / /   | |\ \ (_) | |_) | (_) | |_
*   /_/    |_| \_\___/|____/ \___/'\__|
*			      7robot.fr
*/

#include <stdio.h>
#include <stdlib.h>
#include <delays.h>
#include <p18f2680.h>


/////*CONFIGURATION*/////
#pragma config OSC = IRCIO67
#pragma config FCMEN = OFF
#pragma config IESO = OFF
#pragma config PWRT = OFF
#pragma config WDT = OFF
#pragma config MCLRE = ON
#pragma config LPT1OSC = OFF
#pragma config PBADEN = OFF
#pragma config DEBUG = OFF
#pragma config XINST = OFF
#pragma config BBSIZ = 1024
#pragma config LVP = OFF

/////*CONSTANTES*/////
#define led PORTCbits.RC0


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

    /* Boucle principale. */
     while(1)
    {
         Delay10KTCYx(25);
         led = led^1;
    }
}

