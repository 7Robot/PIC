/*
* Template PIC18F
* Compiler : Microchip C18
* µC : 18f2550
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
#include <p18f2550.h>
#include <delays.h>


/////*CONFIGURATION*/////
#pragma config FOSC = HS
#pragma config FCMEN = ON
#pragma config IESO = OFF
#pragma config PWRT = OFF
#pragma config BOR = ON
#pragma config BORV = 2
#pragma config VREGEN = ON
#pragma config WDT = OFF
#pragma config MCLRE = ON
#pragma config LPT1OSC = OFF
#pragma config PBADEN = OFF
#pragma config CCP2MX = ON
#pragma config LVP = OFF
#pragma config DEBUG = OFF
#pragma config PLLDIV = 4

/////*CONSTANTES*/////
#define led PORTCbits.RC0

/////*VARIABLES GLOBALES*/////



/////*PROGRAMME PRINCIPAL*/////
void main (void)
{
    /* Initialisations. */
    CMCON   =  0b00000111; /* Désactive les comparateurs. */
    ADCON0  = 0b00000000;  /* Désactive le module A/D. */	
    ADCON1  = 0b00001111;
    WDTCON  = 0 ;
    OSCCON  = 0b01111100;
    UCON    = 0 ;           /* Désactive l'USB. */
    UCFG    = 0b00001000 ;

    /* Configurations. */
    TRISC   = 0b11111000 ; /* Orientation des ports. */
    TRISA   = 0b11000011 ; /* 0 : output , 1 : input. */
    TRISB   = 0b01111111 ;

    
    /*Boucle principale. */
    while(1)
    {
   	Delay10KTCYx(255);
	led = led^1;
    }
}


