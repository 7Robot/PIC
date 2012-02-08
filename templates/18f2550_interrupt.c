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

/////*PROTOTYPES*/////
void high_isr(void);
void low_isr(void);

/////*VARIABLES GLOBALES*/////


/////*INTERRUPTIONS*/////

#pragma code high_vector=0x08
void high_interrupt(void)
{
     _asm GOTO high_isr _endasm
}
#pragma code low_vector=0x18
void low_interrupt(void)
{
     _asm GOTO low_isr _endasm
}
#pragma code

#pragma interrupt high_isr
void high_isr(void)
{

}

#pragma interrupt low_isr
void low_isr(void)
{

}


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
    TRISA   = 0b11000011 ;
    TRISB   = 0b01111111 ;
    TRISC   = 0b11111000 ;
    

    INTCONbits.GIE = 1; /* Autorise interruptions. */

    /* Boucle principale. */
     while(1)
    {

    }
}

