/*
* Programme ARM petit robot
* Eurobot 2012
* Compiler : Microchip C18
* µC : 18f2680
* Jan.21 2011
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
#include <timers.h>
#include <p18f2680.h>


/////*CONFIGURATION*/////
#pragma config OSC = HS
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
#define XTAL    20000000
#define led     PORTAbits.RA5
/*
#define GchA    
#define GchB
#define DchA
#define DchB
*/

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
    if(INTCONbits.TMR0IE && INTCONbits.TMR0IF)
    {
        led = led^1;
        INTCONbits.TMR0IF = 0;
    }

}

#pragma interrupt low_isr
void low_isr(void)
{

}


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
    
    OpenTimer0(TIMER_INT_ON & T0_SOURCE_INT & T0_16BIT & T0_PS_1_8);

    INTCONbits.GIE = 1; /* Autorise interruptions. */

    /* Boucle principale. */
     while(1)
    {

    }
}

