/*
* Programme odometrie petit robot
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
#include <portb.h>


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


#define GchA PORTBbits.RB0
#define GchB PORTBbits.RB4
#define DchA PORTBbits.RB1
#define DchB PORTBbits.RB5

#define riseGchA INTCON2bits.INTEDG0
#define riseDchA INTCON2bits.INTEDG1

#define led PORTAbits.RA5

/////*PROTOTYPES*/////
void high_isr(void);
void low_isr(void);

/////*VARIABLES GLOBALES*/////

long gTicks = 0, dTicks =0 ;
char Gsens = 1, Dsens =1;
char prevGchB = 0, prevDchB = 0;

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
    led = led^1;
    if(INTCONbits.TMR0IE && INTCONbits.TMR0IF)
    {
        led = led^1;
        INTCONbits.TMR0IF = 0;
    }

   if(INTCONbits.RBIE && INTCONbits.RBIF && prevGchB != GchB)
   {
       prevGchB = GchB;
       gTicks++;
       INTCONbits.RBIF = 0;
   }

    INTCONbits.RBIF = 0;

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
    TRISB   = 0xFF ;
    TRISC   = 0b11111000 ;
    
    OpenTimer0(TIMER_INT_OFF & T0_SOURCE_INT & T0_16BIT & T0_PS_1_8);


      /*Interruptions portB*/
    OpenRB0INT( PORTB_CHANGE_INT_OFF & RISING_EDGE_INT & PORTB_PULLUPS_OFF);
    OpenRB1INT( PORTB_CHANGE_INT_OFF & RISING_EDGE_INT & PORTB_PULLUPS_OFF);
    OpenPORTB( PORTB_CHANGE_INT_ON & PORTB_PULLUPS_OFF);

    /*TODO : siganl de démarrage comme dans asservP.X !!! */

    INTCONbits.GIE = 1; /* Autorise interruptions. */

    /* Boucle principale. */
     while(1)
    {

    }
}

