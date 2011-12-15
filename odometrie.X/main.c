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
#include <p18f2550.h>
#include <string.h>
#include <delays.h>
#include <math.h>


/////*CONFIGURATION*/////
#pragma config FOSC = HSPLL_HS
//#pragma config FOSC = INTOSC_HS
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
#pragma config PLLDIV = 5
#pragma config CPUDIV = OSC1_PLL2

/////*CONSTANTES*/////
#define led PORTAbits.RA0
#define DTHETADEMI 0.000115
#define DTHETATOT  0.00023
#define DHDEMI 0.00115
#define DHTOT  0.0023

/////*PROTOTYPES*/////
void high_isr(void);
void low_isr(void);

/////*VARIABLES GLOBALES*/////
unsigned char AD = 0, oldAD = 0, BD = 0, oldBD = 0;
unsigned char AG = 0, oldAG = 0, BG = 0, oldBG = 0;
unsigned char IN = 0;
unsigned char calculer = 0;
volatile char sensD = 0, sensG = 0;
volatile  long nbTICSD = 0, nbTICSG = 0, nbTICS = 0;
unsigned char rgChaine = 0;
char chaine[60] ;
float nombre = 1.0;
volatile float mX = 0, mY = 0, THETA = 0, CTHETA = 1, STHETA = 0;


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
    if (INTCONbits.RBIE && INTCONbits.RBIF)//modification de l'etat du portB
    {
        IN = PORTB;

        oldAD = AD;
        oldBD = BD;
        oldAG = AG;
        oldBG = BG;
        AD = IN & 0b01000000;
        BD = IN & 0b10000000;
        AG = IN & 0b00010000;
        BG = IN & 0b00100000;

        //analyse des modifications
        if (AD != oldAD)
        {
            if (BD != oldBD)
            {
                sensD = 0;
            }
            else if (AD)
            {
                if (BD)
                    sensD = 2;
                else
                    sensD = 1;
            }
            else
            {
                if (BD)
                    sensD = 1;
                else
                    sensD = 2;
            }
        }
        else if (BD != oldBD)
        {
            led = 1;
            if (BD)
            {
                if (AD)
                    sensD = 1;
                else
                    sensD = 2;
            }
            else
            {
                if (AD)
                    sensD = 2;
                else
                    sensD = 1;
            }
        }
        else
            sensD = 0;
        if (AG != oldAG)
        {
            if (BG != oldBG)
                sensG = 0;
            else if (AG)
            {
                if (BG)
                    sensG = 2;
                else
                    sensG = 1;
            }
            else
            {
                if (BG)
                    sensG = 1;
                else
                    sensG = 2;
            }
        }
        else if (BG != oldBG)
        {
            if (BG)
            {
                if (AG)
                    sensG = 1;
                else
                    sensG = 2;
            }
            else
            {
                if (AG)
                    sensG = 2;
                else
                    sensG = 1;
            }
        }
        else
            sensG = 0;


        //mis à jour des variables "positions"
        if (sensD == 1)
        {
            nbTICSD++;
            if (sensG == 1)
            {
                nbTICSG++;
                mX += DHTOT * CTHETA;
                mY += DHTOT * STHETA;
            }
            else if (sensG == 2)
            {
                nbTICSG--;
                THETA += DTHETATOT;
            }
            else
            {
                THETA += DTHETADEMI;
                mX += DHDEMI * CTHETA;
                mY += DHDEMI * STHETA;
            }
        }
        else if (sensD == 2)
        {
            nbTICSD--;
            if (sensG == 2)
            {
                nbTICSG--;
                mX -= DHTOT * CTHETA;
                mY -= DHTOT * STHETA;
            }
            else if (sensG == 1)
            {
                nbTICSG++;
                THETA -= DTHETATOT;
            }
            else
            {
                THETA -= DTHETADEMI;
                mX -= DHDEMI * CTHETA;
                mY -= DHDEMI * STHETA;
            }
        }
        else if (sensG == 1)
        {
            nbTICSG++;
            THETA -= DTHETADEMI;
            mX += DHDEMI * CTHETA;
            mY += DHDEMI * STHETA;
        }
        else if (sensG == 2)
        {
            nbTICSG--;
            THETA += DTHETADEMI;
            mX -= DHDEMI * CTHETA;
            mY -= DHDEMI * STHETA;
        }
        calculer = 2;

        INTCONbits.RBIF = 0;

    }
}

#pragma interrupt low_isr
void low_isr(void)
{
    if  (INTCONbits.TMR0IE && INTCONbits.TMR0IF)
    {
        //led = 1-led;
        //sprintf (chaine, "coucou\n");
        //sprintf(chaine, "D=%ld\nG=%ld\nT=%ld\n\n", nbTICSD, nbTICSG, nbTICS);
        sprintf(chaine, "X = %ld \nY = %ld \ntheta = %ld \nD=%ld\nG=%ld\nT=%ld\n\n\n", (long)(mX*100), (long)(mY*100), (long)(THETA*100),nbTICSD, nbTICSG, nbTICS );

        rgChaine = 0;
        TXSTAbits.TXEN = 1;
        INTCONbits.TMR0IF = 0;
    }

    if (PIE1bits.TXIE && PIR1bits.TXIF)
    {
        if (chaine[rgChaine] != NULL)
        {
            TXREG = chaine[rgChaine];
        }
        else
        {
            TXSTAbits.TXEN = 0;
        }
        PIR1bits.TXIF = 0;
        rgChaine ++;
    }

}


/////*PROGRAMME PRINCIPAL*/////
void main (void)
{
    /* Initialisations. */
    CMCON   = 0b00000111; /* Désactive les comparateurs. */
    ADCON0  = 0b00000000;  /* Désactive le module A/D. */
    ADCON1  = 0b00001111;
    WDTCON  = 0 ;
    OSCCON  = 0b01111100;
    UCON    = 0 ;           /* Désactive l'USB. */
    UCFG    = 0b00001000 ;

    /* Configurations. */
    TRISA   = 0b11000010 ;
    TRISB   = 0b11111111 ;
    TRISC   = 0b10111111 ;

    SPBRG   = 76;
    TXSTA   = 0b00000010 ;
    RCSTA   = 0b10000000 ;
    BAUDCON = 0b00000000 ;

    IPR1bits.TXIP = 0;  // donne la priorité basse à l'interruption de la transmission série
    PIE1bits.TXIE = 1;  //active l'interruption


    RCONbits.IPEN = 1;          //autorise differents niveaux d'interuption

    INTCONbits.GIE = 1; /* Autorise interruptions. */
    INTCONbits.PEIE =1;
    INTCONbits.TMR0IE = 1;      //autorise l'interupt TMR0
    INTCON2bits.TMR0IP = 0;     //priorité basse pour TMR0

    INTCONbits.RBIE = 1;        //autorise l'interuption de changement du port B (RB4->RB7)
    INTCON2bits.RBIP = 1;       //donne prioirté haute aux modif de bort B 4->7
    INTCON2bits.RBPU = 1;       //desactive les pull-UP du portB



    T0CON = 0b10000111;

    /* Boucle principale. */
    led =1;
     while(1)
    {
         if (calculer)
         {
             CTHETA = cos(THETA);
             calculer--;
         }
         if (calculer)
         {
             STHETA = sin(THETA);
             calculer--;
         }
         //led = 1-led;

    }
}
