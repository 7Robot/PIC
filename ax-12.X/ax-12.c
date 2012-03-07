/*
 * Programme de test des servos AX-12
 * Eurobot 2012
 * Compiler : Microchip C18
 * µC : 18f25K80
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
#include <delays.h>
#include <p18f25k80.h>
#include <usart.h>
#include "ax12.h"


/////*CONFIGURATION*/////
#pragma config RETEN = OFF
#pragma config XINST = OFF
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
#pragma config MSSPMSK = MSK5
#pragma config MCLRE = ON
#pragma config STVREN = ON
#pragma config BBSIZ = BB2K



/////*Prototypes*/////
void DelayMS(int delay);
void high_isr(void);
void low_isr(void);


/////*CONSTANTES*/////
char uincoming = 0;

/////*VARIABLES GLOBALES*/////
unsigned int i = 0;
char x = 100;


/////*Interruptions*/////
#pragma code high_vector=0x08

void high_interrupt(void) {
    _asm GOTO high_isr _endasm
}
#pragma code low_vector=0x18

void low_interrupt(void) {
    _asm GOTO low_isr _endasm
}
#pragma code

#pragma interrupt high_isr

void high_isr(void) {
    InterruptAX();
}

#pragma interrupt low_isr

void low_isr(void) {
   Nop();
}


/////*PROGRAMME PRINCIPAL*/////

void main(void) {
    /* Initialisations. */
    ADCON1 = 0x0F;
    ADCON0 = 0b00000000;
    ANCON1 = 0x00;
    WDTCON = 0;

    /* Configurations. */
    TRISA = 0b11111100;
    TRISB = 0xFF;
    TRISC = 0b10001111;
    LATC = 0xFF;

    /*Configuration module USART*/
    Open1USART(USART_TX_INT_OFF & USART_RX_INT_ON
            & USART_ASYNCH_MODE & USART_EIGHT_BIT
            & USART_CONT_RX & USART_BRGH_HIGH, 129); //9600, 0,2% err...
    SetRX();

    /*Interrupts*/
    INTCONbits.PEIE = 1;
    INTCONbits.GIEH = 1;
    INTCONbits.GIEL = 1;


    while (1)
    {
        //PutAX(2 + (i % 2), AX_LED, (i / 2) % 2);
        PutAX(3, AX_LED, i);
        i++;
        DelayMS(1000);
    }

}

///// Définition des fonctions du programme. /////

void DelayMS(int delay) {
    /*Attente en ms, sur cette carte c'est utile, et vu que le Quart est soudé,
     il y a peu de raisons pour que ça change...*/
    int cp = 0;
    for (cp = 0; cp < delay; cp++) {
        Delay1KTCYx(5); // 20Mhz pour les tests
    }
}
