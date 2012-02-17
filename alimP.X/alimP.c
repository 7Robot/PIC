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
#include "../libcan/can18xx8.h"

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
#define led     PORTCbits.RC0


/////*PROTOTYPES*/////
void high_isr(void);
void low_isr(void);
void DelayMS(int delay);

/////*VARIABLES GLOBALES*/////
int i=0;
CANmsg message;

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
    if(PIE3bits.RXB0IE && PIR3bits.RXB0IF)
    {

        /*On stocke la valeur. */
        while(CANIsRxReady())
        {
            CANReceiveMessage(&message.id,message.data,
                          &message.len,&message.flags);
        }
        led = led^1;


        PIR3bits.RXB0IF=0;
        PIR3bits.RXB1IF=0;
        PIR3bits.ERRIF=0;

    }

    if(PIE3bits.ERRIE && PIR3bits.ERRIF)
    {
        /*On stocke la valeur. */
        while(CANIsRxReady())
        {
            CANReceiveMessage(&message.id,message.data,
                          &message.len,&message.flags);
        }

        led = led^1;


        //On stocke le messgae mais on n'incremente pas le buffer
        PIR3bits.RXB0IF=0;
        PIR3bits.RXB1IF=0;
        PIR3bits.ERRIF=0;
    }


}


/////*PROGRAMME PRINCIPAL*/////
void main (void)
{
    /* Initialisations. */
    ADCON1 = 0x0F ;
    ADCON0 = 0b00000000;
    WDTCON = 0 ;

    /* Configurations. */
    TRISA   = 0xFF ;
    TRISB   = 0xFF ;
    TRISC   = 0b11111110 ;

    // Interruptions Buffer1
/*    IPR3bits.RXB1IP=1;// : priorité haute par defaut du buff 1
    PIE3bits.RXB1IE=1;//autorise int sur buff1
    PIR3bits.RXB1IF=0;//mise a 0 du flag*/

    // Interruption Buffer 0
    IPR3bits.RXB0IP=0;// : priorité basse par defaut du buff 0
    PIE3bits.RXB0IE=1;//autorise int sur buff0
    PIR3bits.RXB0IF=0;//mise a 0 du flag
    PIR3bits.ERRIF=0; //flag erreur a 0
    PIE3bits.ERRIE=1; //autorise int erreur

       // Configuration des masques et filtres
    // Set CAN module into configuration mode
    CANSetOperationMode(CAN_OP_MODE_CONFIG);
    // Set Buffer 1 Mask value
    CANSetMask(CAN_MASK_B1, 0b00010000000,CAN_CONFIG_STD_MSG);
    // Set Buffer 2 Mask value
    CANSetMask(CAN_MASK_B2, 0xFFFFFF ,CAN_CONFIG_STD_MSG );
    // Set Buffer 1 Filter values
    CANSetFilter(CAN_FILTER_B1_F1,0b00010000000,CAN_CONFIG_STD_MSG );
    CANSetFilter(CAN_FILTER_B1_F2,0b0000,CAN_CONFIG_STD_MSG );
    CANSetFilter(CAN_FILTER_B2_F1,0b0000,CAN_CONFIG_STD_MSG );
    CANSetFilter(CAN_FILTER_B2_F2,0b0000,CAN_CONFIG_STD_MSG );
    CANSetFilter(CAN_FILTER_B2_F3,0b0000,CAN_CONFIG_STD_MSG );
    CANSetFilter(CAN_FILTER_B2_F4,0b0000,CAN_CONFIG_STD_MSG );
    // Set CAN module into Normal mode
    CANSetOperationMode(CAN_OP_MODE_NORMAL);


    /* Signal de démarrage du programme. */
    led = 0;
    for(i=0;i<20;i++)
    {
        led=led^1;
        DelayMS(50);
    }

    INTCONbits.GIE = 0; /* Autorise interruptions. */
    led = 0;


    /* Boucle principale. */
     while(1)
    {

    }
}

///// Définition des fonctions du programme. /////


