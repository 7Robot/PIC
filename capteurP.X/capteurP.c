/*
* Programme carte capteur petit robot
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
#include <usart.h>


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
#define led     PORTAbits.RA4
#define arriere   PORTCbits.RC2
#define avant PORTCbits.RC3


/////*PROTOTYPES*/////
void high_isr(void);
void low_isr(void);
void DelayMS(int delay);

/////*VARIABLES GLOBALES*/////
int i=0;
char message[8]="";
char message1[8]="un";
char message2[8]="deux";
char message3[8]="trois";
char messagetest[8] = "ENSEE";
char lengh = 0;
char prevC;

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
        if(PORTC != prevC)
        {
            if(!arriere)
            {
                led = led^1;
                CANSendMessage(257,&prevC,1,
                        CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME );
            }
            else if(!avant)
            {
                led = led^1;
                CANSendMessage(258,&prevC,1,
                        CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME );
            }
        }
        prevC = PORTC;
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
    TRISA   = 0b11101111 ;
    TRISB   = 0b01111111 ;
    TRISC   = 0b11111111 ;

    /*Configuration du CAN*/
    CANInitialize(1,5,7,6,2,CAN_CONFIG_VALID_STD_MSG);
    Delay10KTCYx(200);

    /*Timer de rafraichissement des BP*/
    OpenTimer0( TIMER_INT_ON & T0_8BIT & T0_SOURCE_INT & T0_PS_1_256 ); //76Hz

   /* // Interruptions Buffer1
    IPR3bits.RXB1IP=1;// : priorité haute par defaut du buff 1
    PIE3bits.RXB1IE=1;//autorise int sur buff1
    PIR3bits.RXB1IF=0;//mise a 0 du flag

    // Interruption Buffer 0
    IPR3bits.RXB0IP=1;// : priorité haute par defaut du buff 1
    PIE3bits.RXB0IE=1;//autorise int sur buff1
    PIR3bits.RXB0IF=0;//mise a 0 du flag*/


    // Configuration des masques et filtres
    // Set CAN module into configuration mode
    CANSetOperationMode(CAN_OP_MODE_CONFIG);
    // Set Buffer 1 Mask value
    CANSetMask(CAN_MASK_B1, 0b1111,CAN_CONFIG_STD_MSG);
    // Set Buffer 2 Mask value
    CANSetMask(CAN_MASK_B2, 0b1111,CAN_CONFIG_STD_MSG );
    // Set Buffer 1 Filter values
    CANSetFilter(CAN_FILTER_B1_F1,0b0011,CAN_CONFIG_STD_MSG );
    CANSetFilter(CAN_FILTER_B1_F2,0b0000,CAN_CONFIG_STD_MSG );
    CANSetFilter(CAN_FILTER_B2_F1,0b1100,CAN_CONFIG_STD_MSG );
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

    led = 0;
    prevC = PORTC;

    INTCONbits.GIE = 1; /* Autorise interruptions. */
    

    /* Boucle principale. */
     while(1)
    {

         /*Programme de test pour flooder le bus.*/
         /*DelayMS(500);
         while(!CANIsTxReady());
         lengh = 2;
         CANSendMessage(0b00000000001,message1,lengh,CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME );
         led = led^1;
         DelayMS(500);
         while(!CANIsTxReady());
         lengh = 4;
         CANSendMessage(0b00000000010,message2,lengh,CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME );
         led = led^1;
         DelayMS(500);
         while(!CANIsTxReady());
         lengh = 5;
         CANSendMessage(0b00000000100,message3,lengh,CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME );
         led = led^1;
         DelayMS(500);
         while(!CANIsTxReady());
         lengh = 5;
         CANSendMessage(0xAA,messagetest,lengh,CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME );
         led = led^1;*/

    

     }
}

///// Définition des fonctions du programme. /////


