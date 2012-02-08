/*
* Programme espion Xbee petit robot
* Eurobot 2012
* Compiler : Microchip C18
* �C : 18f2680
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
#include <usart.h>
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
#define led     PORTAbits.RA0


/////*PROTOTYPES*/////
void high_isr(void);
void low_isr(void);
void DelayMS(int delay);

/////*VARIABLES GLOBALES*/////
int i=0;
char * p;
long id=0;
char data[8]= "";
char len=0;
enum CAN_RX_MSG_FLAGS flag;

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
    ADCON1 = 0x0F ;
    ADCON0 = 0b00000000;
    WDTCON = 0 ;

    /* Configurations. */
    TRISA   = 0b11111100 ;
    TRISB   = 0b01111111 ;
    TRISC   = 0b11111111 ;

    /*Configuration du port s�rie*/
    OpenUSART( USART_TX_INT_OFF & USART_RX_INT_OFF
                & USART_ASYNCH_MODE & USART_EIGHT_BIT
                & USART_CONT_RX & USART_BRGH_HIGH, 10); //115200, 1,2% err...

    /*Configuration du CAN*/
    CANInitialize(1,5,7,6,2,CAN_CONFIG_VALID_STD_MSG);
    Delay10KTCYx(200);

    // Interruptions Buffer1
    IPR3bits.RXB1IP=1;// : priorit� haute par defaut du buff 1
    PIE3bits.RXB1IE=1;//autorise int sur buff1
    PIR3bits.RXB1IF=0;//mise a 0 du flag

    // Interruption Buffer 0
    IPR3bits.RXB0IP=1;// : priorit� haute par defaut du buff 1
    PIE3bits.RXB0IE=1;//autorise int sur buff1
    PIR3bits.RXB0IF=0;//mise a 0 du flag


    // Configuration des masques et filtres
    // Set CAN module into configuration mode
    CANSetOperationMode(CAN_OP_MODE_CONFIG);
    // Set Buffer 1 Mask value
    CANSetMask(CAN_MASK_B1, 0b1111,CAN_CONFIG_STD_MSG);
    // Set Buffer 2 Mask value
    CANSetMask(CAN_MASK_B2, 0b0,CAN_CONFIG_STD_MSG );
    // Set Buffer 1 Filter values
    CANSetFilter(CAN_FILTER_B1_F1,0b0011,CAN_CONFIG_STD_MSG );
    CANSetFilter(CAN_FILTER_B1_F2,0b0000,CAN_CONFIG_STD_MSG );
    CANSetFilter(CAN_FILTER_B2_F1,0b0000,CAN_CONFIG_STD_MSG );
    CANSetFilter(CAN_FILTER_B2_F2,0b0000,CAN_CONFIG_STD_MSG );
    CANSetFilter(CAN_FILTER_B2_F3,0b0000,CAN_CONFIG_STD_MSG );
    CANSetFilter(CAN_FILTER_B2_F4,0b0000,CAN_CONFIG_STD_MSG );
    // Set CAN module into Normal mode
    CANSetOperationMode(CAN_OP_MODE_NORMAL);


    /* Signal de d�marrage du programme. */
    led = 0;
    for(i=0;i<20;i++)
    {
        led=led^1;
        DelayMS(50);
    }
    led = 0;

    INTCONbits.GIE = 0; /* Autorise interruptions. */
    INTCONbits.PEIE = 0;

    printf("Debut du programme !\n");

    /* Boucle principale. */
     while(1)
    {

            

            if(CANIsRxReady()){
            CANReceiveMessage(&id,data,&len,&flag);
            /*
            i = id;
            printf("id : %d ,",i);
            printf("m : %s \n",data);
             */
            led = led^1;
            
            i = len ;
            len = len << 4 ;
            p = &id; //conversion de pointeur
            p+= 3; // on pointe sur la fin du long
            len = len | *p;
            p++; // fin du pointeur

            WriteUSART(0xFD);
            WriteUSART(len);
            WriteUSART(*p);
            p = data;
            while(i>1)
            {
                WriteUSART(*p);
                p++;
                i--;
            }
            WriteUSART(0xBF);
            }
            
            //DelayMS(800);
        

    }
}

///// D�finition des fonctions du programme. /////
void DelayMS(int delay)
{
    /*Attente en ms, sur cette carte c'est utile, et vu que le Quart est soud�,
     il y a peu de raisons pour que �a change...*/
    int cp = 0;
    for(cp=0; cp<delay; cp++)
    {
        Delay1KTCYx(5);
    }
}

