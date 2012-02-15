/*
* Programme PIC ARM petit robot
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
#include <usart.h>
#include <delays.h>
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
#define on      PORTAbits.RA1

#define attenteFD 0
#define attenteSize 1
#define attenteIdL 2
#define collecteData 3
#define verifBF 4


/////*PROTOTYPES*/////
void high_isr(void);
void low_isr(void);


/////*VARIABLES GLOBALES*/////
int i=0;


char rescp = 0; /*Compteur de réception USART. */
char Rstate =0; /*Etat de réception*/
char incoming=0; /*Byte entrant */



/*Buffers de réception.*/
CANmsg INbuffer[4];
CANmsg OUTbuffer[4];

CANmsg imessage;
CANmsg * pimessage;

CANmsg umessage;
CANmsg * pumessage;

CANmsg message;
CANmsg * pmessage;

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

    if(PIE1bits.RCIE && PIR1bits.RCIF)
    {
        // [ FD ] [ size | 0 | id10..8 ] [ id7..0] [ M1 ] [ M2 ] ? [ M8 ] [ BF ]
        incoming = ReadUSART();
        if(incoming == 0xFD && Rstate == attenteFD)
        {
            Rstate = attenteSize;
        }
        else if(Rstate == attenteSize)
        {
            umessage.len = (incoming & 0xF0) >> 4; //TESTER 0 ET LEN
            ((char*)&umessage.id)[1] = incoming & 0b00000111;
            Rstate = attenteIdL;
        }
        else if(Rstate == attenteIdL)
        {
            *(char*)&umessage.id = incoming ;
            Rstate = collecteData;
            rescp = 0;
        }
        else if(Rstate == collecteData)
        {
            umessage.data[rescp] = incoming ;
            rescp ++;
            if(rescp >=umessage.len)
            {
                Rstate = verifBF ;
            }
        }
        else if(Rstate == verifBF)
        {
            if(incoming == 0xBF)
            {
                pumessage = TrouverPlace(INbuffer);
                *pumessage = umessage;
                if(PlacesRestantes(INbuffer)==0)
                    printf("INbFULL!\n");
            }
            Rstate=attenteFD;
        }

        PIR1bits.RCIF = 0;
    }

        if(PIE3bits.RXB0IE && PIR3bits.RXB0IF)
    {

        /*On stocke la valeur. */
        while(CANIsRxReady())
        {
            CANReceiveMessage(&imessage.id,imessage.data,
                          &imessage.len,&imessage.flags);
        }

        /*On cherche une place dans la buffer... et on y place */
        pimessage = TrouverPlace(OUTbuffer);
        *pimessage = imessage;

        if(PlacesRestantes(OUTbuffer)==0)
            printf("OUTbFULL!\n");

        PIR3bits.RXB0IF=0;
        PIR3bits.RXB1IF=0;
        PIR3bits.ERRIF=0;

    }

    if(PIE3bits.ERRIE && PIR3bits.ERRIF)
    {
        /*On stocke la valeur. */
        while(CANIsRxReady())
        {
            CANReceiveMessage(&imessage.id,imessage.data,
                          &imessage.len,&imessage.flags);
        }


        //On stocke le messgae mais on n'incremente pas le buffer
        PIR3bits.RXB0IF=0;
        PIR3bits.RXB1IF=0;
        PIR3bits.ERRIF=0;
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
    TRISA   = 0b11111100 ;
    TRISB   = 0b01111111 ;
    TRISC   = 0b11111111 ;
    on = 0;
    /*Configuration du port série*/
    OpenUSART( USART_TX_INT_OFF & USART_RX_INT_ON
                & USART_ASYNCH_MODE & USART_EIGHT_BIT
                & USART_CONT_RX & USART_BRGH_HIGH, 10); //115200, 1,2% err...

    /*Configuration du CAN*/
    CANInitialize(1,5,7,6,2,CAN_CONFIG_VALID_STD_MSG);
    Delay10KTCYx(200);

    // Interruptions Buffer1
/*    IPR3bits.RXB1IP=1;// : priorité haute par defaut du buff 1
    PIE3bits.RXB1IE=1;//autorise int sur buff1
    PIR3bits.RXB1IF=0;//mise a 0 du flag*/

    // Interruption Buffer 0
    IPR3bits.RXB0IP=1;// : priorité haute par defaut du buff 0
    PIE3bits.RXB0IE=1;//autorise int sur buff0
    PIR3bits.RXB0IF=0;//mise a 0 du flag
    PIR3bits.ERRIF=0;
    PIE3bits.ERRIE =1;

    // Configuration des masques et filtres
    // Set CAN module into configuration mode
    CANSetOperationMode(CAN_OP_MODE_CONFIG);
    // Set Buffer 1 Mask value
    CANSetMask(CAN_MASK_B1, 0b0,CAN_CONFIG_STD_MSG);
    // Set Buffer 2 Mask value
    CANSetMask(CAN_MASK_B2, 0b0,CAN_CONFIG_STD_MSG );
    // Set Buffer 1 Filter values
    CANSetFilter(CAN_FILTER_B1_F1,0b0000,CAN_CONFIG_STD_MSG );
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
    led = 0;

    ResetBuffer(OUTbuffer);
    ResetBuffer(INbuffer);

    printf("Reset ARM!\n");
    on = 1;

    INTCONbits.GIE = 1; /* Autorise interruptions. */
    INTCONbits.PEIE = 1;


    /* Boucle principale. */
     while(1)
    {
         /**/
         if(PlacesRestantes(INbuffer) < 4)
         {
             pmessage = TrouverMessage(INbuffer);
             message = *pmessage;
             led = led^1;
              while(CANIsTxReady())
              {
                CANSendMessage(message.id,message.data,
                        message.len,CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME );
              }
             pmessage->len = 0;
         }

         /* On envoie 1 message du buffer USART*/
         if(PlacesRestantes(OUTbuffer) < 4)
         {
             pmessage = TrouverMessage(OUTbuffer);
             led = led^1;
             CANtoUSART(pmessage);
             pmessage->len = 0;
         }
    }
}

///// Définition des fonctions du programme. /////









