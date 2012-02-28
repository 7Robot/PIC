/*
* Programme carte alim petit robot
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
#include <delays.h>
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
#define led     PORTCbits.RC0

#define servo   PORTCbits.RC4
#define laser   PORTBbits.RB1
#define temoin  PORTCbits.RC6
#define tempsMin 0.33               //Avec ces valeurs on a 180
#define tempsMax 2.08  //2.44 max
#define omega 7.81 // Vitesse angulaire de rotation du servomoteur


/////*PROTOTYPES*/////
void high_isr(void);
void low_isr(void);

void InterruptServo() ;
void InterruptLaser();
void WriteAngle(int a);
void GetData();
void Mesures();

//unsigned long max(unsigned long a, unsigned long b);
//unsigned long min (unsigned long a, unsigned long b);
//char max(char a, char b);
//char min(char a, char b);


/////*VARIABLES GLOBALES*/////
int i = 0;
CANmsg message;
CANmsg incoming;

char tempo[4] = {0};
int ip = 0;
int hh = 0;
volatile unsigned int angle = 0 ;
int pulse = 0 ;
int dir = 1;
unsigned long temps[5]  = {0};
unsigned long distance[5] = {0};
unsigned long position[5] = {0};
volatile unsigned int timeData[20] = {0};

int nbrepoint = 0;
unsigned long pointMax[5] = {0};
unsigned long pointMin[5]= {0};
int nbreBalises = 0;

char mesures = 0;
char broadcast = 0;

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
    InterruptServo() ;
}

#pragma interrupt low_isr
void low_isr(void)
{
    InterruptLaser();
    // RÈception CAN.
    if(PIE3bits.RXB0IE && PIR3bits.RXB0IF)
    {

        while(CANIsRxReady()) {
            CANReceiveMessage(&incoming.id, incoming.data, &incoming.len, &incoming.flags);
        }

        switch (incoming.id) {
                    case 132: //Renvoyer distace/angle objet
                        CANSendMessage(133,message.data,2*nbreBalises,
                        CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME);
                      break;
                    case 134: //Broadcast OFF
                        broadcast = 0;
                      break;
                    case 135: //Boradcast ON
                        broadcast = 1;
                      break;
                    case 136: //Mesures OFF
                        mesures = 0;
                      break;
                    case 137: //Mesures ON
                        mesures = 1;
                      break;

                    default:
                      // Rien
                      break;
                    }
        led = led ^1;



        PIR3bits.RXB0IF = 0;
    }

}

/////*PROGRAMME PRINCIPAL*/////
void main (void) {
    // Initialisations.
    ADCON1 = 0x0F;
    ADCON0 = 0b00000000;
    WDTCON = 0;

    // Configurations.
    TRISA  = 0b11111111;
    TRISB  = 0b11111111;
    TRISC  = 0b10101110;

    servo = 0 ;

    OpenTimer0(TIMER_INT_OFF & T0_16BIT & T0_SOURCE_INT & T0_PS_1_32 /* Internal oscillator of 20MHz */);
    T0CONbits.TMR0ON = 0; /*On ne d»marre pas le TMR0*/
    WriteTimer0(0);

    OpenTimer3(TIMER_INT_ON & T3_16BIT_RW & T3_SOURCE_INT & T3_PS_1_2 & T3_SYNC_EXT_OFF);

    INTCONbits.TMR0IF = 0; /* Flag de TMR0*/
    INTCON2bits.TMR0IP = 1; /* L'interuption sur TMR0 est en haute priorit»*/

    PIR2bits.TMR3IF = 0; /*Flag de TMR3*/
    IPR2bits.TMR3IP = 1; /*L'interuption de TMR3 est en haute priorit»*/

    INTCON3bits.INT1E = 1; /*Enable interrupt on RB1*/
    INTCON3bits.INT1F = 0; /*External Interrupt Flag bit of RB1*/
    INTCON2bits.INTEDG1 = 1; /* On RB1 : 1:interrupt on risong edge  0:interrupt on falling edge */
    INTCON3bits.INT1IP = 0;  /*INT1 is a low level interrup*/


    // Configuration du CAN.
    CANInitialize(1, 5, 7, 6, 2, CAN_CONFIG_VALID_STD_MSG);
    // Configuration des masques et filtres.
    CANSetOperationMode(CAN_OP_MODE_CONFIG);
    // Set Masks values.
    CANSetMask(CAN_MASK_B1, 0b00010000000, CAN_CONFIG_STD_MSG);
    CANSetMask(CAN_MASK_B2, 0xFFFFFF, CAN_CONFIG_STD_MSG);
    // Set Buffer 1 Filter values.
    CANSetFilter(CAN_FILTER_B1_F1, 0b00010000000, CAN_CONFIG_STD_MSG);
    CANSetFilter(CAN_FILTER_B1_F2, 0b00010000000, CAN_CONFIG_STD_MSG);
    // Set CAN module into Normal mode.
    CANSetOperationMode(CAN_OP_MODE_NORMAL);

    // Interruption Buffer 0.
    IPR3bits.RXB0IP = 0; // Priorit» basse.
    PIE3bits.RXB0IE = 1; // activÈe
    PIR3bits.RXB0IF = 0;
    // Interruptions Buffer 1.
    PIE3bits.RXB1IE = 0; // Interdite.


    mesures = 0; //mesures off au dÈmarrage

    // Signal de d»marrage du programme.
    led = 0;
    for(i = 0; i < 20; i++) {
        led = led ^ 1;
        DelayMS(50);
    }

    //Autorisation des interruptions
    RCONbits.IPEN = 1;
    INTCONbits.GIEH = 1;
    INTCONbits.GIEL = 1;


    while(1) {
        Mesures(mesures);
    }
}


void WriteAngle(int a)
{
    angle = a;
    WriteTimer0(0); /*On initialise TMR1*/

    for (ip = 0; ip < 20; ip++)
    {
        timeData[ip] = 0;
    }

    ip = 0; /*On initialise le tableau timeData[] sur la premiÀre case*/
    T0CONbits.TMR0ON = 1; /*On d»marre le TMR0 pour le tableau timeData[]*/
}

void GetData()
{
   T0CONbits.TMR0ON = 0;
   
   nbrepoint = ip;
   pointMin[0] = timeData[0];
   pointMax[0] = timeData[1];

   nbreBalises = 1;
   ip = 1;
   while((timeData[2*ip] - timeData[2*ip - 1])*6.4 < 800 && (2*ip) < nbrepoint)
   {
        pointMax[0] = timeData[2*ip + 1];
        ip++;
   }
   temps[0] = (pointMax[0] - pointMin[0])*6.4;
   if (2*ip < nbrepoint)
   {
        pointMin[1] = timeData[2*ip];
        pointMax[1] = timeData[2*ip + 1];
        ip++;
        while((timeData[2*ip] - timeData[2*ip - 1])*6.4 < 800 && (2*ip) < nbrepoint)
        {
            pointMax[1] = timeData[2*ip + 1];
            ip++;
        }
        temps[1] = (pointMax[1] - pointMin[1])*6.4;
        nbreBalises = 2;
        if (2*ip < nbrepoint)
        {
            pointMin[2] = timeData[2*ip];
            pointMax[2] = timeData[2*ip + 1];
            ip++;
            while((timeData[2*ip] - timeData[2*ip - 1])*6.4 < 800 && (2*ip) < nbrepoint)
            {
                pointMax[2] = timeData[2*ip + 1];
                ip++;
            }
            temps[2] = (pointMax[2] - pointMin[2])*6.4;
            nbreBalises = 3;
            if (2*ip < nbrepoint)
            {
                pointMin[3] = timeData[2*ip];
                pointMax[3] = timeData[2*ip + 1];
                ip++;
                while((timeData[2*ip] - timeData[2*ip - 1])*6.4 < 800 && (2*ip) < nbrepoint)
                {
                    pointMax[3] = timeData[2*ip + 1];
                    ip++;
                }
                temps[3] = (pointMax[3] - pointMin[3])*6.4;
                nbreBalises = 4;
            }
         }
    }


   for(hh = 0; hh < nbreBalises; hh++)
   {
       distance[hh] = 6.7/(2*omega*temps[hh]*0.000001/2); //distance en cm
       position[hh] =  (omega * (pointMax[hh] + pointMin[hh])/2 * 6.4 * 0.000001)*180/3.14159;
       if (angle == 0)
           position[hh] = 180 - position[hh];
       message.data[2*hh] = (char)distance[hh];
       tempo[hh] = (char)distance[hh];
       message.data[2*hh + 1] = (char)position[hh];
   }
   
   /*for(hh = 0; hh < nbreBalises; hh++)
   {
       message.data[hh] = temps[hh];
   }

*/
   if(broadcast)
   {
    CANSendMessage(133,message.data,2*nbreBalises,    /*ATTENTION Remttre 2*nbreBalises pour aprËs !!!*/
                       CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME);
   }

}

void Mesures(int a)
{
    if(a)
    {   TRISCbits.RC4 = 0;
        WriteAngle(0);
        DelayMS(500);
        GetData();
        WriteAngle(180);
        DelayMS(500);
        GetData();
    }
    else
    {
        TRISCbits.RC4 = 1;
    }
}


void InterruptServo()
{
    if(PIE2bits.TMR3IE && PIR2bits.TMR3IF)
     {
        if (pulse == 1)
            {
            WriteTimer3(65535 - tempsMin*65535/26.214  - angle * (65535/26.214) * (tempsMax - tempsMin) / 180);
            servo = 1;
            pulse = 0;
            }
     else
            {
            WriteTimer3(15535); // 20ms
             servo = 0;
             pulse = 1;
            }
        PIR2bits.TMR3IF = 0;
    }
}

void InterruptLaser()
{
    if (INTCON3bits.INT1E && INTCON3bits.INT1F)
    {
        //temoin ^= 1;
        timeData[ip++] = ReadTimer0();
        INTCON2bits.INTEDG1 ^= 1;
        INTCON3bits.INT1F = 0;
    }
 }


//unsigned long max(unsigned long a, unsigned long b)
//{
//    if(a >= b)
//        return a;
//    return b;
//}
//
//unsigned long min(unsigned long a, unsigned long b)
//{
//    if(a<= b)
//        return a;
//    return b;
//}
//
//char max (char a, char b)
//{
//    if(a >= b)
//        return a;
//    return b;
//}
//
//char min (char a, char b)
//{
//    if(a <= b)
//        return a;
//    return b;
//}


/*
 * TODO
 * - arrÍt/dÈpart sur balise
*/