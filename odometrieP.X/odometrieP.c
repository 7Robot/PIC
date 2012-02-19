/*
* Programme PIC carte d'alim petit robot
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
#include <p18f2680.h>
#include <delays.h>
#include <timers.h>
#include <math.h>
#include <portb.h>
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
#define led     PORTAbits.RA5

// Calibration.
#define DTHETA  (2 * 3.14159265 / 16300.) // radian
#define DL      (51. / 7050.) // cm

#define RISE_gA INTCON2bits.INTEDG0
#define RISE_dA INTCON2bits.INTEDG1

#define PIN_gA PORTBbits.RB0
#define PIN_gB PORTBbits.RB4
#define PIN_dA PORTBbits.RB1
#define PIN_dB PORTBbits.RB5

/////*PROTOTYPES*/////
void high_isr(void);
void low_isr(void);
void DelayMS(int delay);

/////*VARIABLES GLOBALES*/////
int i=0;

char prevB = 0; // Valeur précédente du PORTB, pour déterminer type de front

float x = 0;
float y = 0;
float theta = 0;

// On garde les valeurs pour le debug.
float cosinus = 0;
float sinus = 0;

// Nombre de ticks à traiter (signés).
volatile int gTicks = 0;
volatile int dTicks = 0;

volatile int t = 0; // Chronomètre.

//CAN
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
   // Interruptions gA
   if(INTCONbits.INT0IE && INTCONbits.INT0IF)
   {
       if(RISE_gA == PIN_gB)
           gTicks++;
       else
           gTicks--;

       RISE_gA ^= 1; // Change le sens de sensibilité.
       INTCONbits.INT0IF = 0;

   }

   // Interruptions dA
   if(INTCON3bits.INT1E && INTCON3bits.INT1IF)
   {
       if(RISE_dA == PIN_dB)
           dTicks++;
       else
           dTicks--;

       RISE_dA ^= 1; // Change le sens de sensibilité.
       INTCON3bits.INT1IF = 0;
   }

   // Interruptions gB et dB
   if(INTCONbits.RBIE && INTCONbits.RBIF)
   {
       char newB = PORTB;
       INTCONbits.RBIF = 0; // On autorise tôt l'interruption suivante pour ne rien rater.

       if((newB ^ prevB) & 0b00010000) // front sur gB
       {
            if(PIN_gB == PIN_gA)
                gTicks--;
            else
                gTicks++;
       }
       else if((newB ^ prevB) & 0b00100000) // front sur dB
       {
            if(PIN_dB == PIN_dA)
                dTicks--;
            else
                dTicks++;
       }

       prevB = newB; /* Sauvegarde de la valeur du PORTB */
   }
}

#pragma interrupt low_isr
void low_isr(void)
{
    if(PIE3bits.RXB0IE && PIR3bits.RXB0IF)
    {
        //Interruption de réception CAN.

        /*On stocke la valeur. */
        led  = led^1;
        while(CANIsRxReady())
        {
            CANReceiveMessage(&message.id,message.data,
                          &message.len,&message.flags);
        }

        PIR3bits.RXB0IF=0;
        PIR3bits.RXB1IF=0;
        PIR3bits.ERRIF=0;

    }

    if(PIE3bits.ERRIE && PIR3bits.ERRIF)
    {
        // Interruptiton erreur CAN
        led = led^1;

        /*On stocke la valeur. */
        while(CANIsRxReady())
        {
            CANReceiveMessage(&message.id,message.data,
                          &message.len,&message.flags);
        }
        //On stocke le messgae mais on n'incremente pas le buffer
        PIR3bits.RXB0IF=0;
        PIR3bits.RXB1IF=0;
        PIR3bits.ERRIF=0;
    }

}


void calibration()
{
    /*
     * Utilisation : dans le main.
     * x prend le nombre algébrique de ticks de gauche, y ceux de droite.
     * Pour calibrer DL faire la moyenne de x et y sur une distance de 50cm
     * Pour calibrer DTHETA faire un tour complet sur une roue avec l'autre immobile
     */
    while(1) {
        x -= gTicks;
        gTicks = 0;
        y += dTicks;
        dTicks = 0;

        while(gTicks == 0 && dTicks == 0)
        {} // On attend un tick à traiter.
    }
}

/////*PROGRAMME PRINCIPAL*/////
void main (void)
{
    /* Initialisations. */
    ADCON1 = 0x0F ;
    ADCON0 = 0x00 ;
    WDTCON = 0x00 ;

    /* Configurations. */
    TRISA   = 0b11011111; // TODO par maxime: vérifier (il y avait un 0x1101...)
    TRISB   = 0xFF;
    PORTC   = 0xFF; 

     // Interruptions Buffer1
    IPR3bits.RXB1IP=0;//basse
    PIE3bits.RXB1IE=0;//off
    PIR3bits.RXB1IF=0;//flag

    // Interruption Buffer 0
    IPR3bits.RXB0IP=0;  //priorité BASSE
    PIE3bits.RXB0IE=1;  //autorise int sur buff0
    PIR3bits.RXB0IF=0;  //mise a 0 du flag
    PIE3bits.ERRIE=0;   //interruption erreur

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


    // Interruptions du PORTB (haute priorité par défaut)
    OpenRB0INT(PORTB_CHANGE_INT_ON & RISING_EDGE_INT & PORTB_PULLUPS_OFF);
    OpenRB1INT(PORTB_CHANGE_INT_ON & RISING_EDGE_INT & PORTB_PULLUPS_OFF);
    OpenPORTB(PORTB_CHANGE_INT_ON & PORTB_PULLUPS_OFF);

    // Timer0 pour chronométrer les opérations flottantes.
    OpenTimer0(TIMER_INT_OFF & T0_16BIT & T0_SOURCE_INT & T0_PS_1_1);

    /* Signal de démarrage du programme. */
    led = 1;
    for(i=0;i<20;i++)
    {
        led=led^1;
        DelayMS(50);
    }
    led = 0;

    INTCONbits.GIE = 1;

    //calibration();

    while(1) {
        int gTicksTmp, dTicksTmp;
        float distance;

        gTicksTmp = gTicks;
        gTicks = 0; // Viiiite.
        dTicksTmp = dTicks;
        dTicks = 0; // Viiiite.

        theta -= (dTicksTmp + gTicksTmp) * DTHETA;

        distance = (gTicksTmp - dTicksTmp) * DL;
        x += distance * cosinus;
        y += distance * sinus;

        cosinus = cos(theta); // 534 cycles pour 0
        sinus = sin(theta); // 520 cycles pour 0

        while(gTicks == 0 && dTicks == 0)
        {} // On attend un tick à traiter.
    }


//    while(1) {
//        cosinus = cos(theta); // 534 cycles pour 0
//        sinus = sin(theta); // 520 cycles pour 0
//        t = ReadTimer0() - 18;
//        Nop();
//        theta += 0.1;
//    }

// 249 cycles pour une multiplication
// 1 / 5e6 * 1054 = 0.0002108s temps pour un cos et un sin
// 1 / 30 / 80    = 0.0004166s temps entre deux tics
}


///// Définition des fonctions du programme. /////
