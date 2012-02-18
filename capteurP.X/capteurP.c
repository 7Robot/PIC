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
#define XTAL        20000000
#define led         PORTAbits.RA4


/////*PROTOTYPES*/////
void high_isr(void);
void low_isr(void);
void DelayMS(int delay);
void trigger_us0();
void trigger_us1();

/////*VARIABLES GLOBALES*/////
int i=0;
char message[8]="";
char message1[8]="un";
char message2[8]="deux";
char message3[8]="trois";
char messagetest[8] = "ENSEE";
char lengh = 0;

// Derniers états des interrupteurs.
char bas_arriere = 0; // le capteur n'est pas enfoncé
char bas_avant   = 0;

// Variables des deux sonars.
char us0_underthres = 0; // dessus ou dessous du seuil
char us1_underthres = 0;
char us0_autosend = 0; // diffusion systématique de la distance
char us1_autosend = 0;
unsigned int us0_pulse_start; // ticks depuis le début du pulse echo
unsigned int us1_pulse_start;
unsigned int us0_threshold = 0x600; // seuil par défaut (0 = désactivé)
unsigned int us1_threshold = 0x600;
int us0_echo = 0x7FFF; // initialement à la valeur max
int us1_echo = 0x7FFF;


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
    unsigned int time = ReadTimer0();

    // sonar1 (us0)
    if(INTCONbits.INT0IE && INTCONbits.INT0IF)
    {
        if(INTCON2bits.INTEDG0) { // Début du pulse, on enregistre le temps.
            us0_pulse_start = time;
            INTCON2bits.INTEDG0 = 0; // falling
        }
        else { // Fin du pulse.
            char passage;
            us0_echo = time - us0_pulse_start;
            // marche aussi si le timer a débordé car non signé

            if((us0_echo < us0_threshold) != us0_underthres) // passage d'un coté ou de l'autre du seuil
                passage = 0x2; // décalé d'un bit pour pouvoir faire un ET dans l'id direct
            else
                passage = 0;

            if(us0_autosend || passage)
            {
                us0_underthres = (us0_echo < us0_threshold); // nouveau coté du seuil
                CANSendMessage(320 | us0_underthres | passage, (BYTE*)&us0_echo, 2,
                    CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME );
                led = led^1;
            }

            INTCONbits.INT0IE = 0; // Fin de la mesure.
        }
        INTCONbits.INT0IF = 0;
    }

    // COPIER-COLLER pour le sonar2 (us1)
    if(INTCON3bits.INT1IE && INTCON3bits.INT1IF)
    {
        if(INTCON2bits.INTEDG1) { // Début du pulse, on enregistre le temps.
            us1_pulse_start = time;
            INTCON2bits.INTEDG1 = 0; // falling
        }
        else { // Fin du pulse.
            char passage;
            us1_echo = time - us1_pulse_start;
            // marche aussi si le timer a débordé car non signé

            if((us1_echo < us1_threshold) != us1_underthres) // passage d'un coté ou de l'autre du seuil
                passage = 0x2; // décalé d'un bit pour pouvoir faire un ET dans l'id direct
            else
                passage = 0;

            if(us1_autosend || passage)
            {
                us1_underthres = (us1_echo < us1_threshold); // nouveau coté du seuil
                CANSendMessage(352 | us1_underthres | passage, (BYTE*)&us1_echo, 2,
                    CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME );
                led = led^1;
            }

            INTCON3bits.INT1E = 0; // Fin de la mesure.
        }
        INTCON3bits.INT1IF = 0;
    }
}

#pragma interrupt low_isr
void low_isr(void)
{
    // La génération du pulse est bloquante, donc lente, donc low.
    if(INTCONbits.TMR0IE && INTCONbits.TMR0IF) // Vingt fois par secondes, non stop.
    {
        INTCONbits.TMR0IF = 0;

        PORTCbits.RC6 = 1; // Trigger pulse pour sonar1.
        PORTCbits.RC7 = 1; // Trigger pulse pour sonar2.

        if(PORTCbits.RC2 == bas_avant) // Evenement sur bouton avant.
        {
            bas_avant = !PORTCbits.RC2;
            led = led^1;
            CANSendMessage(256 | bas_avant,NULL, 0, /* ou 257 */
                    CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME );
        }
        if(PORTCbits.RC3 == bas_arriere) // Evenement sur bouton arriere.
        {
            bas_arriere = !PORTCbits.RC3;
            led = led^1;
            CANSendMessage(258 | bas_arriere, NULL, 0, /* ou 259 */
                    CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME );
        }

        // TODO: wait moins de 10us, cf remarque

        Delay10TCYx(5); // 10us minimum, mais ce code a surement duré plus longtemps.
        PORTCbits.RC7 = 0;
        PORTCbits.RC6 = 0;

        // Début de l'attente des echos.
        INTCON2bits.INTEDG0 = 1; // rising echo0
        INTCON2bits.INTEDG1 = 1; // rising echo1
        INTCON3bits.INT1IE = 1;
        INTCONbits.INT0IE = 1;
    }
}

/////*PROGRAMME PRINCIPAL*/////
void main (void)
{
    // Initialisations.
    ADCON1 = 0x0F ;
    ADCON0 = 0b00000000;
    WDTCON = 0 ;

    // Configurations.
    TRISA   = 0b11101111 ;
    TRISB   = 0b11111111 ;
    TRISC   = 0b00111111 ;

    // Timer de rafraichissement des BP et de chronométrage des sonars
    OpenTimer0( TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_4); // 19Hz
    INTCON2bits.TMR0IP = 0; // priorité basse, car les pulses sonars prennent du temps

    // il faut laisser 50ms (20Hz) entre deux débordements (limite des sonars)
    // du coup on ne vérifie pas les boutons plus souvent, mais pas grave.

    // Interruptions sur les pins "Echo output" des sonars (AN0 et AN1).
    // OpenRBxINT faits à la main (sauf pullup, par défaut)

    // Configuration du CAN
    CANInitialize(1,5,7,6,2,CAN_CONFIG_VALID_STD_MSG);
    Delay10KTCYx(200);

   /* // Interruptions Buffer1
    IPR3bits.RXB1IP=1;// : priorité haute par defaut du buff 1
    PIE3bits.RXB1IE=1;//autorise int sur buff1
    PIR3bits.RXB1IF=0;//mise a 0 du flag

    // Interruption Buffer 0
    IPR3bits.RXB0IP=1;// : priorité haute par defaut du buff 1
    PIE3bits.RXB0IE=1;//autorise int sur buff1
    PIR3bits.RXB0IF=0;//mise a 0 du flag


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

    RCONbits.IPEN = 1;
    INTCONbits.GIEH = 1; // Autorise interruptions.
    INTCONbits.GIEL = 1;
    
    /* Boucle principale. */
    while(1)
    {
        /*Programme de test pour l'US.*/
        //trigger_us0();
        //trigger_us1();
        //DelayMS(2000);

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
