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
#include <usart.h>
#include <delays.h>
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
#define XTAL        20000000
#define led         PORTAbits.RA4


/////*PROTOTYPES*/////
void high_isr(void);
void low_isr(void);

/////*VARIABLES GLOBALES*/////
int i;

// Derniers états des interrupteurs.
char bump_back = 0; // Enfoncement du capteur.
char bum_front = 0;

// Variables des deux sonars.
char us0_underthres = 0; // Sur ou sous le seuil.
char us1_underthres = 0;
char us0_unmuted = 0; // Broadcast de la distance.
char us1_unmuted = 0;
unsigned int us0_pulse_start; // Ticks comptés depuis le début de l'echo.
unsigned int us1_pulse_start;
unsigned int us0_threshold = 0x500; // Seuil par défaut (0 = désactivé).
unsigned int us1_threshold = 0x500;
int us0_echo = 0x7FFF; // On commence avec des echos à la valeur maxi.
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

            if(us0_unmuted || passage)
            {
                us0_underthres = (us0_echo < us0_threshold); // nouveau coté du seuil
                while(!CANSendMessage(320 | us0_underthres | passage, (BYTE*)&us0_echo, 2,
                    CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME )) {
                }
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

            if(us1_unmuted || passage)
            {
                us1_underthres = (us1_echo < us1_threshold); // nouveau coté du seuil
                while(!CANSendMessage(352 | us1_underthres | passage, (BYTE*)&us1_echo, 2,
                    CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME )) {
                }
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
    // Réception CAN.
    if(PIE3bits.RXB0IE && PIR3bits.RXB0IF)
    {
        unsigned long id;
        BYTE data[8];
        BYTE len;
        enum CAN_RX_MSG_FLAGS flags;

        while(CANIsRxReady()) {
            CANReceiveMessage(&id, data, &len, &flags);
        }
        PIR3bits.RXB0IF = 0;

        led = led ^ 1;
        
        if(id == 324) { // sonar1Req
            while(!CANSendMessage(320 | us0_underthres, (BYTE*)&us0_echo, 2,
                CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME )) {
            }
        }
        else if(id == 356) { // sonar2Req
            while(!CANSendMessage(352 | us1_underthres, (BYTE*)&us1_echo, 2,
                CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME )) {
            }
        }
        else if(id == 332) { // sonar1Mute
            us0_unmuted = 0;
        }
        else if(id == 333) { // sonar1Unmute
            us0_unmuted = 1;
        }
        else if(id == 364) { // sonar2Mute
            us1_unmuted = 0;
        }
        else if(id == 365) { // sonar2Unmute
            us1_unmuted = 1;
        }
        else if(id == 328 && len == 2) { // sonar1Thres
            us0_threshold = ((unsigned int*) data)[0];
        }
        else if(id == 360 && len == 2) { // sonar2Thres
            us1_threshold = ((unsigned int*) data)[0];
        }
        else {
            led = led ^ 1; // On annule la commutation précédente de la LED.
        }
    }


    // Génération des pulses sonars et polling des bumpers.
    if(INTCONbits.TMR0IE && INTCONbits.TMR0IF) // Vingt fois par secondes, non stop.
    {
        INTCONbits.TMR0IF = 0;

        if(PORTCbits.RC2 == bum_front) // Evenement sur bouton avant.
        {
            bum_front = !PORTCbits.RC2;
            led = led^1;
            while(!CANSendMessage(256 | bum_front,NULL, 0, /* ou 257 */
                    CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME )) {
            }
        }
        if(PORTCbits.RC3 == bump_back) // Evenement sur bouton arriere.
        {
            bump_back = !PORTCbits.RC3;
            led = led^1;
            while(!CANSendMessage(258 | bump_back, NULL, 0, /* ou 259 */
                    CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME )) {
            }
        }

        // On alterne les triggers pulse, et pas besoin
        // d'attendre plus car ils restent allumés la moitié du temps (50ms).
        if(PORTCbits.RC6) {
            PORTCbits.RC6 = 0; // Fin du pulse => déclenchement.
            PORTCbits.RC7 = 1; // Trigger pulse pour us1.

            INTCON2bits.INTEDG0 = 1; // Rising (echo us0).
            INTCONbits.INT0IE = 1;
        }
        else {
            PORTCbits.RC6 = 1; // Début pulse pour us0.
            PORTCbits.RC7 = 0; // Fin du pulse => déclenchement.

            INTCON2bits.INTEDG1 = 1; // Rising (echo us1).
            INTCON3bits.INT1IE = 1;
        }
        // Début de l'attente des echos.
    }
}

/////*PROGRAMME PRINCIPAL*/////
void main (void) {
    // Initialisations.
    ADCON1 = 0x0F;
    ADCON0 = 0;
    WDTCON = 0;

    // Configurations.
    TRISA = 0b11101111;
    TRISB = 0b11111111;
    TRISC = 0b00111111;

    // Timer de rafraichissement des BP et de chronométrage des sonars
    OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_2); // 38Hz
    INTCON2bits.TMR0IP = 0; // priorité basse, car les pulses sonars prennent du temps

    // Il faut laisser 50ms (20Hz) entre deux débordements (limite des sonars).
    // Mais on alterne les salves donc 38Hz est parfait.
    // Du coup on ne vérifie pas les boutons plus souvent, mais pas grave.

    // Interruptions sur les pins "Echo output" des sonars (AN0 et AN1).
    // OpenRBxINT faits à la main (sauf pullup, par défaut).


    // Configuration du CAN.
    CANInitialize(1, 5, 7, 6, 2, CAN_CONFIG_VALID_STD_MSG);
    // Configuration des masques et filtres.
    CANSetOperationMode(CAN_OP_MODE_CONFIG);
    // Set Buffer 1 Mask value.
    CANSetMask(CAN_MASK_B1, 0b00100000000, CAN_CONFIG_STD_MSG);
    // Set Buffer 1 Filter values.
    CANSetFilter(CAN_FILTER_B1_F1, 0b00100000000, CAN_CONFIG_STD_MSG);
    CANSetFilter(CAN_FILTER_B1_F2, 0b00100000000, CAN_CONFIG_STD_MSG);
    // Set CAN module into Normal mode.
    CANSetOperationMode(CAN_OP_MODE_NORMAL);


    // Interruption Buffer 0.
    IPR3bits.RXB0IP = 0; // Priorité basse.
    PIE3bits.RXB0IE = 1; // Activée.
    PIR3bits.RXB0IF = 0;

    // Autorisation des interruptions.
    RCONbits.IPEN = 1;
    INTCONbits.GIEH = 1;
    INTCONbits.GIEL = 1;

    // Signal de démarrage.
    led = 0;
    for(i = 0; i < 20; i++) {
        led = led ^ 1;
        DelayMS(50);
    }


    while(1) {

    }
}
