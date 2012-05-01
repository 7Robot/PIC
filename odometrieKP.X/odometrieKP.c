/*
* Programme PIC carte d'alim petit robot
* Eurobot 2012
* Compiler : Microchip C18
* µC : 18f25k80
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
#include <p18f25k80.h>
#include <delays.h>
#include <timers.h>
#include <math.h>
#include <portb.h>
#include "../libcan/can18xxk8.h"


/////*CONFIGURATION*/////
#pragma config RETEN = OFF
#pragma config XINST = OFF
#pragma config INTOSCSEL = HIGH
#pragma config SOSCSEL = HIGH
#pragma config FOSC = HS2
#pragma config PLLCFG = ON
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

/////*CONSTANTES*/////
#define FCY 16000000 // Fréquence d'exécution des instructions = 4*16Mhz/4 = 16MHz.
#define led LATAbits.LATA5

// Calibration optimiste.
#define TICKS_PER_TURN (4 * 12 * 64) // Deux fronts * deux canaux * 12 pales * démultiplication.
#define PI      3.14159265 // ? (sometimes written pi) is a mathematical constant that is the ratio of any Euclidean circle's circumference to its diameter.
#define CDEG    5729.57795 // (18000 / PI) // centi-degrees
#define ENTRAXE (211. - 15.) // Rayon de courbure pour une seule roue en mouvement (mm).
#define RAYON   36.5 // Rayon des roues (mm).

#define MM      0.0746537317 // (2.* PI * RAYON / TICKS_PER_TURN) // Multiply ticks to get millimeters.
#define DTHETA  0.000380886386 // (MM / ENTRAXE) // Delta de theta pour un tick (rad).

#define RISE_gA INTCON2bits.INTEDG0
#define RISE_dA INTCON2bits.INTEDG1

#define PIN_gA PORTBbits.RB0
#define PIN_gB PORTBbits.RB4
#define PIN_dA PORTBbits.RB1
#define PIN_dB PORTBbits.RB5

/////*PROTOTYPES*/////
void high_isr(void);
void low_isr(void);
void SendPosition();
void DelayMS(int delay);

/////*VARIABLES GLOBALES*/////
int i = 0;

char prevB = 0; // Valeur précédente du PORTB, pour déterminer le type de front

// Pour initialiser : Odoset
// x = -2000 + 250 = 0x6D6
// y = -1500 + 250 = 0x4E2
// theta = 9000 = 0x2328
// 517   214 6   226 4   40 35

volatile float x = 0; // En ticks.
volatile float y = 0; // En ticks.
volatile long theta = 0; // En ticks.

// Nombre de ticks à traiter (signés).
volatile char gTicks = 0;
volatile char dTicks = 0;

char unmuted = 1; // Broadcast de la position.

volatile int t = 0; // Chronomètre.


// CAN
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
    if(INTCONbits.INT0IE && INTCONbits.INT0IF) { // Interruptions gA
        if(RISE_gA == PIN_gB)
            gTicks++;
        else
            gTicks--;

        RISE_gA ^= 1; // Change le sens de sensibilité.
        INTCONbits.INT0IF = 0;
    }

    if(INTCON3bits.INT1E && INTCON3bits.INT1IF) { // Interruptions dA
        if(RISE_dA == PIN_dB)
            dTicks++;
        else
            dTicks--;

        RISE_dA ^= 1; // Change le sens de sensibilité.
        INTCON3bits.INT1IF = 0;
    }

    if(INTCONbits.RBIE && INTCONbits.RBIF) { // Interruptions gB et dB
        char newB = PORTB;
        INTCONbits.RBIF = 0; // On autorise tôt l'interruption suivante pour ne rien rater.

        if((newB ^ prevB) & 0b00010000) { // front sur gB
            if(PIN_gB == PIN_gA)
                gTicks--;
            else
                gTicks++;
        }
        else if((newB ^ prevB) & 0b00100000) { // front sur dB
            if(PIN_dB == PIN_dA)
                dTicks--;
            else
                dTicks++;
        }

        prevB = newB; // Sauvegarde de la valeur du PORTB.
    }
}

#pragma interrupt low_isr
void low_isr(void)
{
    // Réception CAN.
    if(PIE5bits.RXB0IE && PIR5bits.RXB0IF) {
        while(CANIsRxReady()) {
            CANReceiveMessage(&message.id, message.data,
                              &message.len, &message.flags);
        }

        led = led ^ 1;

        if(message.id == 513) { // odoReq
            SendPosition();
        }
        else if(message.id == 514) { // odoMute
            unmuted = 0;
        }
        else if(message.id == 515) { // odoUnmute
            unmuted = 1;
        }
        else if(message.id == 517) { // odoSet
            if(message.len == 0) { // Par défaut 0 0 0 0 0 0.
                x = 0;
                y = 0;
                theta = 0;
            }
            else {
                x = (float)(((int*)message.data)[0] * 2) / MM;
                y = (float)(((int*)message.data)[1] * 2) / MM;
                theta = (long)(((unsigned int*)message.data)[2] / CDEG / DTHETA);
            }
        }
        else {
            led = led ^ 1; // On annule la commutation précédente de la LED.
        }

        PIR5bits.RXB0IF = 0;
    }

    if(INTCONbits.TMR0IE && INTCONbits.TMR0IF) {
        INTCONbits.TMR0IF = 0;

        if(unmuted) {
            led = led ^ 1;
            SendPosition();
        }
    }
}

volatile float f;
/////*PROGRAMME PRINCIPAL*/////
void main (void) {
    float cosinus = 1;
    float sinus = 0;

    // Initialisations.
    ADCON1 = 0x0F;
    ADCON0 = 0b00000000;
    ANCON1 = 0x00;
    WDTCON = 0;

    // Configurations.
    TRISA = 0b11011111;
    TRISB = 0b11111111;
    TRISC = 0b00000011;
    PORTC = 0b11111111;

    // Interruptions du PORTB (haute priorité par défaut).
    OpenRB0INT(PORTB_CHANGE_INT_ON & RISING_EDGE_INT & PORTB_PULLUPS_OFF);
    OpenRB1INT(PORTB_CHANGE_INT_ON & RISING_EDGE_INT & PORTB_PULLUPS_OFF);
    OpenPORTB(PORTB_CHANGE_INT_ON & PORTB_PULLUPS_OFF);
   // prevB = PORTB;
    IOCB = 0b00110000;

    // Timer0 pour le broadcast de l'odométrie.
    OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_8); // 8 * 2^16 / 5e6 = 0.104 fois par seconde
    INTCON2bits.TMR0IP = 0; // Priorité basse, car ce n'est pas critique.


    // Configuration du CAN.
    CANInitialize(1, 16, 7, 6, 2, CAN_CONFIG_VALID_STD_MSG);
    // Configuration des masques et filtres.
    CANSetOperationMode(CAN_OP_MODE_CONFIG);
    // Set mask values.
    CANSetMask(CAN_MASK_B1, 0b01000000000, CAN_CONFIG_STD_MSG);
    CANSetMask(CAN_MASK_B2, 0xFFFFFF, CAN_CONFIG_STD_MSG);
    // Set Buffer 1 filter values.
    CANSetFilter(CAN_FILTER_B1_F1, 0b01000000000, CAN_CONFIG_STD_MSG);
    CANSetFilter(CAN_FILTER_B1_F2, 0b01000000000, CAN_CONFIG_STD_MSG);
    // Set CAN module into Normal mode.
    CANSetOperationMode(CAN_OP_MODE_NORMAL);

    // Interruption Buffer 0.
    IPR5bits.RXB0IP = 0; // Priorité basse.
    PIE5bits.RXB0IE = 1; // Activée.
    PIR5bits.RXB0IF = 0;

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
        char gTicksTmp, dTicksTmp;
        char distance;

        INTCONbits.GIEL = 0;
        INTCONbits.GIEH = 0;
        gTicksTmp = gTicks;
        gTicks = 0; // Viiiite.
        dTicksTmp = dTicks;
        dTicks = 0; // Viiiite.
        INTCONbits.GIEH = 1;

        theta -= dTicksTmp + gTicksTmp;

        distance = gTicksTmp - dTicksTmp; // En double ticks.
        
        x += distance * cosinus;
        y += distance * sinus;

        cosinus = cos(theta * DTHETA); // 534 cycles pour 0
        sinus = sin(theta * DTHETA); // 520 cycles pour 0

        INTCONbits.GIEL = 1;
        while(fabs(gTicks)+fabs(dTicks) < 4)
        {} // On attend un tick à traiter.
    }
    // Timer0 pour chronométrer les opérations flottantes.
    //    OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_1);
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


void DelayMS(int delay) {
    /*Attente en ms, sur cette carte c'est utile, et vu que le Quart est soudé,
     il y a peu de raisons pour que ça change...*/
    int cp = 0;
    for (cp = 0; cp < delay; cp++) {
        Delay1KTCYx(16);
    }
}


void SendPosition() { // Utilisé en réponse à odoReq et dans l'autosend.
    int data[3];

    long thetaCentiDegrees = theta * DTHETA * CDEG; // Risque d'under/overflow avec un unsigned int.


    data[0] = (int)(x * MM / 2.);
    data[1] = (int)(y * MM / 2.);
    ((unsigned int*)data)[2] = (unsigned int)(((thetaCentiDegrees % 36000) + 36000) % 36000); // Contournement du modulo négatif.


    while(!CANSendMessage(516, (BYTE*)data, 6, // 516 odoPosition
        CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME )) {
    }
}