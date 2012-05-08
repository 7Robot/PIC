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
#include "ax12.h"
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

/////*PROTOTYPES*/////
void high_isr(void);
void low_isr(void);

void InterruptServo(void);
void InterruptLaser(void);
void WriteAngle(int a);
void GetData(void);
void Mesures(void);
void CalculBalise(void);

unsigned int LectureAnalogique(char pin); // Fonctionne de AN0 à AN4.
void NiveauBatterie(void);

/////*CONSTANTES*/////
#define XTAL    20000000
#define led     PORTCbits.RC0

#define batterie 4
#define servo   PORTCbits.RC4
#define laser   PORTBbits.RB1
#define temoin  PORTCbits.RC6
#define tempsMin 0.33               //Avec ces valeurs on a 180
#define tempsMax 2.08  //2.44 max
#define omega 7.81 // Vitesse angulaire de rotation du servomoteur


/////*VARIABLES GLOBALES*/////
CANmsg message;
CANmsg incoming;
int i = 0;

volatile int dataCount = 0;
int hh = 0;
volatile unsigned int angle = 0;
int pulse = 0;
int dir = 1;
unsigned long temps[5] = {0};
unsigned long distance[5] = {0};
unsigned long position[5] = {0};
volatile unsigned int timeData[20] = {0};

int nbrepoint = 0;
unsigned long pointMax[5] = {0};
unsigned long pointMin[5] = {0};
int nbreBalises = 0;

unsigned int batteryLevel = 0;

volatile char mesures = 0;
char broadcast = 0;
char checkBatterie = 0;

char k = 0;
int consigne_angle_g = 220, consigne_angle_d = 220;
volatile int angle_g = 0;
volatile int angle_d = 0;
volatile char ordre_240 = 0;
volatile char ordre_241 = 0;
int consigne_couple_g = 1023, consigne_couple_d = 1023;
int couple_g = 0, couple_d = 0;
volatile char ordre_224 = 0;
volatile char ordre_225 = 0;

///// cf capteurP.c ///////
typedef struct {
    char unmuted; // Broadcast désactivé pour 0.
    char state; // 1 pour au dessous du seuil.
    unsigned int threshold; // Seuil désactivé pour 0.
    unsigned int value;
    unsigned int pulse_start; // Ticks comptés depuis le début de l'echo.
} ranger_finder;

volatile ranger_finder ranger = {0};
///////////////////////////


/////*INTERRUPTIONS*/////
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
    if (PIE2bits.TMR3IE && PIR2bits.TMR3IF) {
        InterruptServo();
    }

    if (PIE1bits.RCIE && PIR1bits.RCIF) {
        InterruptAX();
        if (responseReadyAX == 1 && ordre_240 == 1) {
            CANSendMessage(248, (BYTE*) responseAX.params, 2, CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME);
            ordre_240 = 0;
        }
        if (responseReadyAX == 1 && ordre_241 == 1) {
            CANSendMessage(249, (BYTE*) & angle_g, 2, CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME);
            ordre_241 = 0;
        }
        if (responseReadyAX == 1 && ordre_224 == 1) {
            CANSendMessage(232, (BYTE*) & couple_g, 2, CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME);
            ordre_224 = 0;
        }
        if (responseReadyAX == 1 && ordre_225 == 1) {
            CANSendMessage(233, (BYTE*) & couple_g, 2, CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME);
            ordre_225 = 0;
        }
    }

    if(INTCONbits.INT0IE && INTCONbits.INT0IF) // Cf capteurP.c
    {
        unsigned int time = ReadTimer1();
        INTCONbits.INT0IF = 0;

        if(INTCON2bits.INTEDG0) { // Début du pulse, on enregistre le temps.
             ranger.pulse_start = time;
        }
        else { // Fin du pulse.
            char new_state;
            char state_changed;

            ranger.value = time - ranger.pulse_start;
            // marche aussi si le timer a débordé car non signé
            // TODO échelle

            new_state = (ranger.value < ranger.threshold);
            state_changed = (ranger.state != new_state);

            if(ranger.unmuted || state_changed)
            {
                ranger.state = new_state;

                while(!CANSendMessage(359 | new_state << 4 | state_changed << 3, (BYTE*)&(ranger.value), 2,
                    CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME )) {
                }
                led = led ^ 1;
            }
        }
        INTCON2bits.INTEDG0 ^= 1; // On écoutera l'autre sens.
    }
}
#pragma interrupt low_isr

void low_isr(void) {
    if (INTCON3bits.INT1E && INTCON3bits.INT1F) {
        InterruptLaser();
    }
    // Réception CAN.
    if (PIE3bits.RXB0IE && PIR3bits.RXB0IF) {

        while (CANIsRxReady()) {
            CANReceiveMessage(&incoming.id, incoming.data, &incoming.len, &incoming.flags);
        }

        switch (incoming.id) {
            case 132: //Renvoyer distace/angle objet
                CANSendMessage(133, message.data, 2 * nbreBalises,
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
            case 193: //Demande de niveau de batterie
                NiveauBatterie();
                break;
            case 252: //Reception angle AX12 gauche
                consigne_angle_g = ((int*) &incoming.data)[0];
                break;
            case 240: //Emission angle AX12 gauche
                ordre_240 = 1;
                responseReadyAX = 0;
                GetAX(AX_GAUCHE, AX_PRESENT_POSITION);
                break;
            case 236: //Reception couple AX12 gauche
                consigne_couple_g = ((int*) &incoming.data)[0];
                break;
            case 224: //Emission couple AX12 gauche
                ordre_224 = 1;
                responseReadyAX = 0;
                GetAX(AX_DROIT, AX_PRESENT_LOAD);
                break;
            case 253: //Reception angle AX12 droit
                consigne_angle_d = ((int*) &incoming.data)[0];
                break;
            case 241: //Emission angle AX12 droit
                ordre_241 = 1;
                responseReadyAX = 0;
                GetAX(AX_DROIT, AX_PRESENT_POSITION);
                break;
            case 237: //Reception couple AX12 gauche
                consigne_couple_d = ((int*) &incoming.data)[0];
                break;
            case 225: //Emission couple AX12 gauche
                ordre_225 = 1;
                responseReadyAX = 0;
                GetAX(AX_GAUCHE, AX_PRESENT_LOAD);
                break;
            case 327: // rangerReq
                while(!CANSendMessage(359 | (ranger.value < ranger.threshold), (BYTE*)ranger.value, 2,
                    CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME )) {
                }
                break;
            case 335: // rangerThres
                ranger.threshold = ((unsigned int*) message.data)[0];
                break;
            case 343: // rangerMute
                ranger.unmuted = 0;
                break;
            case 351: // rangerUnmute
                ranger.unmuted = 1;
                break;
            default:
                // On annule le clignotement de la LED.
                led = led ^1;
                break;
        }
        led = led ^1;


        PIR3bits.RXB0IF = 0;
    }

    // Génération des pulses sonars.
    if(PIE1bits.TMR1IE && PIR1bits.TMR1IF) // Vingt fois par secondes, non stop.
    {
        PIR1bits.TMR1IF = 0;

        // Cf code capeurs.

        if(PORTAbits.RA0) {
            // Tourelle : CloseRB1INT();
            OpenRB0INT(PORTB_CHANGE_INT_ON & RISING_EDGE_INT & PORTB_PULLUPS_OFF);

            PORTAbits.RA0 = 0; // Fin du pulse => déclenchement.
            // Tourelle : PORTAbits.RA1 = 1;
        }
        else {
            CloseRB0INT();
            // Tourelle : OpenRB1INT(PORTB_CHANGE_INT_ON & RISING_EDGE_INT & PORTB_PULLUPS_OFF);

            PORTAbits.RA0 = 1;
            // Tourelle : PORTAbits.RA1 = 0;
        }
    }

    //Gestion des AX12
    if (PIE1bits.TMR2IE && PIR1bits.TMR2IF) {

        if (k < 60) k++;
        else {
            k = 0;
            if (consigne_angle_g != angle_g) {
                PutAX(AX_GAUCHE, AX_GOAL_POSITION, 1023 - consigne_angle_g);
                angle_g = consigne_angle_g;
            }
            if (consigne_angle_d != angle_d) {
                PutAX(AX_DROIT, AX_GOAL_POSITION, consigne_angle_d);
                angle_d = consigne_angle_d;
            }
            if (consigne_couple_g != couple_g) {
                PutAX(AX_GAUCHE, AX_TORQUE_LIMIT, consigne_couple_g);
                couple_g = consigne_couple_g;
            }
            if (consigne_couple_d != couple_d) {
                PutAX(AX_DROIT, AX_TORQUE_LIMIT, consigne_couple_d);
                couple_d = consigne_couple_d;
            }

        }
        PIR1bits.TMR2IF = 0;
    }
}

/////*PROGRAMME PRINCIPAL*/////

void main(void) {
    // Initialisations.
    ADCON1 = 0x0F;
    ADCON0 = 0b00000000;
    WDTCON = 0;

    // Configurations.
    TRISA = 0b11111110;
    TRISB = 0b11111111;
    TRISC = 0b10101010;

    servo = 0;

    /*Configuration module USART*/
    OpenUSART(USART_TX_INT_OFF & USART_RX_INT_ON
            & USART_ASYNCH_MODE & USART_EIGHT_BIT
            & USART_CONT_RX & USART_BRGH_HIGH, 129); //9600, 0,2% err...
    SetRX();

    OpenTimer0(TIMER_INT_OFF & T0_16BIT & T0_SOURCE_INT & T0_PS_1_32 /* Internal oscillator of 20MHz */);
    T0CONbits.TMR0ON = 0; /*On ne démarre pas le TMR0*/
    WriteTimer0(0);

    OpenTimer1(TIMER_INT_ON & T1_16BIT_RW & T1_SOURCE_INT & T1_PS_1_2
            & T1_OSC1EN_OFF & T1_SYNC_EXT_OFF ); // Sonar.
    IPR1bits.TMR1IP = 0;

    OpenTimer2(TIMER_INT_ON & T2_PS_1_16 & T2_POST_1_16); // AX-12.
    IPR1bits.TMR2IP = 0;

    OpenTimer3(TIMER_INT_ON & T3_16BIT_RW & T3_SOURCE_INT & T3_PS_1_2 & T3_SYNC_EXT_OFF);

    INTCONbits.TMR0IF = 0; /* Flag de TMR0*/
    INTCON2bits.TMR0IP = 1; /* L'interuption sur TMR0 est en haute prioritÈ*/

    PIR2bits.TMR3IF = 0; /*Flag de TMR3*/
    IPR2bits.TMR3IP = 1; /*L'interuption de TMR3 est en haute prioritÈ*/

    INTCON3bits.INT1E = 1; /*Enable interrupt on RB1*/
    INTCON3bits.INT1F = 0; /*External Interrupt Flag bit of RB1*/
    INTCON2bits.INTEDG1 = 1; /* On RB1 : 1:interrupt on risong edge  0:interrupt on falling edge On commence avec un rising edge pour ne pas avoir de problème si l'on commence sur une balise*/
    INTCON3bits.INT1IP = 0; /*INT1 is a low level interrup*/


    // Configuration du CAN.
    CANInitialize(1, 5, 7, 6, 2, CAN_CONFIG_VALID_STD_MSG);
    // Configuration des masques et filtres.
    CANSetOperationMode(CAN_OP_MODE_CONFIG);
    // Set Masks values.
    CANSetMask(CAN_MASK_B1, 0b00010000000, CAN_CONFIG_STD_MSG);
    CANSetMask(CAN_MASK_B2, 0b11111000111, CAN_CONFIG_STD_MSG);
    // Set Buffer 1 Filter values.
    CANSetFilter(CAN_FILTER_B1_F1, 0b00010000000, CAN_CONFIG_STD_MSG);
    CANSetFilter(CAN_FILTER_B1_F2, 0b00010000000, CAN_CONFIG_STD_MSG);
    // Set Buffer 2 Filter values.
    CANSetFilter(CAN_FILTER_B2_F1, 0b00101000111, CAN_CONFIG_STD_MSG);
    CANSetFilter(CAN_FILTER_B2_F2, 0b00101000111, CAN_CONFIG_STD_MSG);
    CANSetFilter(CAN_FILTER_B2_F3, 0b00101000111, CAN_CONFIG_STD_MSG);
    CANSetFilter(CAN_FILTER_B2_F4, 0b00101000111, CAN_CONFIG_STD_MSG);
    // Set CAN module into Normal mode.
    CANSetOperationMode(CAN_OP_MODE_NORMAL);

    // Interruption Buffer 0.
    IPR3bits.RXB0IP = 0; // PrioritÈ basse.
    PIE3bits.RXB0IE = 1; // activée
    PIR3bits.RXB0IF = 0;
    // Interruptions Buffer 1.
    PIE3bits.RXB1IE = 0; // Interdite.


    mesures = 0; //mesures off au démarrage

    PutAX(AX_BROADCAST, AX_ALARM_SHUTDOWN, 0);
    PutAX(AX_BROADCAST, AX_ALARM_LED, 0);
    //    PutAX(AX_GAUCHE, AX_CW_ANGLE_LIMIT, 0);         /* Permet de régler les angles limites des deux pinces */
    //    PutAX(AX_GAUCHE, AX_CCW_ANGLE_LIMIT, 1023);
    //    PutAX(AX_DROIT, AX_CW_ANGLE_LIMIT, 0);
    //    PutAX(AX_DROIT, AX_CCW_ANGLE_LIMIT, 1023);

    // Signal de démarrage du programme.
    led = 0;
    for (i = 0; i < 20; i++) {
        led = led ^ 1;
        DelayMS(50);
    }

    // Autorisation des interruptions
    RCONbits.IPEN = 1;
    INTCONbits.GIEH = 1;
    INTCONbits.GIEL = 1;


    while (1) {
        if (mesures) {
            if (mesures)
                Mesures();
        } else {
            PIE2bits.TMR3IE = 0;
            INTCON3bits.INT1E = 0;
        }
    }
}

void WriteAngle(int a) {
    angle = a;
    WriteTimer0(0); /*On initialise TMR0*/
    dataCount = 0; /*On initialise le tableau timeData[] sur la premiËre case*/
    T0CONbits.TMR0ON = 1; /*On dÈmarre le TMR0 pour le tableau timeData[]*/
}

void GetData() {
    T0CONbits.TMR0ON = 0;

    nbrepoint = dataCount;
    if (nbrepoint % 2) // Permet de ne pas avoir de problème si l'on s'arrête sur une tourelle
        nbrepoint--;
    pointMin[0] = timeData[0];
    pointMax[0] = timeData[1];

    nbreBalises = 0;
    dataCount = 0;
    hh = 0;

    while (2 * dataCount < nbrepoint) {
        CalculBalise();
    }

    for (hh = 0; hh < nbreBalises; hh++) {
        if ((distance[hh] = 857874.5198 / temps[hh]) < 180) //distance en cm
        {
            //distance[hh] = 6.7/(2*omega*temps[hh]*0.000001/2); //distance en cm
            position[hh] = 0.001431936 * (pointMax[hh] + pointMin[hh]);
            //position[hh] =  (omega * (pointMax[hh] + pointMin[hh])/2 * 6.4 * 0.000001)*180/3.14159;
            if (angle == 0)
                position[hh] = 200 - position[hh];
            message.data[2 * hh] = (char) distance[hh];
            message.data[2 * hh + 1] = (char) position[hh];
        } else {
            for (dataCount = hh; dataCount < (nbreBalises - 1); dataCount++) // Permet d'enlever les balises aberrantes
                temps[dataCount] = temps[dataCount + 1];
            nbreBalises--;
            hh--;
        }
    }

    if (broadcast) {
        CANSendMessage(133, message.data, 2 * nbreBalises, /*ATTENTION Remttre 2*nbreBalises pour après 2*nbreBalises !!!*/
                CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME);
    }
}

void CalculBalise() {
    nbreBalises++;
    pointMin[hh] = timeData[2 * dataCount];
    pointMax[hh] = timeData[2 * dataCount + 1];
    dataCount++;
    while ((timeData[2 * dataCount] - timeData[2 * dataCount - 1])*6.4 < 800 && (2 * dataCount) < nbrepoint) {
        pointMax[hh] = timeData[2 * dataCount + 1];
        dataCount++;
    }
    temps[hh] = (pointMax[hh] - pointMin[hh])*6.4;
    hh++;
}

void Mesures() {
    PIE2bits.TMR3IE = 1;
    INTCON3bits.INT1E = 1;
    TRISCbits.RC4 = 0;
    WriteAngle(0);
    DelayMS(500);
    GetData();
    WriteAngle(180);
    DelayMS(500);
    GetData();
}

void InterruptServo() {
    if (pulse == 1) {
        WriteTimer3(65535 - tempsMin * 2500 - angle * (tempsMax - tempsMin)*13.888889);
        //WriteTimer3(65535 - tempsMin*65535/26.214  - angle * (65535/26.214) * (tempsMax - tempsMin) / 180);
        servo = 1;
        pulse = 0;
    } else {
        WriteTimer3(15535); // 20ms
        servo = 0;
        pulse = 1;
    }
    PIR2bits.TMR3IF = 0;
}

void InterruptLaser() {
    timeData[dataCount++] = ReadTimer0();
    INTCON2bits.INTEDG1 ^= 1;
    INTCON3bits.INT1F = 0;
}

unsigned int LectureAnalogique(char pin) { // pas utilisé
    // ATTENTION, ne marche que de AN0 à AN4.
    // pin = 0 => on sélectionne AN0, pin = 1 => AN1 etc...

    unsigned int tempo = 0;
    unsigned int val = 0;
    pin = pin << 2;
    ADCON0 = 0b00000001 + pin; // Selectionne le bon AN en lecture analogique
    ADCON1 = 0b00001010; // Configuration

    //ADCON2 peut être configuré si on le souhaite

    ADCON0bits.GO_DONE = 1;
    while (ADCON0bits.GO_DONE); //attend

    val = ADRESL; // Get the 8 bit LSB result
    val = ADRESL >> 6;
    tempo = ADRESH;
    tempo = tempo << 2; // Get the 2 bit MSB result
    val = val + tempo;

    return (val);
}

void NiveauBatterie() {

    unsigned int tempo = 0;
    unsigned int val = 0;

    char pin = batterie << 2;
    ADCON0 = 0b00000001 + pin; // Selectionne le bon AN en lecture analogique
    ADCON1 = 0b00001010; // Configuration


    //ADCON2 peut être configuré si on le souhaite

    ADCON0bits.GO_DONE = 1;
    while (ADCON0bits.GO_DONE); //attend

    val = ADRESL; // Get the 8 bit LSB result
    val = ADRESL >> 6;
    tempo = ADRESH;
    tempo = tempo << 2; // Get the 2 bit MSB result
    val = val + tempo;


    batteryLevel = val;
    CANSendMessage(194, (BYTE*) & batteryLevel, 2,
            CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME);


    checkBatterie = 0;

}
