/*
 * Programme carte alim petit robot
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

unsigned int LectureAnalogique(char pin); // Fonctionne de AN0 � AN4.
void NiveauBatterie(void);

/////*CONSTANTES*/////
#define XTAL    20000000
#define led     PORTCbits.RC0

#define batterie 4
#define servoPin   PORTCbits.RC4
#define laser   PORTBbits.RB1
#define temoin  PORTCbits.RC6
#define tempsMin 0.33               //Avec ces valeurs on a 180
#define tempsMax 2.08  //2.44 max
#define omega 7.81 // Vitesse angulaire de rotation du servomoteur
#define TIMEDATA_SIZE 50 /*grosse marge sur le nombre de glitch*/

/////*VARIABLES GLOBALES*/////
CANmsg messageTourelle;
CANmsg incoming;
int i = 0;

volatile int timeCount = 0;
volatile unsigned int timeData[TIMEDATA_SIZE];

int cur_balise = 0;
volatile unsigned int angle = 0;
int servoLatch;
unsigned long temps[5] = {0};
unsigned long distance[5] = {0};
unsigned long position[5] = {0};
int pas_de_balise = 0;

int nbrepoint = 0;
unsigned long pointMax[5] = {0};
unsigned long pointMin[5] = {0};
int nbreBalises = 0;

unsigned int batteryLevel = 0;

volatile char mesures = 0;
char broadcast = 0; // inutilis�
char checkBatterie = 0;

char k = 0;
int consigne_angle_g = 200, consigne_angle_d = 200;
volatile int angle_g = 0;
volatile int angle_d = 0;
int conversion_angle;
volatile char ordre_240 = 0;
volatile char ordre_241 = 0;
int consigne_couple_g = 500, consigne_couple_d = 500;
int couple_g = 0, couple_d = 0;
volatile char ordre_224 = 0;
volatile char ordre_225 = 0;

char pulse_sonar = 0;

///// cf capteurP.c ///////

typedef struct {
    char unmuted; // Broadcast d�sactiv� pour 0.
    char state; // 1 pour au dessous du seuil.
    unsigned int threshold; // Seuil d�sactiv� pour 0.
    unsigned int value;
    unsigned int pulse_start; // Ticks compt�s depuis le d�but de l'echo.
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
            //conversion_angle = 1023 -(responseAX.params[1]*256 + responseAX.params[0]);
            *((int*) responseAX.params) = 1023 - *((int*) responseAX.params);
            CANSendMessage(248, (BYTE*) responseAX.params, 2, CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME);
            ordre_240 = 0;
        }
        if (responseReadyAX == 1 && ordre_241 == 1) {
            CANSendMessage(249, (BYTE*) responseAX.params, 2, CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME);
            ordre_241 = 0;
        }
        if (responseReadyAX == 1 && ordre_224 == 1) {
            responseAX.params[1] = responseAX.params[1] % 4;
            CANSendMessage(232, (BYTE*) responseAX.params, 2, CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME);
            ordre_224 = 0;
        }
        if (responseReadyAX == 1 && ordre_225 == 1) {
            CANSendMessage(233, (BYTE*) responseAX.params, 2, CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME);
            ordre_225 = 0;
        }
    }

    if (INTCONbits.INT0IE && INTCONbits.INT0IF) // Cf capteurP.c
    {
        unsigned int time = ReadTimer1();
        INTCONbits.INT0IF = 0;

        if (INTCON2bits.INTEDG0) { // D�but du pulse, on enregistre le temps.
            ranger.pulse_start = time;
        }
        else { // Fin du pulse.
            char new_state;
            char state_changed;

            ranger.value = time - ranger.pulse_start;
            // marche aussi si le timer a d�bord� car non sign�
            // TODO �chelle

            new_state = (ranger.value < ranger.threshold);
            state_changed = (ranger.state != new_state);

            if (ranger.unmuted || state_changed) {
                ranger.state = new_state;

                CANSendMessage(231 | new_state << 4 | state_changed << 3, (BYTE*)&(ranger.value), 2,
                        CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME);
                led = led ^ 1;
            }
        }
        INTCON2bits.INTEDG0 ^= 1; // On �coutera l'autre sens.
    }
}

#pragma interrupt low_isr

void low_isr(void) {
    if (INTCON3bits.INT1E && INTCON3bits.INT1F) {
        InterruptLaser();
    }

    // R�ception CAN.
    if (PIR3bits.RXB0IF || PIR3bits.RXB1IF) {

        CANReceiveMessage(&incoming.id, incoming.data, &incoming.len, &incoming.flags);

        if (PIR3bits.RXB0IF)
            PIR3bits.RXB0IF = 0;
        else // Ne d�sactive pas les deux � la fois si ils sont tous les deux pleins.
            PIR3bits.RXB1IF = 0;


        switch (incoming.id) {
                /* TOURELLE */
            case 132: //Renvoyer distance/angle objet
                CANSendMessage(133, messageTourelle.data, 2 * nbreBalises,
                        CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME);
                break;
            case 134: //Broadcast OFF
                broadcast = 0;
                break;
            case 135: //Broadcast ON
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

                /* AX GAUCHE */
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
                GetAX(AX_GAUCHE, AX_PRESENT_LOAD);
                break;

                /* AX DROIT */
            case 253: //Reception angle AX12 droit
                consigne_angle_d = ((int*) &incoming.data)[0];
                break;
            case 241: //Emission angle AX12 droit
                ordre_241 = 1;
                responseReadyAX = 0;
                GetAX(AX_DROIT, AX_PRESENT_POSITION);
                break;
            case 237: //Reception couple AX12 droit
                consigne_couple_d = ((int*) &incoming.data)[0];
                break;
            case 225: //Emission couple AX12 droit
                ordre_225 = 1;
                responseReadyAX = 0;
                GetAX(AX_DROIT, AX_PRESENT_LOAD);
                break;

                /* ULTRASON */
            case 199: // rangerReq
                while (!CANSendMessage(231 | (ranger.value < ranger.threshold) << 4, (BYTE*)&(ranger.value), 2,
                        CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME)) {
                }
                break;
            case 207: // rangerThres
                ranger.threshold = ((unsigned int*) incoming.data)[0];
                break;
            case 215: // rangerMute
                ranger.unmuted = 0;
                break;
            case 223: // rangerUnmute
                ranger.unmuted = 1;
                break;

            case 1946: // emergencyOn
                mesures = 0; // Arr�t de la tourelle.
                PutAX(AX_GAUCHE, AX_TORQUE_LIMIT, 0); // Free run.
                PutAX(AX_DROIT, AX_TORQUE_LIMIT, 0); // Free run.
                break;
            case 1938: // emergencyOff
                //mesures = 1;
                PutAX(AX_GAUCHE, AX_TORQUE_LIMIT, consigne_couple_g); // Free run.
                PutAX(AX_DROIT, AX_TORQUE_LIMIT, consigne_couple_d); // Free run.
                break;

            default:
                // On annule le clignotement de la LED.
                led = led ^1;
                break;
        }
        led = led ^1;
    }

    // G�n�ration des pulses sonars.
    if (PIE1bits.TMR1IE && PIR1bits.TMR1IF) // Vingt fois par secondes, non stop.
    {
        PIR1bits.TMR1IF = 0;

        // Cf code capeurs.

        if (pulse_sonar) {
            // Tourelle : CloseRB1INT();
            OpenRB0INT(PORTB_CHANGE_INT_ON & RISING_EDGE_INT & PORTB_PULLUPS_OFF);

            PORTAbits.RA0 = 0; // Fin du pulse => d�clenchement.
            // Tourelle : PORTAbits.RA1 = 1;
        } else {
            CloseRB0INT();
            // Tourelle : OpenRB1INT(PORTB_CHANGE_INT_ON & RISING_EDGE_INT & PORTB_PULLUPS_OFF);

            PORTAbits.RA0 = 1;
            // Tourelle : PORTAbits.RA1 = 0;
        }
        pulse_sonar ^= 1;
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

    servoPin = 0;
    servoLatch = 0;

    /*Configuration module USART*/
    OpenUSART(USART_TX_INT_OFF & USART_RX_INT_ON
            & USART_ASYNCH_MODE & USART_EIGHT_BIT
            & USART_CONT_RX & USART_BRGH_HIGH, 129); //9600, 0,2% err...
    SetRX();

    OpenTimer0(TIMER_INT_OFF & T0_16BIT & T0_SOURCE_INT & T0_PS_1_32 /* Internal oscillator of 20MHz */);
    T0CONbits.TMR0ON = 0; /*On ne d�marre pas le TMR0*/
    WriteTimer0(0);

    OpenTimer1(TIMER_INT_ON & T1_16BIT_RW & T1_SOURCE_INT & T1_PS_1_2
            & T1_OSC1EN_OFF & T1_SYNC_EXT_OFF); // Sonar.
    IPR1bits.TMR1IP = 0;

    OpenTimer2(TIMER_INT_ON & T2_PS_1_16 & T2_POST_1_16); // AX-12.
    IPR1bits.TMR2IP = 0;

    OpenTimer3(TIMER_INT_ON & T3_16BIT_RW & T3_SOURCE_INT & T3_PS_1_2 & T3_SYNC_EXT_OFF);

    INTCONbits.TMR0IF = 0; /* Flag de TMR0*/
    INTCON2bits.TMR0IP = 1; /* L'interuption sur TMR0 est en haute priorit�*/

    PIR2bits.TMR3IF = 0; /*Flag de TMR3*/
    IPR2bits.TMR3IP = 1; /*L'interuption de TMR3 est en haute priorit�*/

    INTCON3bits.INT1E = 1; /*Enable interrupt on RB1*/
    INTCON3bits.INT1F = 0; /*External Interrupt Flag bit of RB1*/
    INTCON2bits.INTEDG1 = 1; /* On RB1 : 1:interrupt on risong edge  0:interrupt on falling edge On commence avec un rising edge pour ne pas avoir de probl�me si l'on commence sur une balise*/
    INTCON3bits.INT1IP = 0; /*INT1 is a low level interrup*/


    // Configuration du CAN.
    CANInitialize(1, 5, 7, 6, 2, CAN_CONFIG_VALID_STD_MSG);
    // Configuration des masques et filtres.
    CANSetOperationMode(CAN_OP_MODE_CONFIG);
    // Set Masks values.
    CANSetMask(CAN_MASK_B1, 0b00010000000, CAN_CONFIG_STD_MSG);
    CANSetMask(CAN_MASK_B2, 0b11111100111, CAN_CONFIG_STD_MSG);
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
    IPR3bits.RXB0IP = 0; // Priorit� basse.
    PIE3bits.RXB0IE = 1; // activ�e
    PIR3bits.RXB0IF = 0;

    IPR3bits.RXB1IP = 0; // Priorit� basse.
    PIE3bits.RXB1IE = 1; // Buffer 1 activ� !
    PIR3bits.RXB1IF = 0;


    mesures = 0; //mesures off au d�marrage

    // Signal de d�marrage du programme.
    led = 0;
    for (i = 0; i < 20; i++) {
        led = led ^ 1;
        DelayMS(50);
    }


    PutAX(AX_BROADCAST, AX_ALARM_SHUTDOWN, 0);
    PutAX(AX_BROADCAST, AX_ALARM_LED, 0);
    PutAX(AX_BROADCAST, AX_MOVING_SPEED, 200);
    /* Permet de r�gler les angles limites des deux pinces */
    //    PutAX(AX_GAUCHE, AX_CW_ANGLE_LIMIT, 1023 - 600);          //Limite basse pince gauche
    //    PutAX(AX_GAUCHE, AX_CCW_ANGLE_LIMIT, 1023 - 200);         //Limite haute pince gauche
    //    PutAX(AX_DROIT, AX_CW_ANGLE_LIMIT, 200);                  //Limite haute pince droite
    //    PutAX(AX_DROIT, AX_CCW_ANGLE_LIMIT, 610);                   //Limite basse pince droite


    // Autorisation des interruptions
    RCONbits.IPEN = 1;
    INTCONbits.GIEH = 1;
    INTCONbits.GIEL = 1;

    WriteAngle(180); // Cot� qui force pas.
    while (1) {
        if (mesures)
            if (mesures)
                Mesures();
    }
}

void Mesures() {
    INTCON3bits.INT1E = 1; // InterruptLaser
    WriteAngle(0);
    DelayMS(500);
    INTCON3bits.INT1E = 0;
    GetData();
    INTCON3bits.INT1E = 1;
    WriteAngle(180);
    DelayMS(500);
    INTCON3bits.INT1E = 0;
    GetData();
}

void InterruptLaser() {
    timeData[timeCount++] = ReadTimer0();
    INTCON2bits.INTEDG1 ^= 1;
    INTCON3bits.INT1F = 0;
    if (timeCount >= TIMEDATA_SIZE) {
        timeCount--;
        INTCON3bits.INT1E = 0; // On accepte plus de mesures.
    }
}

void WriteAngle(int a) {
    angle = a;
    WriteTimer0(0); /*On initialise TMR0*/
    timeCount = 0; /*On initialise le tableau timeData[] sur la premi�re case*/
    T0CONbits.TMR0ON = 1; /*On d�marre le TMR0 pour le tableau timeData[]*/
}

void GetData() {
    T0CONbits.TMR0ON = 0;

    nbrepoint = timeCount;
    if (nbrepoint % 2) // Permet de ne pas avoir de probl�me si l'on s'arr�te sur une tourelle
        nbrepoint--;
    pointMin[0] = timeData[0];
    pointMax[0] = timeData[1];

    nbreBalises = 0;
    timeCount = 0;
    cur_balise = 0;

    while (2 * timeCount < nbrepoint) {
        CalculBalise();
    }

    for (cur_balise = 0; cur_balise < nbreBalises; cur_balise++) {
        if ((distance[cur_balise] = 857874.5198 / temps[cur_balise]) < 180) //distance en cm
        {
            //distance[hh] = 6.7/(2*omega*temps[hh]*0.000001/2); //distance en cm
            position[cur_balise] = 0.001431936 * (pointMax[cur_balise] + pointMin[cur_balise]);
            //position[hh] =  (omega * (pointMax[hh] + pointMin[hh])/2 * 6.4 * 0.000001)*180/3.14159;
            if (angle == 0) {
                if (distance[cur_balise] < 25)
                    position[cur_balise] = 180 - position[cur_balise];
                else
                    position[cur_balise] = 190 - position[cur_balise];
            }
            position[cur_balise] -= 5; // Le 5 c'est empirique...
            messageTourelle.data[2 * cur_balise] = (char) distance[cur_balise];
            messageTourelle.data[2 * cur_balise + 1] = (char) position[cur_balise];
        } else {
            for (timeCount = cur_balise; timeCount < (nbreBalises - 1); timeCount++) // Permet d'enlever les balises aberrantes
                temps[timeCount] = temps[timeCount + 1];
            nbreBalises--;
            cur_balise--;
        }
    }
    if (!nbreBalises)
        pas_de_balise++;
    else
        pas_de_balise = 0;

    if (pas_de_balise < 3) { // On emet que si on a d�tect� quelque chose ou qu'on a rien vu depuis deux fois.
        CANSendMessage(133, messageTourelle.data, 2 * nbreBalises, /*ATTENTION Remttre 2*nbreBalises pour apr�s 2*nbreBalises !!!*/
                CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME);
    }
}

void CalculBalise() {
    nbreBalises++;
    pointMin[cur_balise] = timeData[2 * timeCount];
    pointMax[cur_balise] = timeData[2 * timeCount + 1];
    timeCount++;
    while ((timeData[2 * timeCount] - timeData[2 * timeCount - 1])*6.4 < 800 && (2 * timeCount) < nbrepoint) {
        pointMax[cur_balise] = timeData[2 * timeCount + 1];
        timeCount++;
    }
    temps[cur_balise] = (pointMax[cur_balise] - pointMin[cur_balise])*6.4;
    cur_balise++;
}

void InterruptServo() {
    if (servoLatch == 0) {
        WriteTimer3(65535 - tempsMin * 2500 - angle * (tempsMax - tempsMin) * 13.888889);
        //WriteTimer3(65535 - tempsMin*65535/26.214  - angle * (65535/26.214) * (tempsMax - tempsMin) / 180);
        servoPin = 1;
        servoLatch = 1;
    } else {
        WriteTimer3(15535); // 20ms
        servoPin = 0;
        servoLatch = 0;
    }
    PIR2bits.TMR3IF = 0;
}

unsigned int LectureAnalogique(char pin) { // pas utilis�
    // ATTENTION, ne marche que de AN0 � AN4.
    // pin = 0 => on s�lectionne AN0, pin = 1 => AN1 etc...

    unsigned int tempo = 0;
    unsigned int val = 0;
    pin = pin << 2;
    ADCON0 = 0b00000001 + pin; // Selectionne le bon AN en lecture analogique
    ADCON1 = 0b00001010; // Configuration

    //ADCON2 peut �tre configur� si on le souhaite

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


    //ADCON2 peut �tre configur� si on le souhaite

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
