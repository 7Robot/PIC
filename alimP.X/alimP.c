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


/////*PROTOTYPES*/////
void high_isr(void);
void low_isr(void);

void InterruptServo();
void InterruptLaser();
void WriteAngle(int a);
void GetData();
void Mesures();
void CalculBalise();

unsigned int LectureAnalogique(char pin); // Fonctionne de AN0 à AN4.
void NiveauBatterie ();


/////*VARIABLES GLOBALES*/////
CANmsg message;
CANmsg incoming;
int i = 0;

int dataCount = 0;
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

char mesures = 0;
char broadcast = 0;
volatile char checkBatterie = 0;

char k = 0;
int consigne_g = 810, consigne_d = 800;
int angle_g = 0, angle_d = 0;

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
    else if(PIE1bits.RCIE && PIR1bits.RCIF)
    {
        InterruptAX();
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
            case 193 : //Demande de niveau de batterie
                checkBatterie = 1;
                break;
            case 248: //Reception AX12 gauche
                consigne_g = ((int*)&incoming.data)[0];
                break;
            case 224: //Emission AX12 gauche
                CANSendMessage(240, (BYTE*)&angle_g, 2, CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME);
                break;
            case 249: //Reception AX12 droit
                consigne_d = ((int*)&incoming.data)[0];
                break;
            case 225: //Emission AX12 droit
                CANSendMessage(241, (BYTE*)&angle_d, 2, CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME);
                break;

            default:
                // Rien
                break;
        }
        led = led ^1;



        PIR3bits.RXB0IF = 0;
    }

    //Gestion des AX12
    else if(PIE1bits.TMR2IE && PIR1bits.TMR2IF)  {

        if(k<60) k++;
        else {
            k = 0;
            if (consigne_g != angle_g) {
                PutAX(3, AX_GOAL_POSITION, consigne_g);
                angle_g = consigne_g;
            }
            if (consigne_d != angle_d) {
                PutAX(2, AX_GOAL_POSITION, 1023-consigne_d);
                angle_d = consigne_d;
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
    TRISA = 0b11111111;
    TRISB = 0b11111111;
    TRISC = 0b10101010;

    servo = 0;

    /*Configuration module USART*/
    OpenUSART(USART_TX_INT_OFF & USART_RX_INT_ON
            & USART_ASYNCH_MODE & USART_EIGHT_BIT
            & USART_CONT_RX & USART_BRGH_HIGH, 129); //9600, 0,2% err...
    SetRX();

    OpenTimer0(TIMER_INT_OFF & T0_16BIT & T0_SOURCE_INT & T0_PS_1_32 /* Internal oscillator of 20MHz */);
    T0CONbits.TMR0ON = 0; /*On ne dÈmarre pas le TMR0*/
    WriteTimer0(0);

    OpenTimer2( TIMER_INT_ON & T2_PS_1_16 & T2_POST_1_16 );
    IPR1bits.TMR2IP = 0;

    OpenTimer3(TIMER_INT_ON & T3_16BIT_RW & T3_SOURCE_INT & T3_PS_1_2 & T3_SYNC_EXT_OFF);

    INTCONbits.TMR0IF = 0; /* Flag de TMR0*/
    INTCON2bits.TMR0IP = 1; /* L'interuption sur TMR0 est en haute prioritÈ*/

    PIR2bits.TMR3IF = 0; /*Flag de TMR3*/
    IPR2bits.TMR3IP = 1; /*L'interuption de TMR3 est en haute prioritÈ*/

    INTCON3bits.INT1E = 1; /*Enable interrupt on RB1*/
    INTCON3bits.INT1F = 0; /*External Interrupt Flag bit of RB1*/
    INTCON2bits.INTEDG1 = 1; /* On RB1 : 1:interrupt on risong edge  0:interrupt on falling edge */
    INTCON3bits.INT1IP = 0; /*INT1 is a low level interrup*/


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
    IPR3bits.RXB0IP = 0; // PrioritÈ basse.
    PIE3bits.RXB0IE = 1; // activée
    PIR3bits.RXB0IF = 0;
    // Interruptions Buffer 1.
    PIE3bits.RXB1IE = 0; // Interdite.


    mesures = 0; //mesures off au démarrage

    PutAX(254, AX_ALARM_SHUTDOWN, 0);
    PutAX(254, AX_ALARM_LED, 0);

    // Signal de dÈmarrage du programme.
    led = 0;
    for (i = 0; i < 20; i++) {
        led = led ^ 1;
        DelayMS(50);
    }

    //Autorisation des interruptions
    RCONbits.IPEN = 1;
    INTCONbits.GIEH = 1;
    INTCONbits.GIEL = 1;


    while (1) {
        if (mesures)
            if (mesures)
                Mesures();
        else
            TRISCbits.RC4 = 1; // On stoppe le servo moteur

        if (checkBatterie)
            if (checkBatterie)
                NiveauBatterie(batterie);
        

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
        }
        else
        {
            for(dataCount = hh; dataCount < (nbreBalises - 1); dataCount++) // Permet d'enlever les balises aberrantes
                temps[dataCount] = temps[dataCount+1];
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


unsigned int LectureAnalogique(char pin){   // pas utilisé
    // ATTENTION, ne marche que de AN0 à AN4.
    // pin = 0 => on sélectionne AN0, pin = 1 => AN1 etc...

    unsigned int tempo = 0;
    unsigned int val = 0;
    pin = pin << 2;
    ADCON0 = 0b00000001 + pin; // Selectionne le bon AN en lecture analogique
    ADCON1 = 0b00001010; // Configuration

    //ADCON2 peut être configuré si on le souhaite

    ADCON0bits.GO_DONE=1;
    while(ADCON0bits.GO_DONE); //attend

    val=ADRESL;           // Get the 8 bit LSB result
    val=ADRESL>>6;
    tempo=ADRESH;
    tempo=tempo<<2;         // Get the 2 bit MSB result
    val = val + tempo;

    return (val);
}

void NiveauBatterie (){

    unsigned int tempo = 0;
    unsigned int val = 0;

    char pin = batterie << 2;
    ADCON0 = 0b00000001 + pin; // Selectionne le bon AN en lecture analogique
    ADCON1 = 0b00001010; // Configuration


    //ADCON2 peut être configuré si on le souhaite

    ADCON0bits.GO_DONE=1;
    while(ADCON0bits.GO_DONE); //attend

    val=ADRESL;           // Get the 8 bit LSB result
    val=ADRESL>>6;
    tempo=ADRESH;
    tempo=tempo<<2;         // Get the 2 bit MSB result
    val = val + tempo;


    batteryLevel = val;
    CANSendMessage(194, (BYTE*)&batteryLevel, 2,
        CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME);

   
    checkBatterie = 0;

}
/*
 * TODO
 * - arrêt/départ sur balise
 */

 