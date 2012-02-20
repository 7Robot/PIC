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
#define XTAL        20000000
#define led         PORTCbits.RC0


/////*PROTOTYPES*/////
void high_isr(void);
void low_isr(void);


/////*VARIABLES GLOBALES*/////
int i = 0;
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

}

#pragma interrupt low_isr
void low_isr(void)
{
    // Réception CAN.
    if(PIE3bits.RXB0IE && PIR3bits.RXB0IF)
    {

        while(CANIsRxReady()) {
            CANReceiveMessage(&message.id, message.data, &message.len, &message.flags);
        }
        
        switch (message.id) {
                    case 132: //Renvoyer distace/angle objet le plus proche
                      //CODE
                      break;
                    case 133: //Renvoyer toutes les infos
                      //CODE
                      break;
                    case 134: //Broadcast ON
                      //CODE
                      break;
                    case 135: //Boradcast OFF
                      //CODE
                      break;
                    case 136: //Mesures OFF
                      //CODE
                      break;
                    case 137: //Mesures ON
                      //CODE
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
    TRISC  = 0b11111110;

    
    // Configuration du CAN.
    CANInitialize(1, 5, 7, 6, 2, CAN_CONFIG_VALID_STD_MSG);
    // Configuration des masques et filtres.
    CANSetOperationMode(CAN_OP_MODE_CONFIG);
    // Set Buffer 1 Mask value.
    CANSetMask(CAN_MASK_B1, 0b00010000000, CAN_CONFIG_STD_MSG);
    // Set Buffer 1 Filter values.
    CANSetFilter(CAN_FILTER_B1_F1, 0b00010000000, CAN_CONFIG_STD_MSG);
    CANSetFilter(CAN_FILTER_B1_F2, 0b00010000000, CAN_CONFIG_STD_MSG);
    // Set CAN module into Normal mode.
    CANSetOperationMode(CAN_OP_MODE_NORMAL);
    // Interruption Buffer 0.
    IPR3bits.RXB0IP = 0; // Priorité basse.
    PIE3bits.RXB0IE = 1; // Activée.
    PIR3bits.RXB0IF = 0;
    // Interruptions Buffer 1.
    PIE3bits.RXB1IE = 0; // Interdite.


    // Signal de démarrage du programme.
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

    }
}
