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
#include <adc.h>
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
void check_sonar(char pos, char rising);
void check_button(char pos, char pin);

/////*VARIABLES GLOBALES*/////
int i;

// Derniers états des interrupteurs.
char button_values[4] = {0}; // Capteurs d'enfoncement.


typedef struct {
    char unmuted; // Broadcast désactivé pour 0.
    char state; // 1 pour au dessous du seuil.
    unsigned int threshold; // Seuil désactivé pour 0.
    unsigned int value;

    // Spécifique aux sonars.
    unsigned int pulse_start; // Ticks comptés depuis le début de l'echo.
} ranger_finder;

ranger_finder rangers[6] = {0}; // Sonar ou Sharp.


// Le sharp en cours de lecture (0 à 3).
char adc_channel = 0;




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


unsigned int time;
#pragma interrupt high_isr
void high_isr(void)
{
    time = ReadTimer0();

    if(INTCONbits.INT0IE && INTCONbits.INT0IF)
    {
        INTCONbits.INT0E = 0; // Fin de la mesure.
        INTCONbits.INT0IF = 0;
        check_sonar(0, INTCON2bits.INTEDG0);
        INTCON2bits.INTEDG0 ^= 1; // On écoutera l'autre sens.
    }

    if(INTCON3bits.INT1IE && INTCON3bits.INT1IF)
    {
        INTCON3bits.INT1E = 0; // Fin de la mesure.
        INTCON3bits.INT1IF = 0;
        check_sonar(1, INTCON2bits.INTEDG1);
        INTCON2bits.INTEDG1 ^= 1; // On écoutera l'autre sens.
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
        char pos;
        char cmd;

        while(CANIsRxReady()) {
            CANReceiveMessage(&id, data, &len, &flags);
        }
        PIR3bits.RXB0IF = 0;

        led = led ^ 1;

        pos = id & 0x07;
        cmd = id & 0xF8;
        
        if(cmd == 324) { // ranger request
            while(!CANSendMessage(320 | pos, (BYTE*)rangers[pos].value, 2,
                CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME )) {
            }
        }
        else if(cmd == 332) { // ranger mute
            rangers[pos].unmuted = 0;
        }
        else if(cmd == 333) { // ranger unmute
            rangers[pos].unmuted = 1;
        }
        else if(cmd == 360 && len == 2) { // ranger threshold
            rangers[pos].threshold = ((unsigned int*) data)[0];
        }
        else {
            led = led ^ 1; // On annule la commutation précédente de la LED.
        }
    }


    // Génération des pulses sonars et polling des bumpers.
    if(INTCONbits.TMR0IE && INTCONbits.TMR0IF) // Vingt fois par secondes, non stop.
    {
        INTCONbits.TMR0IF = 0;

        check_button(0, PORTCbits.RC2);
        check_button(1, PORTCbits.RC3);
        check_button(2, PORTCbits.RC3); // TODO pin
        check_button(3, PORTCbits.RC3); // TODO pin

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

        /*TODO On démarre la première conversion analogique (Sharp 1).
        SetChanADC(ADC_CH0);
        adc_channel = 0;
        ConvertADC();
        //*/
    }

    /*TODO Lecture de l'ADC et passage à la mesure suivante.
    if(PIE1bits.ADIE && PIR1bits.ADIF)
    {
        BYTE data[2];
        ((int*)data)[0] = ReadADC();

        sharp_values[adc_channel] = ADRESH; // TODO : pré-traitement

        if(sharp_unmuted[adc_channel] && i++ % 4 == 0) {
            // TODO : gérer seuils
            while(!CANSendMessage(272 | adc_channel << 1, &sharp_values[adc_channel], 1,
                    CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME )) {
            }
            led = led ^ 1;
        }

        PIR1bits.ADIF = 0;
        adc_channel++;

        if(adc_channel < 4) { // Lancement de la conversion suivante.
            ADCON0bits.CHS = adc_channel; // Plus simple que SetChanADC().
            ConvertADC();
        }
    }*/
}


void check_sonar(char pos, char rising) // TODO inline ?
{
    ranger_finder* ranger = &rangers[pos];

    if(rising) { // Début du pulse, on enregistre le temps.
         ranger->pulse_start = time;
    }
    else { // Fin du pulse.
        char new_state;
        char state_changed;

        ranger->value = time - ranger->pulse_start;
        // marche aussi si le timer a débordé car non signé

        new_state = (ranger->value < ranger->threshold);
        state_changed = (ranger->state != new_state);

        if(ranger->unmuted || state_changed)
        {
            ranger->state = new_state;

            while(!CANSendMessage(352 | pos | state_changed << 42 /*TODO*/ | new_state << 55, (BYTE*)ranger.value, 2,
                CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME )) {
            }
            led = led ^ 1;
        }
    }
}

void check_button(char pos, char pin)
{
    if(pin == button_values[pos]) // Évènement sur la pin.
    {
        button_values[pos] = !pin; // Résistances de pull-up donc niveaux inversés.
        // TODO
        while(!CANSendMessage(258  | pos | !pin << 4, NULL, 0,
            CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME )) {
        }
        led = led ^ 1;
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
    // Les sharps actualisent leur valeur toutes les 40ms, mais on va les
    // échantillonner plus vite (38Hz), tant mieux si on choppe les nouvelles
    // valeurs rapidement.

    // Interruptions sur les pins "Echo output" des sonars (AN0 et AN1).
    // OpenRBxINT faits à la main (sauf pullup, par défaut).

    // TODO marche pas
    OpenADC(ADC_FOSC_4 // Tosc < 5.7 MHz.
            & ADC_LEFT_JUST // On pourra ignorer l'octet de poid faible (2 bits non nuls).
            & ADC_20_TAD, // 15µs * 5 MHz / 4 [cf ci-dessus] = 18.75 Tosc
            ADC_CH0 // Changé par le timer de toute façon.
            & ADC_INT_ON & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS,
            0b00001011); // Pins analogiques pour AN0 à AN3.

    // Interruption ADC.
    IPR1bits.ADIP = 0; // Priorité basse.
    PIE1bits.ADIE = 1; // Activée.
    PIR1bits.ADIF = 0;

    // Configuration du CAN.
    CANInitialize(1, 5, 7, 6, 2, CAN_CONFIG_VALID_STD_MSG);
    // Configuration des masques et filtres.
    CANSetOperationMode(CAN_OP_MODE_CONFIG);
    // Set Buffer 1 Mask value.
    CANSetMask(CAN_MASK_B1, 0b00100000000, CAN_CONFIG_STD_MSG);
    CANSetMask(CAN_MASK_B2, 0xFFFFFF, CAN_CONFIG_STD_MSG);
    // Set Buffer 1 filter values.
    CANSetFilter(CAN_FILTER_B1_F1, 0b00100000000, CAN_CONFIG_STD_MSG);
    CANSetFilter(CAN_FILTER_B1_F2, 0b00100000000, CAN_CONFIG_STD_MSG);
    // Set CAN module into Normal mode.
    CANSetOperationMode(CAN_OP_MODE_NORMAL);

    // Interruption Buffer 0.
    IPR3bits.RXB0IP = 0; // Priorité basse.
    PIE3bits.RXB0IE = 1; // Activée.
    PIR3bits.RXB0IF = 0;

    // Signal de démarrage.
    led = 0;
    for(i = 0; i < 20; i++) {
        led = led ^ 1;
        DelayMS(50);
    }

    // Autorisation des interruptions.
    RCONbits.IPEN = 1;
    INTCONbits.GIEH = 1;
    INTCONbits.GIEL = 1;


    while(1) {
    }
}
