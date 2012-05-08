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
void check_sonar(char, char);
void check_button(char, char);

unsigned int LectureAnalogique(void); // Fonctionne de AN0 à AN4.

/////*VARIABLES GLOBALES*/////
int i;

// Derniers états des interrupteurs.
char switches[4] = {0}; // Capteurs d'enfoncement.


typedef struct {
    char unmuted; // Broadcast désactivé pour 0.
    char state; // 1 pour au dessous du seuil.
    unsigned int threshold; // Seuil désactivé pour 0.
    unsigned int value;

    // Spécifique aux sonars.
    unsigned int pulse_start; // Ticks comptés depuis le début de l'echo.
} ranger_finder;

volatile ranger_finder rangers[6] = {0}; // Sonar ou Sharp.


// Le sharp en cours de lecture (0 à 3).
char cur_sharp = 2;  //cur_sharp va de 2 à 5.




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
        INTCONbits.INT0IF = 0;
        check_sonar(0, INTCON2bits.INTEDG0);
        INTCON2bits.INTEDG0 ^= 1; // On écoutera l'autre sens.
    }

    if(INTCON3bits.INT1IE && INTCON3bits.INT1IF)
    {
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
        char num;
        short cmd;
        BYTE data[8];
        BYTE len;
        enum CAN_RX_MSG_FLAGS flags;

        while(CANIsRxReady()) {
            CANReceiveMessage(&id, data, &len, &flags);
        }
        PIR3bits.RXB0IF = 0;

        led = led ^ 1;

        num = id & 0x07;
        cmd = id & 0xFFF8;

        if(cmd == 320) { // rangerReq
            id = 352 | (rangers[num].value < rangers[num].threshold) << 4 | num;
            while(!CANSendMessage(id, (BYTE*)rangers[num].value, 2,
                CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME )) {
            }
        }
        else if(cmd == 328) { // rangerThres
            rangers[num].threshold = ((unsigned int*) data)[0];
        }
        else if(cmd == 336) { // rangerMute
            rangers[num].unmuted = 0;
        }
        else if(cmd == 344) { // rangerUnmute
            rangers[num].unmuted = 1;
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
        check_button(2, PORTCbits.RC4);
        check_button(3, PORTCbits.RC5);

        // On alterne les triggers pulse, et pas besoin
        // d'attendre plus car ils restent allumés la moitié du temps (50ms).
        if(PORTCbits.RC7) {
            CloseRB1INT();
            OpenRB0INT(PORTB_CHANGE_INT_ON & RISING_EDGE_INT & PORTB_PULLUPS_OFF);

            PORTCbits.RC6 = 1;
            PORTCbits.RC7 = 0; // Fin du pulse => déclenchement.
        }
        else {
            CloseRB0INT();
            OpenRB1INT(PORTB_CHANGE_INT_ON & RISING_EDGE_INT & PORTB_PULLUPS_OFF);

            PORTCbits.RC6 = 0;
            PORTCbits.RC7 = 1; // Fin du pulse => déclenchement.
        }
        // Début de l'attente des echos.

        // Lecture de l'ADC pour les sharps et passage à la mesure suivante.
        rangers[cur_sharp].value = LectureAnalogique();

        if(rangers[cur_sharp].unmuted) {
            while(!CANSendMessage(352 | cur_sharp, (BYTE*)&(rangers[cur_sharp].value), 2,
                    CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME )) {
            }
            led = led ^ 1;
        }

        if(++cur_sharp > 5)
            cur_sharp = 2;

        // Lancement de la conversion suivante.
        ADCON0 = 0b00000001 + ((cur_sharp - 2) << 2); // Selectionne le bon AN en lecture analogique



    //ADCON2 peut être configuré si on le souhaite

    ADCON0bits.GO_DONE=1;



        //ADCON0bits.CHS = cur_sharp; // Plus simple que SetChanADC(). //TODO
        //ConvertADC(); //TODO
    }   
}


void check_sonar(char num, char rising)
{
    if(rising) { // Début du pulse, on enregistre le temps.
         rangers[num].pulse_start = time;
    }
    else { // Fin du pulse.
        char new_state;
        char state_changed;

        rangers[num].value = time - rangers[num].pulse_start;
        // marche aussi si le timer a débordé car non signé
        // TODO échelle

        new_state = (rangers[num].value < rangers[num].threshold);
        state_changed = (rangers[num].state != new_state);

        if(rangers[num].unmuted || state_changed)
        {
            rangers[num].state = new_state;

            while(!CANSendMessage(352 | new_state << 4 | state_changed << 3 | num, (BYTE*)&(rangers[num].value), 2,
                CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME )) {
            }
            led = led ^ 1;
        }
    }
}

void check_button(char num, char pin)
{
    if(pin == switches[num]) // Évènement sur la pin.
    {
        switches[num] = !pin; // Résistances pull-up => niveaux inversés.
        while(!CANSendMessage(272 | (!pin << 3) | num, NULL, 0,
            CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME )) {
        }
        led = led ^ 1;
    }
}

unsigned int LectureAnalogique(){
    // ATTENTION, ne marche que de AN0 à AN4.
    // pin = 0 => on sélectionne AN0, pin = 1 => AN1 etc...


    unsigned int tempo = 0;
    unsigned int val = 0;
    val=ADRESL;           // Get the 8 bit LSB result
    val=ADRESL>>6;
    tempo=ADRESH;
    tempo=tempo<<2;         // Get the 2 bit MSB result
    val = val + tempo;

    return (val);
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

    ADCON1 = 0b00001011; // Configuration ADC de AN0 à AN3

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

    /*OpenADC(ADC_FOSC_4 // Tosc < 5.7 MHz.
            & ADC_LEFT_JUST // On pourra ignorer l'octet de poid faible (2 bits non nuls).
            & ADC_20_TAD, // 15µs * 5 MHz / 4 [cf ci-dessus] = 18.75 Tosc
            ADC_CH0 // Changé par le timer de toute façon.
            & ADC_INT_ON & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS,
            0b00001011); // Pins analogiques pour AN0 à AN3.


    // Interruption ADC.
    IPR1bits.ADIP = 0; // Priorité basse.
    PIE1bits.ADIE = 1; // Activée.
    PIR1bits.ADIF = 0;
//*/
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