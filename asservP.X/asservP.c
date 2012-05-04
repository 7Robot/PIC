/*
 * Programme d'asservissement vitesse et position du */
#define GROS
 /*  robot
 * Eurobot 2012
 * Compiler : Microchip C18
 * �C : 18f25K80
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
#include <delays.h>
#include <p18f25k80.h>
#include <portb.h>
#include <timers.h>
#include <pwm.h>
#include <math.h>
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



/////*Prototypes*/////
void GsetDC(int dc);
void DsetDC(int dc);
void high_isr(void);
void low_isr(void);
void DelayMS(int delay);
void Vconsigne(int Vg, int Vd);
void resetTicks(void);

/////*CONSTANTES*/////
#define FCY 16000000 // Fr�quence d'ex�cution des instructions = 4*16Mhz/4 = 16MHz.

#define led LATAbits.LATA5


#define in1  LATCbits.LATC5
#define in2  LATCbits.LATC4
#define in3  LATCbits.LATC3
#define in4  LATCbits.LATC2

#define GchA PORTBbits.RB0
#define GchB PORTBbits.RB4
#define DchA PORTBbits.RB1
#define DchB PORTBbits.RB5

#define riseGchA INTCON2bits.INTEDG0
#define riseDchA INTCON2bits.INTEDG1

#ifdef GROS
    #define TourRoue  4096 // Ticks par tour de roue (th�orique).
    #define TourRobot 17726 // Ticks pour 360 degres (th�orique).
    #define Vmax 200        // Vitesse max
    //#define TicksAcc 3*TourRoue // Pente des rampes.
    #define Kp 2
    #define Ki 5
    #define Kd 0
#else
    #define TourRoue  3072 // Ticks par tour de roue (th�orique).
    #define TourRobot 8424 // Ticks pour 360 degres (th�orique).
    #define Vmax 80        // Vitesse de plateau des rampes.
    //#define TicksAcc 3*TourRoue // Pente des rampes.
    #define Kp 10
    #define Ki 10
    #define Kd 8
#endif



/////*VARIABLES GLOBALES*/////
unsigned int i = 0;

// Variables calcul vitesse.
long gTicks = 0, dTicks = 0; // Compteurs incr�mentaux. cf Interruptions PORTB0,1,4,5.
long dTicksp = 0, gTicksp = 0; // Valeurs interm�diaires pour calcul vitesse. cf interrupt TMR0.
int gVitesse = 0, dVitesse = 0; // Vitesses instantan�es.
char Gsens = 1, Dsens = 1; // Sens de rotation (+1 ou -1).
char prevB = 0; // Valeur pr�c�dente du PORTB, pour d�terminer type de front.

// Variables asserv vitesse PI(D).
int gConsigne = 0, dConsigne = 0; // Consignes de vitesse des moteurs gauche et droit.
int dErreur = 0, gErreur = 0, gIErreur = 0, dIErreur = 0; // Termes d'erreurs P et I.
int dDErreur = 0, dErreurP = 0, gDErreur = 0, gErreurP = 0; // Termes D.
int Emax = 0; // Erreur maxi pour r�glages.

// Variables asserv position P.
long r = 0; // Position rectiligne ou angulaire.
int Rconsigne = 0;
long Eposition = 0; // Consigne et erreur de position (anglulaire ou rectiligne).
char mode = 0; // Mode de fonctionnement 0:off, 1:ligne, -1:rotation.
char aru = 0; // proc�dure d'arr�t d'urgence
//int  aruResidu = 0; // distance parcourue depuis derni�re consigne 1025 ou 1026
BYTE residu = 0; // Erreur r�siduelle
int convi = 0;
int dVFinale = 0, gVFinale = 0; // Pour changement de vitesse avec rampe.

//Pour ARM
long ticksElie=0;

// Variables CAN inutile mais cool.
CANmsg message;


/////*Interruptions*/////
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
    /*
    Les interruptions de haut niveau concernent uniquement le comptage des ticks,
    en effet, elles sont fr�quentes et ne doivent pas �tre "manqu�es" pour assurer
    de la pr�cision dans l'asserv de position.
    
    Elles sont tr�s rapides et bien qu'elles soient asynchrones les unes par rapport
    aux autres, il est peu probable que deux d'entre elles d�clenchent en m�me temps.
     */

    if (INTCONbits.INT0IE && INTCONbits.INT0IF) { // Interruptions gA
        if (riseGchA == GchB)
            gTicks++;
        else
            gTicks--;

        riseGchA ^= 1; // Change le sens de sensibilit�.
        INTCONbits.INT0IF = 0;
    }

    if (INTCON3bits.INT1E && INTCON3bits.INT1IF) { // Interruptions dA
        if (riseDchA == DchB)
            dTicks--;
        else
            dTicks++;

        riseDchA ^= 1; // Change le sens de sensibilit�.
        INTCON3bits.INT1IF = 0;
    }

    if (INTCONbits.RBIE && INTCONbits.RBIF) { // Interruptions gB et dB
        char newB = PORTB;
        INTCONbits.RBIF = 0; // On autorise t�t l'interruption suivante pour ne rien rater.

        if ((newB ^ prevB) & 0b00010000) { // front sur gB
            if (GchB == GchA)
                gTicks--;
            else
                gTicks++;
        } else if ((newB ^ prevB) & 0b00100000) { // front sur dB
            if (DchB == DchA)
                dTicks++;
            else
                dTicks--;
        }

        prevB = newB; // Sauvegarde de la valeur du PORTB.
    }
}

#pragma interrupt low_isr

void low_isr(void) {
    if (INTCONbits.TMR0IE && INTCONbits.TMR0IF) {
        /* L'interruption TMR0 g�re la fr�quence de calcul des asservs. Elle soit �tre assez
        rapide pour assurer de ma r�activit�, mais pas trop longue pour collecter assez de ticks
        entre deux calculs. La fr�quence est fix�e � environ 30Hz. */

        /* Calcul vitesse */
        gVitesse = gTicks - gTicksp;
        dVitesse = dTicks - dTicksp;
        gTicksp = gTicks;
        dTicksp = dTicks;
        
#ifdef GROS
        gVitesse = gVitesse;
        dVitesse = dVitesse;
#endif

        /* Calcul erreur vitesse */
        dErreur = dConsigne - dVitesse;
        gErreur = gConsigne - gVitesse;

        dIErreur += dErreur;
        gIErreur += gErreur;

        dDErreur = dErreur - dErreurP;
        gDErreur = gErreur - gErreurP;
        dErreurP = dErreur;
        gErreurP = gErreur;

        if (dVitesse > Emax)
            Emax = dVitesse;

        /* Commande des moteurs */
#ifdef GROS
        GsetDC(Kp * gErreur + (Ki*gIErreur)/10 + Kd * gDErreur);
        DsetDC(Kp * dErreur + (Ki*dIErreur)/10 + Kd * dDErreur);
#else
        GsetDC(Kp * gErreur + Ki * gIErreur + Kd * gDErreur);
        DsetDC(Kp * dErreur + Ki * dIErreur + Kd * dDErreur);
#endif

        /* Calcul profil droit ou rotation. */
        if ((mode == 1 || mode == -1) && !aru) {
            r = (gTicks + dTicks) / 2;
            if(mode==-1)
                r = (gTicks - dTicks) / 2;
            Eposition = sqrt(fabs(Rconsigne) - fabs(2 * fabs(r) - fabs(Rconsigne)));
#ifdef GROS
            Eposition = 2 * Eposition ;
            if (Eposition == 0 && r < 100)
                Eposition = 10;
#else
            Eposition = 3 * Eposition / 2;
            if (Eposition == 0 && r < 20)
                Eposition = 1;
#endif
            if (Eposition > Vmax)
                Eposition = Vmax;

            if (Rconsigne < 0)
                Eposition = -Eposition;

            if(mode == 1)
                Vconsigne(Eposition, Eposition);
            else
                Vconsigne(Eposition, -Eposition);

            if (gConsigne == 0 || dConsigne == 0 || fabs(r - Rconsigne) < 100) {
                residu = fabs(r - Rconsigne);
                Vconsigne(0, 0);
                resetTicks();
                mode = 0;
                led = led^1;
                CANSendMessage(1040, &residu, 1,
                        CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME); //Idle, byte quelconque
            }
        }


        /* Changement de vitesse avec rampe */
        if (mode == 2 && !aru) {
            if (dConsigne < dVFinale) {
                dConsigne += 2;
            }
            else if (dConsigne > dVFinale) {
                dConsigne -= 2;
            }
            if (gConsigne < gVFinale) {
                gConsigne += 2;
            }
            else if (gConsigne > gVFinale) {
                gConsigne -= 2;
            }
        }

        /*Arr�t d'urgence*/
        if(aru)
        {
            //Calcul de la distance parcourue depuis de debut de la consigne
            if(mode == 1)
                r = (gTicks + dTicks) / 2;
            if(mode == -1)
                r = (gTicks - dTicks) / 2;

            //Arr�t
            if (dConsigne < 0) {
                dConsigne += 3;
            }
            else if (dConsigne > 0) {
                dConsigne -= 3;
            }
            if (gConsigne < 0) {
                gConsigne += 3;
            }
            else if (gConsigne > 0) {
                gConsigne -= 3;
            }
            if(dConsigne < 3 && dConsigne > -3)
                dConsigne = 0;
            if(gConsigne < 3 && gConsigne > -3)
                gConsigne = 0;

            //renvoi position
            if(dConsigne == 0 && gConsigne == 0)
            {
                if(mode == 1)
                    CANSendMessage(1041, &r, 2,
                        CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME);
                if(mode == -1)
                    CANSendMessage(1042, &r, 2,
                        CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME);
                aru  = 0;
                mode = 0;                
            }
            
        }

        INTCONbits.TMR0IF = 0;
    }

    if (PIE5bits.RXB0IE && PIR5bits.RXB0IF) {

        while (CANIsRxReady()) {
            CANReceiveMessage(&message.id, message.data, &message.len, &message.flags);
        }

        led = led ^ 1;

        switch (message.id) {
            case 1032: // Consigne en vitesse
                mode = 0;
                Vconsigne(((char*)&message.data)[0], ((char*)&message.data)[1]);
                break;

            case 1033: // Changement de vitesse par rampe
                mode = 2;
                if (mode != 2) {
                    resetTicks();
                }
                gVFinale = ((char*)&message.data)[0];
                dVFinale = ((char*)&message.data)[1];
                if (gVFinale > Vmax)
                    gVFinale = Vmax;
                else if (gVFinale < -Vmax)
                    gVFinale = -Vmax;
                if (dVFinale > Vmax)
                    dVFinale = Vmax;
                else if (dVFinale < -Vmax)
                    dVFinale = -Vmax;
                break;

            case 1025: // Consigne ligne
                resetTicks();
                mode = 1;
                Rconsigne = ((int*)&message.data)[0];
                break;

            case 1026: // Consigne rotation
                resetTicks();
                mode = -1;
                Rconsigne = ((int*)&message.data)[0];
                break;

            case 1028: // Arr�t
                INTCONbits.TMR0IE = 0;
                mode = 0;
                break;

            case 1029: // Marche
                resetTicks();
                mode = 0;
                INTCONbits.TMR0IE = 1;
                break;

            case 1151: // Stop
                aru = 1;
                break;

            case 1043: //reset ticks counter
                resetTicks();
                ticksElie = 0;
                break;

            case 1044:
                resetTicks();
                CANSendMessage(1045, &ticksElie, 4,
                        CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME);
                break;

            default:
                // Rien
                break;
        }

        PIR5bits.RXB0IF = 0;
        PIR5bits.ERRIF = 0;
    }
}


/////*PROGRAMME PRINCIPAL*/////

void main(void) {
    /* Initialisations. */
    ADCON1 = 0x0F;
    ADCON0 = 0b00000000;
    ANCON1 = 0x00;
    WDTCON = 0;

    /* Configurations. */
    TRISA = 0x00;
    TRISB = 0xFF;
    TRISC = 0b00000011;
    PORTC = 0x00;

    /* Configuration des PWM */
    OpenTimer2(TIMER_INT_OFF & T2_PS_1_4 & T2_POST_1_1);
    OpenPWM3(0xFF, 2);
    OpenPWM4(0xFF, 2);
    Delay10KTCYx(255);
    SetDCPWM3(0);
    SetDCPWM4(0);

    /*Interruptions du PORTB (haute priorit� par d�faut) */
    OpenRB0INT(PORTB_CHANGE_INT_ON & RISING_EDGE_INT & PORTB_PULLUPS_OFF);
    OpenRB1INT(PORTB_CHANGE_INT_ON & RISING_EDGE_INT & PORTB_PULLUPS_OFF);
    OpenPORTB(PORTB_CHANGE_INT_ON & PORTB_PULLUPS_OFF);
    IOCB = 0b00110000;

    /*Timer0 interrupt pour calculs asserv*/
#ifdef GROS
    OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_4);
#else
    OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_4);
#endif

    INTCON2bits.TMR0IP = 0; /*Basse priorit� pour ne pas rater de ticks */

    /*Configuration du CAN*/
    CANInitialize(1, 16, 7, 6, 2, CAN_CONFIG_VALID_STD_MSG);
    Delay10KTCYx(200);
    // Configuration des masques et filtres
    // Set CAN module into configuration mode
    CANSetOperationMode(CAN_OP_MODE_CONFIG);
    // Set Buffer 1 Mask value
    CANSetMask(CAN_MASK_B1, 0b10000000000, CAN_CONFIG_STD_MSG);
    // Set Buffer 2 Mask value
    CANSetMask(CAN_MASK_B2, 0xFFFFFF, CAN_CONFIG_STD_MSG);
    // Set Buffer 1 Filter values
    CANSetFilter(CAN_FILTER_B1_F1, 0b10000000000, CAN_CONFIG_STD_MSG);
    CANSetFilter(CAN_FILTER_B1_F2, 0b10000000000, CAN_CONFIG_STD_MSG);
    // Set CAN module into Normal mode
    CANSetOperationMode(CAN_OP_MODE_NORMAL);
    // Interruption Buffer 0
    IPR5bits.RXB0IP = 0; // : priorit� basse du buff 0
    PIE5bits.RXB0IE = 1; //autorise int sur buff0
    PIR5bits.RXB0IF = 0; //mise a 0 du flag
    // Interruptions Buffer1
    PIE5bits.RXB1IE = 0; //Interdite


    /* Signal de d�marrage du programme. */
    led = 0;
    for (i = 0; i < 20; i++) {
        led = led ^ 1;
        DelayMS(50);
    }

    RCONbits.IPEN = 1; /* On active les niveaux d'interruption... */
    INTCONbits.GIEH = 1; /* Autorisation des interruptions de haut niveau. */
    INTCONbits.GIEL = 1; /* Autorisation des interruptions de bas niveau. */


    while (1);
}

///// D�finition des fonctions du programme. /////

void DelayMS(int delay) {
    /*Attente en ms, sur cette carte c'est utile, et vu que le Quart est soud�,
     il y a peu de raisons pour que �a change...*/
    int cp = 0;
    for (cp = 0; cp < delay; cp++) {
        Delay1KTCYx(16);
    }
}

/*Les fonctions GsetDC et DsetDC permettent la commande du pont en H int�gr� � la carte.
 En plus de modifier le rapport cyclique des PWM3 et 4 elles modifient le sens de rotation
 du moteur (ie. la tension � ses bornes) en fonction du signe de l'argument (dc). */
void GsetDC(int dc) {
    if (dc >= 0) {
        in1 = 1;
        in2 = 0;
    } else {
        in1 = 0;
        in2 = 1;
        dc = -dc;
    }

    if (dc > 1023)
        dc = 1023;

    SetDCPWM4(dc);
}

void DsetDC(int dc) {
    if (dc > 0) {
        in3 = 0;
        in4 = 1;
    }
    else {
        in3 = 1;
        in4 = 0;
        dc = -dc;
    }
    if (dc > 1023)
        dc = 1023;

    SetDCPWM3(dc);
}

void Vconsigne(int Vg, int Vd) {
    /* Consigne de vitesse pour les moteurs entre 0 et Vmax.
       Permet de g�rer la saturation en vitesse. */
    if (Vg >= Vmax)
        gConsigne = Vmax;
    else
        gConsigne = Vg;

    if (Vd >= Vmax)
        dConsigne = Vmax;
    else
        dConsigne = Vd;
}

void resetTicks(void) {
    /* Reset le comptage des ticks, utile entre deux consignes de position... */
    /* Attention, on reset aussi l'asserv !*/
    if((dConsigne*gConsigne)<0)
        ticksElie += (gTicks - dTicks) / 2;
    else
        ticksElie += (gTicks + dTicks) / 2;
    
    gTicks = 0;
    dTicks = 0;
    gTicksp = 0;
    dTicksp = 0;

    INTCONbits.TMR0IE = 0; // TODO: wut ?
    dErreur = 0;
    gErreur = 0;
    gIErreur = 0;
    dIErreur = 0;
    INTCONbits.TMR0IE = 1;
}
