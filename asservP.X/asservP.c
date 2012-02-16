/*
 * Programme d'asservissement vitesse et position du petit robot
 * Eurobot 2012
 * Compiler : Microchip C18
 * µC : 18f25K80
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
#define FCY 16000000  /*  Fréquence d'exécution des instructions = 4*16Mhz/4 = 16MHz. */

#define led LATAbits.LATA5 //Essayer de mettre PORT à la place
                            // ne devrait rien changer avec le jeu d'instruction réduit...


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

#define TourRoue 3072   /* Ticks par tour de roue (théorique) */
#define TourRobot 8424  /* Ticks pour 360 degres (théorique) */
#define Vmax 80         /* Vitesse de plateau des rampes. */
//#define TicksAcc 3*TourRoue /* Pente des rampes. */

#define Kp 10
#define Ki 10

#define Kr 1 // Conversion ticks-mm




/////*VARIABLES GLOBALES*/////
unsigned int i=0; /* Variable à usage général... */

/*Variables calcul vitesse */
long gTicks=0, dTicks=0;    /* Compteurs incrémentaux. cf Interruptions PORTB0,1,4,5 */
long dTicksp=0, gTicksp=0;  /* Valeurs intérmédaires pour calcul vitesse. cf interrupt TMR0*/
int gVitesse=0, dVitesse=0; /* Vitesses instantanées. */
char Gsens = 1, Dsens = 1;  /* Sens de rotation (+1 ou -1)*/
char prevB = 0;             /*Valeur précédente du PORTB, pour déterminer type de front*/

/* Variables asserv vitesse PI(D) */
int gConsigne = 0, dConsigne=0;     /* Consignes de vitesse des moteurs gauche et droit. */
int dErreur=0, gErreur=0, gIErreur=0, dIErreur=0; /* Termes d'erreurs P et I*/

/* Variables asserv position P */
long r = 0; /* Position rectiligne ou angulaire. */
long Rconsigne=0, Eposition=0; /*Consigne et erreur de position (anglulaire ou rectiligne)*/
char mode=0; /* Mode de fonctionnement 0:off, 1:ligne, -1:rotation */

/*Variables CAN*/
CANmsg message;


/////*Interruptions*/////
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
   /*Les interruptions de haut niveau concernent uniquement le comptage des ticks,
    en effet, elles sont fréquentes et ne doivent pas être "manquées" pour assurer
    de la précision dans l'asserv de position.
    
    Elles sont très rapides et bien qu'elles soient asynchrones les unes par rapport
    aux autres, il est peu probables que deux d'entre elles déclenchent en même temps.
    */

   /* Interruptions GchA, detecte fronts, determine le sens, et in(dé)cremente. */
   if(INTCONbits.INT0IE && INTCONbits.INT0IF)
   {
       Gsens = -1;
       if((riseGchA && GchB) || (!riseGchA && !GchB)) /* Si front montant. */
          Gsens = 1;
       riseGchA = riseGchA^1; /* Inversion du type de front déclenchant l'interruption. */
       gTicks += Gsens;
       INTCONbits.INT0IF = 0;
   }

   /* Interruptions DchA, deteche fronts, determine sens, et in(dé)crémente*/
   if(INTCON3bits.INT1E && INTCON3bits.INT1IF)
   {

       Dsens = 1;
       if((riseDchA && DchB) || (!riseDchA && !DchB)) /* Si front montant. */
          Dsens = -1;

       riseDchA = riseDchA^1; /* Inversion du type de front déclenchant l'interruption. */
       dTicks += Dsens;
       INTCON3bits.INT1IF = 0;
   }

   /*Interruptions GchB et DchB, détecte fronts, utilise le sens déterminé par les autres*/
   if(INTCONbits.RBIE && INTCONbits.RBIF)
   {
       if((PORTB^prevB) & 0b00010000) gTicks += Gsens;      /* Si GchB a déclenché. */
       else if((PORTB^prevB) & 0b00100000) dTicks += Dsens; /* Si DchB a déclenché. */
       prevB = PORTB; /* Sauvegarde de la valeur du PORTB */
       INTCONbits.RBIF = 0; 
   }   
}

#pragma interrupt low_isr
void low_isr(void)
{
    
    if(INTCONbits.TMR0IE && INTCONbits.TMR0IF)
    {
     /*L'interruption TMR0 gère la fréquence de calcul des asservs. Elle soit être assez
     rapide pour assurer de ma réactivité, mais pas trop longue pour collecter assez de ticks
     entre deux calculs. La fréquence est fixée à environ 30Hz.*/
        
        /* Calcul vitesse*/
        gVitesse = gTicks - gTicksp;
        dVitesse = dTicks - dTicksp;
        gTicksp = gTicks;
        dTicksp = dTicks;

        /* Calcul erreur vitesse*/
        dErreur = dConsigne - dVitesse;
        gErreur = gConsigne - gVitesse;

        dIErreur += dErreur;
        gIErreur += gErreur;

        /* Commande des moteurs */
        GsetDC(Kp*gErreur + Ki*gIErreur);
        DsetDC(Kp*dErreur + Ki*dIErreur);

        
        /* Calcul profil droit. */
       if(mode == 1)
        {
            r = Kr*(gTicks+dTicks)/2;
            Eposition = fabs(Rconsigne)-fabs(2*fabs(r)-fabs(Rconsigne));
            Eposition = Eposition/50;
            if(Eposition > Vmax) Eposition = Vmax;
            if(Eposition == 0 && r < 100) Eposition = 1;
            if(Rconsigne < 0) Eposition = -Eposition;
            Vconsigne(Eposition,Eposition);

            if(fabs(r-Rconsigne) < 30)
            {   
                //DelayMS(100);
                //resetTicks();
                //Rconsigne = 0;
                //position=0;
                //r=0;
                //Eposition=0;
                
                Vconsigne(0,0);
                resetTicks();
                mode = 0;
                CANSendMessage(1028,&prevB,1,
                        CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME ); //Idle, byte quelconque



            }
        }

        /* Calcul profil rotation. */
        if(mode == -1)
        {
            r = Kr*(gTicks-dTicks)/2;
            Eposition = fabs(Rconsigne)-fabs(2*fabs(r)-fabs(Rconsigne));
            Eposition = Eposition/50;
            if(Eposition > Vmax) Eposition = Vmax;

            if(Eposition == 0 && r < 100) Eposition = 1;
            if(Rconsigne < 0) Eposition = -Eposition;

            Vconsigne(Eposition,-Eposition);

            if(fabs(r-Rconsigne) < 30)
            {
                //DelayMS(100);
                //resetTicks();
                //Rconsigne = 0;
                //position=0;
                //r=0;
                //Eposition=0;
                led = 1;
                Vconsigne(0,0);
                resetTicks();
                mode = 0;
                CANSendMessage(1028,&prevB,1,
                        CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME ); //Idle, byte quelconque

            }
        }

        INTCONbits.TMR0IF = 0;
    }
    
    if( PIE5bits.RXB0IE && PIR5bits.RXB0IF)
    {

       while(CANIsRxReady()){
        CANReceiveMessage(&message.id,message.data,&message.len,&message.flags);
        }

       switch (message.id) {
                    case 1029: //Consigne en vitesse
                        mode = 0;
                        led  = led^1;
                        Vconsigne(message.data[0],message.data[1]);
                      break;

                    case 1025: //Consigne ligne
                        led  = led^1;
                        mode = 1;
                        Rconsigne = 0;
                        Rconsigne = (255*message.data[0]) | message.data[1]; /*Poid des bits inversé...*/
                        /*Pas beau mais permet de gérer le signe...*/
                      break;

                    case 1026: //Consigne rotation
                        led  = led^1;
                        mode = -1;
                        Rconsigne = 0;
                        Rconsigne = (255*message.data[0]) | message.data[1]; /*Poid des bits inversé...*/
                        /*Pas beau mais permet de gérer le signe...*/
                      break;

                    case 1030: //Marche-Arrêt
                        led = led^1;
                        if(message.data[0] == 'A')
                        {
                            INTCONbits.TMR0IE = 0;
                            mode = 0;
                        }
                        else if(message.data[0] == 'M')
                        {
                            resetTicks();
                            mode = 0;
                            INTCONbits.TMR0IE = 1;
                        }
                      break;

                    case 1051: //Stop
                        mode = 0;
                        led = led^1;
                        Vconsigne(0,0);
                      break;

                      
                    default:
                      // Rien
                      break;
                    }
       
        PIR5bits.RXB0IF=0;
        PIR5bits.ERRIF=0;
    }
    
}


/////*PROGRAMME PRINCIPAL*/////
void main (void)
{
    /* Initialisations. */
    ADCON1 = 0x0F ;
    ADCON0 = 0b00000000;
    ANCON1 = 0x00;
    WDTCON = 0 ;

    /* Configurations. */
    TRISA   = 0x00 ;
    TRISB   = 0xFF ;
    TRISC   = 0b00000011 ;
    PORTC = 0x00;
    
    /* Configuration des PWM */
    OpenTimer2(TIMER_INT_OFF & T2_PS_1_4 & T2_POST_1_1);
    OpenPWM3(0xFF,2);
    OpenPWM4(0xFF,2);
    Delay10KTCYx(255);
    SetDCPWM3(0);
    SetDCPWM4(0);

    /*Interruptions du PORTB (haute priorité par défaut) */
    OpenRB0INT( PORTB_CHANGE_INT_ON & RISING_EDGE_INT & PORTB_PULLUPS_OFF);
    OpenRB1INT( PORTB_CHANGE_INT_ON & RISING_EDGE_INT & PORTB_PULLUPS_OFF);
    OpenPORTB( PORTB_CHANGE_INT_ON & PORTB_PULLUPS_OFF);
    IOCB = 0b00110000 ;

    /*Timer0 interrupt pour calculs asserv*/
    OpenTimer0( TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_4 );
    INTCON2bits.TMR0IP = 0 ; /*Basse priorité pour ne pas rater de ticks */

    /*Configuration du CAN*/
    CANInitialize(1,16,7,6,2,CAN_CONFIG_VALID_STD_MSG);
    Delay10KTCYx(200);

    
    // Interruptions Buffer1
    IPR5bits.RXB1IP=0;// : priorité haute par defaut du buff 1
    PIE5bits.RXB1IE=0;//autorise pas  int sur buff1
    PIR5bits.RXB1IF=0;//mise a 0 du flag*/

    // Interruption Buffer 0
    IPR5bits.RXB0IP=0;// : priorité basse du buff 0
    PIE5bits.RXB0IE=1;//autorise int sur buff0
    PIR5bits.RXB0IF=0;//mise a 0 du flag


    // Configuration des masques et filtres
    // Set CAN module into configuration mode
    CANSetOperationMode(CAN_OP_MODE_CONFIG);
    // Set Buffer 1 Mask value
    CANSetMask(CAN_MASK_B1, 0b10000000000,CAN_CONFIG_STD_MSG);
    // Set Buffer 2 Mask value
    CANSetMask(CAN_MASK_B2, 0xFFFFFF ,CAN_CONFIG_STD_MSG );
    // Set Buffer 1 Filter values
    CANSetFilter(CAN_FILTER_B1_F1,0b10000000000,CAN_CONFIG_STD_MSG );
    CANSetFilter(CAN_FILTER_B1_F2,0b0000,CAN_CONFIG_STD_MSG );
    CANSetFilter(CAN_FILTER_B2_F1,0b0000,CAN_CONFIG_STD_MSG );
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

    RCONbits.IPEN = 1; /* On active les niveaux d'interruption... */
    INTCONbits.GIEH = 1; /* Autorisation des interruptions de haut niveau. */
    INTCONbits.GIEL = 1; /* Autorisation des interruptions de bas niveau. */


    while(1);

}

///// Définition des fonctions du programme. /////
void DelayMS(int delay)
{
    /*Attente en ms, sur cette carte c'est utile, et vu que le Quart est soudé,
     il y a peu de raisons pour que ça change...*/
    int cp = 0;
    for(cp=0; cp<delay; cp++)
    {
        Delay1KTCYx(16);
    }
}

/*Les fonctions GsetDC et DsetDC permettent la commande du pont en H intégré à la carte.
 En plus de modifier le rapport cyclique des PWM3 et 4 elles modifient le sens de rotation
 du moteur (ie. la tension à ses bornes) en fonction du signe de l'argument (dc). */
void GsetDC(int dc)
{

    if(dc >= 0)
    {
        in1 = 1;
        in2 = 0;
    }
    else
    {
        in1 = 0;
        in2 = 1;
        dc = -dc;
    }

   if(dc > 1023)
      dc = 1023;

    SetDCPWM4(dc);
}
void DsetDC(int dc)
{
     if(dc > 0)
    {
        in3 = 0;
        in4 = 1;
    }
    else
    {
        in3 = 1;
        in4 = 0;
        dc = -dc;
    }
    if(dc > 1023)
        dc = 1023;

    SetDCPWM3(dc);
}

void Vconsigne(int Vg, int Vd)
{
    /* Consigne de vitesse pour les moteurs entre 0 et Vmax.
       Permet de gérer la saturation en vitesse. */
    if(Vg >= Vmax) gConsigne= Vmax;
    else gConsigne = Vg;
    if(Vd >= Vmax) dConsigne= Vmax;
    else dConsigne = Vd;
}

void resetTicks(void)
{
    /*Reset le comptage des ticks, utile entre deux consignes de position...*/
    /* Attention, on reset aussi l'asserv !*/
    gTicks = 0;
    dTicks = 0;
    gTicksp = 0;
    dTicksp = 0;

    INTCONbits.TMR0IE = 0;
    dErreur=0;
    gErreur=0;
    gIErreur=0;
    dIErreur=0;
    INTCONbits.TMR0IE = 1;
}

