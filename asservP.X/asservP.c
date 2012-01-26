#include <stdio.h>
#include <stdlib.h>
#include <delays.h>
#include <p18f25k80.h>
#include <portb.h>
#include <timers.h>
#include <pwm.h>
#include <math.h>

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
#pragma config MSSPMSK = MSK5 /// A voir
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


#define TourRoue 3072 // ticks par tour de roue
#define Vmax 80
#define TicksAcc 3*TourRoue



#define Kp 10
#define Ki 10

#define Kr 1 // Conversion ticks-mm
 //Pente acc/decceleration

/////*VARIABLES GLOBALES*/////
unsigned int i=0;
long gTicks=0, dTicks=0, r=0;
long Rconsigne=0, Eposition=0;
long dTicksp=0, gTicksp=0;
int gVitesse=0, dVitesse=0;
int gConsigne = 0, dConsigne=0, dErreur=0, gErreur=0, gIErreur=0, dIErreur=0;
char Gsens = 1, Dsens = 1, prevB = 0,position=0;


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
   
   /* Interruptions GchA, detecte fronts, determine le sens, et in(dé)cremente*/
   if(INTCONbits.INT0IE && INTCONbits.INT0IF)
   {
       INTCONbits.INT0IF = 0; // flag au debut pour capter autre
       Gsens = -1;
       if((riseGchA && GchB) || (!riseGchA && !GchB)) //front montant
          Gsens = 1;

       riseGchA = riseGchA^1;
       gTicks += Gsens;
   }

   /* Interruptions DchA, deteche fronts, determine sens, et in(dé)crémente*/
   if(INTCON3bits.INT1E && INTCON3bits.INT1IF)
   {
       INTCON3bits.INT1IF = 0;
       Dsens = 1;
       if((riseDchA && DchB) || (!riseDchA && !DchB)) //front montant
          Dsens = -1;

       riseDchA = riseDchA^1;
       dTicks += Dsens;
   }

   /*Interruptions GchB et DchB, détecte fronts, utilise le sens déterminé par les autres*/
   if(INTCONbits.RBIE && INTCONbits.RBIF)
   {
       if((PORTB^prevB) & 0b00010000) gTicks += Gsens;
       else if((PORTB^prevB) & 0b00100000) dTicks += Dsens;
       prevB = PORTB; //très important
       INTCONbits.RBIF = 0; // mettre à la fin car modifier par PORTB
   }

    
}

#pragma interrupt low_isr
void low_isr(void)
{
    
    if(INTCONbits.TMR0IE && INTCONbits.TMR0IF)
    {
        /* Calcul vitesse*/
        gVitesse = gTicks - gTicksp;
        dVitesse = dTicks - dTicksp;
        gTicksp = gTicks;
        dTicksp = dTicks;

        /* Calcul erreur */
        dErreur = dConsigne - dVitesse;
        gErreur = gConsigne - gVitesse;

        dIErreur += dErreur;
        gIErreur += gErreur;

        /* Commande des moteurs */
        GsetDC(Kp*gErreur + Ki*gIErreur);
        DsetDC(Kp*dErreur + Ki*dIErreur);

        
        /* Calcul profil. */
        if(position)
        {
            r = Kr*(gTicks+dTicks)/2;
            Eposition = (Rconsigne-fabs(2*r-Rconsigne))/2;
            Eposition = Eposition/20 ;
            if(Eposition > Vmax) Eposition = Vmax;
            Vconsigne(Eposition+1,Eposition+1);
        }

        led = led^1;
        INTCONbits.TMR0IF = 0; //On efface tous les flags pour faire comme
        INTCONbits.INT0IF = 0; // si rien ne c'etait passé
        INTCON3bits.INT1IF = 0;
        INTCONbits.RBIF = 0;
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

    /*Interruptions portB*/
    OpenRB0INT( PORTB_CHANGE_INT_ON & RISING_EDGE_INT & PORTB_PULLUPS_OFF);
    OpenRB1INT( PORTB_CHANGE_INT_ON & RISING_EDGE_INT & PORTB_PULLUPS_OFF);
    OpenPORTB( PORTB_CHANGE_INT_ON & PORTB_PULLUPS_OFF);
    IOCB = 0b00110000 ;

    /*Timer0 interrupt pour calcul asserv*/
    OpenTimer0( TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_4 );
    INTCON2bits.TMR0IP = 0 ; //Basse priorité pour ne pas rater de ticks 
    /* Signal de démarrage du programme. */
    led = 0;
    for(i=0;i<20;i++)
    {
        led=led^1;
        DelayMS(50);
    }

    RCONbits.IPEN = 1; // priorités, par défaut : haute
    INTCONbits.GIEH = 1;
    INTCONbits.GIEL = 1;

    resetTicks();
    position = 1;
    Rconsigne = 1*TourRoue;


        while(1);
  
}


void DelayMS(int delay)
{
    int cp = 0;
    for(cp=0; cp<delay; cp++)
    {
        Delay1KTCYx(16);
    }
}

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
    /* Consigne de vitesse pour les moteurs entre 0 et Vmax. */
    gConsigne = Vg;
    dConsigne = Vd;
    if(Vg >= Vmax) gConsigne= Vmax;
    if(Vd >= Vmax) dConsigne= Vmax;
}

void resetTicks(void)
{
    gTicks = 0;
    dTicks = 0;
}