#include <stdio.h>
#include <stdlib.h>
#include <delays.h>
#include <p18f25k80.h>
#include <portb.h>
#include <timers.h>
#include <pwm.h>

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

#define Kp 10
/////*VARIABLES GLOBALES*/////
unsigned int i=0;
long gTicks=0, dTicks=0;
long dTicksp=0, gTicksp=0;
int gVitesse=0, dVitesse=0;
int cg = 0, cd=0;


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

    //Encodeur gauche
    if(INTCONbits.INT0IE && INTCONbits.INT0IF) //GchA interrupt
    {
        if(GchB) gTicks ++;
        else gTicks--;

        INTCONbits.INT0IF = 0;
    }

   /* if(INTCONbits.RBIE && INTCONbits.RBIF && GchB) //GchB interrupt
    {
        INTCONbits.RBIF = 0; // au debut au cas ou DchB redeclenche

        if(!GchA) gTicks ++;
        else gTicks--;
    }*/


    //Encodeur droit
    if(INTCON3bits.INT1IE && INTCON3bits.INT1IF) //DchA interrupt
    {
        if(!DchB) dTicks ++;
        else dTicks--;
        INTCON3bits.INT1IF = 0;
    }

   /* if(INTCONbits.RBIE && INTCONbits.RBIF && DchB) //DchB interrupt
    {
        INTCONbits.RBIF = 0; // au debut au cas ou GchB redeclenche
       if(DchA) dTicks ++;
        else dTicks--;
    }*/

    //Calcul asserv
    if(INTCONbits.TMR0IE && INTCONbits.TMR0IF)
    {

        gVitesse = gTicks - gTicksp;
        dVitesse = dTicks - dTicksp;

        gTicksp = gTicks;
        dTicksp = dTicks;

        GsetDC(Kp*(cg-gVitesse));
        DsetDC(Kp*(cg-dVitesse));
        led = led^1;
        INTCONbits.TMR0IF = 0; //On efface tous les flags pour faire comme
        INTCONbits.INT0IF = 0; // si rien ne c'etait passé
        INTCON3bits.INT1IF = 0;
        INTCONbits.RBIF = 0;
    }

}

#pragma interrupt low_isr
void low_isr(void)
{

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
    OpenPORTB( PORTB_CHANGE_INT_ON & PORTB_PULLUPS_ON);
    IOCB = 0b00110000;

    /*Timer0 interrupt pour calcul asserv*/
    OpenTimer0( TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_8 );


    INTCONbits.GIE = 1;


   
       
        cg = 25 ;
        cd = 25 ;
        for(i=0;i<=10;i++) Delay10KTCYx(255);
        cd=0;
        cg=0;
        while(1);
  
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
