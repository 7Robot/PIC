///INCLUDES///
#include <stdio.h>
#include <p18f2550.h>
#include <i2c.h>
#include <timers.h>
#include <delays.h>
#include <pwm.h>
#define idle PORTCbits.RC0
#define ledv PORTCbits.RC1
#define longTram 2

///CONFIG BITS///
#pragma config FOSC = HS
#pragma config FCMEN = ON
#pragma config IESO = OFF
#pragma config PWRT = OFF
#pragma config BOR = ON
#pragma config BORV = 2
#pragma config VREGEN = ON
#pragma config WDT = OFF
#pragma config MCLRE = ON
#pragma config LPT1OSC = OFF
#pragma config PBADEN = OFF
#pragma config LVP = OFF
#pragma config DEBUG = ON
#pragma config CCP2MX = ON



/////VARIABLES GLOBALES ////

/*Buffer de reception I2C*/
char tab[longTram] ;
char *i = &tab[0] ;
char masque ;

/*Phases du moteur A sur RA2,3,4,5*/
char a[5] =  {0b00000100,0b00010000,0b0001000,0b00100000,'\0'} ;
	
/*Phases du moteur B sur RB2,3,4,5*/
char b[5] =  {0b00000100,0b00010000,0b0001000,0b00100000,'\0'} ;
	
/*Pointeurs position courante*/
char * apos= &a[0];
char * bpos= &b[0];

/* données de mouvement*/
unsigned char maitreA ;
unsigned char npas ; 
unsigned char inv ;
unsigned char ratio,j ;
unsigned char vit ;
unsigned char x = 0, test[16] ;
int mli = 0 ;


//////*PROTOTYPES*////////
void high_isr(void);
void i2c_isr(void);
void nextA(void);
void prevA(void);
void nextB(void);
void prevB(void);
unsigned char lin(unsigned char vit );


/////*INTERRUPTIONS*/////
#pragma code high_vector=0x08
void high_interrupt(void)
{
     _asm GOTO high_isr _endasm
}
#pragma code

#pragma interrupt high_isr
void high_isr(void)
{
	if(PIE1bits.SSPIE && PIR1bits.SSPIF)
	{
		if(SSPSTATbits.R_W) // lecture
        {
            ledv = ledv^1;
            putsI2C("reponse");
        }
        else // ecriture
	        {
				*i = SSPBUF ; // on stocke le byte
					
				if(!SSPSTATbits.D_A) // si c'était l'addresse
				{
					i=tab; // on initialise le pointeur	
				}
				/*if(*i == 'r')
				{
					Reset();
				}*/
				else // si c'était de la donnée
				{
					i++; // on pointe la case suivante
				}
		
				if(i - &tab[0] == longTram)
					{
					// initialisation des données de mouvement
					maitreA = tab[0] & 0b10000000;
					maitreA = maitreA >> 7 ;
				
					npas = tab[0] & 0b01111111 ;
				
					inv = tab[1] & 0b10000000 ;
					inv = inv >> 7 ;
						
					ratio = tab[1] & 0b01110000 ;
					ratio = ratio >> 4 ;
					j = ratio ;
					
					mli = 1023 ;
					vit = tab[1] & 0b00001111 ;
					vit = lin(vit) ;
					/*
					if(mli <= 1) //basse vitesse
					{
					mli = 500;
					}
					else
					{
					mli = 1023; // 850 + mli*10;
					}*/
					
					SetDCPWM1(mli);
						
					idle = 0 ; // on n'est plus inactif
					
					if(npas != 0)
					{
						INTCONbits.TMR0IE = 1 ; //on réautorise les interruptions timer0
						WriteTimer0(vit) ; //// A REGLER !!!!! 
					}
					else // relâcher les moteurs
					{
						PORTA = PORTA & 0b11000011;
						PORTB = PORTB & 0b11000011;
					}
				}
			}
				SSPCON1bits.SSPOV=0;
		PIR1bits.SSPIF = 0 ; // on réautorise l'interruption
	}
	
	if(INTCONbits.TMR0IE & INTCONbits.TMR0IF)
	{
		char motA = 0;
		char motB = 0;
			
		WriteTimer0(vit);
		
		if(ratio == 1) // avancer/recluler
		{
			motA = 0b00000001 + inv	;
			motB = 0b00000001 + inv	;
		}
		
		if(ratio == 0) // pivoter
		{
			if(maitreA)
			{
				motA = 0b00000001 + inv ;
				motB = 0b00000010 - inv ;
			}
			else
			{
				motB = 0b00000001 + inv ;
				motA = 0b00000010 - inv ;
			}
		}
		else // tourner
		{
			if(maitreA) // moteur A maitre
			{
				motA = 0b00000001 + inv ;
				j-- ;
				if(j) motB = 0;
				else
				{
					motB = 0b00000001 + inv ;
					j = ratio;
				}
			}
			else // moteur B maitre
			{
				motB = 0b00000001 + inv ;
				j-- ;
				if(j) motA = 0;
				else
				{
					motA = 0b00000001 + inv ;
					j = ratio;
				}
			}	
		}
		
		if(motA == 0b00000001) nextA();
		if(motA == 0b00000010) prevA();
		if(motB == 0b00000001) nextB();
		if(motB == 0b00000010) prevB();
		
		npas--;
		if(npas==0)
		{
			INTCONbits.TMR0IE = 0 ;
			idle = 1 ;
			//PORTA = PORTA & 0b11000011;
			//PORTB = PORTB & 0b11000011;
		}
		INTCONbits.TMR0IF = 0 ; //on réautorise l'interruption
	}
}	


////////*PROGRAMME PRINCIPAL*////////

void main (void)
{
//initialisations
	//CMCON=  0b00000111; // turn off comparators
	ADCON0= 0b00000000;
	ADCON1= 0x0F ;//0b00001111;
	WDTCON = 0 ;
	OSCCON = 0b01111111;
	
	UCON = 0 ;            // turn off usb
	UCFG = 0b00001000 ;

	TRISA = 0b11000011 ;
	TRISB = 0b00000011 ; // SCL et SDA en entrées
	TRISC = 0b11111000;
	PORTA = 0b11000011 ;
	PORTB = 0b11000011 ;
	PORTC = 0b00000000 ;
//	i = &tab[0];



//i2c interrupt

	PIE1bits.SSPIE = 1; 
	PIR1bits.SSPIF = 0; //Clear any pending interrupt
	OpenI2C(SLAVE_7, SLEW_OFF);
	SSPADD = 0b10000000 ; //addresse du pic
	SSPCON1bits.CKP = 1; //On relache l'horloge
	

//timer0
	OpenTimer0(TIMER_INT_ON & T0_SOURCE_INT & T0_8BIT & T0_PS_1_256);
	INTCONbits.TMR0IE = 0 ; // activé par la réception d'une trame
	idle = 1 ;  	// on est inactif au démarrage

//PWM

	OpenTimer2(TIMER_INT_OFF & T2_PS_1_1 & T2_POST_1_1);
	OpenPWM1(0xF0);
	SetDCPWM1(0); //500
	
//autoriser les interruptions
	RCONbits.IPEN=1;
	INTCONbits.GIE = 1;
 	INTCONbits.PEIE = 0;

	SSPBUF = 0 ;
	SSPCON1bits.SSPOV=0;
	
	
	
	
	while(1)
	{
	
	}
}


///////*FONCTIONS*///////
void nextA(void)
{
	apos++;
	if(apos > &a[3]) // si on est au bout
	{
	apos = &a[0];
	masque = PORTA & 0b11000011 ; // masquage
	PORTA = masque | *apos ;
	}
	else
	{
	masque = PORTA & 0b11000011 ; // masquage
	PORTA = masque | *apos ;
	}
}

void prevA(void)
{
	apos--;
	if(apos < &a[0]) // si on est au bout
	{
	apos = &a[3];
	masque = PORTA & 0b11000011 ; // masquage
	PORTA = masque | *apos ;
	}
	else
	{
	masque = PORTA & 0b11000011 ; // masquage
	PORTA = masque | *apos ;
	}
}
void nextB(void)
{
	bpos++;
	if(bpos > &b[3]) // si on est au bout
	{
	bpos = &b[0];
	masque = PORTB & 0b11000011 ; // masquage
	PORTB = masque | *bpos ;
	}
	else
	{
	masque = PORTB & 0b11000011 ; // masquage
	PORTB = masque | *bpos ;
	}
}

void prevB(void)
{
	bpos--;
	if(bpos < &b[0]) // si on est au bout
	{
	bpos = &b[3];
	masque = PORTB & 0b11000011 ; // masquage
	PORTB = masque | *bpos ;
	}
	else
	{
	masque = PORTB & 0b11000011 ; // masquage
	PORTB = masque | *bpos ;
	}
}

unsigned char lin(unsigned char vit ){
    unsigned char tinit;
    switch(vit)
    {
     case 0:
        tinit=0;
        mli = 0;
        break;
    case 1:
        tinit=105;
        mli =0;
         break;
    case 2:
        tinit=150;
        mli = 1000;
         break;
    case 3:
        tinit=173;
         break;
    case 4:
        tinit=195;
         break;
    case 5:
        tinit=202;
         break;
    case 6:
        tinit=210;
         break;
    case 7:
        tinit=214;
         break;
    case 8:
        tinit=218;
         break;
    case 9:
        tinit=222;
         break;
    case 10:
        tinit=227;
         break;
    case 11:
        tinit=228;
         break;
    case 12:
        tinit=230
        ;
         break;
    case 13:
        tinit=231;
         break;
    case 14:
        tinit=232; 
         break;
    case 15:
        tinit=233;
         break;

    }
    return(tinit);
}
