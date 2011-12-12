/////////////////////////////////////////////////////////////////////////////
///////////////////////// ROBOT ASCENSEUR ///////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

#include <p18f2550.h>
#include <delays.h>
#include <i2c.h>

//Fonctionnement : on compare en permanence la "position" effective à la commande de "consigne"
//Si la consigne > position, alors on monte, sinon on descend
//Si consigne = position, alors on passe RA2 à 1 pour indiquer qu'on est à la position désirée.
//Voir avec Baptiste pour I2C

// 1) Vpp -> N.C.
// 2) Ra0 -> N.C.
// 3) Ra1 -> N.C.
// 4) Ra2 -> N.C.
// 5) Ra3 -> N.C.
// 6) Ra4 -> N.C.
// 7) Ra5 -> N.C.
// 8) Vss -> Masse (voir pin 19)
// 9) Osc1/Clki -> N.C.
// 10) Clk0 -> N.C.
// 11) T13Ckl -> N.C.
// 12) Rc1 -> Bornes 1 du pont en H (sens montée)
// 13) Rc2 -> Bornes 2 du pont en H (sens descente)
// 14) Vusb -> N.C.
// 15) Rc4 -> N.C.
// 16) Rc5 -> N.C.
// 17) Rc6 -> N.C.
// 18) Rc7 -> Capteur inférieur de la pince
// 19) Masse (voir pin 8)
// 20) Vdd -> Alimentation positive
// 21) Int0 -> I²C data
// 22) Int1 -> I²C clock
// 23) Int2 -> Broche A du codeur rotatif optique
// 24) Rb3 -> Broche B du codeur rotatif optique
// 25) Rb4 -> N.C.
// 26) Rb5 -> Sortie de retour
// 27) Rb6 -> N.C.
// 28) Rb7 -> N.C.

#define TOLERANCE 50    //Tolérance sur la consigne
#define PLATEAU 1000    //Nombre d'impulsions par plateau
#define CONSIGNE_MAX 8000    // Hauteur maximale de l'ascenseur
#define PIN_MONTEE PORTCbits.RC1    //Pin de montée (Attention, dépend de TRISx)
#define PIN_DESCENTE PORTCbits.RC2 //Pin de descente (Attention, dépend de TRISx)
#define PIN_RETOUR PORTBbits.RB5  //Pin de retour (Attention, dépend de TRISx)
#define PIN_CAPTEUR PORTCbits.RC7 //Pin du capteur bas de la pince (Attention, dépend de TRISx)
#define ADRESSE_PIC 0b10000010;  //Adresse I²C du PIC (la doc dit une adresse de 7 bits ????? A verifier)

#define longTram 3 //Ne pas modifier (nécessaire pour le buffer I2C)


//**************************************************************************************//
//*************************************** CONFIG ***************************************//
//**************************************************************************************//

// Définition des fonctions à utiliser
void Mouvement(void);
void Reception_i2c(void);
void Remise_a_zero(void);
void Monter(void);
void Descendre(void);
void Stop_moteur(void);
void Configuration_PIC(void);


// Configuration préprocesseur
#pragma config PLLDIV = 1 // Pas de prescale division
#pragma config CPUDIV = OSC1_PLL2 // Pas de postscale division car oscillateur interne
#pragma config USBDIV = 1 // On s'en fout de l'USB !!

#pragma config FOSC = INTOSC_EC //Mode INTCKO (Internal oscillator, CLKOUT on RA6, EC used by USB)
#pragma config FCMEN = OFF // Fail-Safe Clock Monitor désactivé (inutile pour oscillateur interne)
#pragma config IESO = OFF // Oscillator Switchover mode désactivé (même chose)

#pragma config PWRT = OFF // Power-up Timer désactivé
#pragma config BOR = OFF // Brown-Out Reset désactivé
#pragma config BORV = 0 // Brown-Out Reset niveau maximum (inutile)
#pragma config VREGEN = OFF // Régulateur de tension USB désactivé

#pragma config WDT = OFF // Watchdog-Timer désactivé
#pragma config WDTPS = 1 // Postscaler du Watchdog 1:1 (inutile)

#pragma config MCLRE = ON // Master Clear désactivé
#pragma config LPT1OSC = ON // Timer 1 configuré faible consommation
#pragma config PBADEN = OFF // Porte B configuré en digital
#pragma config CCP2MX = ON // CCP2 = RC1

#pragma config STVREN = OFF // Pas de reset en cas d'Overflow
#pragma config LVP = OFF // Pas de programmation en-circuit
#pragma config XINST = OFF // On se contentera des fonctions de base
#pragma config DEBUG = ON // Pas de débogage en-circuit

#pragma config CP0 = OFF // Pas de protection des blocs 0 à 3
#pragma config CP1 = OFF // Pas de protection des blocs 0 à 3
#pragma config CP2 = OFF // Pas de protection des blocs 0 à 3
#pragma config CP3 = OFF // Pas de protection des blocs 0 à 3

#pragma config CPB = OFF // Pas de protection du bloc de boot
#pragma config CPD = OFF // Pas de protection de l'EEPROM

#pragma config WRT0 = OFF // Pas de protection en écriture des blocs 0 à 3
#pragma config WRT1 = OFF // Pas de protection en écriture des blocs 0 à 3
#pragma config WRT2 = OFF // Pas de protection en écriture des blocs 0 à 3
#pragma config WRT3 = OFF // Pas de protection en écriture des blocs 0 à 3

#pragma config WRTB = OFF // Pas de protection en écriture du secteur de boot
#pragma config WRTC = OFF // Pas de protection en écriture des blocs de configuration
#pragma config WRTD = OFF // Pas de protection en écriture des blocs EEPROM

#pragma config EBTR0 = OFF // Pas de protection table de lecture (?)
#pragma config EBTR1 = OFF // Pas de protection table de lecture (?)
#pragma config EBTR2 = OFF // Pas de protection table de lecture (?)
#pragma config EBTR3 = OFF // Pas de protection table de lecture (?)

#pragma config EBTRB = OFF // Pas de protection table de lecture du secteur de boot (?)
 







// Définition des interruptions
#pragma code highVector=0x008
void HIGH(void)
	{
	_asm GOTO Reception_i2c _endasm
	}
#pragma code lowVector=0x018
void LOW(void)
	{
	_asm GOTO Mouvement _endasm
	}
#pragma code








//**************************************************************************************//
//**************************************** CORPS ***************************************//
//**************************************************************************************//

// Définition des variables
long consigne = 0;
long position = 0;
char MONTER = 'u';		// Chaîne de caractères I²C pour monter
char DESCENDRE = 'd';      //Chaîne de caractères I²C pour descendre
char RESET = 'r';

/*Buffer de reception I2C*/
char buffer[3];
char *i = &buffer[longTram];



void main(void)
	{

	Configuration_PIC(); // On fait la configuration du PIC
	Remise_a_zero(); //Remise à zero de l'ascenseur

	while(1)
		{
		Nop();
		if (position < consigne - TOLERANCE)
			{
			if (!((PIN_MONTEE == 1) && (PIN_DESCENTE == 0) && (PIN_RETOUR == 0)))
				{
				Monter();
				}
			}

		else if ((consigne - TOLERANCE <= position) && (position <= consigne + TOLERANCE)) //Régler la tolérance
			{
			if (!((PIN_MONTEE == 0) && (PIN_DESCENTE == 0) && (PIN_RETOUR == 1)))
				{
				Stop_moteur();
				}
			}

		else // (position > consigne + TOLERANCE)
			{
			if (!((PIN_MONTEE == 0) && (PIN_DESCENTE == 1) && (PIN_RETOUR == 0)))
				{
				Descendre();
				}
			}
		Nop();
		}
	}








//**************************************************************************************//
//************************************ INTERRUPTIONS ***********************************//
//**************************************************************************************//

#pragma interrupt Mouvement
void Mouvement(void)
	{
	if (INTCON3bits.INT2IF) // Si l'interruption INT2 se produit. On sait que on vient d'avoir un front montant de A
		{
		if (PORTBbits.RB3 == 0) //Ca veut dire qu'on est en train de monter
			{
			position++;
			}
		else if (PORTBbits.RB3 == 1) //Ca veut dire qu'on est en train de descendre
			{
			position--;
			}
		INTCON3bits.INT2IF = 0; //On lève l'interruption
		}
	}



#pragma interrupt Reception_i2c
void Reception_i2c(void)
	{
	if(PIE1bits.SSPIE && PIR1bits.SSPIF)
		{
		
		*i = SSPBUF ; // on stocke le byte
			
		if(!SSPSTATbits.D_A) // si c'était l'addresse
			i = buffer;
		else // si c'était de la donnée
			i++;


		SSPCON1bits.SSPOV=0;

		if (buffer[0] == MONTER)
			{
			consigne += PLATEAU;    //On ordonne de monter !
			if (consigne >= CONSIGNE_MAX) //Si la consigne est trop grande, on la met au maximum
				consigne = CONSIGNE_MAX;
			}
		else if (buffer[0] == DESCENDRE)
			{
			consigne += -PLATEAU;   //On ordonne de descendre !
			if (consigne <= 0) //Si la consigne est négative, on la remet à zéro
				consigne = 0;
			}
		else if (buffer[0] == RESET)
			{
			consigne = 0;
			}




		i = buffer;
		buffer[0] = 0 ;
		PIR1bits.SSPIF = 0 ; // on réautorise l'interruption
		//#TODO : Que faire dans les autres cas ?
		}
	}



//**************************************************************************************//
//*************************************** FONCTIONS ************************************//
//**************************************************************************************//

void Remise_a_zero(void)
{
	//--- Remise à zéro de l'ascenseur jusqu'à ce que la pince soit au niveau du sol ---//

	PIN_DESCENTE = 1;		// On fait descendre l'ascenceur
	
	while (!(PIN_CAPTEUR)) //Jusqu'à ce que le capteur inférieur de la pince se déclenche
		{Nop();}

	PIN_DESCENTE = 0;		//On stoppe l'ascenseur

	Delay1KTCYx(2); //On attend 1ms

	position = 0;
	consigne = 0;
	
	//On active les interruptions
	INTCONbits.PEIE = 1; 	  //Enables all low priority peripheral interrupts
	INTCONbits.GIE = 1;  	  //Enables all high priority interrupts
}


void Monter(void)
{
	PIN_DESCENTE = 0;
	PIN_MONTEE = 1; //On monte
	PIN_RETOUR = 0; //On n'est PAS à la position de consigne
}


void Descendre(void)
{
	PIN_MONTEE = 0;
	PIN_DESCENTE = 1; //On descend
	PIN_RETOUR = 0; //On n'est PAS à la position de consigne
}


void Stop_moteur(void)
{
	PIN_MONTEE = 0; //On éteint tout
	PIN_DESCENTE = 0; //On éteint tout
	PIN_RETOUR = 1; //On est à la position de consigne
}



//**************************************************************************************//
//***************************** CONFIGURATION DU PIC ***********************************//
//**************************************************************************************//

void Configuration_PIC(void)
	{
	BSR = 15;

	// OSCCON : Réglage de l'oscillateur interne
		OSCCONbits.IDLEN = 0;	//Device enters Sleep mode on SLEEP instruction
		OSCCONbits.IRCF0 = 1;   //8 MHz (INTOSC drives clock directly)
		OSCCONbits.IRCF1 = 1;
		OSCCONbits.IRCF2 = 1;
		OSCCONbits.SCS0 = 1;	//Internal oscillator block
		OSCCONbits.SCS1 = 1;

	// RCON : Réglage du reset
		RCONbits.SBOREN = 0; // On désactive le BOR
		RCONbits.IPEN = 1; // On active la hiérarchisation de la priorité d'un interrupt

	// INTCON : Réglage des interruptions
		INTCONbits.PEIE = 0; 	  //Disables all low priority peripheral interrupts
		INTCONbits.GIE = 0;  	  //Disables all high priority interrupts
		INTCONbits.TMR0IE = 0;    //Disables the TMR0 overflow interrupt
		INTCONbits.INT0IE = 0;    //Disables the INT0 external interrupt
		INTCONbits.RBIE = 0;      //Disables the RB port change interrupt
		INTCONbits.TMR0IF = 0;    //Timer0 bit status
		INTCONbits.INT0IF = 0;    //Int0 bit status
		INTCONbits.RBIF = 0;      //PortB interrupt-on-change status

	// INTCON2 : Réglage des interruptions
		INTCON2bits.RBPU = 0;     //PORTB pull-ups are enabled by individual port latch values
		INTCON2bits.INTEDG0 = 0;  //Interrupt0 on falling edge
		INTCON2bits.INTEDG1 = 0;  //Interrupt1 on falling edge
		INTCON2bits.INTEDG2 = 1;  //Interrupt2 on rising edge
		INTCON2bits.TMR0IP = 0;   //Timer0 priority low
		INTCON2bits.RBIP = 0;     //PortB interrupt-on-change low priority

	// INTCON3 : Réglage des interruptions
		INTCON3bits.INT2IP = 0;   //Int2 low priority
		INTCON3bits.INT1IP = 0;   //Int1 low priority
		INTCON3bits.INT2IE = 1;   //Enables Int2
		INTCON3bits.INT1IE = 0;   //Disables Int1
		INTCON3bits.INT2IF = 0;   //Int2 bit status
		INTCON3bits.INT1IF = 0;   //Int1 bit status

	// HLVDCON : Réglage du High/Low-Voltage Detect module
		HLVDCONbits.HLVDEN = 0;	  //HLVD disabled

	// WDTCON : Réglage du Watchdog Timer
		WDTCONbits.SWDTEN = 0;	  //Watchdog Timer is off

	// T0CON : Réglage du Timer0
		T0CONbits.TMR0ON = 0;    //Désactivation du Timer0

	// T1CON : Réglages du Timer 1
		T1CONbits.TMR1ON = 0;	  //Disables Timer1

	// T2CON : Réglages du Timer2
		T2CONbits.TMR2ON = 0;	  //Timer2 is off

	// T3CON : Réglages du Timer3
		T3CONbits.TMR3ON = 0;  //Stops Timer3

	// TRISx : Réglages des I/O
		TRISA = 0b00000000; // Paramétrage de la porte A 
		TRISB = 0b00001111; // Paramétrage de la porte B - RB3, RB2, RB1 et RB0 en Input (notamment RB0, RB1 car I²C), RB5 en Output
		TRISC = 0b10000000; // Paramétrage de la porte C - RC1, RC2 en Output, RC7 en Input

	// SSP -> FAIRE LES REGLAGES NECESSAIRES

	// ADCON0/1/2 : Réglages du convertisseur analogique
		ADCON0 = 0;	  //A/D converter module is disabled
		ADCON1bits.PCFG0 = 1; //Réglage des ports
		ADCON1bits.PCFG1 = 1;
		ADCON1bits.PCFG2 = 1;
		ADCON1bits.PCFG3 = 1;

	// CCP1/2CON : Réglages de l'ECCP
		CCP1CON = 0b00000000; // Désactivation de l'ECCP1
		CCP2CON = 0b00000000; // Désactivation de l'ECCP2

	// BAUDCON : Réglages du BAUD rate
		BAUDCONbits.ABDEN = 0; //Baud rate measurement disabled or completed
	
	// CVRCON : Réglages du comparateur
		CVRCONbits.CVREN = 0;  //CVref circuit powered down
		CVRCONbits.CVROE = 0;  //CVref voltage is disconnected from the RA2/AN2/Vref-/CVref pin

	// PIRx : Réglages des interruptions périphériques
		PIR1 = 0b00000000; // Nettoyage des bits d'interruption
		PIR2 = 0b00000000; // Nettoyage des bits d'interruption

	// PIEx : Activation des interruptions périphériques
		PIE1bits.SSPIE = 1;   //Activation de l'interruption sur I²C
		PIE1bits.TMR1IE = 0;  //Désactivation de l'interrution sur débordement du Timer1
		OpenI2C(SLAVE_7, SLEW_OFF);
		SSPADD = ADRESSE_PIC ;  //Addresse du pic
		SSPCON1bits.CKP = 1;  //On relache l'horloge

	// IPRx : Réglages priorités d'interruption
		IPR1bits.SSPIP = 1;  //I²C high interrupt
		IPR1bits.TMR1IP = 0; //Timer1 low interrupt

	// OSCTUNE : Ajustements de l'oscillateur
		OSCTUNEbits.INTSRC = 0;		//31 kHz device clock derived directly from INTRC internal oscillator
		OSCTUNEbits.TUN0 = 0;  //Center frequency
		OSCTUNEbits.TUN1 = 0;
		OSCTUNEbits.TUN2 = 0;
		OSCTUNEbits.TUN3 = 0;
		OSCTUNEbits.TUN4 = 0;
	
	// USB : Réglages
		UIE = 0b00000000; // Désactivation des interruptions USB
		UEIE = 0b00000000; // Désactivation des interruptions erreurs USB

	// PORTx : Nettoyage des I/O
		PORTA = 0;
		PORTB = 0;
		PORTC = 0;
		PORTE = 0;
	}