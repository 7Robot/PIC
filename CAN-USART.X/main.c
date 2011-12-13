#include <stdio.h>
#include <p18f2680.h>
#include <delays.h>
#include <usart.h>
#include "can18xx8.h"



#define XTAL 10000000
#define led PORTCbits.RC0

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



/////VARIABLES GLOBALES ////


/////VARIABLES GLOBALES ////
void high_isr(void);
void low_isr(void);


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
    if (PIE3bits.RXB0IE && PIR3bits.RXB0IF || PIE3bits.RXB1IE && PIR3bits.RXB1IF) {

        char data[8];
        unsigned long id;
        int len;
        enum CAN_RX_MSG_FLAGS flag;
        int i;

        while (CANIsRxReady()) { // Read all available messages.
            if(!CANReceiveMessage(&id, data, &len, &flag))
                break;
            
            led ^= 1;
            //Delay10KTCYx(200);

            // Traitement à faire ici.
            // [ FD ] [ size | 0 | id10..8 ] [ id7..0] [ M1 ] [ M2 ] ? [ M8 ] [ BF ]

            WriteUSART(0xFD);
            WriteUSART(len << 4 | id >> 8);
            WriteUSART(id & 0xFF);
            for(i = 1; i <= len && i <= 8; i++) {
                WriteUSART(data[i]);
            }
            WriteUSART(0xBF);

        }

       PIR3bits.ERRIF = 0;
        if (PIE3bits.RXB0IE)
            PIR3bits.RXB0IF = 0;
        if (PIE3bits.RXB1IE)
            PIR3bits.RXB1IF = 0;
    }
/*
    if (PIE1bits.RXIE && PIR1bits.RCIF) {
        unsigned char c = ReadUSART();
        
        
        if(bufferPos == -1 && c == 0xFD) { // (si c'est pas 0xFD on ignore)
            bufferPos = 0;
        }
        else if(bufferPos == 0) // [ size | 0 | id10..8 ]
        {
        }
        else if(bufferPos == 8) // 
        {
            unsigned long id = (buffer[0] & ) << ;
            CANSendMessage(id, data, buffer[0] >> , CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME);
        }
        else // paquet malformé, on ignore
            bufferPos--;


        buffer[bufferPos] = c;

        PIR1bits.RCIF = 0;
    }
*/
}
unsigned char buffer[10]; // Does not contain 0xFD and 0xFB.
int bufferPos = -1;

#pragma interrupt low_isr

void low_isr(void) {


}

void OpenCAN(unsigned long mask0, unsigned long filter00, unsigned long filter01, unsigned long mask1, unsigned long filter10, unsigned long filter11, unsigned long filter12, unsigned long filter13) {
    CANInitialize(1, 2, 6, 3, 2, CAN_CONFIG_VALID_STD_MSG);

    /*Config interupt CAN- Buffeur 0*/
    IPR3bits.RXB0IP = 1; // priorité haute
    PIE3bits.RXB0IE = 1; // autorise int sur buff1
    PIR3bits.RXB0IF = 0; // mise à 0 du flag

    /*Config interupt CAN- Buffeur 1*/
    IPR3bits.RXB1IP = 1; // priorité haute
    PIE3bits.RXB1IE = 1; // autorise int sur buff1
    PIR3bits.RXB1IF = 0; // mise à 0 du flag

    //config des mask et filtres
    // Set CAN module into configuration mode
    CANSetOperationMode(CAN_OP_MODE_CONFIG);
    // Set Buffer 1 Mask value
    CANSetMask(CAN_MASK_B1, mask0, CAN_CONFIG_STD_MSG);
    // Set Buffer 2 Mask value
    CANSetMask(CAN_MASK_B2, mask1, CAN_CONFIG_STD_MSG);
    // Set Buffer 1 Filter values
    CANSetFilter(CAN_FILTER_B1_F1, filter00, CAN_CONFIG_STD_MSG);
    CANSetFilter(CAN_FILTER_B1_F2, filter01, CAN_CONFIG_STD_MSG);
    CANSetFilter(CAN_FILTER_B2_F1, filter10, CAN_CONFIG_STD_MSG);
    CANSetFilter(CAN_FILTER_B2_F2, filter11, CAN_CONFIG_STD_MSG);
    CANSetFilter(CAN_FILTER_B2_F3, filter12, CAN_CONFIG_STD_MSG);
    CANSetFilter(CAN_FILTER_B2_F4, filter13, CAN_CONFIG_STD_MSG);

    // Set CAN module into Normal mode
    CANSetOperationMode(CAN_OP_MODE_NORMAL);
}

////////*PROGRAMME PRINCIPAL*////////

void main(void) {
    //initialisations
    ADCON1 = 0x0F;
    ADCON0 = 0b00000000;
    WDTCON = 0;

    /* Direction des ports I in, O out*/
    TRISA = 0b11111111;
    TRISB = 0b01111011; //canTX en sortie
    TRISC = 0b11111110;

    /* Etat des sorties */
    PORTA = 0b11111111;
    PORTB = 0b11111111;
    PORTC = 0b11111111;

    /* USART */
    OpenUSART(USART_TX_INT_OFF & USART_RX_INT_ON & USART_ASYNCH_MODE
            & USART_EIGHT_BIT & USART_CONT_RX & USART_BRGH_HIGH, 77);
    IPR1bits.RCIP = 1; // priorité haute comme le CAN pour pas qu'il s'interrompent l'un l'autre

    OpenCAN(0, 0, 0, 0, 0, 0, 0, 0);

    /*Config interupt General*/
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;

    CANSendMessage(77, NULL, 0, CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME);
    while (1) {
       Delay10KTCYx(5);
    
       CANSendMessage(77, NULL, 0, CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME);

    }
}
