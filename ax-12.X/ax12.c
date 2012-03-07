/*
 *  ax12.cpp - C18 library to use the AX-12 servomotor (Dynamixel Series) from
 *  Robotis on the PIC18F family from Microchip.
 *
 *  Tested with PIC18F2550 under MPLAB X.
 *
 *  2011-09-20 - First version by Martin d'Allens <martin.dallens@gmail.com>
 *
 *  TODO : try using high impedances
 *  TODO : helpers for bauds
boolean inverse;

static void AX12init (long baud);
static void autoDetect (int* list_motors, byte num_motors);

void setEndlessTurnMode (boolean onoff);
void endlessTurn (int velocidad);
byte presentPSL (int* PSL);
 *
 *  This library is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <p18f25k80.h>
#include <usart.h>
#include "ax12.h"


/******************************************************************************
 * Wiring dependent functions, that you should customize
 ******************************************************************************/

void SetTX() {
    TX_EN = 1;
    RX_EN = 0;
}

void SetRX() {
    TX_EN = 0;
    RX_EN = 1;
}


/******************************************************************************
 * Functions to read and write command and return packets
 ******************************************************************************/

byte checksumAX;
struct {
    byte id;
    byte len;
    errorAX error;
    byte params[4]; // Could be larger.
} responseAX;
char responseReadyAX = 0;
char posAX = -4;

void PushUSART(byte b) {
    while (Busy1USART());
    Write1USART(b);
    checksumAX += b;
}

/*
 * Write the first bytes of a command packet, assuming a <len> parameters will
 * follow.
 */
void PushHeaderAX(byte id, byte len, byte inst) {
    SetTX();
    
    PushUSART(0xFF);
    PushUSART(0xFF);

    checksumAX = 0; // The first two bytes don't count.
    PushUSART(id);
    PushUSART(len + 2); // Bytes to go : instruction + buffer (len) + checksum.
    PushUSART(inst);
}

/* Write a buffer of given length to the body of a command packet. */
void PushBufferAX(byte len, byte* buf) {
    byte i;
    for (i = 0; i < len; i++) {
        PushUSART(buf[i]);
    }
}

/* Finish a command packet by sending the checksum. */
void PushFooterAX() {
    PushUSART(~checksumAX);
    while (Busy1USART());
    SetRX();
}

/**/
void InterruptAX() {
    if(PIE1bits.RCIE && PIR1bits.RCIF)
    {
        byte b = Read1USART();

        if(posAX == -4 && b == 0xFF)
            posAX = -3;
        else if(posAX == -3 && b == 0xFF) {
            posAX = -2;
            checksumAX = 0;
            responseAX.len = 1;
        }
        else if(posAX == -2) {
            posAX = -1;
            responseAX.id = b;
        }
        else if(posAX == -1 && b < 2 + 4 /*taille de ax.parameters*/) {
            posAX = 0;
            checksumAX = responseAX.id + b;
            responseAX.len = b - 2;
        }
        else if(0 <= posAX && posAX < responseAX.len) {
            ((byte*)&responseAX.params)[posAX++] = b;
            checksumAX += b;
        }
        else if(posAX == responseAX.len && b == ~checksumAX) {
            responseReadyAX = 1;
            posAX = -4;
        }
        else
            posAX = -4; // Erreur.

        // Sert à rien, mais Maxime insiste. Je dois être trop borné pour comprendre.
        PIR1bits.RCIF = 0;
    }
}


/******************************************************************************
 * Instructions Implementation
 ******************************************************************************/

void PingAX(byte id) {
    PushHeaderAX(id, 2, AX_INST_PING);
    PushFooterAX();
}

void ReadAX(byte id, byte address, byte len) {
    PushHeaderAX(id, 2, AX_INST_READ_DATA);
    PushUSART(address);
    PushUSART(len);
    PushFooterAX();
}

void WriteAX(byte id, byte address, byte len, byte* buf) {
    PushHeaderAX(id, 1 + len, AX_INST_WRITE_DATA);
    PushUSART(address);
    PushBufferAX(len, buf);
    PushFooterAX();
}

void RegWriteAX(byte id, byte address, byte len, byte* buf) {
    PushHeaderAX(id, 1 + len, AX_INST_REG_WRITE);
    PushUSART(address);
    PushBufferAX(len, buf);
    PushFooterAX();
}

void ActionAX(byte id) {
    PushHeaderAX(id, 0, AX_INST_ACTION);
    PushFooterAX();
}

void ResetAX(byte id) {
    PushHeaderAX(id, 0, AX_INST_RESET);
    PushFooterAX();
}


/******************************************************************************
 * Convenience Functions
 ******************************************************************************/

byte RegisterLenAX(byte address) {
    switch (address) {
        case  2: case  3: case  4: case  5: case 11: case 12: case 13: case 16:
        case 17: case 18: case 19: case 24: case 25: case 26: case 27: case 28:
        case 29: case 42: case 43: case 44: case 46: case 47:
            return 1;
        case  0: case  6: case  8: case 14: case 20: case 22: case 30: case 32:
        case 34: case 36: case 38: case 40: case 48:
            return 2;
    }
    return 0; // Unexpected.
}

/* Write a value to a registry, guessing its width. */
void PutAX(byte id, byte address, int value) {
    WriteAX(id, address, RegisterLenAX(address),
                   (byte*)&value /* C18 and AX12 are little-endian */);
}

/* Read a value from a registry, guessing its width. */
void GetAX(byte id, byte address) {
    ReadAX(id, address, RegisterLenAX(address));
}
