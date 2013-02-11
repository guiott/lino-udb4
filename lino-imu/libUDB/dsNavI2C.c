// This file is part of MatrixPilot.
//
//    http://code.google.com/p/gentlenav/
//
// Copyright 2009-2011 MatrixPilot Team
// See the AUTHORS.TXT file for a list of authors of MatrixPilot.
//
// MatrixPilot is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// MatrixPilot is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with MatrixPilot.  If not, see <http://www.gnu.org/licenses/>.
#include "I2C.h"
#include "libUDB_internal.h"

//<GUIOTT>
// added I2C communication with dsNav board:
// (http://www.guiott.com/Lino/dsNav/dsNav.htm)


#if (BOARD_TYPE == UDB4_BOARD)

#if ( DSNAV_I2C == 1)

#include "dsNavI2C.h"

// TX Buffer
struct _TxBuff
{
    int VelDes; // mean desired speed mm/s
    int YawDes; // desired orientation angle (set point)(Degx10 0-3599)
    int YawMes; // measured orientation binary angle (process control) (Degx10 0-3599)
    char MasterFlag;// to set the dsNav board as a master
    char NewFlag;   // new values sent. Set at the end to close the cycle
};

union __TxBuff
{
    struct _TxBuff I;// to use as integers or chars, little endian LSB first
    unsigned char C[I2C_BUFF_SIZE_TX];  // to use as bytes to send on I2C buffer
}I2CTxBuff;


// RX Buffer
struct _RxBuff
{
    float PosXmes;  // current X position coordinate
    float PosYmes;  // current Y position coordinate
    int VelInt[4];  // speed in mm/s as an integer for all the wheels
    int ADCValue[4];// 64 sample average ADC also for slave
    unsigned char stasis_err;   // number of times imu and wheels very different
    unsigned char stasis_alarm; // signal too many stasis errors
};

union __RxBuff
{
    struct _RxBuff I;// to use as integers or chars, little endian LSB first
    unsigned char C[I2C_BUFF_SIZE_RX];  // to use as bytes to send on I2C buffer
}I2CRxBuff;

unsigned char DsNavReadIndex[] = {0x00} ;    // Address of the first register to read
unsigned char DsNavWriteIndex[] = {0x00} ;   // Address of the first register to read

void I2C_txDsnav(void) // Transmit the navigation data to the dsNav board
{
    static unsigned char TxCount = 0;

    I2CTxBuff.I.MasterFlag = 1;// to set the dsNav board as a master
    I2CTxBuff.I.NewFlag = 1;   // new values arrived. Set at the end to close the cycle
    if (TxCount > 3)
    {// recall the parameter reception only every 4 TX, i.e.: 25 x 4 = 100ms
        I2C_Write(DSNAV_COMMAND, DsNavWriteIndex, 1, I2CTxBuff.C, I2C_BUFF_SIZE_TX, &I2C_rxDsnav);
        TxCount = 0;
    }
    else
    {
        I2C_Write(DSNAV_COMMAND, DsNavWriteIndex, 1, I2CTxBuff.C, I2C_BUFF_SIZE_TX, &I2C_doneTxDsnavData);
        TxCount ++;
    }
}

void I2C_rxDsnav(boolean I2CtrxOK) // after transmission receive the parameters from the dsNav
{
    I2C_Read(DSNAV_COMMAND, DsNavReadIndex, 1, I2CRxBuff.C, I2C_BUFF_SIZE_RX, &I2C_doneRxDsnavData);
}

void I2C_doneRxDsnavData( boolean I2CtrxOK )
{       
    return ;
}

void I2C_doneTxDsnavData( boolean I2CtrxOK )
{
    return ;
}

#endif

#endif
