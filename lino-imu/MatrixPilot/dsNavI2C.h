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


/* 
 * File:   dsNavI2C.h
 * Author: guiott
 *
 * Created on 5 febbraio 2013, 17.36
 */

#define I2C_BUFF_SIZE_TX 8
#define I2C_BUFF_SIZE_RX 26

#ifndef DSNAVI2C_H
#define	DSNAVI2C_H
#endif	/* DSNAVI2C_H */



#define DSNAV_COMMAND 0X24

void I2C_doneRxDsnavData( boolean I2CtrxOK );
void I2C_doneTxDsnavData( boolean I2CtrxOK );
void I2C_rxDsnav(boolean I2CtrxOK);  // receive from the dsNav board
void I2C_txDsnav(void);  // transmit to the dsNav board

#define I2C_Normal              I2C2_Normal
#define I2C_Read                I2C2_Read
#define I2C_Write               I2C2_Write
#define I2C_reset               I2C2_reset



