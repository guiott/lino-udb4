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

/*<GUIOTT>
 * This is the configuration for testPoint() usage.
 * It is used to verify with a scope the correct timing of critical routines.
 * It toggles the RA1 digital pin, so the period read on that pin is double the
 * real routine execution timing.
 * On each function to test, add the following code:

 //<GUIOTT>
 #if (XXX_TESTPOINT == 1)
    testPoint();
 #endif
 //</GUIOTT>
 *
 *
 * You must enable the routine under test by assigning a value of 1 to the
 * relative #define. Enable the routines once at a time to have a correct
 * functioning of the procedure.
 * if the test function is enabled the RA1 I/O pin is reserved and not useable
 * as SERVO_OUT_PIN_10
 </GUIOTT>*/

#include "libUDB.h"

#ifndef TESTPOINT_H
// switch to 0 the define below to disable testPoint and enable RA1 on SERVO_OUT_PIN_10
#define	TESTPOINT_H 0
#endif


// Enable the routines once at a time to have a correct functioning of the procedure.
#define TWO_HZ_TESTPOINT 0
#define HEARTBEAT_TESTPOINT 0
#define MAG_TESTPOINT 0


void testPoint(void);



