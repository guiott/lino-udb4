//<GUIOTT>

// copyright 2013 Guido Ottaviani
// guido@guiott.com

// This is my porting fron the original MatrixPilot software,
// starting from Version 3.3 R.1880 - Dec. 26th 2012
//
// I'm using the IMU to have a real time measurement of the current orientation
// for an outdoor 4WD rover: http://www.guiott.com/Lino/index.html
// to obtain an autonomous navigation like this one:
// http://www.seattlerobotics.org/encoder/200610/article3/IMU%20Odometry,%20by%20David%20Anderson.htm
// the using of the UDB4 is discussed her:http://www.guiott.com/Lino/IMU/IMU.htm
// it communicate with the basic motor control system for the 4 wheels:
// http://www.guiott.com/Lino/dsNav/dsNav.htm
// The high level navigation system (http://www.guiott.com/Lino/HLS/HLS.htm)
// controls the whole navigation, SLAM, obstacle avoidance and all the rest
//

//</GUIOTT>

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

#include "defines.h"

int main (void)
{
	udb_init() ;
	dcm_init() ;   
	init_servoPrepare() ;
	init_states() ;
  init_behavior() ;
	init_serial() ;

	udb_run() ;
	// This never returns.
	
	return 0 ;
}
