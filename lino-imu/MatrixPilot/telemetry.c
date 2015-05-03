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
#include "../libDCM/libDCM_internal.h" // Needed for access to internal DCM values
#include  "../TimeConvert/time_conversion.h"
#include "dsNavI2C.h"

#if (SERIAL_OUTPUT_FORMAT != SERIAL_MAVLINK) // All MAVLink telemetry code is in MAVLink.c

//Note:  The trap flags need to be moved out of telemetry.c and mavlink.c
volatile int trap_flags __attribute__ ((persistent));
volatile long trap_source __attribute__ ((persistent));
volatile int osc_fail_count __attribute__ ((persistent)) ;

#define _ADDED_C_LIB 1 // Needed to get vsnprintf()
#include <stdio.h>
#include <stdarg.h>


union intbb voltage_milis = {0} ;
union intbb voltage_temp ;

volatile int trap_flags __attribute__ ((persistent));
volatile long trap_source __attribute__ ((persistent));
volatile int osc_fail_count __attribute__ ((persistent));

// Time structure
extern struct _TimeS
{
    unsigned short utcYear;
    unsigned char utcMonth;
    unsigned char utcDay;
    unsigned char utcHour;
    unsigned char utcMinute;
    float utcSeconds;
};

extern union longbbbb SatIdList_gps , Hepe_gps;
extern union u_intbb UtcYear_gps , UtcSeconds_gps;
extern unsigned char UtcMonth_gps , UtcDay_gps , UtcHour_gps , UtcMinute_gps;

extern union __TimeS
{
    struct _TimeS I;// to use as integers or chars, little endian LSB first
    char C[TIME_S_SIZE];  // to use as bytes to send on I2C buffer
}TimeS;

// TX Buffer
extern struct _TxBuff
{
    int VelDes; // mean desired speed mm/s
    int YawDes; // desired orientation angle (set point)(Degx10 0-3599)
    int YawMesAbs;  // absolute value of measured orientation binary angle (process control) (Degx10 0-3599)
    int YawMesRel;  // measured orientation binary angle (process control) (Degx10 0-3599) relative to startup position
    char MasterFlag;// to set the dsNav board as a master
    char NewFlag;   // new values sent. Set at the end to close the cycle
    char OrientFlag;// change the dsNav orientation mode (direct or PID) according to external command
};


extern union __TxBuff
{
    struct _TxBuff I;// to use as integers or chars, little endian LSB first
    unsigned char C[I2C_BUFF_SIZE_TX];  // to use as bytes to send on I2C buffer
}I2CTxBuff;

// RX Buffer
extern struct _RxBuff
{
    float PosXmes;  // current X position coordinate
    float PosYmes;  // current Y position coordinate
    int VelInt[4];  // speed in mm/s as an integer for all the wheels
    int ADCValue[4];// 64 sample average ADC also for slave
    unsigned char stasis_err;   // number of times imu and wheels very different
    unsigned char stasis_alarm; // signal too many stasis errors
};

extern union __RxBuff
{
    struct _RxBuff I;// to use as integers or chars, little endian LSB first
    unsigned char C[I2C_BUFF_SIZE_RX];  // to use as bytes to send on I2C buffer
}I2CRxBuff;

int YawOffset;

// Communication Watch Dog, if no data arrives for a while set speed to zero for safety
int CommWd = 0;
// currently data sent via I2C at 40Hz toward dsNav. 80 means 2 seconds without new data from HLS
#define COMM_WD_TMO 80

void sio_newMsg(unsigned char);
void sio_voltage_low( unsigned char inchar ) ;
void sio_voltage_high( unsigned char inchar ) ;

void sio_fp_data( unsigned char inchar ) ;
void sio_fp_checksum( unsigned char inchar ) ;

void sio_cam_data( unsigned char inchar ) ;
void sio_cam_checksum( unsigned char inchar ) ;

//<GUIOTT>
// following functions decode and control packet parameters
#define HEADER  '@'
#define HEADER_LEN 4//# of bytes composing the header, payload starts from pos.4
void GO_sio_header ( unsigned char inchar );
void GO_sio_id ( unsigned char inchar );
void GO_sio_command ( unsigned char inchar );
void GO_sio_lenght ( unsigned char inchar );
void GO_sio_data ( unsigned char inchar );
void GO_UartRxError(int Err);
void GO_cmd_parser(int Cmd); // only if above are correct, start the command parsing
unsigned char GO_CheckSum(unsigned char* Buffer, int LastIndx);
void GO_serial_output_imu_data_K (void); // exec command K: send IMU params
void GO_serial_output_dsnav_data_b( void ); // exec command b: send dsNav params
void GO_serial_output_time_data_T( void ); // exec command T: send GPS time params
void GO_serial_output_gps_data_G( void );  // exec command G: send GPS service params

void GO_serial_input_nav_data_S(void);   // cmd S: set speed & direction

void GO_I2C_output_yaw(void); // compute and send yaw value to motor controller
void GO_yaw_offset_calib(void); // to compute the yaw at startup
void GO_serial_output_header(char Cmd, int Indx); // fill the buffer with heading info
//</GUIOTT>


char fp_high_byte;
unsigned char fp_checksum;

#if (SERIAL_INPUT_FORMAT  ==    SERIAL_IN_GUIOTT)
void (* sio_parse ) ( unsigned char inchar ) = &GO_sio_header ;
#else
void (* sio_parse ) ( unsigned char inchar ) = &sio_newMsg ;
#endif


#define SERIAL_BUFFER_SIZE 256
unsigned char serial_buffer[SERIAL_BUFFER_SIZE] ;

//<GUIOTT>
unsigned char Tmp_serial_buffer[SERIAL_BUFFER_SIZE] ;
unsigned char RX_serial_buffer[SERIAL_BUFFER_SIZE] ;
int RxBuffindx; // buffer pointer
int RxBuffCnt;  // buffer counter

int sb_index = 0 ;
int end_index = 0 ;

void init_serial()
{
#if ( SERIAL_OUTPUT_FORMAT == SERIAL_OSD_REMZIBI )
	dcm_flags._.nmea_passthrough = 1;
#endif
	
//  udb_serial_set_rate(19200) ;
//	udb_serial_set_rate(38400) ;
 udb_serial_set_rate(57600) ;
//	udb_serial_set_rate(115200) ;
//	udb_serial_set_rate(230400) ;
//	udb_serial_set_rate(460800) ;
//	udb_serial_set_rate(921600) ; // yes, it really will work at this rate
  
	return ;
}


////////////////////////////////////////////////////////////////////////////////
// 
// Receive Serial Commands
//

void udb_serial_callback_received_byte(char rxchar)
{
	(* sio_parse) ( rxchar ) ; // parse the input byte
	return ;
}

//<GUIOTT>
void GO_sio_header ( unsigned char inchar )
{
  if ( inchar == HEADER )
  {
	sio_parse = &GO_sio_id ;
    RxBuffindx = 0;     // reset buffer
    RX_serial_buffer[RxBuffindx] = inchar;
  }
  else // out of command sequence -> error
  {
    GO_UartRxError(-5);
  }
  return ;
}

void GO_sio_id ( unsigned char inchar )
{
  if ( 1 ) // ID is not used in this case
  {
    sio_parse = &GO_sio_command ;
    RxBuffindx ++;     // next location in buffer
    RX_serial_buffer[RxBuffindx] = inchar;
  }
  else // receive but does not decode the command
  {
    // if ID is used set here a flag to avoid executing command
  }
  return ;
}

void GO_sio_command ( unsigned char inchar )
{
	sio_parse = &GO_sio_lenght ;
    RxBuffindx ++;     // next location in buffer
    RX_serial_buffer[RxBuffindx] = inchar;
    return ;
}

void GO_sio_lenght ( unsigned char inchar )
{
	sio_parse = &GO_sio_data ;
    RxBuffindx ++;     // next location in buffer
    RX_serial_buffer[RxBuffindx] = inchar;
    RxBuffCnt = inchar;
    return ;
}

void GO_sio_data ( unsigned char inchar )
{
    RxBuffindx ++;     // next location in buffer
    RX_serial_buffer[RxBuffindx] = inchar;
    RxBuffCnt --;   // receive the right number of characters: the payload
    if (RxBuffCnt ==0)
    {
        sio_parse = &GO_sio_header; // restart packet decoding procedure
        
        if (GO_CheckSum(RX_serial_buffer, RxBuffindx-1) == inchar )
        {
            GO_cmd_parser(RX_serial_buffer[2]); // command is at position 2
        }
        else
        {
            GO_UartRxError(-1); // checksum error
        }  
    }
    return ;
}

void GO_cmd_parser(int Cmd)
{
    switch (Cmd)
    {
        case 'K':
            GO_serial_output_imu_data_K();
            break;

        case 'S':
            GO_serial_input_nav_data_S();
            break;

        case 'T':
            GO_serial_output_time_data_T();
            break;

        case 'b':
            GO_serial_output_dsnav_data_b();
            break;

        case 'G':
            GO_serial_output_gps_data_G();
            break;

        default:
            GO_UartRxError(-1); //	error: not a known command
            break;
    }
}

void GO_UartRxError(int Err)
{
    // do error procedures
    return;
}

unsigned char GO_CheckSum(unsigned char* Buffer, int LastIndx)
{
   unsigned char ChkSum = 0;
   int ChkCnt;

    for(ChkCnt=0; ChkCnt<=LastIndx; ChkCnt++)
    {
        ChkSum+=Buffer[ChkCnt];
    }
   	return ChkSum;
 }

void GO_serial_input_nav_data_S(void)
{// two byte int MSB
    CommWd = 0; // New data -> Clear comm Watch Dog
    I2CTxBuff.I.VelDes = (RX_serial_buffer[HEADER_LEN] << 8) + (RX_serial_buffer[HEADER_LEN +1]);
    I2CTxBuff.I.YawDes = (RX_serial_buffer[HEADER_LEN+2] << 8) + (RX_serial_buffer[HEADER_LEN +3]);
    I2CTxBuff.I.OrientFlag = RX_serial_buffer[HEADER_LEN +4];// set the otientation mode on master dsNav (direct or PID)
}
//</GUIOTT>


void sio_newMsg( unsigned char inchar )
{
	if ( inchar == 'V' )
	{
		sio_parse = &sio_voltage_high ;
	}
	
#if ( FLIGHT_PLAN_TYPE == FP_LOGO )
	else if ( inchar == 'L' )
#else
	else if ( inchar == 'W' )
#endif
	{
		fp_high_byte = -1 ; // -1 means we don't have the high byte yet (0-15 means we do)
		fp_checksum = 0 ;
		sio_parse = &sio_fp_data ;
		flightplan_live_begin() ;
	}
#if (CAM_USE_EXTERNAL_TARGET_DATA == 1)
	else if ( inchar == 'T' )
	{
		fp_high_byte = -1 ; // -1 means we don't have the high byte yet (0-15 means we do)
		fp_checksum = 0 ;
		sio_parse = &sio_cam_data ;
		camera_live_begin() ;
	}
#endif
	else
	{
		// error ?
	}
	return ;
}


void sio_voltage_high( unsigned char inchar )
{
	voltage_temp.BB = 0 ; // initialize our temp variable
	voltage_temp._.B1 = inchar ;
	sio_parse = &sio_voltage_low ;
	return ;
}


void sio_voltage_low( unsigned char inchar )
{
	voltage_temp._.B0 = inchar ;
	voltage_temp.BB = voltage_temp.BB * 2 ; // convert to voltage
	voltage_milis.BB = voltage_temp.BB ;
	sio_parse = &sio_newMsg ;
	return ;
}


char hex_char_val(unsigned char inchar)
{
	if (inchar >= '0' && inchar <= '9')
	{
		return (inchar - '0') ;
	}
	else if (inchar >= 'A' && inchar <= 'F')
	{
		return (inchar - 'A' + 10) ;
	}
	return -1 ;
}


// For UDB Logo instructions, bytes should be passed in using the following format
// (Below, an X represents a hex digit 0-F.  Mulit-digit values are MSB first.)
// L			begin remote Logo command
// XX	byte:	command
// XX	byte:	subcommand
// X	0-1:	do fly
// X	0-1:	use param
// XXXX	word:	argument
// *			done with command data
// XX	byte:	checksum should equal the sum of the 10 bytes before the *, mod 256
// 
// For example: "L0201000005*E8" runs:
// the DO command(02) for subroutine 01 with fly and param off(00) and an argument of 0005


// For classic Waypoints, bytes should be passed in using the following format
// (Below, an X represents a hex digit 0-F.  Mulit-digit values are MSB first.)
// W				begin remote Waypoint command
// XXXXXXXX	long:	waypoint X value
// XXXXXXXX	long:	waypoint Y value
// XXXX		word:	waypoint Z value
// XXXX		word:	flags
// XXXXXXXX	long:	cam view X value
// XXXXXXXX	long:	cam view Y value
// XXXX		word:	cam view Z value
// *				done with command data
// XX		byte:	checksum should equal the sum of the 44 bytes before the *, mod 256
// 
// For example: "W0000006400000032000F0200000000000000000000*67" represents:
// the waypoint { {100, 50, 15}, F_INVERTED, {0, 0, 0} }
// 

void sio_fp_data( unsigned char inchar )
{
	if (inchar == '*')
	{
		fp_high_byte = -1 ;
		sio_parse = &sio_fp_checksum ;
	}
	else
	{
		char hexVal = hex_char_val(inchar) ;
		if (hexVal == -1)
		{
			sio_parse = &sio_newMsg ;
			return ;
		}
		else if (fp_high_byte == -1)
		{
			fp_high_byte = hexVal * 16 ;
		}
		else
		{
			flightplan_live_received_byte(fp_high_byte + hexVal) ;
			fp_high_byte = -1 ;
		}
		fp_checksum += inchar ;
	}
	return ;
}


void sio_fp_checksum( unsigned char inchar )
{
	char hexVal = hex_char_val(inchar) ;
	if (hexVal == -1)
	{
		sio_parse = &sio_newMsg ;
	}
	else if (fp_high_byte == -1)
	{
		fp_high_byte = hexVal * 16 ;
	}
	else
	{
		unsigned char v = fp_high_byte + hexVal ;
		if (v == fp_checksum)
		{
			flightplan_live_commit() ;
		}
		sio_parse = &sio_newMsg ;
	}
	return ;
}


#if (CAM_USE_EXTERNAL_TARGET_DATA == 1)

void sio_cam_data( unsigned char inchar )
{
	if (inchar == '*')
	{
		fp_high_byte = -1 ;
		sio_parse = &sio_cam_checksum ;
	}
	else
	{
		char hexVal = hex_char_val(inchar) ;
		if (hexVal == -1)
		{
			sio_parse = &sio_newMsg ;
			return ;
		}
		else if (fp_high_byte == -1)
		{
			fp_high_byte = hexVal * 16 ;
		}
		else
		{
			unsigned char combined = fp_high_byte + hexVal ;
			camera_live_received_byte(combined) ;
			fp_high_byte = -1 ;
			fp_checksum += combined ;
		}
	}
	return ;
}


void sio_cam_checksum( unsigned char inchar )
{
	char hexVal = hex_char_val(inchar) ;
	if (hexVal == -1)
	{
		sio_parse = &sio_newMsg ;
	}
	else if (fp_high_byte == -1)
	{
		fp_high_byte = hexVal * 16 ;
	}
	else
	{
		unsigned char v = fp_high_byte + hexVal ;
		if (v == fp_checksum)
		{
			camera_live_commit() ;
		}
		sio_parse = &sio_newMsg ;
	}
	return ;
}

#endif


////////////////////////////////////////////////////////////////////////////////
// 
// Output Serial Data
//

// add this text to the output buffer
void serial_output( char* format, ... )
{
	va_list arglist ;
	
	va_start(arglist, format) ;
	
	int start_index = end_index ;
	int remaining = SERIAL_BUFFER_SIZE - start_index ;
	
	if (remaining > 1)
	{
		int wrote = vsnprintf( (char*)(&serial_buffer[start_index]), (size_t)remaining, format, arglist) ;
		end_index = start_index + wrote;
	}
	
	if (sb_index == 0)
	{
		udb_serial_start_sending_data();
	}
	
	va_end(arglist);
	
	return ;
}

//<GUIOTT>
void GO_serial_output_imu_data_K( void )
{
    /* exec command K: send IMU params
     The command string is composed by an array of unsigned char:
     0      Header	 @
     1      Id		   0	ASCII	(not used here, just for compatibility)
     2      Cmd		   K 	ASCII
     3      CmdLen	 Num of bytes (bin) following (checksum included)
	 4-7    Plane gps latitude shown as the number of degrees times 10 to the power 7 North of the equator. (negative means south).
	 8-11   Plane gps longitude shown as the number of degrees times 10 to the power 7 East of the Greenwich Meridian
     12-15  Plane gps altitude in centimeters above mean sea level
     16-17  Speed over the ground from the GPS in meters / second times 100
     18-19  GPS Course over the Ground (2D) of the plane in degrees times 100
     20-21  DCM XpXe - rmat[0]
     22-23  DCM XpYe - rmat[1]
     24-25  DCM XpZe - rmat[2]
     26-27  DCM YpXe - rmat[3]
     28-29  DCM YpYe - rmat[4]
     30-31  DCM YpZe - rmat[5]
     32-33  DCM ZpXe - rmat[6]
     34-35  DCM ZpYe - rmat[7]
     36-27  DCM ZpZe - rmat[8]
     38     Percentage of available cpu power that has been used over the last one second.
     39     Checksum 0-255	obtained by adding in a 8 bit variable all the bytes composing the message (checksum itself excluded)
     */
 
    int Indx = HEADER_LEN;  // Head length, number of characters in buffer before valid data

    Tmp_serial_buffer[Indx++]=lat_gps.__.B3; // MSB first
    Tmp_serial_buffer[Indx++]=lat_gps.__.B2;
    Tmp_serial_buffer[Indx++]=lat_gps.__.B1;
    Tmp_serial_buffer[Indx++]=lat_gps.__.B0;
    Tmp_serial_buffer[Indx++]=long_gps.__.B3; // MSB first
    Tmp_serial_buffer[Indx++]=long_gps.__.B2;
    Tmp_serial_buffer[Indx++]=long_gps.__.B1;
    Tmp_serial_buffer[Indx++]=long_gps.__.B0;
    Tmp_serial_buffer[Indx++]=alt_sl_gps.__.B3; // MSB first
    Tmp_serial_buffer[Indx++]=alt_sl_gps.__.B2;
    Tmp_serial_buffer[Indx++]=alt_sl_gps.__.B1;
    Tmp_serial_buffer[Indx++]=alt_sl_gps.__.B0;
    Tmp_serial_buffer[Indx++]=sog_gps._.B1;
    Tmp_serial_buffer[Indx++]=sog_gps._.B0;
    Tmp_serial_buffer[Indx++]=cog_gps._.B1;
    Tmp_serial_buffer[Indx++]=cog_gps._.B0;
    Tmp_serial_buffer[Indx++]=rmat[0] >> 8;
    Tmp_serial_buffer[Indx++]=rmat[0];
    Tmp_serial_buffer[Indx++]=rmat[1] >> 8;
    Tmp_serial_buffer[Indx++]=rmat[1];
    Tmp_serial_buffer[Indx++]=rmat[2] >> 8;
    Tmp_serial_buffer[Indx++]=rmat[2];
    Tmp_serial_buffer[Indx++]=rmat[3] >> 8;
    Tmp_serial_buffer[Indx++]=rmat[3];
    Tmp_serial_buffer[Indx++]=rmat[4] >> 8;
    Tmp_serial_buffer[Indx++]=rmat[4];
    Tmp_serial_buffer[Indx++]=rmat[5] >> 8;
    Tmp_serial_buffer[Indx++]=rmat[5];
    Tmp_serial_buffer[Indx++]=rmat[6] >> 8;
    Tmp_serial_buffer[Indx++]=rmat[6];
    Tmp_serial_buffer[Indx++]=rmat[7] >> 8;
    Tmp_serial_buffer[Indx++]=rmat[7];
    Tmp_serial_buffer[Indx++]=rmat[8] >> 8;
    Tmp_serial_buffer[Indx++]=rmat[8];
    Tmp_serial_buffer[Indx++]=udb_cpu_load();
    Tmp_serial_buffer[Indx++]=YawOffset >> 8;
    Tmp_serial_buffer[Indx++]=YawOffset;
    
    GO_serial_output_header('K', Indx);

    Tmp_serial_buffer[Indx]=GO_CheckSum(Tmp_serial_buffer, Indx-1);
    GO_serial_output_bin(Indx+1);
	return ;
}

void GO_serial_output_gps_data_G( void )
{
    /* exec command G: send GPS services params
     The command string is composed by an array of unsigned char:
     0      Header	 @
     1      Id		   0	ASCII	(not used here, just for compatibility)
     2      Cmd		   K 	ASCII
     3      CmdLen	 Num of bytes (bin) following (checksum included)
     4-5    Week Number (GPS)
     6-9    Time in micro seconds since Saturday midnight GMT (GPS)
     10     hdop
     11     number of visible satellites
     12-15  Satellite ID list
     16-19
     20     Checksum 0-255	obtained by adding in a 8 bit variable all the bytes composing the message (checksum itself excluded)
     */

    int Indx = HEADER_LEN;  // Head length, number of characters in buffer before valid data

    Tmp_serial_buffer[Indx++]=week_no._.B1;
    Tmp_serial_buffer[Indx++]=week_no._.B0;
    Tmp_serial_buffer[Indx++]=tow.__.B3; // MSB first
    Tmp_serial_buffer[Indx++]=tow.__.B2;
    Tmp_serial_buffer[Indx++]=tow.__.B1;
    Tmp_serial_buffer[Indx++]=tow.__.B0;
    Tmp_serial_buffer[Indx++]=hdop;
    Tmp_serial_buffer[Indx++]=svs;
    Tmp_serial_buffer[Indx++]=SatIdList_gps.__.B3; // MSB first
    Tmp_serial_buffer[Indx++]=SatIdList_gps.__.B2;
    Tmp_serial_buffer[Indx++]=SatIdList_gps.__.B1;
    Tmp_serial_buffer[Indx++]=SatIdList_gps.__.B0;
    Tmp_serial_buffer[Indx++]=Hepe_gps.__.B3; // MSB first
    Tmp_serial_buffer[Indx++]=Hepe_gps.__.B2;
    Tmp_serial_buffer[Indx++]=Hepe_gps.__.B1;
    Tmp_serial_buffer[Indx++]=Hepe_gps.__.B0;

    GO_serial_output_header('G', Indx);

    Tmp_serial_buffer[Indx]=GO_CheckSum(Tmp_serial_buffer, Indx-1);
    GO_serial_output_bin(Indx+1);
	return ;
}


#define CONSOLE_DEBUG 0 // debug GUI protocol sending back test values without dsNav

void GO_serial_output_dsnav_data_b( void )
{    /* exec command b: send dsNav params
     The command string is composed by an array of unsigned char:
     0      Header	 @
     1      Id		   0	ASCII	(not used here, just for compatibility)
     2      Cmd		   K 	ASCII
     3      CmdLen	 Num of bytes (bin) following (checksum included)
     4-7    PosXmes current X position coordinate (MSB first)
     8-11   PosYmes current Y position coordinate (MSB first)
     12-19  VelInt[4] speed in mm/s as an integer for all the wheels (4 ints)
     20-27  ADCValue[4] measured current for each motor (4 ints)
     28     stasis_err number of times imu and wheels very different
     29     stasis_alarm stasis_alarm
     */

    int Indx = HEADER_LEN;  // Head length, number of characters in buffer before valid data

#if(CONSOLE_DEBUG == 1) 
    I2CRxBuff.I.PosXmes += (I2CTxBuff.I.VelDes * 0.1);
    I2CRxBuff.I.PosYmes += (I2CTxBuff.I.VelDes * 0.05);
    I2CRxBuff.I.VelInt[0] = I2CTxBuff.I.VelDes;
    I2CRxBuff.I.VelInt[1] = I2CTxBuff.I.VelDes;
    I2CRxBuff.I.VelInt[2] = I2CTxBuff.I.VelDes;
    I2CRxBuff.I.VelInt[3] = I2CTxBuff.I.VelDes;
    I2CRxBuff.I.ADCValue[0] = abs(I2CTxBuff.I.VelDes)+100;
    I2CRxBuff.I.ADCValue[1] = abs(I2CTxBuff.I.VelDes)+100;
    I2CRxBuff.I.ADCValue[2] = abs(I2CTxBuff.I.VelDes)+100;
    I2CRxBuff.I.ADCValue[3] = abs(I2CTxBuff.I.VelDes)+100;
    I2CRxBuff.I.stasis_err = 4;
    I2CRxBuff.I.stasis_alarm = 2;
#endif

    // fill buffer with char struct
    Tmp_serial_buffer[Indx++]=I2CRxBuff.C[3]; // PosXmes MSB
    Tmp_serial_buffer[Indx++]=I2CRxBuff.C[2];
    Tmp_serial_buffer[Indx++]=I2CRxBuff.C[1];
    Tmp_serial_buffer[Indx++]=I2CRxBuff.C[0];
    Tmp_serial_buffer[Indx++]=I2CRxBuff.C[7]; // PosYmes MSB
    Tmp_serial_buffer[Indx++]=I2CRxBuff.C[6];
    Tmp_serial_buffer[Indx++]=I2CRxBuff.C[5];
    Tmp_serial_buffer[Indx++]=I2CRxBuff.C[4];
    Tmp_serial_buffer[Indx++]=I2CRxBuff.C[9]; // VelInt[0]   MSB
    Tmp_serial_buffer[Indx++]=I2CRxBuff.C[8];
    Tmp_serial_buffer[Indx++]=I2CRxBuff.C[11];// VelInt[1]   MSB
    Tmp_serial_buffer[Indx++]=I2CRxBuff.C[10];
    Tmp_serial_buffer[Indx++]=I2CRxBuff.C[13];// VelInt[2]   MSB
    Tmp_serial_buffer[Indx++]=I2CRxBuff.C[12];
    Tmp_serial_buffer[Indx++]=I2CRxBuff.C[15];// VelInt[3]   MSB
    Tmp_serial_buffer[Indx++]=I2CRxBuff.C[14];
    Tmp_serial_buffer[Indx++]=I2CRxBuff.C[17];// ADCValue[0] MSB
    Tmp_serial_buffer[Indx++]=I2CRxBuff.C[16];
    Tmp_serial_buffer[Indx++]=I2CRxBuff.C[19];// ADCValue[1] MSB
    Tmp_serial_buffer[Indx++]=I2CRxBuff.C[18];
    Tmp_serial_buffer[Indx++]=I2CRxBuff.C[21];// ADCValue[2] MSB
    Tmp_serial_buffer[Indx++]=I2CRxBuff.C[20];
    Tmp_serial_buffer[Indx++]=I2CRxBuff.C[23];// ADCValue[3] MSB
    Tmp_serial_buffer[Indx++]=I2CRxBuff.C[22];
    Tmp_serial_buffer[Indx++]=I2CRxBuff.C[24];// stasis_err number
    Tmp_serial_buffer[Indx++]=I2CRxBuff.C[25];// stasis_alarm

    GO_serial_output_header('b', Indx);

    Tmp_serial_buffer[Indx]=GO_CheckSum(Tmp_serial_buffer, Indx-1);
    GO_serial_output_bin(Indx+1);
	return ;
}

void GO_serial_output_time_data_T( void )
{
    /* exec command T: send GPS time params
     The command string is composed by an array of unsigned char:
     0      Header	 @
     1      Id		   0	ASCII	(not used here, just for compatibility)
     2      Cmd		   K 	ASCII
     3      CmdLen	 Num of bytes (bin) following (checksum included)
	 4-5    utcYear (unsigned short int)
     6      utcMonth (unsigned char)
     7      utcDay (unsigned char)
     8      utcHour (unsigned char)
     9      utcMinute (unsigned char utcMinute)
     10-13  utcSeconds   (float)
     14     Checksum 0-255	obtained by adding in a 8 bit variable all the bytes composing the message (checksum itself excluded)
     */

    int Indx = HEADER_LEN;  // Head length, number of characters in buffer before valid data

    /*TIMECONV_GetUTCTimeFromGPSfix(week_no.BB, (double)tow.WW,
                                 &TimeS.I.utcYear, &TimeS.I.utcMonth,
                                 &TimeS.I.utcDay, &TimeS.I.utcHour,
                                 &TimeS.I.utcMinute, &TimeS.I.utcSeconds);
    */

    Tmp_serial_buffer[Indx++]= UtcYear_gps._.B1; // Year MSB
    Tmp_serial_buffer[Indx++]= UtcYear_gps._.B0; // Year LSB
    Tmp_serial_buffer[Indx++]= UtcMonth_gps; // Month
    Tmp_serial_buffer[Indx++]= UtcDay_gps; // Day
    Tmp_serial_buffer[Indx++]= UtcHour_gps; // Hours
    Tmp_serial_buffer[Indx++]= UtcMinute_gps; // Minutes
    Tmp_serial_buffer[Indx++]= UtcSeconds_gps._.B1; // Seconds MSB
    Tmp_serial_buffer[Indx++]= UtcSeconds_gps._.B0; // Seconds

    GO_serial_output_header('T', Indx);

    Tmp_serial_buffer[Indx]=GO_CheckSum(Tmp_serial_buffer, Indx-1);
    GO_serial_output_bin(Indx+1);
	return ;
}

void GO_serial_output_header(char Cmd, int Indx)
{
    Tmp_serial_buffer[0]= HEADER;
    Tmp_serial_buffer[1]= '0';
    Tmp_serial_buffer[2]= Cmd;
    Tmp_serial_buffer[3]= Indx - HEADER_LEN;//number of valid data=last index-head length+1 (chk)
}

// add this binary data to the output buffer according to the GUIOTT protocol
void GO_serial_output_bin(int BuffLen)
{
	int start_index = end_index ;
	int remaining = SERIAL_BUFFER_SIZE - start_index ;
    int LenCnt;

	if (remaining > 1)
	{
    for(LenCnt=0; LenCnt<BuffLen; LenCnt++)
    {
        serial_buffer[LenCnt+start_index] = Tmp_serial_buffer[LenCnt];
    }
    serial_buffer[LenCnt+start_index]=0; //null terminated
		end_index = start_index + BuffLen;
	}

	if (sb_index == 0)
	{
		udb_serial_start_sending_data();
	}

	return ;
}

void GO_I2C_output_yaw(void)
{// called every 25ms from T6 timer callback
    struct relative2D matrix_accum ;

    matrix_accum.x = rmat[4];
    matrix_accum.y = rmat[1];
    I2CTxBuff.I.YawMesAbs = (int)((float)rect_to_polar16(&matrix_accum))*0.054933633; // Deg x 10 0-3599
    I2CTxBuff.I.YawMesRel = I2CTxBuff.I.YawMesAbs - YawOffset;

    CommWd++; // Communication Watch Dog
    if (CommWd > COMM_WD_TMO)
    {
        I2CTxBuff.I.VelDes = 0X7FFF; // set both wheels speed to 0 for safety if no new commands
    }

    I2C_txDsnav(); // Transmit the navigation parameters to dsNav
}

void GO_yaw_offset_calib(void)
{// to compute the yaw at startup
    struct relative2D matrix_accum ;
    I2CTxBuff.I.VelDes = 0X7FFF; // set both wheels speed to 0 for safety if no new commands
    matrix_accum.x = rmat[4];
    matrix_accum.y = rmat[1];
    YawOffset = (int)((float)rect_to_polar16(&matrix_accum))*0.054933633; // Deg x 10 0-3599
}
//</GUIOTT>

int udb_serial_callback_get_byte_to_send(void)
{
	unsigned char txchar = serial_buffer[ sb_index++ ] ;

  /*<GUIOTT>
   * modified to transmit binary buffer and not only ASCII (NULL terminated)
	if ( txchar )
	{
		return txchar ;
	}
	else
	{
		sb_index = 0 ;
		end_index = 0 ;
	}
  </GUIOTT>*/

  if(sb_index <= end_index)
	{
		return txchar ;
	}
	else
	{
		sb_index = 0 ;
		end_index = 0 ;
	}
	
	return -1;
}


#if ( SERIAL_OUTPUT_FORMAT == SERIAL_DEBUG )

void serial_output_8hz( void )
{
  /*
	serial_output("lat: %li, long: %li, alt: %li\r\nrmat: %i, %i, %i, %i, %i, %i, %i, %i, %i\r\n" ,
		lat_gps.WW , long_gps.WW , alt_sl_gps.WW ,
		rmat[0] , rmat[1] , rmat[2] ,
		rmat[3] , rmat[4] , rmat[5] ,
		rmat[6] , rmat[7] , rmat[8]  ) ;
  */

  //<GUIOTT>

  serial_output("\r\nLat: %li, Long: %li, Alt: %li\r\n"
                "Speed: %i, Course: %i, Climb: %i\r\n"
                "Week: %i, Time of week: %li\r\n"
                "HDOP: %i, Satellites: %i\r\n"
                "rmat: %i, %i, %i, %i, %i, %i, %i, %i, %i\r\n"
                "CPU load: %i\r\n"
                "===================================================\r\n\r\n" ,
		lat_gps.WW , long_gps.WW , alt_sl_gps.WW ,
    sog_gps.BB , cog_gps.BB , climb_gps.BB ,
    week_no.BB , tow.WW,
    hdop,	svs,
		rmat[0] , rmat[1] , rmat[2] ,
		rmat[3] , rmat[4] , rmat[5] ,
		rmat[6] , rmat[7] , rmat[8] ,
    udb_cpu_load());
    if ( _SWR == 0 )
				{
					// if there was not a software reset (trap error) clear the trap data
					trap_flags = trap_source = osc_fail_count = 0 ;
				}

  //</GUIOTT>

	return ;
}

//<GUIOTT>
#elif ( SERIAL_OUTPUT_FORMAT == SERIAL_CONSOLE )
// String of important navigation data in the format of my communication protocol:
// http://www.guiott.com/Rino/CommandDescr/Protocol.htm
void serial_output_8hz( void )
{
    GO_serial_output_nav_data_K;
	return ;
}
//</GUIOTT>


#elif ( SERIAL_OUTPUT_FORMAT == SERIAL_ARDUSTATION )

extern int desiredHeight, waypointIndex ;

void serial_output_8hz( void )
{
	unsigned int mode ;
	struct relative2D matrix_accum ;
	union longbbbb accum ;
	int desired_dir_deg ;  // desired_dir converted to a bearing (0-360)
	
	long earth_pitch ;		// pitch in binary angles ( 0-255 is 360 degreres)
	long earth_roll ;		// roll of the plane with respect to earth frame
	//long earth_yaw ;		// yaw with respect to earth frame
	
	accum.WW  = ( desired_dir * BYTECIR_TO_DEGREE ) + 32768 ;
	desired_dir_deg  = accum._.W1 - 90 ; // "Convert UAV DevBoad Earth" to Compass Bearing
	if ( desired_dir_deg < 0 ) desired_dir_deg += 360 ; 

	if (flags._.GPS_steering == 0 && flags._.pitch_feedback == 0)
		mode = 1 ;
	else if (flags._.GPS_steering == 0 && flags._.pitch_feedback == 1)
		mode = 2 ;
	else if (flags._.GPS_steering == 1 && flags._.pitch_feedback == 1 && udb_flags._.radio_on == 1)
		mode = 3 ;
	else if (flags._.GPS_steering == 1 && flags._.pitch_feedback == 1 && udb_flags._.radio_on == 0)
		mode = 0 ;
	else
		mode = 99 ; // Unknown
	
	//  Roll
	//  Earth Frame of Reference
	matrix_accum.x = rmat[8] ;
	matrix_accum.y = rmat[6] ;
	earth_roll = rect_to_polar(&matrix_accum) ;					// binary angle (0 - 256 = 360 degrees)
	earth_roll = (-earth_roll * BYTECIR_TO_DEGREE) >> 16 ;		// switch polarity, convert to -180 - 180 degrees
	
	//  Pitch
	//  Earth Frame of Reference
	//  Note that we are using the matrix_accum.x
	//  left over from previous rect_to_polar in this calculation.
	//  so this Pitch calculation must follow the Roll calculation
	matrix_accum.y = rmat[7] ;
	earth_pitch = rect_to_polar(&matrix_accum) ;				// binary angle (0 - 256 = 360 degrees)
	earth_pitch = (-earth_pitch * BYTECIR_TO_DEGREE) >> 16 ;	// switch polarity, convert to -180 - 180 degrees
	
	// Yaw
	// Earth Frame of Reference
	// Ardustation does not use yaw in degrees
	// matrix_accum.x = rmat[4] ;
	// matrix_accum.y = rmat[1] ;
	// earth_yaw = rect_to_polar(&matrix_accum) ;				// binary angle (0 - 256 = 360 degrees)
	// earth_yaw = (earth_yaw * BYTECIR_TO_DEGREE) >> 16 ;		// switch polarity, convert to -180 - 180 degrees
	
	
	// The Ardupilot GroundStation protocol is mostly documented here:
	//    http://diydrones.com/profiles/blogs/ardupilot-telemetry-protocol
	
	if (udb_heartbeat_counter % 40 == 0)  // Every 8 runs (5 heartbeat counts per 8Hz)
	{
		serial_output("!!!LAT:%li,LON:%li,SPD:%.2f,CRT:%.2f,ALT:%li,ALH:%i,CRS:%.2f,BER:%i,WPN:%i,DST:%i,BTV:%.2f***\r\n"
					  "+++THH:%i,RLL:%li,PCH:%li,STT:%i,***\r\n",
			lat_gps.WW / 10 , long_gps.WW / 10 , (float)(sog_gps.BB / 100.0), (float)(climb_gps.BB / 100.0),
			(alt_sl_gps.WW - alt_origin.WW) / 100, desiredHeight, (float)(cog_gps.BB / 100.0), desired_dir_deg,
			waypointIndex, tofinish_line, (float)(voltage_milis.BB / 100.0), 
			(int)((udb_pwOut[THROTTLE_OUTPUT_CHANNEL] - udb_pwTrim[THROTTLE_OUTPUT_CHANNEL])/20),
			earth_roll, earth_pitch,
			mode
		) ;
	}
	else if (udb_heartbeat_counter % 10 == 0)  // Every 2 runs (5 heartbeat counts per 8Hz)
	{
		serial_output("+++THH:%i,RLL:%li,PCH:%li,STT:%i,***\r\n",
			(int)((udb_pwOut[THROTTLE_OUTPUT_CHANNEL] - udb_pwTrim[THROTTLE_OUTPUT_CHANNEL])/20),
			earth_roll, earth_pitch,
			mode
		) ;
	}
	
	return ;
}


#elif ( SERIAL_OUTPUT_FORMAT == SERIAL_UDB || SERIAL_OUTPUT_FORMAT == SERIAL_UDB_EXTRA )

int telemetry_counter = 8 ;

#if ( SERIAL_OUTPUT_FORMAT == SERIAL_UDB_EXTRA )
int pwIn_save[NUM_INPUTS + 1] ;
int pwOut_save[NUM_OUTPUTS + 1] ;
#endif

extern int waypointIndex ;

#if (RECORD_FREE_STACK_SPACE == 1)
extern unsigned int maxstack ;
#endif

void serial_output_8hz( void )
{
#if ( SERIAL_OUTPUT_FORMAT == SERIAL_UDB )	// Only run through this function twice per second, by skipping all but every 4 runs through it.
	// Saves CPU and XBee power.
	if (udb_heartbeat_counter % 20 != 0) return ;  // Every 4 runs (5 heartbeat counts per 8Hz)
	
#elif ( SERIAL_OUTPUT_FORMAT == SERIAL_UDB_EXTRA )
	// SERIAL_UDB_EXTRA expected to be used with the OpenLog which can take greater transfer speeds than Xbee
	// F2: SERIAL_UDB_EXTRA format is printed out every other time, although it is being called at 8Hz, this
	//		version will output four F2 lines every second (4Hz updates)
#endif

	switch (telemetry_counter)
	{
		// The first lines of telemetry contain info about the compile-time settings from the options.h file
		case 8:
			if ( _SWR == 0 )
			{
				// if there was not a software reset (trap error) clear the trap data
				trap_flags = trap_source = osc_fail_count = 0 ;
			}
			serial_output("\r\nF14:WIND_EST=%i:GPS_TYPE=%i:DR=%i:BOARD_TYPE=%i:AIRFRAME=%i:RCON=0x%X:TRAP_FLAGS=0x%X:TRAP_SOURCE=0x%lX:ALARMS=%i:"  \
							"CLOCK=%i:FP=%d:\r\n",
				WIND_ESTIMATION, GPS_TYPE, DEADRECKONING, BOARD_TYPE, AIRFRAME_TYPE, RCON , trap_flags , trap_source , osc_fail_count, CLOCK_CONFIG, FLIGHT_PLAN_TYPE ) ;
				RCON = 0 ;
				trap_flags = 0 ;
				trap_source = 0 ;
				osc_fail_count = 0 ;
			break ;
		case 7:
			serial_output("F15:IDA=");
			serial_output(ID_VEHICLE_MODEL_NAME );
			serial_output(":IDB=");
			serial_output(ID_VEHICLE_REGISTRATION );
			serial_output(":\r\n" );
			break ;
		case 6:
			serial_output("F16:IDC=" );
			serial_output( ID_LEAD_PILOT );
			serial_output( ":IDD=");
			serial_output( ID_DIY_DRONES_URL );
			serial_output(":\r\n") ;
			break ;
		case 5:
			serial_output("F4:R_STAB_A=%i:R_STAB_RD=%i:P_STAB=%i:Y_STAB_R=%i:Y_STAB_A=%i:AIL_NAV=%i:RUD_NAV=%i:AH_STAB=%i:AH_WP=%i:RACE=%i:\r\n",
				ROLL_STABILIZATION_AILERONS, ROLL_STABILIZATION_RUDDER, PITCH_STABILIZATION, YAW_STABILIZATION_RUDDER, YAW_STABILIZATION_AILERON,
				AILERON_NAVIGATION, RUDDER_NAVIGATION, ALTITUDEHOLD_STABILIZED, ALTITUDEHOLD_WAYPOINT, RACING_MODE) ;
			break ;
		case 4:
			serial_output("F5:YAWKP_A=%5.3f:YAWKD_A=%5.3f:ROLLKP=%5.3f:ROLLKD=%5.3f:A_BOOST=%3.1f:\r\n",
				YAWKP_AILERON, YAWKD_AILERON, ROLLKP, ROLLKD, AILERON_BOOST ) ;
			break ;
		case 3:
			serial_output("F6:P_GAIN=%5.3f:P_KD=%5.3f:RUD_E_MIX=%5.3f:ROL_E_MIX=%5.3f:E_BOOST=%3.1f:\r\n",
				PITCHGAIN, PITCHKD, RUDDER_ELEV_MIX, ROLL_ELEV_MIX, ELEVATOR_BOOST) ;
			break ;
		case 2:
			serial_output("F7:Y_KP_R=%5.4f:Y_KD_R=%5.3f:RLKP_RUD=%5.3f:RLKD_RUD=%5.3f:RUD_BOOST=%5.3f:RTL_PITCH_DN=%5.3f:\r\n",
				YAWKP_RUDDER, YAWKD_RUDDER, ROLLKP_RUDDER , ROLLKD_RUDDER , RUDDER_BOOST, RTL_PITCH_DOWN) ;
			break ;
		case 1:
			serial_output("F8:H_MAX=%6.1f:H_MIN=%6.1f:MIN_THR=%3.2f:MAX_THR=%3.2f:PITCH_MIN_THR=%4.1f:PITCH_MAX_THR=%4.1f:PITCH_ZERO_THR=%4.1f:\r\n",
				HEIGHT_TARGET_MAX, HEIGHT_TARGET_MIN, ALT_HOLD_THROTTLE_MIN, ALT_HOLD_THROTTLE_MAX,
				ALT_HOLD_PITCH_MIN, ALT_HOLD_PITCH_MAX, ALT_HOLD_PITCH_HIGH) ;
			break ;
		default:
		{
			// F2 below means "Format Revision 2: and is used by a Telemetry parser to invoke the right pattern matching
			// F2 is a compromise between easy reading of raw data in a file and not droppping chars in transmission.
			
#if ( SERIAL_OUTPUT_FORMAT == SERIAL_UDB )
			serial_output("F2:T%li:S%d%d%d:N%li:E%li:A%li:W%i:a%i:b%i:c%i:d%i:e%i:f%i:g%i:h%i:i%i:c%u:s%i:cpu%u:bmv%i:"
				"as%i:wvx%i:wvy%i:wvz%i:\r\n",
				tow.WW, udb_flags._.radio_on, dcm_flags._.nav_capable, flags._.GPS_steering,
				lat_gps.WW , long_gps.WW , alt_sl_gps.WW, waypointIndex,
				rmat[0] , rmat[1] , rmat[2] ,
				rmat[3] , rmat[4] , rmat[5] ,
				rmat[6] , rmat[7] , rmat[8] ,
				(unsigned int)cog_gps.BB, sog_gps.BB, (unsigned int)udb_cpu_load(), voltage_milis.BB,
				air_speed_3DIMU, 
				estimatedWind[0], estimatedWind[1], estimatedWind[2] ) ;
			// Approximate time passing between each telemetry line, even though
			// we may not have new GPS time data each time through.
			if (tow.WW > 0) tow.WW += 500 ;
				
#elif ( SERIAL_OUTPUT_FORMAT == SERIAL_UDB_EXTRA )
			if (udb_heartbeat_counter % 10 != 0)  // Every 2 runs (5 heartbeat counts per 8Hz)
			{
					serial_output("F2:T%li:S%d%d%d:N%li:E%li:A%li:W%i:a%i:b%i:c%i:d%i:e%i:f%i:g%i:h%i:i%i:c%u:s%i:cpu%u:bmv%i:"
					"as%u:wvx%i:wvy%i:wvz%i:ma%i:mb%i:mc%i:svs%i:hd%i:",
					tow.WW, udb_flags._.radio_on, dcm_flags._.nav_capable, flags._.GPS_steering,
					lat_gps.WW , long_gps.WW , alt_sl_gps.WW, waypointIndex,
					rmat[0] , rmat[1] , rmat[2] ,
					rmat[3] , rmat[4] , rmat[5] ,
					rmat[6] , rmat[7] , rmat[8] ,
					(unsigned int)cog_gps.BB, sog_gps.BB, (unsigned int)udb_cpu_load(), voltage_milis.BB,
					air_speed_3DIMU,
					estimatedWind[0], estimatedWind[1], estimatedWind[2],
#if (MAG_YAW_DRIFT == 1)
					magFieldEarth[0],magFieldEarth[1],magFieldEarth[2],
#else
					(int)0, (int)0, (int)0,
#endif
					
					svs, hdop ) ;
				
				// Approximate time passing between each telemetry line, even though
				// we may not have new GPS time data each time through.
				if (tow.WW > 0) tow.WW += 250 ; 
				
				// Save  pwIn and PwOut buffers for printing next time around
				int i ;
				for (i=0; i <= NUM_INPUTS; i++)
					pwIn_save[i] = udb_pwIn[i] ;
				for (i=0; i <= NUM_OUTPUTS; i++)
					pwOut_save[i] = udb_pwOut[i] ;
			}
			else
			{
				int i ;
				for (i= 1; i <= NUM_INPUTS; i++)
					serial_output("p%ii%i:",i,pwIn_save[i]);
				for (i= 1; i <= NUM_OUTPUTS; i++)
					serial_output("p%io%i:",i,pwOut_save[i]);
				serial_output("imx%i:imy%i:imz%i:fgs%X:ofc%i:tx%i:ty%i:tz%i:G%d,%d,%d:",IMUlocationx._.W1 ,IMUlocationy._.W1 ,IMUlocationz._.W1,
					 flags.WW, osc_fail_count, IMUvelocityx._.W1, IMUvelocityy._.W1, IMUvelocityz._.W1, goal.x, goal.y, goal.height );
#if (RECORD_FREE_STACK_SPACE == 1)
				serial_output("stk%d:", (int)(4096-maxstack));
#endif
				serial_output("\r\n");
			}
#endif
			if (flags._.f13_print_req == 1)
			{
				// The F13 line of telemetry is printed when origin has been captured and inbetween F2 lines in SERIAL_UDB_EXTRA
#if ( SERIAL_OUTPUT_FORMAT == SERIAL_UDB_EXTRA )
				if (udb_heartbeat_counter % 10 != 0) return ;
#endif
				serial_output("F13:week%i:origN%li:origE%li:origA%li:\r\n", week_no, lat_origin.WW, long_origin.WW, alt_origin) ;
				flags._.f13_print_req = 0 ;
			}
			
			return ;
		}
	}
	telemetry_counter-- ;
	return ;
}


#elif ( SERIAL_OUTPUT_FORMAT == SERIAL_OSD_REMZIBI )

void serial_output_8hz( void )
{
	// TODO: Output interesting information for OSD.
	// But first we'll have to implement a buffer for passthrough characters to avoid
	// output corruption, or generate NMEA ourselves here.
	return ;
}

#elif ( SERIAL_OUTPUT_FORMAT == SERIAL_MAGNETOMETER )

extern void rxMagnetometer(void) ;
extern int udb_magFieldBody[3] ;
extern unsigned char magreg[6] ;
extern int magFieldEarth[3] ;
extern int udb_magOffset[3] ;
extern int magGain[3] ;
extern int offsetDelta[3] ;
extern int rawMagCalib[3] ;
extern int magMessage ;

extern union longww HHIntegral ;

#define OFFSETSHIFT 1

extern int I2ERROR ;
extern int I2messages ;
extern int I2interrupts ;

#if ( BOARD_TYPE == UDB4_BOARD )
#define I2CCONREG I2C2CON
#define I2CSTATREG I2C2STAT
#else
#define I2CCONREG I2CCON
#define I2CSTATREG I2CSTAT
#endif
/*
void serial_output_8hz( void )
{
	serial_output("MagMessage: %i\r\nI2CCON: %X, I2CSTAT: %X, I2ERROR: %X\r\nMessages: %i\r\nInterrupts: %i\r\n\r\n" ,
		magMessage ,
		I2CCONREG , I2CSTATREG , I2ERROR ,
		I2messages, I2interrupts ) ;
	return ;
}
*/

void serial_output_8hz( void )
{
	if (udb_heartbeat_counter % 10 == 0) // Every 2 runs (5 heartbeat counts per 8Hz)
	{
		serial_output("MagOffset: %i, %i, %i\r\nMagBody: %i, %i, %i\r\nMagEarth: %i, %i, %i\r\nMagGain: %i, %i, %i\r\nCalib: %i, %i, %i\r\nMagMessage: %i\r\nTotalMsg: %i\r\nI2CCON: %X, I2CSTAT: %X, I2ERROR: %X\r\n\r\n" ,
			udb_magOffset[0]>>OFFSETSHIFT , udb_magOffset[1]>>OFFSETSHIFT , udb_magOffset[2]>>OFFSETSHIFT ,
			udb_magFieldBody[0] , udb_magFieldBody[1] , udb_magFieldBody[2] ,
			magFieldEarth[0] , magFieldEarth[1] , magFieldEarth[2] ,
			magGain[0] , magGain[1] , magGain[2] ,
			rawMagCalib[0] , rawMagCalib[1] , rawMagCalib[2] ,
			magMessage ,
			I2messages ,
			I2CCONREG , I2CSTATREG , I2ERROR ) ;
	}
	return ;
}


#elif ( SERIAL_OUTPUT_FORMAT == SERIAL_CAM_TRACK )

void serial_output_8hz( void )
{
	unsigned char checksum = 0 ;
	checksum += ((union intbb)(IMUlocationx._.W1))._.B0 + ((union intbb)(IMUlocationx._.W1))._.B1 ;
	checksum += ((union intbb)(IMUlocationy._.W1))._.B0 + ((union intbb)(IMUlocationy._.W1))._.B1 ;
	checksum += ((union intbb)(IMUlocationz._.W1))._.B0 + ((union intbb)(IMUlocationz._.W1))._.B1 ;
	
	// Send location as TXXXXYYYYZZZZ*CC, at 8Hz
	// Where T marks this as a camera Tracking message
	// XXXX is the relative X location in meters as a HEX value
	// YYYY is the relative Y location in meters as a HEX value
	// ZZZZ is the relative Z location in meters as a HEX value
	// And *CC is an asterisk followed by the checksum byte in HEX.
	// The checksum is just the sum of the previous 6 bytes % 256.
	
	serial_output("T%04X%04X%04X*%02X\r\n",
		IMUlocationx._.W1, IMUlocationy._.W1, IMUlocationz._.W1,
		checksum) ;
	
	return ;
}


#else // If SERIAL_OUTPUT_FORMAT is set to SERIAL_NONE, or is not set

void serial_output_8hz( void )
{
	return ;
}

#endif
#endif //  (SERIAL_OUTPUT_FORMAT != SERIAL_MAVLINK)
