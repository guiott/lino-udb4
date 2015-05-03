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


#include "libUDB_internal.h"
#include "I2C.h"

#define SIM // to avoid hang on simulation


#if (BOARD_IS_CLASSIC_UDB)
#if ( CLOCK_CONFIG == CRYSTAL_CLOCK )
_FOSC( CSW_FSCM_OFF & HS ) ;		// external high speed crystal
#elif ( CLOCK_CONFIG == FRC8X_CLOCK ) 
_FOSC(CSW_FSCM_OFF & FRC_PLL8);
#endif
_FWDT( WDT_OFF ) ;					// no watchdog timer


// Add compatibility for c30 V3.3
#ifndef BORV_20
#define BORV_20 BORV20
#endif
#ifndef _FICD
#define _FICD(x) _ICD(x)
#endif


_FBORPOR( 	PBOR_ON &				// brown out detection on
			BORV_20 &				// brown out set to 2.0 V
			MCLR_EN &				// enable MCLR
			RST_PWMPIN &			// pwm pins as pwm
			PWMxH_ACT_HI &			// PWMH is active high
			PWMxL_ACT_HI ) ;		// PMWL is active high
_FGS( CODE_PROT_OFF ) ;				// no protection
_FICD( 0xC003 ) ;					// normal use of debugging port

#elif (BOARD_TYPE == UDB4_BOARD)
_FOSCSEL(FNOSC_PRI) ; // Primary Oscillator (XT, HS, EC) w/ PLL medium speed XTAL
_FOSC(	FCKSM_CSECMD &   // Clock switching is enabled, Fail-Safe Clock Monitor is disabled
		OSCIOFNC_OFF &   // OSC2 pin has no digital I/O function
		POSCMD_XT ) ;    // XT Oscillator Mode
_FWDT(	FWDTEN_OFF &     // Watchdog timer enabled/disabled by user software
		WINDIS_OFF ) ;   // Watchdog Timer in Non-Window mode
_FGS(	GSS_OFF &        // User program memory is not code-protected
		GCP_OFF &        // User program memory is not code-protected
		GWRP_OFF ) ;     // User program memory is not write-protected
_FPOR(	FPWRT_PWR64 ) ; // 1=POR Timer disabled, otherwise 2-4-8-16-32-64-128ms
_FICD(	JTAGEN_OFF &     // JTAG is Disabled
		ICS_PGD2 ) ;     // Communicate on PGC2/EMUC2 and PGD2/EMUD2

 //<GUIOTT>
  /* CLOCK_FREQ = 8 MHz
     FREQOSC = CLOCK_FREQ*M/(N1*N2) = 8M*40/(2*2)=80Mhz
     CLK_PHASES		2
     FCY = FREQOSC / CLK_PHASES = 40
  */
  #define FCY FREQOSC / CLK_PHASES

  //</GUIOTT>
#endif

#include <libpic30.h>

union udb_fbts_byte udb_flags ;

int defaultCorcon = 0 ;

#if (ANALOG_CURRENT_INPUT_CHANNEL != CHANNEL_UNUSED)
union longww battery_current ;
union longww battery_mAh_used ;
#endif

#if (ANALOG_VOLTAGE_INPUT_CHANNEL != CHANNEL_UNUSED)
union longww battery_voltage;	// battery_voltage._.W1 is in tenths of Volts
#endif

#if (ANALOG_RSSI_INPUT_CHANNEL != CHANNEL_UNUSED)
unsigned char rc_signal_strength ;
#define MIN_RSSI	((long)((RSSI_MIN_SIGNAL_VOLTAGE)/3.3 * 65536))
#define RSSI_RANGE	((long)((RSSI_MAX_SIGNAL_VOLTAGE-RSSI_MIN_SIGNAL_VOLTAGE)/3.3 * 100))
#endif


// Functions only included with nv memory.
#if(USE_NV_MEMORY == 1)
UDB_SKIP_FLAGS udb_skip_flags = {0,0,0};

void udb_skip_radio_trim()
{
	udb_skip_flags.skip_radio_trim = 1;
}

void udb_skip_imu_calibration()
{
	udb_skip_flags.skip_imu_cal = 1;
}

#endif


//#if(USE_NV_MEMORY == 1)
//if(udb_skip_flags.skip_radio_trim == 1)
//if(udb_skip_flags.skip_imu_cal == 1)
//#endif
//

void udb_init(void)
{
	defaultCorcon = CORCON ;
	
#if (BOARD_TYPE == UDB4_BOARD)
  //<GUIOTT>
  /*
   * Clock speed up to 80MHz following the Mark Whitehorn branch on UDB4
   * The following values have been parametrized:
   * DCM frame rate: changed from hardwired 40Hz to parameter HEARTBEAT_HZ
   * PID loop frame rate: changed from hardwired 40Hz to parameter PID_HZ
   * ESC frame rate ESC_HZ

   ** note that the above three rates are not orthogonal

    * Mark Whitehorn notes

    * CPU clock: (limitation: UDB4 only: BOARD_TYPE not in {GREEN,RED,UDB3, RUSTYS}
    * and BOARD_IS_CLASSIC_UDB==0), changed (or added) following parameters and macros
    * CLK_PHASES set to 2
    ** FREQOSC set to 80 MHz (FCY = 40MHz)
    ** PWMOUTSCALE ratio of Timer 3 frequency T3FREQ to original hardwired 2 MHz
    *** T3 prescaler was already at max. value of 8
    ** ADC_CLK speed also increases at 40MIPS since ADCLK_DIV_N_MINUS_1 was at max. value of 63
    ** I2CBRGVAL in magneto_udb4.c now calculated using FREQOSC and CLK_PHASES to achieve 100KHz rate
    ** CPU_RES defined such that cpu_timer units are .01%
    ** CPU_LOAD_PERCENT is changed, I think, but untested since raw cpu_timer values are already in units of .01%
    ** PWINSCALE is added to make pwIn values independent of CPU clock freq.
    ** Timer 1 period now calculated from HEARTBEAT_HZ (sets DCM rate)
    */
    
  PLLFBDbits.PLLDIV = 38 ; // FOSC = 80 MHz (XT = 8.00MHz, N1=2, N2=2, M = 40)
  CLKDIVbits.PLLPOST = 0;  // N1=2
  CLKDIVbits.PLLPRE = 0;   // N2=2
  // Clock switching to incorporate PLL
  __builtin_write_OSCCONH(0x03);		// Initiate Clock Switch to Primary
	// Oscillator with PLL (NOSC=0b011)
  __builtin_write_OSCCONL(0x01);		// Start clock switching

#ifndef SIM	
	while (OSCCONbits.COSC != 0b011);// Wait for Clock switch to occur
	while(OSCCONbits.LOCK!=1) {};	// Wait for PLL to lock
#endif
  //</GUIOT>
#endif
	
	udb_flags.B = 0 ;

#if (ANALOG_CURRENT_INPUT_CHANNEL != CHANNEL_UNUSED)
	battery_current.WW = 0 ;
	battery_mAh_used.WW = 0 ;
#endif
	
#if (ANALOG_VOLTAGE_INPUT_CHANNEL != CHANNEL_UNUSED)
	battery_voltage.WW = 0 ;
#endif
	
#if (ANALOG_RSSI_INPUT_CHANNEL != CHANNEL_UNUSED)
	rc_signal_strength = 0 ;
#endif

  udb_init_leds() ;
	udb_init_ADC() ;
	udb_init_clock() ;
	udb_init_capture() ;
	
#if (MAG_YAW_DRIFT == 1  ||  USE_BAROMETER == 1)
        #if (USE_I2C1_DRIVER == 1)
                I2C1_init();
        #endif
        #if (USE_I2C2_DRIVER == 1)
            I2C2_init(); // NEW I2C QUEUE FUNCTION FOR MULTIPLE SENSOR SUPPORT
        #endif
        //  I2C1_init() ;
#endif
	
	udb_init_GPS() ;
	udb_init_USART() ;
	udb_init_pwm() ;
	
#if (USE_OSD == 1)
	udb_init_osd() ;
#endif
  
	SRbits.IPL = 0 ;	// turn on all interrupt priorities
	
	return ;
}


void udb_run(void)
{
	//  nothing else to do... entirely interrupt driven
	while (1)
	{
		// pause cpu counting timer while not in an ISR
		indicate_loading_main ;
	}
	// Never returns
}

void udb_init_leds( void )
{
	
#if (BOARD_IS_CLASSIC_UDB == 1)
	TRISFbits.TRISF0 = 0 ;
	
#elif (BOARD_TYPE == UDB4_BOARD)
  int LedCnt;
  unsigned int LedDelay=100;
	_TRISE1 = _TRISE2 = _TRISE3 = _TRISE4 = 0 ;
	_LATE1 = _LATE2 = _LATE3 = _LATE4 = LED_OFF ;
  //<GUIOTT>
  for(LedCnt=0;LedCnt<4;LedCnt++)
  {
      LED_BLUE=LED_ON;
      __delay_ms(LedDelay);
      LED_BLUE=LED_OFF;
      LED_ORANGE=LED_ON;
      __delay_ms(LedDelay);
      LED_ORANGE=LED_OFF;
      LED_GREEN=LED_ON;
      __delay_ms(LedDelay);
      LED_GREEN=LED_OFF;
      LED_RED=LED_ON;
      __delay_ms(LedDelay);
      LED_RED=LED_OFF;
      __delay_ms(LedDelay);
  }
  //</GUIOTT>
#endif
 return ;
}

#ifdef INITIALIZE_VERTICAL // for VTOL, vertical initialization
void udb_a2d_record_offsets(void)
{
#if(USE_NV_MEMORY == 1)
	if(udb_skip_flags.skip_imu_cal == 1)
		return;
#endif

	// almost ready to turn the control on, save the input offsets
	UDB_XACCEL.offset = UDB_XACCEL.value ;
	udb_xrate.offset = udb_xrate.value ;
	UDB_YACCEL.offset = UDB_YACCEL.value - ( Y_GRAVITY_SIGN ((int)(2*GRAVITY)) ); // opposite direction
	udb_yrate.offset = udb_yrate.value ;
	UDB_ZACCEL.offset = UDB_ZACCEL.value ; 
	udb_zrate.offset = udb_zrate.value ;
#ifdef VREF
	udb_vref.offset = udb_vref.value ;
#endif
	return ;
}
#else  // horizontal initialization
void udb_a2d_record_offsets(void)
{
#if(USE_NV_MEMORY == 1)
	if(udb_skip_flags.skip_imu_cal == 1)
		return;
#endif

	// almost ready to turn the control on, save the input offsets
	UDB_XACCEL.offset = UDB_XACCEL.value ;
	udb_xrate.offset = udb_xrate.value ;
	UDB_YACCEL.offset = UDB_YACCEL.value ;
	udb_yrate.offset = udb_yrate.value ;
	UDB_ZACCEL.offset = UDB_ZACCEL.value + ( Z_GRAVITY_SIGN ((int)(2*GRAVITY))) ; // same direction
	udb_zrate.offset = udb_zrate.value ;									
#ifdef VREF
	udb_vref.offset = udb_vref.value ;
#endif
	return ;
}
#endif


void udb_servo_record_trims(void)
{
	int i;
	for (i=0; i <= NUM_INPUTS; i++)
		udb_pwTrim[i] = udb_pwIn[i] ;
	
	return ;
}


// saturation logic to maintain pulse width within bounds
int udb_servo_pulsesat ( long pw )
{
	if ( pw > SERVOMAX ) pw = SERVOMAX ;
	if ( pw < SERVOMIN ) pw = SERVOMIN ;
	return (int)pw ;
}


void calculate_analog_sensor_values( void )
{
#if (ANALOG_CURRENT_INPUT_CHANNEL != CHANNEL_UNUSED)
	// Shift up from [-2^15 , 2^15-1] to [0 , 2^16-1]
	// Convert to current in tenths of Amps
	battery_current.WW = (udb_analogInputs[ANALOG_CURRENT_INPUT_CHANNEL-1].value + (long)32768) * (MAX_CURRENT) + (((long)(CURRENT_SENSOR_OFFSET)) << 16) ;
	
	// mAh = mA / 144000 (increment per 40Hz tick is /40*60*60)
	// 90000/144000 == 900/1440
	battery_mAh_used.WW += (battery_current.WW / 1440) ;
#endif

#if (ANALOG_VOLTAGE_INPUT_CHANNEL != CHANNEL_UNUSED)
	// Shift up from [-2^15 , 2^15-1] to [0 , 2^16-1]
	// Convert to voltage in tenths of Volts
	battery_voltage.WW = (udb_analogInputs[ANALOG_VOLTAGE_INPUT_CHANNEL-1].value + (long)32768) * (MAX_VOLTAGE) + (((long)(VOLTAGE_SENSOR_OFFSET)) << 16) ;
#endif

#if (ANALOG_RSSI_INPUT_CHANNEL != CHANNEL_UNUSED)
	union longww rssi_accum ;
	rssi_accum.WW = (((udb_analogInputs[ANALOG_RSSI_INPUT_CHANNEL-1].value + 32768) - (MIN_RSSI)) * (10000 / (RSSI_RANGE))) ;
	if (rssi_accum._.W1 < 0)
		rc_signal_strength = 0 ;
	else if (rssi_accum._.W1 > 100)
		rc_signal_strength = 100 ;
	else
		rc_signal_strength = (unsigned char)rssi_accum._.W1 ;
#endif
}
