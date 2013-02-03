//
//  main.c
//  GNSStimeConv
//
//  Created by Guido Ottaviani on 27/01/13.
//  Copyright (c) 2013 Guido Ottaviani. All rights reserved.
//

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "constants.h"

#include "time_conversion.h"

unsigned short utcYear;
unsigned char utcMonth;
unsigned char utcDay;
unsigned char utcHour;
unsigned char utcMinute;
float utcSeconds;

int main(int argc, const char * argv[])
{
    if(argc == 1)
    {
       // argv[1]="2444244.500000";
        
        
        
        argv[1]="2456321.365486";

        
        
    }
    TIMECONV_GetUTCTimeFromJulianDate(atof(argv[1]), &utcYear, &utcMonth, &utcDay, &utcHour, &utcMinute, &utcSeconds);

    printf("Conversion\n");
    printf("UTC: %hhu-%hhu-%i  %hhu:%hhu:%f, \r\n\r\n", utcDay, utcMonth, utcYear, utcHour, utcMinute, utcSeconds );
    return 0;
}

