/*******************************************************************************

Project:
    Gemini Multi-Conjugate Adaptive Optics Project

File:
    drvXy5320.h

Description:
    Header file for the XIP-5320-000 Industrial I/O Pack
    12-Bit High Density Analog Input Board

Author:
    Andy Foster <ajf@observatorysciences.co.uk>

Created:
    12th November 2002

Copyright (c) 2002 Andy Foster

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

*******************************************************************************/

#ifndef INCdrvXy5320H
#define INCdrvXy5320H

/* Error numbers */

#ifndef M_xy5320
#define M_xy5320  (602 <<16)
#endif

#define S_xy5320_duplicateDevice     (M_xy5320| 1) /*Duplicate xy5320 device definition*/
#define S_xy5320_tooManyChannels     (M_xy5320| 2) /*Too many channels*/
#define S_xy5320_fileOpenFailed      (M_xy5320| 3) /*File open failed*/
#define S_xy5320_fileFormatError     (M_xy5320| 4) /*File format error*/
#define S_xy5320_semFailed           (M_xy5320| 5) /*Semaphore creation failed*/
#define S_xy5320_voltRangeError      (M_xy5320| 6) /*Error in voltage range*/
#define S_xy5320_modeError           (M_xy5320| 7) /*Error in mode specified*/
#define S_xy5320_notValidated        (M_xy5320| 8) /*Card not validated*/
#define S_xy5320_mallocFailed        (M_xy5320| 9) /*Malloc failure*/
#define S_xy5320_cardNotFound        (M_xy5320|10) /*Card not found*/
#define S_xy5320_invalidChannel      (M_xy5320|11) /*Invalid channel number*/
#define S_xy5320_invalidGain         (M_xy5320|12) /*Invalid gain*/
#define S_xy5320_invalidFieldType    (M_xy5320|13) /*Invalid field type*/
#define S_xy5320_noChannels          (M_xy5320|14) /*No channels defined*/
#define S_xy5320_taskCreate          (M_xy5320|15) /*Task create failed*/
#define S_xy5320_invalidChannelIndex (M_xy5320|16) /*Invalid channel index*/
#define S_xy5320_noSpace             (M_xy5320|17) /*No space available for array*/
#define S_xy5320_readError           (M_xy5320|18) /*Read error*/


#define MAX_SE_CHANNELS   40  /* Maximum number of SE inputs      */
#define MAX_DIF_CHANNELS  20  /* Maximum number of DIF inputs     */
#define MAX_LINE_LENGTH   16  /* Maximum length of input line     */
#define CHARS_PER_NUMBER   4  /* Number of characters in a number */

/* Maximum number of samples to average over.              */
/* On the 68k, averaging 256 samples for each of 40        */
/* single-ended inputs at 20 Hz uses 68% of the CPU!!      */
/* Averaging 16 samples for each of 40 single-ended inputs */
/* at 20 Hz uses 17% of the CPU.                           */

#define MAX_SAMPLES      256

/* EPICS Device Support return codes */

#define CONVERT        0
#define DO_NOT_CONVERT 2

/* Types */

#define TYPE_LONG   0   /* code for an array of long's   */
#define TYPE_DOUBLE 1   /* code for an array of double's */

/* mode and gain code definitions */

#define DIF	    1   /* code for differential channel mode */
#define SE	    2   /* code for single ended channel mode */
#define AZV	    3   /* code for auto-zero mode */
#define CAL	    4   /* code for calibration mode */

#define GAIN_X1     1   /* code for gain = 1 */
#define GAIN_X2     2   /* code for gain = 2 */
#define GAIN_X4     4   /* code for gain = 4 */
#define GAIN_X8     8   /* code for gain = 8 */

#define RANGE_5TO5   1 	/* input range */
#define RANGE_10TO10 2 	/* input range */
#define RANGE_0TO10  3	/* special auto zero value for this range */

#define BIT12	(unsigned short)0xFFF0	/* 12 bit data mask */
#define BIT14	(unsigned short)0xFFFC	/* 14 bit data mask */
#define BIT16	(unsigned short)0xFFFF	/* 16 bit data mask */

#define CON12    (long)4096	/* constant for data correction equation */
#define CON14    (long)16384	/* constant for data correction equation */
#define CON16    (long)65536	/* constant for data correction equation */

#define STRIG	     0	/* software trigger */
#define ETRIG	     1	/* external trigger */

/*  board control register bit positions */

#define SEL_SELECT  0x0100	/* single ended low select bit mask */
#define SEH_SELECT  0x0200	/* single ended high select bit mask */
#define AZ_SELECT   0x0300	/* auto-zero select bit mask */
#define CTRIG	    0x8000	/* conversion trigger */

#define CAL0	    20		/* 4.9V cal Voltage */
#define CAL1	    21		/* 2.459V cal Voltage */
#define CAL2	    22		/* 1.225V cal Voltage */
#define CAL3	    23		/* 0.6125V cal Voltage */

/* Memory Map */

struct map5320
{
    unsigned short cntl_reg;	 /* board control register */
    unsigned short unused1[7];   /* undefined */
    unsigned short strt_reg;	 /* start conversion register */
    unsigned short unused2[7];   /* undefined */
    unsigned short ai_reg;	 /* analog input data register */
    unsigned short unused3[15];	 /* undefined */
    unsigned short unused4;      /* undefined */
    unsigned short unused5[31];  /* undefined */
    struct id
    {
      char unused1;	         /* undefined */
      unsigned char prom;	 /* id prom location */
    } id_map[32];
};

/*
    Defined below is the structure for the scan array.  The scan
    array is an array of channel & gain pairs which are used
    when scanning a group of non-sequential input channels or when
    scanning channels using different gain settings.
*/

struct scan_array
{
    unsigned short chan;  /* channel number                              */
    unsigned short gain;  /* gain setting corresponding to above channel */
};
    

/* Configuration structure */

struct config5320
{
    struct config5320 *pnext;                      /* to next device. Must be first member      */
    char              *pName;                      /* Name to identify this card                */
    ushort_t          card;                        /* Number of IP carrier board                */
    ushort_t          slot;                        /* Slot number in carrier board              */
    struct map5320    *brd_ptr;                    /* pointer to base address of board          */
    unsigned char     range;	                   /* input range jumper setting of the board   */
    unsigned char     trigger;	                   /* triggering option software/external       */
    unsigned char     mode;	                   /* the mode                                  */
    unsigned short    average;	                   /* number of samples to average              */
    unsigned short    data_mask;                   /* bit mask for 12 bit/16 bit A/D converters */
    long              bit_constant;                /* constant for data correction equation     */
    unsigned short    raw_data[MAX_SE_CHANNELS];   /* raw data buffer                           */
    unsigned short    az_data[MAX_SE_CHANNELS];    /* auto-zero buffer                          */
    unsigned short    cal_data[MAX_SE_CHANNELS];   /* calibration buffer                        */
    long              cor_data[MAX_SE_CHANNELS];   /* corrected buffer                          */
    double            analogData[MAX_SE_CHANNELS]; /* corrected buffer converted back to analog */
    struct scan_array s_array[MAX_SE_CHANNELS];    /* array of channels and gains               */
    long              numChannels;                 /* Number of channels being used             */
    SEM_ID            semId;                       /* Semaphore to protect calibration & reads  */
    int               cal;                         /* Is the board calibrated?                  */
    int               startTasks;                  /* Do we start the tasks?                    */
};


/* Prototypes */

int            xy5320Report( int interest );
int            xy5320Initialise( void );
int            xy5320CalTask();
int            xy5320ReadTask();
int            xy5320Create( char *pName, ushort_t card, ushort_t slot, char *voltRangeName,
                             char *modeName, int numSamples, char *filename );
long           xy5320SetConfig( char *pName, ushort_t card, ushort_t slot,
                                struct config5320 *pconfig, int voltRange, int mode,
                                int numSamples, char *filename );
void           xy5320ReadInputs( struct config5320 *pconfig );
unsigned short xy5320BuildControl( struct config5320 *pconfig, int index );
void           xy5320CorrectInputs( struct config5320 *pconfig );
long           xy5320ReadChannel( char *name, int channel, unsigned long ftvl, void *prval );
long           xy5320ReadArray( char *name, int startIndex, int numChan, unsigned long ftvl,
                                void *prval );
int            xy5320GetNumChan( void *ptr );
void          *xy5320FindCard( char *name );
int            xy5320FindChannel( void *ptr, int channel, int *chanIndex );
int            xy5320GetNonSpace( char *buf, int start );
int            xy5320GetNonDigit( char *buf, int start );

#endif  /* INCdrvXy5320H */
