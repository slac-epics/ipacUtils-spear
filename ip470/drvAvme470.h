/*******************************************************************************

Project:
    Gemini Multi-Conjugate Adaptive Optics Project

File:
    drvAvme470.h

Description:
    Header file for the Acromag IP470-00x Industrial I/O Pack
    48-Channel TTL Digital Input/Output Module with Interrupts.

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

#ifndef INCdrvAvme470H
#define INCdrvAvme470H

/* Error numbers */

#ifndef M_avme470
#define M_avme470  (600 <<16)
#endif

#define S_avme470_duplicateDevice    (M_avme470| 1) /*Duplicate avme470 device definition*/
#define S_avme470_modeError          (M_avme470| 2) /*Error in mode parameter*/
#define S_avme470_intHandlerError    (M_avme470| 3) /*Error in type of interrupt handler*/
#define S_avme470_intConnectError    (M_avme470| 4) /*Error from intConnect*/
#define S_avme470_validateFailed     (M_avme470| 5) /*Card not validated*/
#define S_avme470_mallocFailed       (M_avme470| 6) /*Malloc Failure*/
#define S_avme470_portError          (M_avme470| 7) /*Error in port number*/
#define S_avme470_bitError           (M_avme470| 8) /*Error in bit number*/
#define S_avme470_readError          (M_avme470| 9) /*Read error*/
#define S_avme470_dataFlagError      (M_avme470|10) /*Error in Data Flag*/
#define S_avme470_cardNotFound       (M_avme470|11) /*Card not found*/
#define S_avme470_noInterrupts       (M_avme470|12) /*Interrupts not supported in STANDARD mode*/
#define S_avme470_invalidRecordType  (M_avme470|13) /*Invalid record type for I/O scan support*/
#define S_avme470_vectorInvalid      (M_avme470|14) /*Interrupt vector number is invalid*/
#define S_avme470_eventRegInvalid    (M_avme470|15) /*Event register invalid*/
#define S_avme470_debounceRegInvalid (M_avme470|16) /*Debounce register invalid*/
#define S_avme470_writeError         (M_avme470|17) /*Write error*/

/* EPICS Device Support return codes */

#define CONVERT        0
#define DO_NOT_CONVERT 2

#define BANK0   (unsigned char)0
#define BANK1   (unsigned char)1
#define BANK2   (unsigned char)2

#define MAXPORTS  6
#define MAXBITS   8

/* Data sizes that can be read */
#define BIT       0
#define NIBBLE    1
#define PORT      2
#define WORD      3

#define RESET     2   /* bit position in interrupt enable reg. */
#define INTEN     1   /* bit position in interrupt enable reg. */

#define STANDARD                 0        /* standard mode configuration */
#define ENHANCED                 1        /* enhanced mode configuration */

#define NOTUSED                  0        /* No interrupt handling       */
#define COS                      1        /* Change-of-state interrupt handling */
#define LEVEL                    2        /* Defined transition interrupt handling */

#define BI                       0        /* bi record type */
#define MBBI                     1        /* mbbi record type */
#define MBBI_DIRECT              2        /* mbbiDirect record type */

/* Parameter mask bit positions */

#define MASK                     2        /* write mask register */
#define EVCONTROL                4        /* event control register */
#define DEBCLOCK                 8        /* debounce clock register */
#define DEBCONTROL            0x10        /* debounce control register */
#define DEBDURATION           0x20        /* debounce duration register */
#define RESET_INTEN           0x40        /* interrupt enable register */
#define VECT                  0x80        /* interrupt vector register */

/* Memory Map for the Avme470 Binary Input/Output Module */

struct map470
{
  struct
   {
     unsigned char nu0;                  
     unsigned char b_select;  /* bank select register */
   } port[8];
   unsigned char nu1;                  
   unsigned char nu2[14];     /* not used */
   unsigned char ier;         /* interrupt enable register */
   unsigned char nu3[15];     /* not used */
   unsigned char ivr;         /* interrupt vector register */
};


/* Structure to hold the board's configuration information */

typedef void (*VOIDFUNPTR)();

struct config470
{
    struct config470 *pnext;                     /* to next device. Must be first member */
    char              *pName;                     /* Name to identify this card           */
    unsigned short    card;                       /* Number of IP carrier board           */
    unsigned short    slot;                       /* Slot number in carrier board         */
    struct map470    *brd_ptr;                   /* base address of the input board      */
    unsigned short    param;                      /* parameter mask for configuring board */
    unsigned char     e_mode;                     /* enhanced operation flag              */
    unsigned char     mask_reg;                   /* output port mask register            */
    unsigned short    ev_control[2];              /* event control register               */
    unsigned char     deb_control;                /* debounce control register            */
    unsigned short    deb_duration;               /* debounce duration registers          */
    unsigned char     deb_clock;                  /* debounce clock select register       */
    unsigned char     enable;                     /* interrupt enable register            */
    unsigned char     vector;                     /* interrupt vector register            */
    unsigned char     id_prom[32];                /* board ID Prom                        */
    unsigned char     ip_pos;                     /* IP under service position            */
    unsigned char     last_chan;                  /* last interrupt input channel number  */
    unsigned char     last_state;                 /* last state of the interrupt channel  */
    unsigned char     intHandler;                 /* interrupt handler flag               */
    VOIDFUNPTR        isr;                        /* Address of Interrupt Service Routine */
    VOIDFUNPTR        usrFunc;                    /* Address of user function             */
#ifndef NO_EPICS
    IOSCANPVT         biScan[MAXPORTS*MAXBITS];   /* One for each bit of each port, bi's  */
    IOSCANPVT         mbbiScan[MAXPORTS*MAXBITS]; /* All possible mbbi's                  */
    IOSCANPVT         mbbiDirectScan[MAXPORTS*MAXBITS]; /* All possible mbbiDirect's      */
#endif
};

int           avme470Report( int interest );
int           avme470Initialise( void );

#ifndef NO_EPICS
int           avme470GetIoScanpvt( char *name, unsigned char port, unsigned char point, 
                                  int mbbiscan, unsigned char intHandler, IOSCANPVT *ppvt );
#endif

int           avme470Create( char *pName, unsigned short card, 
                             unsigned short slot, char *modeName,
                             char *intHandlerName, char *usrFunc, 
                             short vector, short event, short debounce );
void          avme470SetConfig( char *pName, unsigned short card, 
                                unsigned short slot, unsigned char mode,
                                unsigned char intHandler, char *usrFunc, 
                                unsigned short vector, 
                                unsigned short event, unsigned short debounce, 
                                struct config470 *pconfig );
void          avme470Config( struct config470 *pconfig );
unsigned char avme470SelectBank( unsigned char newBank, struct config470 *pconfig );
long          avme470Read( char *name, short port, short bit, int readFlag,
                           unsigned short *pval, int debug );
long          avme470Write( char *name, short port, short bit, int writeFlag,
                            long value, int nobt, int debug );
unsigned char avme470Input( unsigned int *addr );
void          avme470Output( unsigned int *addr, int b );
void          avme470COS( struct config470 *pconfig );
void          avme470LEVEL( struct config470 *pconfig );
void          avme470WhichHandler( char *name, unsigned char *handler );
void         *avme470FindCard( char *name );

#endif  /* INCdrvAvme470H */
