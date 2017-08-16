/*******************************************************************************

Project:
    Gemini Multi-Conjugate Adaptive Optics Project

File:
    drvAvme470.c

Description:
    vxWorks/EPICS Driver for the Acromag 470-00x Industrial I/O Pack
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
/*-----------------------------------------------------------------------------
From: Till Straumann [mailto:strauman@slac.stanford.edu]
Sent: Thu 1/12/2006 5:26 PM
To: Allison, Stephanie
Cc: Wermelskirchen, Clemens
Subject: Re: acromag IP470 ISR


Let me elaborate a little bit:

The basic problem is that for some - alas still unknown -
reason, a bad interrupt vector is sometimes reported during
an IACK cycle as a result of interrupt level 6 being asserted.

As a precaution, the RTEMS BSP disables a VME interrupt (level)
if it receives a vector for which no handler is installed
since it cannot invoke a handler which normally would be responsible
for clearing the interrupt condition. This is done to prevent
'runaway interrupts' (e.g., due to a incorrectly configured board)
from freezing the system.

The strange thing is that I could have 3 IP470s on a single
carrier (the 'middle' one) and the CANbus IP on the 'left'
carrier sharing VME level 6 without problems, i.e., I could
let both, the 470s and the CAN generate thousands of interrupts
without observing a single spurious vector.

As soon as I re-enabled interrupts also on the IP470 which
sit on the rightmost carrier then I get about 3-5 bad vectors
for every 100 interrupts.

Of course, I double-checked that the vector registers
are all programmed correctly (and they are: 0x60..0x6c).
The erroneous vectors reported are frequently 0x00, 0x40, 0x48.

As a workaround, I believe that the 470s can be operated
in 'STANDARD' (as opposed to 'ENHANCED') mode which doesn't
use interrupts.

If interrupt-driven mode is desired then further investigation
is needed to find the cause of the problem.

Most of the time we spent was needed to find a way to reproduce
the symptoms. Now, it seems that we at least have a method
to reproduce the problem within a few seconds.

T.

Allison, Stephanie wrote:
> Hi Clemens,
>
> Till and I (mostly Till) spent quite some time on Monday looking at
> the problem where an interrupt is permanently disabled on iocmu.  We
> discovered that we could reproduce the problem by just hitting the
> orbit interlock reset button many times (sometime just a few hits is
> enough to cause the problem).  Till was able to isolate it to the ISR
> run by the acromag IP470 and for now, he provided an alternate ISR
> that simply disables the interrupt for the IP470 and things are
> running fine since your IP470 databases don't use the interrupt.  I
> don't see anything obviously wrong in the ISR.  (Though I wonder
> about the order of actions at the end of the ISR where the interrupt
> is re-enabled on the carrier before the interrupt is re-enabled on
> the IP module (?)).
------------------------------------------------------------------------*/

#ifdef NO_EPICS
#include <vxWorks.h>
#include <iv.h>
#include <intLib.h>
#include <logLib.h>
#include <taskLib.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DEBUG 0

/* These are the IPAC IDs for this module */
#define IP_MANUFACTURER_ACROMAG 0xa3
#define IP_MODEL_ACROMAG_IP470   0x08
#define PARAM_MASK_STANDARD   0x43   /* Bits 0,1,6 are set         */
#define PARAM_MASK_ENHANCED   0xFF   /* All bits set               */
#define OUTPUT_MASK           0x3F   /* Mask writes to all outputs */

static struct config470 *ptrAvme470First = NULL;

#ifndef NO_EPICS
#include "devLib.h"
#include "drvSup.h"
#include "dbScan.h"
#include "epicsInterrupt.h"
#include "epicsExport.h"
#include "iocsh.h"
#endif

#include "drvIpac.h"
#include "drvAvme470.h"

#include "basicIoOps.h"


#ifndef NO_EPICS
/* EPICS Driver Support Entry Table */

struct drvet drvAvme470 = {
  2,
  (DRVSUPFUN) avme470Report,
  (DRVSUPFUN) avme470Initialise
};
epicsExportAddress(drvet, drvAvme470);
#endif


int avme470Report( int interest )
{
  int               i;
  int               j;
  unsigned char     *idptr;
  unsigned short    val;
  struct config470 *plist;

  plist = ptrAvme470First;
  while( plist )
  {
    avme470SelectBank(BANK1, plist);              /* select I/O bank   */
    if( interest == 0 || interest == 2 )
    {
      /* interrupt enable status */
      plist->enable = avme470Input((unsigned int *)&plist->brd_ptr->ier);
      /* interrupt vector */
      plist->vector = avme470Input((unsigned int *)&plist->brd_ptr->ivr);

      idptr = (unsigned char *)plist->brd_ptr + 0x80;

      for(i = 0, j = 1; i < 32; i++, j += 2)
        plist->id_prom[i] = avme470Input((unsigned int *)&idptr[j]);
    
      printf("\nBoard Status Information: %s\n", plist->pName);
      printf("\nInterrupt Enable Register:   %02x",plist->enable);
      printf("\nInterrupt Vector Register:   %02x",plist->vector);
      printf("\nLast Interrupting Channel:   %02x",plist->last_chan);
      printf("\nLast Interrupting State:     %02x",plist->last_state);
      printf("\nIdentification:              ");
      for(i = 0; i < 4; i++)                 /* identification */
        printf("%c",plist->id_prom[i]);
      printf("\nManufacturer's ID:           %x",(unsigned char)plist->id_prom[4]);
      printf("\nIP Model Number:             %x",(unsigned char)plist->id_prom[5]);
      printf("\nRevision:                    %x",(unsigned char)plist->id_prom[6]);
      printf("\nReserved:                    %x",(unsigned char)plist->id_prom[7]);
      printf("\nDriver I.D. (low):           %x",(unsigned char)plist->id_prom[8]);
      printf("\nDriver I.D. (high):          %x",(unsigned char)plist->id_prom[9]);
      printf("\nTotal I.D. Bytes:            %x",(unsigned char)plist->id_prom[10]);
      printf("\nCRC:                         %x",(unsigned char)plist->id_prom[11]);
      printf("\n\n");
    }

    if( interest == 1 || interest == 2 )
    {
      printf("\nBoard Pattern: %s\n", plist->pName);
      printf("--------------\n");
      printf("              Bits\n");
      printf("         0 1 2 3 4 5 6 7\n");
      printf("         - - - - - - - -\n");
      for( i=0; i<MAXPORTS; i++ )
      {
        printf("Port %d:  ", i);
        for( j=0; j<MAXBITS; j++ )
        {
          avme470Read( plist->pName, i, j, BIT, &val, 0 );
          printf("%d ", val);
        }
        printf("\n");
      }
      printf("\n");
    }
    plist = plist->pnext;
  }
  return(OK);
}


int avme470Initialise( void )
{
  struct config470 *plist;
  int              status;

  plist = ptrAvme470First;
  while( plist )
  {
#ifndef NO_EPICS
    {
      int i;
      for( i=0; i<MAXPORTS*MAXBITS; i++ )
      {
        scanIoInit( &plist->biScan[i]         );
        scanIoInit( &plist->mbbiScan[i]       );
        scanIoInit( &plist->mbbiDirectScan[i] );
      }
    }
#endif

    if( plist->e_mode == ENHANCED )
    {
printf("ipmIntConnect(%d,%d,%d,0x%p,0x%x)\n",
	plist->card, plist->slot, plist->vector, plist->isr, (int)plist);

      if( (status=ipmIntConnect(plist->card, plist->slot, plist->vector, 
                                plist->isr, (int)plist)) )
      {
        printf("avme470Initialise: %s: Error %d from ipmIntConnect\n", 
                plist->pName, status);
        return S_avme470_intConnectError;
      }

      ipmIrqCmd( plist->card, plist->slot, 0, ipac_irqEnable);
    }

    plist = plist->pnext;
  }
  return(OK);
}


#ifndef NO_EPICS
int avme470GetIoScanpvt( char *name, unsigned char port, unsigned char point, 
                        int recType, unsigned char intHandler, IOSCANPVT *ppvt )
{
  struct config470 *plist;
  int               bitNum=0;

  if( intHandler == NOTUSED )
  {
    printf("avme470GetIoScanpvt: %s: Not configured for interrupts\n", name);
    return S_avme470_noInterrupts;
  }
  else if( intHandler == COS )
  {
    bitNum = (port << 2) + point;
    if( point > 3 )
      bitNum -= 4;
  }
  else if( intHandler == LEVEL )
    bitNum = MAXBITS*port + point;

  plist = avme470FindCard(name);
  if( plist )
  {
    if( recType == BI )
      *ppvt = plist->biScan[bitNum];
    else if( recType == MBBI )
      *ppvt = plist->mbbiScan[bitNum];
    else if( recType == MBBI_DIRECT )
      *ppvt = plist->mbbiDirectScan[bitNum];
    else
    {
      printf("avme470GetIoScanpvt: %s: Invalid record type (%d)\n", name, recType);
      return S_avme470_invalidRecordType;
    }
  }
  return(OK);
}
#endif


int avme470Create( char *pName, unsigned short card, unsigned short slot, 
                   char *modeName,
                   char *intHandlerName, char *usrFunc, short vector, 
                   short event, short debounce )
{
  struct config470 *plist;
  struct config470 *pconfig;
  unsigned char     mode;
  unsigned char     intHandler;
  int               status;

  if( !strcmp(modeName, "STANDARD") )
  {
    mode       = STANDARD;
    intHandler = NOTUSED;
  }
  else if( !strcmp(modeName, "ENHANCED") )
  {
    mode = ENHANCED;
    if( !strcmp(intHandlerName, "COS") )
      intHandler = COS;
    else if( !strcmp(intHandlerName, "LEVEL") )
      intHandler = LEVEL;
    else
    {
      printf("avme470Create: %s: Invalid interrupt handler %s\n", pName, intHandlerName);
      return S_avme470_intHandlerError;
    }

    if( vector < 0x0 || vector > 0xFF )
    {
      printf("avme470Create: %s: Interrupt vector invalid 0x%x\n", pName, vector);
      return S_avme470_vectorInvalid;
    }

    if( event < 0x0 || event > 0xFFF )
    {
      printf("avme470Create: %s: Event Register invalid 0x%x\n", pName, event);
      return S_avme470_eventRegInvalid;
    }

    if( debounce < 0x0 || debounce > 0xFF )
    {
      printf("avme470Create: %s: Debounce Register invalid 0x%x\n", pName, debounce);
      return S_avme470_debounceRegInvalid;
    }
  }
  else
  {
    printf("avme470Create: %s: Board Mode Error %s\n", pName, modeName);
    return S_avme470_modeError;
  }

  status = ipmValidate(card, slot, IP_MANUFACTURER_ACROMAG, IP_MODEL_ACROMAG_IP470);
  if( status )
  {
    printf("avme470Create: Error %d from ipmValidate\n", status);
    return S_avme470_validateFailed;
  }

  if( !ptrAvme470First )

  {
    ptrAvme470First = malloc(sizeof(struct config470));
    if( !ptrAvme470First )
    {
      printf("avme470Create: First malloc failed\n");
      return S_avme470_mallocFailed;
    }
    else
    {
      avme470SetConfig( pName, card, slot, mode, intHandler, usrFunc, vector, event, 
                       debounce, ptrAvme470First );

      /* Configure the new board based on above settings */
      avme470Config( ptrAvme470First );
    }
  }
  else
  {
    /* Check for unique card */

    plist = ptrAvme470First;
    while( TRUE )
    {
      if( !strcmp(plist->pName, pName) || ((plist->card == card) && (plist->slot == slot)) )
      {
        printf("avme470Create: Duplicate device (%s, %d, %d)\n", pName, card, slot);
        return S_avme470_duplicateDevice;
      }
      if( plist->pnext == NULL )
        break;
      else
        plist = plist->pnext;
    }

    /* plist now points to the last item in the list */

    pconfig = malloc(sizeof(struct config470));
    if( pconfig == NULL )
    {
      printf("avme470Create: malloc failed\n");
      return S_avme470_mallocFailed;
    }

    /* pconfig is the configuration block for our new card */
    avme470SetConfig( pName, card, slot, mode, intHandler, usrFunc, vector, event,
                     debounce, pconfig );

    /* Update linked-list */
    plist->pnext = pconfig;

    /* Configure the new board based on above settings */
    avme470Config( pconfig );
  }
  return(OK);
}


void avme470SetConfig( char *pName, unsigned short card, unsigned short slot, 
                       unsigned char mode, unsigned char intHandler,
                       char *usrFunc, unsigned short vector,
                       unsigned short event, unsigned short debounce, 
                       struct config470 *pconfig )
{
  pconfig->pnext      = NULL;
  pconfig->pName      = pName;
  pconfig->card       = card;
  pconfig->slot       = slot;
  pconfig->brd_ptr    = (struct map470 *)ipmBaseAddr(card, slot, ipac_addrIO);
  pconfig->mask_reg   = OUTPUT_MASK;  /* Mask writes to all outputs */
  pconfig->mask_reg = 0;	/* CW to test interrupts */
  pconfig->e_mode     = mode;
  pconfig->intHandler = intHandler;

  if( pconfig->e_mode == STANDARD )
  {
    pconfig->param         = PARAM_MASK_STANDARD;
    pconfig->ev_control[0] = 0;
    pconfig->ev_control[1] = 0;
    pconfig->deb_clock     = 0;
    pconfig->deb_control   = 0;
    pconfig->deb_duration  = 0;
    pconfig->enable        = 0;
    pconfig->vector        = 0;
    pconfig->isr           = NULL;
    pconfig->usrFunc       = NULL;
  }
  else if( pconfig->e_mode == ENHANCED )
  {
    pconfig->param = PARAM_MASK_ENHANCED;
    if( pconfig->intHandler == COS )
    {
      pconfig->ev_control[0] = 0xAA;      /* Bi-wiring at the port level will trap both transitions */
      pconfig->ev_control[1] = 0xA;
      pconfig->isr           = avme470COS; /* Connect ISR */
    }
    else if(  pconfig->intHandler == LEVEL )
    {
      pconfig->ev_control[0] = event&0xFF;  /* Events which generate interrupts */
      pconfig->ev_control[1] = (event>>8)&0xFF;
      pconfig->isr           = avme470LEVEL; /* Connect ISR */
    }
    pconfig->usrFunc      = (VOIDFUNPTR)usrFunc; /* User defined function */
    pconfig->deb_clock    = 1;         /* Use the 8 MHz IP bus clock (recommended) */
    pconfig->deb_control  = 0x3F;      /* Enable debounced operation for all bits of all ports */
    pconfig->deb_duration = debounce;  /* Debounce duration */
    pconfig->enable       = INTEN;     /* Enable interrupts */
    pconfig->vector       = vector;    /* Interrupt vector  */
  }
}


void avme470Config( struct config470 *pconfig )
{
  int i;

  if((pconfig->param & RESET_INTEN) && (pconfig->enable & RESET))
    avme470Output((unsigned int *)&pconfig->brd_ptr->ier, RESET); /* set reset bit */

/*
  Put the card in Enhanced Mode if this has been selected
*/

  if((pconfig->param & ENHANCED) && (pconfig->e_mode != 0))
  {
    avme470Output((unsigned int *)&pconfig->brd_ptr->port[7].b_select, 0x07);
    avme470Output((unsigned int *)&pconfig->brd_ptr->port[7].b_select, 0x0D);
    avme470Output((unsigned int *)&pconfig->brd_ptr->port[7].b_select, 0x06);
    avme470Output((unsigned int *)&pconfig->brd_ptr->port[7].b_select, 0x12);

  }

/*
  Check to see if the Interrupt Vector Register is to be updated.
  Update the Vector Register before enabling Global Interrupts so that
  the board will always output the correct vectors upon interrupt.
*/

  if(pconfig->param & VECT )
    avme470Output((unsigned int *)&pconfig->brd_ptr->ivr, pconfig->vector);

/*
  If in standard mode and the Mask Register is to be updated, then update it.
*/

  if((pconfig->e_mode == 0) && (pconfig->param & MASK))
  {
    avme470SelectBank(BANK0, pconfig);
    avme470Output((unsigned int *)&pconfig->brd_ptr->port[7].b_select, (pconfig->mask_reg & 0x3F));
  }

/*
  Configure Enhanced Mode if this has been selected
*/
    
  if((pconfig->param & ENHANCED) && (pconfig->e_mode != 0))
  {
    if(pconfig->param & MASK)   /* Update Mask Register */
    {
      avme470SelectBank(BANK0, pconfig);
      avme470Output((unsigned int *)&pconfig->brd_ptr->port[7].b_select, (pconfig->mask_reg & 0x3F));
    }

    if(pconfig->param & EVCONTROL)  /* Update Event Control Registers */
    {
      /* Note: avme470SelectBank 1 writes the event sense polarity for R1,P7,B1 */
      avme470SelectBank(BANK1, pconfig);
      avme470Output((unsigned int *)&pconfig->brd_ptr->port[6].b_select, pconfig->ev_control[0]);
      avme470Output((unsigned int *)&pconfig->brd_ptr->port[7].b_select, pconfig->ev_control[1]);
    }

    if(pconfig->param & DEBCONTROL)  /* Update Debounce Control Register */
    {
      avme470SelectBank(BANK2, pconfig);
      avme470Output((unsigned int *)&pconfig->brd_ptr->port[0].b_select, pconfig->deb_control);
    }

    if(pconfig->param & DEBDURATION)  /* Update Debounce Duration Register */
    {
      avme470SelectBank(BANK2, pconfig);
      avme470Output((unsigned int *)&pconfig->brd_ptr->port[1].b_select, pconfig->deb_duration&0xFF);
      avme470Output((unsigned int *)&pconfig->brd_ptr->port[2].b_select, (pconfig->deb_duration>>8)&0xF);
    }

    if(pconfig->param & DEBCLOCK)   /* Update Debounce Clock Register */
    {
      avme470SelectBank(BANK2, pconfig);
      avme470Output((unsigned int *)&pconfig->brd_ptr->port[3].b_select, pconfig->deb_clock);
    }

    if((pconfig->param & RESET_INTEN) && (pconfig->enable & INTEN)) /* Int. Enable Reg. */
    {
      avme470SelectBank(BANK1, pconfig);
      for(i = 0; i < 6; i++)
      {
        avme470Output((unsigned int *)&pconfig->brd_ptr->port[i].b_select, (unsigned char)0);
        avme470Output((unsigned int *)&pconfig->brd_ptr->port[i].b_select, (unsigned char)0xFF);
      }
      avme470Output((unsigned int *)&pconfig->brd_ptr->ier, INTEN);
    }
  }  /* End of Enhanced Mode set-up */
}


unsigned char avme470SelectBank( unsigned char newBank, struct config470 *pconfig )
{
  unsigned char oldBank;   /* old bank number */
  unsigned char bankBits;  /* bank select info */

  bankBits = avme470Input((unsigned int *)&pconfig->brd_ptr->port[7].b_select); /*get current*/
  oldBank  = ((bankBits & 0xC0) >> 6);                      /* isolate bank select bits */
   
  if(oldBank == newBank)                                    /* same bank? */
    return(oldBank);                                        /* no need to change bits */
   
  if(oldBank == BANK1)                                      /* special treatment required */
    bankBits = pconfig->ev_control[1];                      /* Must use ev_control bits */

  bankBits &= 0x3F;                                         /* save all but bank sel. bits */
  bankBits |= (newBank << 6);                               /* OR in new bank bits */

  avme470Output((unsigned int *)&pconfig->brd_ptr->port[7].b_select, bankBits);
                                                            /* read back to ensure it is written into the hardware register */
/*  avme470Input((unsigned int *)&pconfig->brd_ptr->port[7].b_select); */

  return(oldBank);
}


long avme470Read( char *name, short port, short bit, int readFlag,
                      unsigned short *pval, int debug )
{
  struct config470 *plist;
  struct map470    *map_ptr;
  unsigned char     ports[3];
  unsigned int      res, n;
  int               shift;

  if( (port < 0) || (port >= MAXPORTS) )
  {
    printf("avme470Read: %s: port number out of range %d\n", name, port );
    return S_avme470_portError;
  }
  else if( (bit < 0) || (bit >= MAXBITS) )
  {
    printf("avme470Read: %s: bit number out of range %d\n", name, bit );
    return S_avme470_bitError;
  }
  else
  {
    plist = avme470FindCard(name);
    if( plist )
    {
      avme470SelectBank(BANK0, plist);      /* select I/O bank */
      map_ptr = plist->brd_ptr;
      if( readFlag == BIT || readFlag == PORT )
      {
        *pval = avme470Input((unsigned *)&map_ptr->port[port].b_select);
        if( readFlag == BIT )
        {
          if( *pval & (1 << bit) )
            *pval = 1;
          else 
            *pval = 0;
        }

        if( debug )
          printf("avme470Read: name = %s, port = %d, bit = %d, pval = %d\n", 
                  name, port, bit, *pval);
      }
      else if( readFlag == NIBBLE || readFlag == WORD )
      {
        ports[2] = ports[1] = ports[0] = 0;
        for ( n = 0; n < 3  &&  port < MAXPORTS; n++, port++ ) {
          ports[n] = avme470Input((unsigned *)&map_ptr->port[port].b_select);
        }

        /* Combine into a 32-bit integer */
        res   =  (ports[2]<<16) + (ports[1]<<8) + ports[0];

        /* Calculate position in integer where we want to be */
        shift = bit;

        if( readFlag == NIBBLE )
          *pval = (res >> shift) & 0xF;
        else if( readFlag == WORD )
          *pval = (res >> shift) & 0xFFFF;

        if( debug )
          printf("avme470Read: name = %s, port = %d, bit = %d, pval = %d\n", 
                  name, port, bit, *pval);
      }
      else
      {
        printf("avme470Read: %s: Data flag error (%d)\n", name, readFlag );
        return S_avme470_dataFlagError;
      }
    }
    else
    {
      printf("avme470Read: Card %s not found\n", name);
      return S_avme470_cardNotFound;
    }
  }
  return(OK);
}


long avme470Write( char *name, short port, short bit, int writeFlag,
                   long value, int nobt, int debug )
{
  struct config470 *plist;
  struct map470    *map_ptr;
  unsigned char    bpos, oldport, newport;
  int              nBits = nobt;

  unsigned long    zeroMask, zeroOut, uvalue = value;

  if( (port < 0) || (port >= MAXPORTS) )
  {
    printf("avme470Write: %s: port numbver out of range %d\n", name, port );
    return S_avme470_portError;
  }
  else if( (bit < 0) || (bit >= MAXBITS) )
  {
    printf("avme470Write: %s: bit number out of range %d\n", name, bit );
    return S_avme470_bitError;
  }
  else
  {
    plist = avme470FindCard( name );
    if( plist )
    {
      if( debug )
        printf("avme470Write: name = %s, port = %d, bit = %d, writeFlag = %d, value = 0x%lx\n",
                              name, port, bit, writeFlag, value);
      map_ptr = plist->brd_ptr;
      if( writeFlag == BIT )
      {
        if( value < 0 || value > 1 )
        {
          printf("avme470Write: %s: BIT value out of range = %ld\n", name, value);
          return S_avme470_writeError;
        }
        else
        {
          bpos  = 1 << bit;
          value = value << bit;
          avme470Output( (unsigned *)&map_ptr->port[port].b_select,
                         (int)((avme470Input((unsigned *)&map_ptr->port[port].b_select) & ~bpos) | value));
        }
      }
      else if( writeFlag == PORT )
      {
        if( value < 0 || value > 0xFF )
        {
          printf("avme470Write: %s: PORT value out of range = %ld\n", name, value);
          return S_avme470_writeError;
        }
        else
          avme470Output( (unsigned *)&map_ptr->port[port].b_select, (int)value);
      }
      else if( writeFlag == NIBBLE || writeFlag == WORD )
      {
        if( (writeFlag == NIBBLE) && (uvalue > 0xF) )
        {
          printf("avme470Write: %s: NIBBLE value out of range = %ld\n", name, value);
          return S_avme470_writeError;
        }
        else if( (writeFlag == WORD) && (uvalue > 0xFFFF) )
        {
          printf("avme470Write: %s: WORD value out of range = %ld\n", name, value);
          return S_avme470_writeError;
        }
        else
        {
          if ( nBits > 16 ) nBits = 16;
          if( writeFlag == NIBBLE  &&  nBits > 4 ) nBits = 4;
          zeroMask = (1<<nBits) - 1;

                                                /* get new data port aligned */
          zeroMask = zeroMask << bit;
          uvalue   = uvalue   << bit;

          for ( ; nBits > 0  &&  port < MAXPORTS; ) {

            /* Zero-out the bits we want to change */
            zeroOut = ~zeroMask;

            /* Current value AND zeroOut will zero-out the bits.   */
            /* Now OR this with the new bit pattern shifted by the */
            /* appropriate amount                                  */

            oldport = avme470Input((unsigned *)&map_ptr->port[port].b_select);
            newport = ( (oldport & zeroOut) | uvalue ) & 0xFF;

            if ( debug ) { 
              printf("avme470Write: port=%d, nBits=%d, zeroMask=0x%04lx, uvalue=0x%04lx\n",
                port, nBits, zeroMask, uvalue);
              printf("              oldport=0x%04x, newport=0x%04x\n", oldport, newport);
            } 

            if ( newport != oldport ) {
                                          /* Write new port value */
              avme470Output((unsigned *)&map_ptr->port[port].b_select, newport);
            }

            nBits    = nBits - (8-bit);
            uvalue   = uvalue >> (8-bit);
            zeroMask = zeroMask >> (8-bit);
            bit = 0;
            port++;
          }
        }
      }
      else
      {
        printf("avme470Write: %s: Data flag error (%d)\n", name, writeFlag );
        return S_avme470_dataFlagError;
      }
    }
    else
    {
      printf("avme470Write: Card %s not found\n", name);
      return S_avme470_cardNotFound;
    }
  }
  return(OK);
}
    
 
unsigned char avme470Input( unsigned int *addr ) 
{
/*  return((unsigned char) *((char *)addr)); */
  return (unsigned char) in_8((volatile unsigned char*)addr);
}


void avme470Output( unsigned int *addr, int b )
{
/*  *((char *)addr) = (char)b; */
	out_8((volatile unsigned char*)addr,(unsigned char)b);
}


void avme470COS( struct config470 *plist )
{
  unsigned char   saved_bank; /* saved bank value */
  unsigned char   i_stat;     /* interrupt status */
  unsigned char   p_mask;     /* port interrupt bit mask */
  unsigned char   i_pend;     /* interrupting bit */
  unsigned char   b_mask;     /* bit interrupt bit mask */
  unsigned char   mbit;       /* event control mask */
  int             i;          /* loop control over ports */
  int             j;          /* loop control over bits */
  int             cos_bit;    /* COS bit number 0-15 */
  int             state;      /* state of changed bit */

  /* disable interrupts for this carrier and slot */
  if (ipmIrqCmd(plist->card, plist->slot, 0, ipac_irqDisable) == S_IPAC_badAddress) {    
#ifdef NO_EPICS
      logMsg("avme2470COS: Error in card or slot number\n", 0, 0, 0, 0, 0, 0);
#else
      epicsInterruptContextMessage("avme2470COS: Error in card or slot number");
#endif
  }

  saved_bank = avme470SelectBank(BANK1, plist);  /* set & save bank select */
        
  for(i = 0; i < MAXPORTS; i++)
  {
    i_stat = avme470Input((unsigned int *)&plist->brd_ptr->port[6].b_select); /* interrupt status */
    p_mask = ( 1 << i);         /* form port interrupt bit mask */
    if((p_mask & i_stat) != 0)  /* port with interrupt pending? */
    {
      for(j = 0; j < MAXBITS; j++)
      {
        i_pend = avme470Input((unsigned int *)&plist->brd_ptr->port[i].b_select); /* interrupt sense */
	b_mask = ( 1 << j);         /* form bit interrupt mask */
	if((b_mask & i_pend) != 0)  /* bit in port with interrupt pending? */
        {  
          /* write 0 to clear the interrupting bit */
          avme470Output((unsigned int *)&plist->brd_ptr->port[i].b_select,(~b_mask));

#if DEBUG
          logMsg("avme470COS: Interrupt on port %d, bit %d\n", i, j, 0, 0, 0, 0);
#endif
          /*        
          At this time the interrupting port and bit is known.
          The port number (0-3) is in 'i' and the bit number (0-7) is in 'j'.

          The following code converts from port:bit format to change of state
          format bit number:state (0 thru 15:0 or 1).
          */

	  cos_bit = (i << 2) + j;      /* compute COS bit number */   
	  mbit    = (1 << (i << 1));   /* generate event control mask bit */
	  if(j > 3)                    /* correct for nibble encoding */
	  {
	    mbit    <<= 1;
	    cos_bit  -= 4;             /* correct COS bit number */
	  }
          if((plist->ev_control[0] & mbit) != 0)  /* state 0 or 1 */
            state = 1;
	  else
            state = 0;

          /* Save the change of state bit number and the state value */

          plist->last_chan  = cos_bit;  /* correct channel number */
          plist->last_state = state;    /* correct state for channel */

#ifndef NO_EPICS
          {
            int k;

            /* Make bi records process */

            if( plist->biScan[cos_bit] )
              scanIoRequest(plist->biScan[cos_bit]);

            /* Make all mbbi records which contain this bit process        */
            /* Remember, the mbbiScan is an array of IOSCANPVT's defined   */
            /* on the records first bit (LSB). This is why we go backwards */
            /* in the loop below.                                          */

            for( k=0; k<4; k++ )    /* 4 bits max in an mbbi */
            {
              if( cos_bit-k >= 0 && plist->mbbiScan[cos_bit-k] )
                scanIoRequest(plist->mbbiScan[cos_bit-k]);
            }

            /* Same for mbbiDirect records */

            for( k=0; k<16; k++ )   /* 16 bits max in an mbbiDirect */
            {
              if( cos_bit-k >= 0 && plist->mbbiDirectScan[cos_bit-k] )
                scanIoRequest(plist->mbbiDirectScan[cos_bit-k]);
            }
          }
#endif
          /* If the user has passed in a function, then call it now  */
          /* with the name of the board, the port number and the bit */
          /* number.                                                 */

          if( plist->usrFunc )
            (plist->usrFunc)( plist->pName, i, j );
        }
      }
      /* re-enable sense inputs */
      avme470Output((unsigned int *)&plist->brd_ptr->port[i].b_select, 0xFF);
    }
  }
  /* restore bank select */
  avme470SelectBank(saved_bank, plist);

  /* Clear and Enable Interrupt from Carrier Board Registers */
  if (ipmIrqCmd(plist->card, plist->slot, 0, ipac_irqEnable) == S_IPAC_badAddress) {    
#ifdef NO_EPICS
      logMsg("avme2470COS: Error in card or slot number\n", 0, 0, 0, 0, 0, 0);
#else
      epicsInterruptContextMessage("avme2470COS: Error in card or slot number");
#endif
  }
}

void (*tillcb)(struct config470 *plist)=0;

void avme470COSTest( struct config470 *plist )
{

  /* disable interrupts for this carrier and slot */
  if (ipmIrqCmd(plist->card, plist->slot, 0, ipac_irqDisable) == S_IPAC_badAddress) {    
#ifdef NO_EPICS
      logMsg("avme2470COS: Error in card or slot number\n", 0, 0, 0, 0, 0, 0);
#else
      epicsInterruptContextMessage("avme2470COS: Error in card or slot number");
#endif
  }

  if ( tillcb )
	tillcb(plist);
  else
    avme470Output((unsigned int*)&plist->brd_ptr->ier, 0);/* disable interrupt */

  /* Clear and Enable Interrupt from Carrier Board Registers */
  if (ipmIrqCmd(plist->card, plist->slot, 0, ipac_irqEnable) == S_IPAC_badAddress) {    
#ifdef NO_EPICS
      logMsg("avme2470COS: Error in card or slot number\n", 0, 0, 0, 0, 0, 0);
#else
      epicsInterruptContextMessage("avme2470COS: Error in card or slot number");
#endif
  }
}


void setLed(unsigned char val) {
typedef struct{unsigned char led[8][8];} LED;
LED *led = (void*)0xFFEFFE80;
int i,j;
 for (i=0;i<8;i++) {
  led->led[i][0] = val; j=val; val = val >> 1;
 }
 for (i=0;i<2000000;i++) led->led[7][0]=j; 
}

void dumpLed(unsigned int val) {
unsigned int pl;
pl = val;
for (;;){
 setLed((unsigned char)(pl>>24)&0xFF);
 setLed((unsigned char)(pl>>16)&0xFF);
 setLed((unsigned char)(pl>> 8)&0xFF);
 setLed((unsigned char)(pl    )&0xFF);
 setLed((unsigned char)0xff);
 setLed((unsigned char)0xff);
}
}

void avme470LEVEL( struct config470 *plist )
{
  unsigned char   saved_bank;  /* saved bank value */
  unsigned char   i_stat;      /* interrupt status */
  unsigned char   p_mask;      /* port interrupt bit mask */
  unsigned char   i_pend;      /* interrupting bit */
  unsigned char   b_mask;      /* bit interrupt bit mask */
  unsigned char   mbit;        /* event control mask */
  int             i;           /* loop control over ports */
  int             j;           /* loop control over bits  */
  int             lev_bit;     /* LEV bit number 0-23 */
  int             state;       /* state of changed bit */

  /* disable interrupts for this carrier and slot */
  if (ipmIrqCmd(plist->card, plist->slot, 0, ipac_irqDisable) == S_IPAC_badAddress) {    
#ifdef NO_EPICS
      logMsg("avme2470COS: Error in card or slot number\n", 0, 0, 0, 0, 0, 0);
#else
      epicsInterruptContextMessage("avme2470COS: Error in card or slot number");
#endif
  }

  avme470Output((unsigned int*)&plist->brd_ptr->ier, 0);/* disable interrupt */

  saved_bank = avme470SelectBank(BANK1, plist); /* set & save bank select */

/*  i_stat = 0x3F; */	/* until we can read port6, assume all ports! */

  i_stat = avme470Input((unsigned int*)&plist->brd_ptr->port[6].b_select); 
/* interrupt status */
/*  dumpLed((unsigned int)i_stat); */

  for(i=0; i<MAXPORTS; i++)
  {

    p_mask = ( 1 << i );         /* form port interrupt bit mask */

    if((p_mask & i_stat) != 0)  /* port with interrupt pending? */
    {
      i_pend = avme470Input((unsigned int *)&plist->brd_ptr->port[i].b_select); /* interrupt sense */

      for(j = 0; j < MAXBITS; j++)
      {
	b_mask = ( 1 << j );        /* form bit interrupt mask */
	if((b_mask & i_pend) != 0)  /* bit in port with interrupt pending? */
	{
          /* write 0 to clear the interrupting bit */
          avme470Output((unsigned int *)&plist->brd_ptr->port[i].b_select,(~b_mask));

#if DEBUG
          logMsg("avme470LEVEL: Interrupt on port %d, bit %d\n", i, j, 0, 0, 0, 0);
#endif

          /*        
          At this time the interrupting port and bit is known.
          The port number (0-(MAXPORTS-1)) is in 'i' 
          and the bit number (0-(MAXBITS-1)) is in 'j'.

          The following code converts from port:bit format to level
          format bit number:state (0 thru (MAXPORTS*MAXBITS-1):0 or 1).
          */

          lev_bit = i*MAXBITS + j;    /* compute bit number */   
	  mbit    = (1 << (i << 1));  /* generate event control mask bit */
	  if(j > 3)                   /* correct for nibble encoding */
            mbit <<= 1;

		/* the following still needs to be extended for 48 bits! */
          if((plist->ev_control[0] & mbit) != 0)  /* state 0 or 1 */
            state = 1;
	  else
            state = 0;

          /* Save the bit number and the state value */

          plist->last_chan  = lev_bit;  /* correct channel number */
          plist->last_state = state;    /* correct state for channel */

#ifndef NO_EPICS
          {
            int k;

            /* Make bi records process */

            if( plist->biScan[lev_bit] )
              scanIoRequest(plist->biScan[lev_bit]);

            /* Make all mbbi records which contain this bit process        */
            /* Remember, the mbbiScan is an array of IOSCANPVT's defined   */
            /* on the records first bit (LSB). This is why we go backwards */
            /* in the loop below.                                          */

            for( k=0; k<4; k++ )    /* 4 bits max in an mbbi */
            {
              if( lev_bit-k >= 0 && plist->mbbiScan[lev_bit-k] )
                scanIoRequest(plist->mbbiScan[lev_bit-k]);
            }

            /* Same for mbbiDirect records */

            for( k=0; k<16; k++ )   /* 16 bits max in an mbbiDirect */
            {
              if( lev_bit-k >= 0 && plist->mbbiDirectScan[lev_bit-k] )
                scanIoRequest(plist->mbbiDirectScan[lev_bit-k]);
            }
          }
#endif
          /* If the user has passed in a function, then call it now  */
          /* with the name of the board, the port number and the bit */
          /* number.                                                 */

          if( plist->usrFunc )
            (plist->usrFunc)( plist->pName, i, j );
	}
      }
      /* re-enable sense inputs */
      avme470Output((unsigned int *)&plist->brd_ptr->port[i].b_select, 0xFF);
    }
  }

  /* restore bank select */
  avme470SelectBank(saved_bank, plist);

  /* Clear and Enable Interrupt from Carrier Board Registers */
  if (ipmIrqCmd(plist->card, plist->slot, 0, ipac_irqEnable) == S_IPAC_badAddress) {    
#ifdef NO_EPICS
      logMsg("avme2470COS: Error in card or slot number\n", 0, 0, 0, 0, 0, 0);
#else
      epicsInterruptContextMessage("avme2470COS: Error in card or slot number");
#endif
  }
  avme470Output((unsigned int *)&plist->brd_ptr->ier, INTEN); /* enable interrupts again */
}


void avme470WhichHandler( char *name, unsigned char *handler )
{
  struct config470 *plist;

  plist = avme470FindCard( name );
  if( plist )
    *handler = plist->intHandler;
}


void *avme470FindCard( char *name )
{
  struct config470 *plist;
  int               foundCard = 0;

  plist = ptrAvme470First;
  while( plist )
  {
    if( !strcmp(plist->pName, name) )
    {
      foundCard = 1;
      break;
    }
    plist = plist->pnext;
  }

  if( !foundCard )
    return NULL;
  else
    return plist;
}


/*******************************************************************************
* EPICS iocsh Command registry
*/

#ifndef NO_EPICS

/* avme470Report(int interest) */
static const iocshArg avme470ReportArg0 = {"interest", iocshArgInt};
static const iocshArg * const avme470ReportArgs[1] = {&avme470ReportArg0};
static const iocshFuncDef avme470ReportFuncDef =
    {"avme470Report",1,avme470ReportArgs};
static void avme470ReportCallFunc(const iocshArgBuf *args)
{
    avme470Report(args[0].ival);
}

/* avme470Create( char *pName, unsigned short card, unsigned short slot,
                  char *modeName,
                  char *intHandlerName, char *usrFunc, short vector, 
                  short event, short debounce ) */
static const iocshArg avme470CreateArg0 = {"pName",iocshArgPersistentString};
static const iocshArg avme470CreateArg1 = {"card", iocshArgInt};
static const iocshArg avme470CreateArg2 = {"slot", iocshArgInt};
static const iocshArg avme470CreateArg3 = {"modeName",iocshArgString};
static const iocshArg avme470CreateArg4 = {"intHandlerName",iocshArgString};
static const iocshArg avme470CreateArg5 = {"usrFunc",iocshArgString};
static const iocshArg avme470CreateArg6 = {"vector", iocshArgInt};
static const iocshArg avme470CreateArg7 = {"event", iocshArgInt};
static const iocshArg avme470CreateArg8 = {"debounce", iocshArgInt};
static const iocshArg * const avme470CreateArgs[9] = {
    &avme470CreateArg0, &avme470CreateArg1, &avme470CreateArg2, &avme470CreateArg3,
    &avme470CreateArg4, &avme470CreateArg5, &avme470CreateArg6, &avme470CreateArg7,
    &avme470CreateArg8};
static const iocshFuncDef avme470CreateFuncDef =
    {"avme470Create",9,avme470CreateArgs};
static void avme470CreateCallFunc(const iocshArgBuf *arg)
{
    avme470Create(arg[0].sval, arg[1].ival, arg[2].ival, arg[3].sval,
                  arg[4].sval, arg[5].sval, arg[6].ival, arg[7].ival,
                  arg[8].ival);
}

LOCAL void drvAvme470Registrar(void) {
    iocshRegister(&avme470ReportFuncDef,avme470ReportCallFunc);
    iocshRegister(&avme470CreateFuncDef,avme470CreateCallFunc);
}
epicsExportRegistrar(drvAvme470Registrar);

#endif
