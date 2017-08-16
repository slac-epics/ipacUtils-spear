/*******************************************************************************

Project:
    Gemini Multi-Conjugate Adaptive Optics Project

File:
    drvXy5320.c

Description:
    EPICS Driver for the XIP-5320-000 Industrial I/O Pack
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

#include <vxWorks.h>
#include <sysLib.h>
#include <taskLib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "drvIpac.h"
#include "drvXy5320.h"

#define DEBUG 0

/* These are the IPAC IDs for this module */
#define IP_MANUFACTURER_XYCOM 0xa3
#define IP_MODEL_XYCOM_5320   0x32

LOCAL struct config5320 *ptrXy5320First = NULL;

#define XY5320_CAL_NAME   "xy5320Cal"
#define XY5320_CAL_PRI    170
#define XY5320_CAL_OPT    VX_FP_TASK
#define XY5320_CAL_STACK  8192

#define XY5320_READ_NAME  "xy5320Read"
#define XY5320_READ_PRI   170
#define XY5320_READ_OPT   VX_FP_TASK
#define XY5320_READ_STACK 8192

#define READ_TRIGGER     0xFFFF


#ifndef NO_EPICS
#include <drvSup.h>
#include <dbScan.h>
#include <taskwd.h>
#include <task_params.h>

/* EPICS Driver Support Entry Table */

struct drvet drvXy5320 = {
  2,
  (DRVSUPFUN) xy5320Report,
  (DRVSUPFUN) xy5320Initialise
};
#endif


int xy5320Report( int interest )
{
  int               i;
  struct map5320    *map_ptr;
  struct config5320 *plist;

  plist = ptrXy5320First;
  while( plist )
  {
    map_ptr = plist->brd_ptr;
    printf("\nBoard Status Information: %s\n\n", plist->pName);
    printf("Board Control Register: %04x\n", map_ptr->cntl_reg);
    printf("Identification:         ");
    for(i = 0; i < 4; i++)                    /* identification */
      printf("%c", map_ptr->id_map[i].prom);

    printf("\nManufacturer's I.D.:    %x", map_ptr->id_map[4].prom);
    printf("\nIP Model Number:        %x", map_ptr->id_map[5].prom);
    printf("\nRevision:               %x", map_ptr->id_map[6].prom);
    printf("\nReserved:               %x", map_ptr->id_map[7].prom);
    printf("\nDriver I.D. (low):      %x", map_ptr->id_map[8].prom);
    printf("\nDriver I.D. (high):     %x", map_ptr->id_map[9].prom);
    printf("\nTotal I.D. Bytes:       %x", map_ptr->id_map[10].prom);
    printf("\nCRC:                    %x", map_ptr->id_map[11].prom);
    printf("\n\n");
    for(i=0; i<plist->numChannels; i++)
      printf("Chan = %2d: raw = 0x%x, auto-zero = 0x%x, cal = 0x%x, corrected = 0x%lx, analog = %+f\n",
              plist->s_array[i].chan, plist->raw_data[i], plist->az_data[i], plist->cal_data[i], 
              plist->cor_data[i], plist->analogData[i] );
    printf("\n");

    plist = plist->pnext;
  }
  return(OK);
}


int xy5320Initialise( void )
{
  int tid;
  int status;

  if( ptrXy5320First && ptrXy5320First->startTasks )
  {
    /* Is this task already running? */
    if( (tid = taskNameToId(XY5320_CAL_NAME)) != ERROR )
    {
#ifndef NO_EPICS
      taskwdRemove(tid);
#endif
      taskDelete(tid);
    }

    /* Start off the task which periodically (20 minutes) calibrates the board */
    status = taskSpawn( XY5320_CAL_NAME, XY5320_CAL_PRI, XY5320_CAL_OPT, XY5320_CAL_STACK,
                        xy5320CalTask, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
    if( status == ERROR )
    {
      printf("xy5320Initialise: Failed to create calibration task\n");
      return S_xy5320_taskCreate;
    }
#ifndef NO_EPICS
    else
      taskwdInsert(status,NULL,NULL);
#endif

    /* Is this task already running? */
    if( (tid = taskNameToId(XY5320_READ_NAME)) != ERROR )
    {
#ifndef NO_EPICS
      taskwdRemove(tid);
#endif
      taskDelete(tid);
    }

    /* Start off the task which reads the inputs */
    status = taskSpawn( XY5320_READ_NAME, XY5320_READ_PRI, XY5320_READ_OPT, 
                        XY5320_READ_STACK, xy5320ReadTask, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
    if( status == ERROR )
    {
      printf("xy5320Initialise: Failed to create read task\n");
      return S_xy5320_taskCreate;
    }
#ifndef NO_EPICS
    else
      taskwdInsert(status,NULL,NULL);
#endif
  }
  return(OK);
}


int xy5320CalTask()
{
  struct config5320 *plist;
  unsigned short    temp;

#ifndef NO_EPICS
  for(;;)
  {
    if( interruptAccept )     /* Wait for iocInit to set this true */
      break;
    taskDelay( sysClkRateGet()/20 );   /* 20 Hz */
  }
#endif

  for(;;)
  {
    plist = ptrXy5320First;
    while( plist )
    {
      semTake(plist->semId, WAIT_FOREVER);
      temp        = plist->mode;    /* Remember old mode */
      plist->mode = AZV;
      xy5320ReadInputs( plist );
      plist->mode = CAL;
      xy5320ReadInputs( plist );
      plist->cal  = 1;
      plist->mode = temp;
      semGive(plist->semId);
      plist = plist->pnext;
    }
    taskDelay( sysClkRateGet() * 60 * 20 );   /* Every 20 minutes */
  }
  return(OK);
}


int xy5320ReadTask()
{
  struct config5320 *plist;

#ifndef NO_EPICS
  for(;;)
  {
    if( interruptAccept )     /* Wait for iocInit to set this true */
      break;
    taskDelay( sysClkRateGet()/20 );   /* 20 Hz */
  }
#endif

  for(;;)
  {
    plist = ptrXy5320First;
    while( plist )
    {
      semTake(plist->semId, WAIT_FOREVER);
      if( plist->cal )                     /* Only read if the board is calibrated */
      {
        xy5320ReadInputs(plist);
        xy5320CorrectInputs(plist);        /* Correct the inputs based on calibration */
#if DEBUG
        printf("\n");
#endif
      }
      semGive(plist->semId);
      plist = plist->pnext;
    }
#if DEBUG
    taskDelay( sysClkRateGet() );     /* 1 Hz */
#else
    taskDelay( sysClkRateGet()/20 );  /* 20 Hz */
#endif
  }
  return(OK);
}


int xy5320Create( char *pName, ushort_t card, ushort_t slot, char *voltRangeName,
                  char *modeName, int numSamples, char *filename )
{
  struct config5320 *plist;
  struct config5320 *pconfig;
  int               status;
  int               voltRange;
  int               mode;

  if( !strcmp(voltRangeName, "-5TO5") )
    voltRange = 1;
  else if( !strcmp(voltRangeName, "-10TO10") )
    voltRange = 2;
  else if( !strcmp(voltRangeName, "0TO10") )
    voltRange = 3;
  else
  {
    printf("xy5320Create: Voltage Range Error (%s)\n", voltRangeName);
    return S_xy5320_voltRangeError;
  }

  if( !strcmp(modeName, "DIF") )
    mode = DIF;
  else if( !strcmp(modeName, "SE") )
    mode = SE;
  else
  {
    printf("xy5320Create: Mode Name Error (%s)\n", modeName);
    return S_xy5320_modeError;
  }

  if( numSamples < 1 )
  {
    numSamples = 1;
    printf("xy5320Create: Setting number of samples to 1\n");
  }
  else if( numSamples > MAX_SAMPLES )
  {
    numSamples = MAX_SAMPLES;
    printf("xy5320Create: Setting number of samples to the maximum permitted: %d\n", numSamples);
  }

  status = ipmValidate(card, slot, IP_MANUFACTURER_XYCOM, IP_MODEL_XYCOM_5320);
  if( status )
  {
    printf("xy5320Create: Error %d from ipmValidate\n", status);
    return S_xy5320_notValidated;
  }

  if( !ptrXy5320First )
  {
    ptrXy5320First = malloc(sizeof(struct config5320));
    if( !ptrXy5320First )
    {
      printf("xy5320Create: First malloc failed\n");
      return S_xy5320_mallocFailed;
    }

    status = xy5320SetConfig( pName, card, slot, ptrXy5320First, voltRange, mode,
                              numSamples, filename );
    if( status )
      return status;
  }
  else
  {
    /* Check for unique card */

    plist = ptrXy5320First;
    while( TRUE )
    {
      if( !strcmp(plist->pName, pName) || ((plist->card == card) && (plist->slot == slot)) )
      {
        printf("xy5320Create: Duplicate device (%s, %d, %d)\n", pName, card, slot);
        return S_xy5320_duplicateDevice;
      }
      if( plist->pnext == NULL )
        break;
      else
        plist = plist->pnext;
    }

    /* plist now points to the last item in the list */

    pconfig = malloc(sizeof(struct config5320));
    if( pconfig == NULL )
    {
      printf("xy5320Create: malloc failed\n");
      return S_xy5320_mallocFailed;
    }

    /* pconfig is the configuration block for our new card */
    status = xy5320SetConfig( pName, card, slot, pconfig, voltRange, mode,
                              numSamples, filename );
    if( status )
      return status;

    /* Update linked-list */
    plist->pnext = pconfig;
  }
  return(OK);
}


long xy5320SetConfig( char *pName, ushort_t card, ushort_t slot, 
                      struct config5320 *pconfig, int voltRange, int mode,
                      int numSamples, char *filename )
{
  FILE  *fd;
  int   numRead;
  short cc;
  short gg;
  short maxChannels;
  char  buffer[MAX_LINE_LENGTH];
  char  num1[CHARS_PER_NUMBER];
  char  num2[CHARS_PER_NUMBER];
  int   status;
  int   i;
  int   p1;
  int   p2;
  int   p3;
  int   p4;

  status  = 0;
  numRead = 0;
  if( mode == DIF )
    maxChannels = MAX_DIF_CHANNELS;
  else
    maxChannels = MAX_SE_CHANNELS;

  fd = fopen( filename, "r");
  if( fd )
  {
    while( fgets( buffer, MAX_LINE_LENGTH, fd ) )
    {
      p1 = xy5320GetNonSpace(buffer, 0);
      if( p1 == -1 )
      {
        printf("Error! xy5320SetConfig: File \"%s\" has format error: \"%s\"\n", filename, buffer);
        status = S_xy5320_fileFormatError;
        break;
      }

      p2 = xy5320GetNonDigit(buffer, p1);
      if( p2 == -1 )
      {
        printf("Error! xy5320SetConfig: File \"%s\" has format error: \"%s\"\n", filename, buffer);
        status = S_xy5320_fileFormatError;
        break;
      }

      p3 = xy5320GetNonSpace(buffer, p2);
      if( p3 == -1 )
      {
        printf("Error! xy5320SetConfig: File \"%s\" has format error: \"%s\"\n", filename, buffer);
        status = S_xy5320_fileFormatError;
        break;
      }

      p4 = xy5320GetNonDigit(buffer, p3);
      if( p4 == -1 )
      {
        printf("Error! xy5320SetConfig: File \"%s\" has format error: \"%s\"\n", filename, buffer);
        status = S_xy5320_fileFormatError;
        break;
      }

      strncpy( num1, buffer+p1, p2-p1+1 );
      strncpy( num2, buffer+p3, p4-p3+1 );
      cc = atoi(num1);
      gg = atoi(num2);

      if( numRead > maxChannels-1 )
      {
        printf("Error! xy5320SetConfig: File \"%s\" too many channels: %d\n", filename, numRead);
        status = S_xy5320_tooManyChannels;
        break;
      }
      else if( (cc < 0) || (cc > maxChannels-1) )
      {
        printf("Error! xy5320SetConfig: File \"%s\" invalid channel: %d\n", filename, cc);
        status = S_xy5320_invalidChannel;
        break;
      }
      else if( (gg != GAIN_X1) && (gg != GAIN_X2) && (gg != GAIN_X4) && (gg != GAIN_X8) )
      {
        printf("Error! xy5320SetConfig: File \"%s\" invalid gain: %d\n", filename, gg);
        status = S_xy5320_invalidGain;
        break;
      }
      else
      {
        pconfig->s_array[numRead].chan = cc;  /* channel */
        pconfig->s_array[numRead].gain = gg;  /* gain    */
      }
      numRead++;
    }
    fclose(fd);

    if( !status )
    {
      pconfig->pnext        = NULL;
      pconfig->pName        = pName;
      pconfig->card         = card;
      pconfig->slot         = slot;
      pconfig->brd_ptr      = (struct map5320 *)ipmBaseAddr(card, slot, ipac_addrIO);
      pconfig->range        = voltRange;
      pconfig->trigger      = STRIG;            /* software triggering              */
      pconfig->mode         = mode;             /* How do we collect the inputs     */
      pconfig->average      = numSamples;       /* number of samples to average     */
      pconfig->data_mask    = BIT12;            /* A/D converter mask               */
      pconfig->bit_constant = CON12;            /* constant for correction equation */
      pconfig->numChannels  = numRead;          /* Number of channels in file       */
      pconfig->cal          = 0;                /* Board not calibrated             */
      if( !numRead )
      {
        printf("Error! xy5320SetConfig: File \"%s\" no channels defined\n", filename);
        status = S_xy5320_noChannels;
      }

      if( !status )
      {
        for( i=0; i<pconfig->numChannels; i++ )   /* Initialise data buffers */
        {
          pconfig->raw_data[i] = 0;  /* raw data           */
          pconfig->az_data[i]  = 0;  /* auto-zero data     */
          pconfig->cal_data[i] = 0;  /* calibration buffer */
          pconfig->cor_data[i] = 0;  /* corrected buffer   */
        }

        /* Create a semaphore to protect access to the board's inputs */
        /* when swapping between calibration and normal read mode     */

        pconfig->semId = semMCreate(SEM_Q_PRIORITY | SEM_DELETE_SAFE | SEM_INVERSION_SAFE);
        if( !pconfig->semId )
        {
          printf("Error! xy5320SetConfig: semMCreate failed\n");
          status = S_xy5320_semFailed;
        }
      }
    }
  }
  else
  {
    printf("Error! xy5320SetConfig: File open failed on \"%s\"\n", filename);
    status = S_xy5320_fileOpenFailed;
  }

  if( status )
  {
    pconfig->pnext      = NULL;
    pconfig->startTasks = 0;
  }
  else
    pconfig->startTasks = 1;

  return(status);
}


void xy5320ReadInputs( struct config5320 *pconfig )
{
  struct map5320 *map_ptr;     /* pointer to board memory map              */
  float          sum_data;     /* sum of all reads of a given channel      */
  unsigned short i;            /* loop control                             */
  unsigned short j;            /* channel index                            */
  unsigned short control_reg;  /* storage for control word of next channel */
  unsigned short *buff_ptr;    /* pointer to current position in buffer    */

  j                 = 0;
  map_ptr           = pconfig->brd_ptr;               /* initialize memory map pointer */
  map_ptr->cntl_reg = xy5320BuildControl(pconfig, j); /* control reg. for first channel in list */

  switch(pconfig->mode)
  {
    case DIF:    /* Differential inputs */
      buff_ptr = pconfig->raw_data;
#if DEBUG
      printf("DIF: Control Register for channel %d = 0x%x\n", pconfig->s_array[j].chan, map_ptr->cntl_reg);
#endif
      break;

    case SE:     /* Single-ended inputs */
      buff_ptr = pconfig->raw_data;
#if DEBUG
      printf("SE: Control Register for channel %d = 0x%x\n", pconfig->s_array[j].chan, map_ptr->cntl_reg);
#endif
      break;

    case AZV:    /* Auto-Zero */
      buff_ptr = pconfig->az_data;
#if DEBUG
      printf("AZV: Control Register for channel %d = 0x%x\n", pconfig->s_array[j].chan, map_ptr->cntl_reg);
#endif
      break;

    case CAL:    /* Calibration */
      buff_ptr = pconfig->cal_data;
#if DEBUG
      printf("CAL: Control Register for channel %d = 0x%x\n", pconfig->s_array[j].chan, map_ptr->cntl_reg);
#endif
      break;

    default:
      printf("xy5320ReadInputs: Mode must be DIF, SE, AZV or CAL\n");
      break;
  }

  while( j < pconfig->numChannels )
  {
    sum_data = 0.0;

    /* Build the control word for the next channel, while the */
    /* A/D is settling on the the current channel.            */

    if( j+1 < pconfig->numChannels )
    {
      control_reg = xy5320BuildControl(pconfig, j+1); /* control for next channel in list */
#if DEBUG
      switch(pconfig->mode)
      {
        case DIF:    /* Differential inputs */
          printf("DIF: Control Register for channel %d = 0x%x\n", pconfig->s_array[j+1].chan, control_reg);
          break;

        case SE:     /* Single-ended inputs */
          printf("SE: Control Register for channel %d = 0x%x\n", pconfig->s_array[j+1].chan, control_reg);
          break;

        case AZV:    /* Auto-Zero */
          printf("AZV: Control Register for channel %d = 0x%x\n", pconfig->s_array[j+1].chan, control_reg);
          break;

        case CAL:    /* Calibration */
          printf("CAL: Control Register for channel %d = 0x%x\n", pconfig->s_array[j+1].chan, control_reg);
          break;

        default:
          printf("xy5320ReadInputs: Mode must be DIF, SE, AZV or CAL\n");
          break;
      }
#endif
    }

    if((map_ptr->cntl_reg & CTRIG) != 0)              /* old data may be present */
      *buff_ptr = map_ptr->ai_reg;                    /* read data register to clear flag */

    for(i=0; i<pconfig->average; i++)
    {
      if(pconfig->trigger == ETRIG)                   /* check external trigger */
      {
        while((map_ptr->cntl_reg & CTRIG) == 0)       /* wait for trigger       */
          ;
      }
      else
        map_ptr->strt_reg = READ_TRIGGER;             /* trigger conversion     */

      /* Read and sum the data samples */

      sum_data += (float)(map_ptr->ai_reg & pconfig->data_mask);  /* read data register */
    }

    /* Average data */
    *buff_ptr++ = (unsigned short)(sum_data/(float)pconfig->average);

    map_ptr->cntl_reg = control_reg;                  /* set in new control register */
    j++;
  }
}


unsigned short xy5320BuildControl( struct config5320 *pconfig, int index )
{
  unsigned short control;  /* board control register     */
  unsigned char  cal_ch;   /* calibration channel number */

  control = 0;             /* initialize control register value */

  /* switch on range and then gain */
  switch(pconfig->range)
  {
    case RANGE_5TO5:
      switch(pconfig->s_array[index].gain)
      {
        case GAIN_X1:         /* gain = 1 */
          cal_ch = CAL0;      /* number of calibration channel */
#if DEBUG
          printf("RANGE_5TO5: gain = 1\n");
#endif
          break;

        case GAIN_X2:
          cal_ch   = CAL1;    /* number of calibration channel */
          control |= 0x0040;  /* OR in gain = 2 */
          break;

        case GAIN_X4:
          cal_ch   = CAL2;    /* number of calibration channel */
          control |= 0x0080;  /* OR in gain = 4 */
          break;

        case GAIN_X8:
          cal_ch   = CAL3;    /* number of calibration channel */
          control |= 0x00C0;  /* OR in gain = 8 */
          break;

        default:
          printf("xy5320BuildControl (-5,+5): Channel %d: Gain = %d not allowed, must be 1, 2, 4 or 8\n", pconfig->s_array[index].chan, pconfig->s_array[index].gain );
          break;
      }
      break;

    case RANGE_0TO10:
    case RANGE_10TO10:
      switch(pconfig->s_array[index].gain)
      {
        case GAIN_X1:         /* gain = 1                      */
          cal_ch = CAL0;      /* number of calibration channel */
#if DEBUG
          printf("RANGE_0TO10, RANGE_10TO10: gain = 1\n");
#endif
          break;

        case GAIN_X2:
          cal_ch = CAL0;      /* number of calibration channel */
          control |= 0x0040;  /* OR in gain = 2 */
          break;

        case GAIN_X4:
          cal_ch = CAL1;      /* number of calibration channel */
          control |= 0x0080;  /* OR in gain = 4 */
          break;

        case GAIN_X8:
          cal_ch   = CAL2;    /* number of calibration channel */
          control |= 0x00C0;  /* OR in gain = 8 */
          break;

        default:
          printf("xy5320BuildControl (0,+10), (-10,+10): Channel %d: Gain = %d not allowed, must be 1, 2, 4 or 8\n", pconfig->s_array[index].chan, pconfig->s_array[index].gain );
          break;
      }
      break;

    default:
      printf("xy5320BuildControl: Voltage range must be one of (-5,+5), (0,+10), (-10,+10)\n");
      break;
  }

  /* switch on mode */
  switch(pconfig->mode)
  {
    case DIF:                                             /* differential input mode          */
      control |= pconfig->s_array[index].chan;            /* OR in channel bits               */
      break;

    case SE:                                              /* single-ended input mode          */
      if( pconfig->s_array[index].chan < CAL0 )
      {
        control |= SEL_SELECT;                            /* OR in single-ended low mode bits */
        control |= pconfig->s_array[index].chan;          /* OR in channel bits               */
      }
      else
      {
        control |= SEH_SELECT;                            /* OR in single-ended high mode bits */
        control |= (pconfig->s_array[index].chan - CAL0); /* OR in high channel bits           */
      }
      break;

    case AZV:                                             /* auto-zero mode                    */
      if(pconfig->range == RANGE_0TO10)
        control |= CAL3;                                  /* select CAL3                       */
      else
      {
        control |= AZ_SELECT;                             /* OR in auto-zero bits              */
#if DEBUG
        printf("AZV mode: OR in auto-zero bits\n");
#endif
      }
      break;

    case CAL:                                             /* calibration mode                  */
      control |= cal_ch;                                  /* OR in calibration channel bits    */
      break;

    default:
      printf("xy5320BuildControl: Mode must be one of DIF, SE, AZV or CAL\n");
      break;
  }
#if DEBUG
  printf("xy5320BuildControl returns control = 0x%x\n", control);
#endif
  return(control);
}


void xy5320CorrectInputs( struct config5320 *pconfig )
{
  int   i;        /* Loop index */
  float i_span;   /* ideal span value */
  float i_zero;   /* ideal zero value */
  float calhi;    /* high calibration input voltage */
  float callo;    /* low calibration input voltage */
  float slope;    /* slope from equation 2 */
  float temp;

  i = 0;
  while( i < pconfig->numChannels )
  {
    /* Select calibration voltages and ideal zero and span values */

    callo = 0.0;
    switch(pconfig->range)
    {
      case RANGE_5TO5:
        switch( pconfig->s_array[i].gain )
        {
	  case GAIN_X1:
            calhi = 4.9000;
	    break;

	  case GAIN_X2:
            calhi = 2.4500;
	    break;

	  case GAIN_X4:
            calhi = 1.2250;
	    break;

	  case GAIN_X8:
            calhi = 0.6125;
	    break;

          default:
            printf("xy5320CorrectInputs (-5,+5): Channel %d: Gain = %d not allowed, must be 1, 2, 4 or 8\n", pconfig->s_array[i].chan, pconfig->s_array[i].gain );
            break;
	}
	i_zero = -5.0000;
	i_span = 10.0000;
        break;

      case RANGE_10TO10:
        switch( pconfig->s_array[i].gain )
        {
	  case GAIN_X1:
            calhi = 4.9000;
	    break;

	  case GAIN_X2:
            calhi = 4.9000;
  	    break;

	  case GAIN_X4:
            calhi = 2.4500;
	    break;

	  case GAIN_X8:
            calhi = 1.2250;
	    break;

          default:
            printf("xy5320CorrectInputs (-10,+10): Channel %d: Gain = %d not allowed, must be 1, 2, 4 or 8\n", pconfig->s_array[i].chan, pconfig->s_array[i].gain );
            break;
        }
        i_zero = -10.0000;
        i_span =  20.0000;
        break;

      case RANGE_0TO10:
        callo = 0.6125;
	switch( pconfig->s_array[i].gain )
    	{
	  case GAIN_X1:
            calhi = 4.9000;
	    break;

	  case GAIN_X2:
            calhi = 4.9000;
	    break;

	  case GAIN_X4:
            calhi = 2.4500;
	    break;

	  case GAIN_X8:
            calhi = 1.2250;
	    break;

          default:
            printf("xy5320CorrectInputs (0,+10): Channel %d: Gain = %d not allowed, must be 1, 2, 4 or 8\n", pconfig->s_array[i].chan, pconfig->s_array[i].gain );
            break;
	}
	i_zero =  0.0;
	i_span = 10.0000;
        break;

      default:
        printf("xy5320CorrectInputs: Range must be one of: (-5,+5), (-10,+10), (0,+10)\n");
        break;
    }

    slope = ((float)pconfig->s_array[i].gain) * (calhi - callo) / 
            (float)(pconfig->cal_data[i] - pconfig->az_data[i]);

    temp  = (((float)pconfig->bit_constant * slope) / i_span) * 
      ((float)pconfig->raw_data[i] + (((callo * (float)pconfig->s_array[i].gain) - i_zero) 
            / slope) - (float)pconfig->az_data[i]);

    pconfig->cor_data[i]   = (long)temp;  /* update corrected data buffer */

    /* This should be the value of the original analog source */
    pconfig->analogData[i] = (i_span * (temp/(float)pconfig->bit_constant)) + i_zero;

#if DEBUG
    if( i==0 || i==1 )
    {
      printf("xy5320CorrectInputs: (%d) gain = %f, calhi = %f, callo = %f, slope = %f, bit_constant = %ld, i_span = %f, i_zero = %f\n", pconfig->s_array[i].chan, (float)pconfig->s_array[i].gain, calhi, callo, slope, pconfig->bit_constant, i_span, i_zero);

      printf("xy5320CorrectInputs: (%d) az_data = %d, cal_data = %d, raw_data = %d, cor_data = %ld, analogData = %f\n", pconfig->s_array[i].chan, pconfig->az_data[i], pconfig->cal_data[i], pconfig->raw_data[i], pconfig->cor_data[i], pconfig->analogData[i]);
    }
#endif

    i++;
  }
}


long xy5320ReadChannel( char *name, int channel, unsigned long ftvl, void *prval )
{
  struct config5320 *plist;
  int               chanIndex;
  int               error;

  plist = xy5320FindCard( name );
  if( plist )
  {
    error = xy5320FindChannel( plist, channel, &chanIndex );
    if( error )
    {
      printf("xy5320ReadChannel: Channel %d (%s) not configured\n", channel, name); 
      return S_xy5320_invalidChannel;
    }
    else
    {
      semTake(plist->semId, WAIT_FOREVER);
      if( ftvl == TYPE_LONG )
        *(long *)prval = plist->cor_data[chanIndex];
      else if( ftvl == TYPE_DOUBLE )
        *(double *)prval = plist->analogData[chanIndex];
      else
      {
        printf("xy5320ReadChannel: Invalid field type %ld\n", ftvl);
        semGive(plist->semId);
        return S_xy5320_invalidFieldType;
      }
      semGive(plist->semId);
    }
  }
  else
  {
    printf("xy5320ReadChannel: Card %s not found\n", name);
    return S_xy5320_cardNotFound;
  }
  return(OK);
}


long xy5320ReadArray( char *name, int startIndex, int space, unsigned long ftvl, void *prval )
{
  struct config5320 *plist;
  int               i;
  int               numRead;
  int               numChan;

  plist = xy5320FindCard( name );
  if( plist )
  {
    if( (startIndex < 0) || (startIndex > plist->numChannels - 1) )
    {
      printf("xy5320ReadArray: Card %s, invalid channel index (%d)\n", name, startIndex);
      return S_xy5320_invalidChannelIndex;
    }
    else if( space <= 0 )
    {
      printf("xy5320ReadArray: Insufficient space for array values (%s)\n", name);
      return S_xy5320_noSpace;
    }
    else
    {
      numChan = (space - 1)/2;
      if( startIndex+numChan <= plist->numChannels )
        numRead = numChan;
      else
        numRead = plist->numChannels - startIndex;

      semTake(plist->semId, WAIT_FOREVER);
        
      if( ftvl == TYPE_LONG )
      {
        *((long *)prval) = numRead;
        for( i=0; i<numRead; i++ )
        {
          *((long *)prval+i+1)         = plist->s_array[startIndex+i].chan;
          *((long *)prval+i+1+numRead) = plist->cor_data[startIndex+i];
        }
      }
      else if( ftvl == TYPE_DOUBLE )
      {
        *((double *)prval) = numRead;
        for( i=0; i<numRead; i++ )
        {
          *((double *)prval+i+1)         = plist->s_array[startIndex+i].chan;
          *((double *)prval+i+1+numRead) = plist->analogData[startIndex+i];
        }
      }
      else
      {
        printf("xy5320ReadArray: Invalid field type %ld\n", ftvl);
        semGive(plist->semId);
        return S_xy5320_invalidFieldType;
      }
      semGive(plist->semId);
    }
  }
  else
  {
    printf("xy5320ReadArray: Card %s not found\n", name);
    return S_xy5320_cardNotFound;
  }
  return(OK);
}


int xy5320FindChannel( void *ptr, int channel, int *chanIndex )
{
  struct config5320 *plist;
  int               i;
  int               foundChannel;

  foundChannel = 0;
  plist        = (struct config5320 *)ptr;
  for( i=0; i<plist->numChannels; i++ )
  {
    if( plist->s_array[i].chan == channel )
    {
      *chanIndex   = i;
      foundChannel = 1;
      break;
    }
  }

  if( !foundChannel )
  {
    printf("xy5320FindChannel: Channel %d (\"%s\") not found\n", channel, plist->pName);
    return S_xy5320_invalidChannel;
  }

  return(OK);
}


int xy5320GetNumChan( void *ptr )
{
  struct config5320 *plist;

  plist = (struct config5320 *)ptr;
  return(plist->numChannels);
}


void *xy5320FindCard( char *name )
{
  struct config5320 *plist;
  int               foundCard = 0;

  plist = ptrXy5320First;
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


int xy5320GetNonSpace( char *buf, int start )
{
  int i;
  int ret;

  ret = -1;
  for(i=start; i<strlen(buf); i++)
  {
    if( !isspace(*(buf+i)) )
      break;
  }

  if( isdigit(*(buf+i)) )
    ret = i;
  else if( (*(buf+i) == '-') && isdigit(*(buf+i+1)) )
    ret = i;
  else if( (*(buf+i) == '+') && isdigit(*(buf+i+1)) )
    ret = i;

  return(ret);
}


int xy5320GetNonDigit( char *buf, int start )
{
  int i;
  int ret;

  ret = -1;
  for(i=start; i<strlen(buf); i++)
  {
    if( !isdigit(*(buf+i)) && (*(buf+i) != '-') && isdigit(*(buf+i-1)) )
      break;
    if( !isdigit(*(buf+i)) && (*(buf+i) != '+') && isdigit(*(buf+i-1)) )
      break;
  }

  if( isspace(*(buf+i)) )
    ret = i;

  return(ret);
}
