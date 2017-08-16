/*******************************************************************************

Project:
    Gemini Multi-Conjugate Adaptive Optics Project

File:
    devXy5320.c

Description:
    EPICS Device Support for the XIP-5320-000 Industrial I/O Pack
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

#include	<vxWorks.h>
#include	<stdlib.h>
#include	<stdio.h>
#include	<string.h>

#include	<alarm.h>
#include	<cvtTable.h>
#include	<dbDefs.h>
#include        <devLib.h>
#include	<dbAccess.h>
#include        <recSup.h>
#include	<devSup.h>
#include	<link.h>
#include	<aiRecord.h>
#include	<waveformRecord.h>
#include        "drvXy5320.h"
#include        "xipIo.h"

static long init_ai();
static long read_ai();
static long init_wf();
static long read_wf();


typedef struct {
	long		number;
	DEVSUPFUN	report;
	DEVSUPFUN	init;
	DEVSUPFUN	init_record;
        DEVSUPFUN       get_ioint_info;
	DEVSUPFUN	read_ai;
	DEVSUPFUN	special_linconv;
        } ANALOGDSET;

ANALOGDSET devAiXy5320 = { 6, NULL, NULL, init_ai, NULL, read_ai, NULL };
ANALOGDSET devWfXy5320 = { 6, NULL, NULL, init_wf, NULL, read_wf, NULL };

/* Support Function */
static void handleError( void *prec, int *status, int error, char *errString, int pactValue );


static long init_ai( struct aiRecord *pai )
{
  xipIo_t *pxip;
  int     status;
  int     chanIndex;
  double  value;
  void    *ptr;

  switch(pai->inp.type)
  {
    case(INST_IO):
      pxip = (xipIo_t *)malloc(sizeof(xipIo_t));
      if( !pxip )
      {
        handleError(pai, &status, S_dev_noMemory,
                    "devAiXy5320 (init_ai) malloc failed", TRUE);
      }
      else
      {
        /* Convert the address string into members of the xipIo structure */
        status = xipIoParse(pai->inp.value.instio.string, pxip, 'A');
        if( status )
        {
          handleError(pai, &status, S_xip_badAddress,
                      "devAiXy5320 (init_ai) XIP address string format error", TRUE);
        }
        else
        {
          ptr = xy5320FindCard(pxip->name);
          if( ptr )
          {
            status = xy5320FindChannel(ptr, pxip->channel, &chanIndex);
            if( status )
            {
              handleError(pai, &status, S_xy5320_invalidChannel,
                          "devAiXy5320 (init_ai) Channel not configured", TRUE);
            }
            else
            {
              pai->dpvt = pxip;
              status    = xy5320ReadChannel( pxip->name, pxip->channel, TYPE_DOUBLE, &value );
              if( status )
              {
                handleError(pai, &status, S_xy5320_readError,
                            "devAiXy5320 (init_ai) Read error", TRUE);
              }
              else
                pai->val = value;
            }
          }
          else
          {
            handleError(pai, &status, S_xy5320_cardNotFound,
                        "devAiXy5320 (init_ai) Card not found", TRUE);
          }
        }
      }
      break;

    default:
      handleError(pai, &status, S_db_badField,
                  "devAiXy5320 (init_ai) illegal INP field", TRUE);
      break;
  }
  return(status);
}


static long init_wf( struct waveformRecord *pwf )
{
  xipIo_t *pxip;
  int     status;
  int     maxChanIndex;
  int     fieldType;
  void    *ptr;

  switch(pwf->inp.type)
  {
    case(INST_IO):
      pxip = (xipIo_t *)malloc(sizeof(xipIo_t));
      if( !pxip )
      {
        handleError(pwf, &status, S_dev_noMemory,
                    "devWfXy5320 (init_wf) malloc failed", TRUE);
      }
      else
      {
        /* Convert the address string into members of the xipIo structure */
        status = xipIoParse(pwf->inp.value.instio.string, pxip, 'A');
        if( status )
        {
          handleError(pwf, &status, S_xip_badAddress,
                      "devWfXy5320 (init_wf) XIP address string format error", TRUE);
        }
        else
        {
          ptr = xy5320FindCard(pxip->name);
          if( ptr )
          {
            maxChanIndex = xy5320GetNumChan(ptr) - 1;
            if( (pxip->channel < 0) || (pxip->channel > maxChanIndex) )
            {
              handleError(pwf, &status, S_xy5320_invalidChannelIndex,
                          "devWfXy5320 (init_wf) Invalid channel index", TRUE);
            }
            else
            {
              pwf->dpvt = pxip;
              if( (pwf->ftvl != DBR_LONG) && (pwf->ftvl != DBR_DOUBLE) )
                handleError(pwf, &status, S_db_badField, "devWfXy5320 (init_wf) illegal ftvl", TRUE);
              else
              {
                if( pwf->ftvl == DBR_LONG )
                  fieldType = TYPE_LONG;
                else
                  fieldType = TYPE_DOUBLE;
                status = xy5320ReadArray( pxip->name, pxip->channel, pwf->nelm, fieldType, pwf->bptr );
                if( status )
                {
                  handleError(pwf, &status, S_xy5320_readError, 
                              "devWfXy5320 (init_wf) read error", TRUE);
                }
                else
                {
                  if( pwf->ftvl == DBR_LONG )
                    pwf->nord = 2 * (*(long *)pwf->bptr) + 1;
                  else
                    pwf->nord = 2 * (long)(*(double *)pwf->bptr) + 1;
                }
              }
            }
          }
          else
          {
            handleError(pwf, &status, S_xy5320_cardNotFound,
                        "devWfXy5320 (init_wf) Card not found", TRUE);
          }
        }
      }
      break;

    default:
      handleError(pwf, &status, S_db_badField,
                  "devWfXy5320 (init_wf) illegal INP field", TRUE);
      break;
  }
  return(status);
}


static long read_ai( struct aiRecord *pai )
{
  xipIo_t *pxip;
  double   value;
  int      status;
	
  pxip   = (xipIo_t *)pai->dpvt;
  status = xy5320ReadChannel( pxip->name, pxip->channel, TYPE_DOUBLE, &value );
  if( status )
  {
    handleError(pai, &status, S_xy5320_readError, "devAiXy5320 (read_ai) read error", FALSE);
    recGblSetSevr(pai,READ_ALARM,INVALID_ALARM);
  }
  else
    pai->val = value;

  return(DO_NOT_CONVERT);
}


static long read_wf( struct waveformRecord *pwf )
{
  xipIo_t *pxip;
  int      status;
  int      fieldType;

  pxip = (xipIo_t *)pwf->dpvt;
  if( (pwf->ftvl != DBR_LONG) && (pwf->ftvl != DBR_DOUBLE) )
  {
    handleError(pwf, &status, S_db_badField, "devWfXy5320 (read_wf) illegal ftvl", FALSE);
    recGblSetSevr(pwf,READ_ALARM,INVALID_ALARM);
  }
  else
  {
    if( pwf->ftvl == DBR_LONG )
      fieldType = TYPE_LONG;
    else
      fieldType = TYPE_DOUBLE;
    status = xy5320ReadArray( pxip->name, pxip->channel, pwf->nelm, fieldType, pwf->bptr );
    if( status )
    {
      handleError(pwf, &status, S_xy5320_readError, "devWfXy5320 (read_wf) read error", FALSE);
      recGblSetSevr(pwf,READ_ALARM,INVALID_ALARM);
    }
    else
    {
      if( pwf->ftvl == DBR_LONG )
        pwf->nord = 2 * (*(long *)pwf->bptr) + 1;
      else
        pwf->nord = 2 * (long)(*(double *)pwf->bptr) + 1;
    }
  }
  return(DO_NOT_CONVERT);
}


static void handleError( void *prec, int *status, int error, char *errString, int pactValue )
{
  struct dbCommon *pCommon;

  pCommon = (struct dbCommon *)prec;
  if( pactValue )
    pCommon->pact = TRUE;

  *status = error;
/*
  errlogPrintf("%s (%d): \"%s\"\n", pCommon->name, error, errString);
*/
  printf("%s (%d): \"%s\"\n", pCommon->name, error, errString);
}
