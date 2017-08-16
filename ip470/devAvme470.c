/*******************************************************************************

Project:
    Gemini Multi-Conjugate Adaptive Optics Project

File:
    devAvme470.c

Description:
    EPICS Device Support for the Acromag IP470-00x Industrial I/O Pack
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

#include	<stdlib.h>
#include	<stdio.h>
#include	<string.h>

#include	"alarm.h"
#include	"cvtTable.h"
#include	"dbDefs.h"
#include        "devLib.h"
#include	"dbAccess.h"
#include	"recGbl.h"
#include        "recSup.h"
#include	"devSup.h"
#include	"dbScan.h"
#include	"link.h"
#include	"biRecord.h"
#include	"boRecord.h"
#include	"mbbiRecord.h"
#include	"mbbiDirectRecord.h"
#include	"mbboRecord.h"
#include	"mbboDirectRecord.h"
#include	"drvAvme470.h"
#include	"xipIo.h"
#include        "epicsExport.h"

#define	DEBUG 0

static long init_bi();
static long bi_ioinfo();
static long read_bi();

static long init_bo();
static long write_bo();

static long init_mbbi();
static long mbbi_ioinfo();
static long read_mbbi();

static long init_mbbiDirect();
static long mbbiDirect_ioinfo();
static long read_mbbiDirect();

static long init_mbbo();
static long write_mbbo();

static long init_mbboDirect();
static long write_mbboDirect();

typedef struct {
	long		number;
	DEVSUPFUN	report;
	DEVSUPFUN	init;
	DEVSUPFUN	init_record;
	DEVSUPFUN	get_ioint_info;
	DEVSUPFUN	read_write;
	} BINARYDSET;


BINARYDSET devBiAvme470         = {6, NULL, NULL, init_bi,   bi_ioinfo,   read_bi};
epicsExportAddress(dset, devBiAvme470);
BINARYDSET devBoAvme470		= {6, NULL, NULL, init_bo,   NULL,        write_bo};
epicsExportAddress(dset, devBoAvme470);
BINARYDSET devMbbiAvme470       = {6, NULL, NULL, init_mbbi, mbbi_ioinfo, read_mbbi};
epicsExportAddress(dset, devMbbiAvme470);
BINARYDSET devMbbiDirectAvme470 = {6, NULL, NULL, init_mbbiDirect, mbbiDirect_ioinfo, 
                                  read_mbbiDirect};
epicsExportAddress(dset, devMbbiDirectAvme470);
BINARYDSET devMbboAvme470	= {6, NULL, NULL, init_mbbo, NULL,        write_mbbo};
epicsExportAddress(dset, devMbboAvme470);
BINARYDSET devMbboDirectAvme470	= {6, NULL, NULL, init_mbboDirect, NULL, write_mbboDirect};
epicsExportAddress(dset, devMbboDirectAvme470);

/* Support Function */
static void handleError( void *prec, int *status, int error, char *errString, int pactValue );


static long init_bi(struct biRecord *pbi)
{
  xipIo_t        *pxip;
  int            status;
  unsigned char  handler;
  unsigned short value;
  void           *ptr;

  switch(pbi->inp.type) 
  {
    case(INST_IO):
      pxip = (xipIo_t *)malloc(sizeof(xipIo_t));
      if( !pxip )
      {
        handleError(pbi, &status, S_dev_noMemory, 
                    "devBiAvme470 (init_bi) malloc failed", TRUE);
      }
      else
      {
        /* Convert the address string into members of the xipIo structure */
        status = xipIoParse(pbi->inp.value.instio.string, pxip, 'B');
        if( status )
        {
          handleError(pbi, &status, S_xip_badAddress, 
                      "devBiAvme470 (init_bi) XIP address string format error", TRUE);
        }
        else
        {
          ptr = avme470FindCard(pxip->name);
          if( ptr )
          {
            if( (pxip->port < 0) || (pxip->port >= MAXPORTS) )
            {
              handleError(pbi, &status, S_avme470_portError,
                          "devBiAvme470 (init_bi) port out of range", TRUE);
            }
            else if( (pxip->bit  < 0) || (pxip->bit  >= MAXBITS)  )
            {
              handleError(pbi, &status, S_avme470_bitError,
                          "devBiAvme470 (init_bi) bit out of range", TRUE);
            }
            else
            {
              /* Ask driver which interrupt handler do I have ? */
              avme470WhichHandler( pxip->name, &handler );
              pxip->intHandler = handler;
              pbi->dpvt        = pxip;
              status           = avme470Read(pxip->name, pxip->port, pxip->bit, BIT, &value, 0);
              if( status )
              {
                handleError(pbi, &status, S_avme470_readError,
                            "devBiAvme470 (init_bi) error from avme470Read", TRUE);
              }
              else
              {
                pbi->val  = value;
                pbi->rval = value;
              }
            }
          }
          else
          {
            handleError(pbi, &status, S_avme470_cardNotFound,
                        "devBiAvme470 (init_bi) Card not found", TRUE);
          }
        }
      }
      break;

    default:
      handleError(pbi, &status, S_db_badField,
                  "devBiAvme470 (init_bi) illegal INP field", TRUE);
      break;
  }
  return(status);
}


static long bi_ioinfo( int cmd, struct biRecord *pbi, IOSCANPVT	*ppvt )
{
  xipIo_t *pxip;
  int     status;
  int     recType = BI;

  pxip   = (xipIo_t *)pbi->dpvt;
  status = avme470GetIoScanpvt( pxip->name, pxip->port, pxip->bit, recType, 
                               pxip->intHandler, ppvt );
  if( status )
    handleError(pbi, &status, status, "devBiAVme470 (bi_ioinfo) error", FALSE);

  return(status);
}


static long read_bi(struct biRecord *pbi)
{
  xipIo_t        *pxip;
  int            status;
  unsigned short value;
	
  pxip   = (xipIo_t *)pbi->dpvt;
  status = avme470Read(pxip->name, pxip->port, pxip->bit, BIT, &value, 0);
  if( status )
  {
    handleError(pbi, &status, S_avme470_readError, "devBiAvme470 (read_bi) error", FALSE);
    recGblSetSevr(pbi,READ_ALARM,INVALID_ALARM);
    status = DO_NOT_CONVERT;
  }
  else
  {
    pbi->rval = value;
    status    = CONVERT;
  }
  return(status);
}


static long init_bo(struct boRecord *pbo)
{
  xipIo_t	 *pxip;
  int		 status;
  unsigned short value;
  void		 *ptr;

  switch(pbo->out.type)
  {
    case(INST_IO):
      pxip = (xipIo_t *)malloc(sizeof(xipIo_t));
      if( !pxip )
      {
        handleError(pbo, &status, S_dev_noMemory,
                    "devBoAvme470 (init_bo) malloc failed", TRUE);
      }
      else
      {
        /* Convert the address tring into members of the xipIo structure */
        status = xipIoParse(pbo->out.value.instio.string, pxip, 'B');
        if( status )
        {
          handleError(pbo, &status, S_xip_badAddress,
                      "devBoAvme470 (init_bo) XIP address string format error", TRUE);
        }
        else
        {
          ptr = avme470FindCard(pxip->name);
          if( ptr )
          {
            if( (pxip->port < 0) || (pxip->port >= MAXPORTS) )
            {
              handleError(pbo, &status, S_avme470_portError,
                          "devBoAvme470 (init_bo) port out of range", TRUE);
            }
            else if( (pxip->bit < 0) || (pxip->bit >= MAXBITS) )
            {
              handleError(pbo, &status, S_avme470_bitError,
                          "devBiAvme470 (init_bo) bit out of range", TRUE);
            }
            else
            {
              pbo->dpvt        = pxip;
              status           = avme470Read(pxip->name, pxip->port, pxip->bit, BIT, &value, 0);
              if( status )
              {
                handleError(pbo, &status, S_avme470_readError,
                            "devBoAvme470 (init_bo) error from avme470Read", TRUE);
              }
              else
              {
                pbo->val  = value;
                pbo->rval = value;
              }
            }
          }
          else
          {
            handleError(pbo, &status, S_avme470_cardNotFound,
                        "devBoAvme470 (init_bo) Card not found", TRUE);
          }
        }
      }
      break;

    default:
      handleError(pbo, &status, S_db_badField,
                  "devBoAvme470 (init_bo) illegal INP field", TRUE);
      break;
  }
  return(status);
}


static long write_bo(struct boRecord *pbo)
{
  xipIo_t	 *pxip;
  int	  	 status;
  unsigned short value;
  int		 debug = DEBUG;

  pxip   = (xipIo_t *)pbo->dpvt;
  status = avme470Write( pxip->name, pxip->port, pxip->bit, BIT, pbo->rval, 1, debug );
  if( status )
  {
    handleError(pbo, &status, S_avme470_writeError, "devBoAvme470 (write_bo) error", FALSE);
    recGblSetSevr(pbo,WRITE_ALARM,INVALID_ALARM);
  }
  else
  {
    status = avme470Read( pxip->name, pxip->port, pxip->bit, BIT, &value, debug );
    if( status )
    {
      handleError(pbo, &status, S_avme470_readError, "devBoAvme470 (write_bo) error", FALSE);
      recGblSetSevr(pbo,READ_ALARM,INVALID_ALARM);
    }
    else
      pbo->rbv = value;
  }
  return(status);
}


static long init_mbbo(struct mbboRecord *pmbbo)
{
  unsigned short value;
  xipIo_t        *pxip;
  int            status;
  int            debug = DEBUG;
  void           *ptr;

  switch(pmbbo->out.type)
  {
    case(INST_IO):
      pxip = (xipIo_t *)malloc(sizeof(xipIo_t));
      if( !pxip )
      {
        handleError(pmbbo, &status, S_dev_noMemory,
                    "devMbboAvme470 (init_mbbo) malloc failed", TRUE);
      }
      else
      {
        /* Convert the address string into members of the xipIo structure */
        status = xipIoParse(pmbbo->out.value.instio.string, pxip, 'B');
        if( status )
        {
          handleError(pmbbo, &status, S_xip_badAddress,
                      "devMbboAvme470 (init_mbbo) XIP address string format error", TRUE);
        }
        else
        {
          ptr = avme470FindCard(pxip->name);
          if( ptr )
          {
            if( (pxip->port < 0) || (pxip->port >= MAXPORTS) )
            {
              handleError(pmbbo, &status, S_avme470_portError,
                          "devMbboAvme470 (init_mbbo) port out of range", TRUE);
            }
            else if( (pxip->bit  < 0) || (pxip->bit  >= MAXBITS)  )
            {
              handleError(pmbbo, &status, S_avme470_bitError,
                          "devMbboAvme470 (init_mbbo) bit out of range", TRUE);
            }
            else
            {
              pmbbo->dpvt = pxip;
              status      = avme470Read( pxip->name, pxip->port, pxip->bit, NIBBLE, &value, debug );
              if( status )
              {
                handleError(pmbbo, &status, S_avme470_readError,
                            "devMbboAvme470 (init_mbbo) error from avme470Read", TRUE);
              }
              else
              {
                pmbbo->rbv  = value & pmbbo->mask;
                pmbbo->rval = value & pmbbo->mask;
              }
            }
          }
          else
          {
            handleError(pmbbo, &status, S_avme470_cardNotFound,
                        "devMbboAvme470 (init_mbbo) Card not found", TRUE);
          }
        }
      }
      break;

    default:
      handleError(pmbbo, &status, S_db_badField,
                  "devMbboAvme470 (init_mbbo) illegal OUT field", TRUE);
      break;
  }
  return(status);

}


static long write_mbbo(struct mbboRecord *pmbbo)
{
  xipIo_t       *pxip;
  int            status;
  unsigned short value;
  int            debug = DEBUG;

  pxip   = (xipIo_t *)pmbbo->dpvt;
  status = avme470Write( pxip->name, pxip->port, pxip->bit, NIBBLE, (pmbbo->rval
& pmbbo->mask), 4, debug );
  if( status )
  {
    handleError(pmbbo, &status, S_avme470_writeError, "devMbboAvme470 (write_mbbo) error", FALSE);
    recGblSetSevr(pmbbo,WRITE_ALARM,INVALID_ALARM);
  }
  else
  {
    status = avme470Read( pxip->name, pxip->port, pxip->bit, NIBBLE, &value, debug );
    if( status )
    {
      handleError(pmbbo, &status, S_avme470_readError,
                  "devMbboAvme470 (write_mbbo) error", FALSE);
      recGblSetSevr(pmbbo,READ_ALARM,INVALID_ALARM);
    }
    else
      pmbbo->rbv = value;
  }
  return(status);
}


static long init_mbboDirect(struct mbboDirectRecord *pmbboDirect)
{
  unsigned short value;
  xipIo_t       *pxip;
  int            status;
  int            debug = DEBUG;
  void          *ptr;

  switch(pmbboDirect->out.type)
  {
    case(INST_IO):
      pxip = (xipIo_t *)malloc(sizeof(xipIo_t));
      if( !pxip )
      {
        handleError(pmbboDirect, &status, S_dev_noMemory,
                    "devMbboDirectAvme470 (init_mbboDirect) malloc failed", TRUE);
      }
      else
      {
        /* Convert the address string into members of the xipIo structure */
        status = xipIoParse(pmbboDirect->out.value.instio.string, pxip, 'B');
        if( status )
        {
          handleError(pmbboDirect, &status, S_xip_badAddress,
               "devMbboDirectAvme470 (init_mbboDirect) XIP address string format error", TRUE);
        }
        else
        {
          ptr = avme470FindCard(pxip->name);
          if( ptr )
          {
            if( (pxip->port < 0) || (pxip->port >= MAXPORTS) )
            {
              handleError(pmbboDirect, &status, S_avme470_portError,
                          "devMbboDirectAVme470 (init_mbboDirect) port out of range", TRUE);
            }
            else if( (pxip->bit  < 0) || (pxip->bit  >= MAXBITS)  )
            {
              handleError(pmbboDirect, &status, S_avme470_bitError,
                          "devMbboDirectAvme470 (init_mbboDirect) bit out of range", TRUE);
            }
            else
            {
              pmbboDirect->dpvt = pxip;
              status            = avme470Read( pxip->name, pxip->port, pxip->bit, WORD, &value, debug );
              if( status )
              {
                handleError(pmbboDirect, &status, S_avme470_readError,
                            "devMbboDirectAvme470 (init_mbboDirect) error from avme470Read", TRUE);
              }
              else
              {
                pmbboDirect->rbv  = value & pmbboDirect->mask;
                pmbboDirect->rval = value & pmbboDirect->mask;

                /* ajf - This is a kludge BUT if this is not done       */
                /* the value entered the first time in SUPERVISORY mode */
                /* is overwritten with the entries in the B0-BF fields  */
                pmbboDirect->sevr = NO_ALARM;
              }
            }
          }
          else
          {
            handleError(pmbboDirect, &status, S_avme470_cardNotFound,
                        "devMbboDirectAvme470 (init_mbboDirect) Card not found", TRUE);
          }
        }
      }
      break;

    default:
      handleError(pmbboDirect, &status, S_db_badField,
                  "devMbboDirectAvme470 (init_mbboDirect) illegal OUT field", TRUE);
      break;
  }
  return(status);
}


static long write_mbboDirect(struct mbboDirectRecord *pmbboDirect)
{
  xipIo_t        *pxip;
  int             status;
  unsigned short  value;
  int             debug = DEBUG;

  pxip   = (xipIo_t *)pmbboDirect->dpvt;
  status = avme470Write( pxip->name, pxip->port, pxip->bit, WORD,
                        (pmbboDirect->rval & pmbboDirect->mask), 
                        pmbboDirect->nobt, debug );
  if( status )
  {
    handleError(pmbboDirect, &status, S_avme470_writeError,
                "devMbboDirectAvme470 (write_mbboDirect) error", FALSE);
    recGblSetSevr(pmbboDirect,WRITE_ALARM,INVALID_ALARM);
  }
  else
  {
    status = avme470Read( pxip->name, pxip->port, pxip->bit, WORD, &value, debug );
    if( status )
    {
      handleError(pmbboDirect, &status, S_avme470_readError,
                  "devMbboDirectAvme470 (write_mbboDirect) error", FALSE);
      recGblSetSevr(pmbboDirect,READ_ALARM,INVALID_ALARM);
    }
    else
      pmbboDirect->rbv = value;
  }
  return(status);
}


static long init_mbbi(struct mbbiRecord *pmbbi)
{
  xipIo_t        *pxip;
  int            status;
  unsigned char  handler;
  unsigned short value;
  void           *ptr;

  switch(pmbbi->inp.type) 
  {
    case(INST_IO):
      pxip = (xipIo_t *)malloc(sizeof(xipIo_t));
      if( !pxip )
      {
        handleError(pmbbi, &status, S_dev_noMemory,
                    "devMbbiAvme470 (init_mbbi) malloc failed", TRUE);
      }
      else
      {
        /* Convert the address string into members of the xipIo structure */
        status = xipIoParse(pmbbi->inp.value.instio.string, pxip, 'B');
        if( status )
        {
          handleError(pmbbi, &status, S_xip_badAddress,
                      "devMbbiAvme470 (init_mbbi) XIP address string format error", TRUE);
        }
        else
        {
          ptr = avme470FindCard(pxip->name);
          if( ptr )
          {
            if( (pxip->port < 0) || (pxip->port >= MAXPORTS) )
            {
              handleError(pmbbi, &status, S_avme470_portError,
                          "devMbbiAvme470 (init_mbbi) port out of range", TRUE);
            }
            else if( (pxip->bit  < 0) || (pxip->bit  >= MAXBITS)  )
            {
              handleError(pmbbi, &status, S_avme470_bitError,
                          "devMbbiAvme470 (init_mbbi) bit out of range", TRUE);
            }
            else
            {
              /* Ask driver which interrupt handler do I have ? */
              avme470WhichHandler( pxip->name, &handler );
              pxip->intHandler = handler;
              pmbbi->dpvt      = pxip;
              status           = avme470Read(pxip->name, pxip->port, pxip->bit, NIBBLE, &value, 0);
              if( status )
              {
                handleError(pmbbi, &status, S_avme470_readError,
                            "devMbbiAvme470 (init_mbbi) error from avme470Read", TRUE);
              }
              else
              {
                pmbbi->val  = value & pmbbi->mask;
                pmbbi->rval = value & pmbbi->mask;
              }
            }
          }
          else
          {
            handleError(pmbbi, &status, S_avme470_cardNotFound,
                        "devMbbiAvme470 (init_mbbi) Card not found", TRUE);
          }
        }
      }
      break;

    default:
      handleError(pmbbi, &status, S_db_badField,
                  "devMbbiAvme470 (init_mbbi) illegal INP field", TRUE);
      break;
  }
  return(status);
}


static long mbbi_ioinfo( int cmd, struct mbbiRecord *pmbbi, IOSCANPVT *ppvt )
{
  xipIo_t *pxip;
  int     status;
  int     recType = MBBI;

  pxip   = (xipIo_t *)pmbbi->dpvt;
  status = avme470GetIoScanpvt( pxip->name, pxip->port, pxip->bit, recType, 
                               pxip->intHandler, ppvt );
  if( status )
    handleError(pmbbi, &status, status, "devMbbiAvme470 (mbbi_ioinfo) error", FALSE);

  return(status);
}


static long read_mbbi(struct mbbiRecord	*pmbbi)
{
  xipIo_t        *pxip;
  int            status;
  unsigned short value;
	
  pxip   = (xipIo_t *)pmbbi->dpvt;
  status = avme470Read(pxip->name, pxip->port, pxip->bit, NIBBLE, &value, 0);
  if( status )
  {
    handleError(pmbbi, &status, S_avme470_readError, "devMbbiAvme470 (read_mbbi) error", FALSE);
    recGblSetSevr(pmbbi,READ_ALARM,INVALID_ALARM);
    status = DO_NOT_CONVERT;
  }
  else
  {
    pmbbi->rval = value & pmbbi->mask;
    status      = CONVERT;
  }
  return(status);
}


static long init_mbbiDirect(struct mbbiDirectRecord *pmbbiDirect)
{
  xipIo_t        *pxip;
  int            status;
  unsigned char  handler;
  unsigned short value;
  void           *ptr;

  switch(pmbbiDirect->inp.type) 
  {
    case(INST_IO):
      pxip = (xipIo_t *)malloc(sizeof(xipIo_t));
      if( !pxip )
      {
        handleError(pmbbiDirect, &status, S_dev_noMemory,
                    "devMbbiDirectAvme470 (init_mbbiDirect) malloc failed", TRUE);
      }
      else
      {
        /* Convert the address string into members of the xipIo structure */
        status = xipIoParse(pmbbiDirect->inp.value.instio.string, pxip, 'B');
        if( status )
        {
          handleError(pmbbiDirect, &status, S_xip_badAddress,
                "devMbbiDirectAvme470 (init_mbbiDirect) XIP address string format error", TRUE);
        }
        else
        {
          ptr = avme470FindCard(pxip->name);
          if( ptr )
          {
            if( (pxip->port < 0) || (pxip->port >= MAXPORTS) )
            {
              handleError(pmbbiDirect, &status, S_avme470_portError,
                          "devMbbiDirectAvme470 (init_mbbiDirect) port out of range", TRUE);
            }
            else if( (pxip->bit  < 0) || (pxip->bit  >= MAXBITS)  )
            {
              handleError(pmbbiDirect, &status, S_avme470_bitError,
                          "devMbbiDirectAvme470 (init_mbbiDirect) bit out of range", TRUE);
            }
            else
            {
              /* Ask driver which interrupt handler do I have ? */
              avme470WhichHandler( pxip->name, &handler );
              pxip->intHandler  = handler;
              pmbbiDirect->dpvt = pxip;
              status            = avme470Read(pxip->name, pxip->port, pxip->bit, WORD, &value, 0);
              if( status )
              {
                handleError(pmbbiDirect, &status, S_avme470_readError,
                            "devMbbiDirectAvme470 (init_mbbiDirect) error from avme470Read", TRUE);
              }
              else
              {
                pmbbiDirect->val  = value & pmbbiDirect->mask;
                pmbbiDirect->rval = value & pmbbiDirect->mask;
              }
            }
          }
          else
          {
            handleError(pmbbiDirect, &status, S_avme470_cardNotFound,
                        "devMbbiDirectAvme470 (init_mbbiDirect) Card not found", TRUE);
          }
        }
      }
      break;

    default:
      handleError(pmbbiDirect, &status, S_db_badField,
                  "devMbbiDirectAvme470 (init_mbbiDirect) illegal INP field", TRUE);
      break;
  }
  return(status);
}


static long mbbiDirect_ioinfo( int cmd, struct mbbiDirectRecord *pmbbiDirect, 
                               IOSCANPVT *ppvt )
{
  xipIo_t *pxip;
  int     status;
  int     recType = MBBI_DIRECT;

  pxip   = (xipIo_t *)pmbbiDirect->dpvt;
  status = avme470GetIoScanpvt( pxip->name, pxip->port, pxip->bit, recType, 
                               pxip->intHandler, ppvt );
  if( status )
    handleError(pmbbiDirect, &status, status, 
                "devMbbiDirectAvme470 (mbbiDirect_ioinfo) error", FALSE);

  return(status);
}


static long read_mbbiDirect( struct mbbiDirectRecord *pmbbiDirect )
{
  xipIo_t        *pxip;
  int            status;
  unsigned short value;
	
  pxip   = (xipIo_t *)pmbbiDirect->dpvt;
  status = avme470Read(pxip->name, pxip->port, pxip->bit, WORD, &value, 0);
  if( status )
  {
    handleError(pmbbiDirect, &status, S_avme470_readError, 
                "devMbbiDirectAvme470 (read_mbbiDirect) error", FALSE);
    recGblSetSevr(pmbbiDirect,READ_ALARM,INVALID_ALARM);
    status = DO_NOT_CONVERT;
  }
  else
  {
    pmbbiDirect->rval = value & pmbbiDirect->mask;
    status            = CONVERT;
  }
  return(status);
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
