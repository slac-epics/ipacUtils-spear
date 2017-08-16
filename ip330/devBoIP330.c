/****************************************************************/
/* $Id: devBoIP330.c,v 1.1 2010/07/01 22:17:05 saa Exp $    */
/* This file implements BO record device support for IP330 DAC  */
/* Author: Sheng Peng, pengs@slac.stanford.edu, 650-926-3847    */
/****************************************************************/
#include <stdio.h>
#include <string.h>

#include <epicsVersion.h>

#if EPICS_VERSION>=3 && EPICS_REVISION>=14
#include <epicsExport.h>
#endif

#include <devLib.h>
#include <dbAccess.h>
#include <dbScan.h>
#include <callback.h>
#include <cvtTable.h>
#include <link.h>
#include <recSup.h>
#include <recGbl.h>
#include <devSup.h>
#include <drvSup.h>
#include <dbCommon.h>
#include <alarm.h>
#include <cantProceed.h>
#include <boRecord.h>
#include <errlog.h>

#include <ptypes.h>
#include <drvIP330Lib.h>

#define MAX_CA_STRING_SIZE (40)

/* define function flags */
typedef enum {
        IP330_START_CONVERT,
} IP330FUNC;

static struct PARAM_MAP
{
        char param[MAX_CA_STRING_SIZE];
        int  funcflag;
} param_map[1] = {
    {"START", IP330_START_CONVERT}
};
#define N_PARAM_MAP (sizeof(param_map)/sizeof(struct PARAM_MAP))

typedef struct IP330_DEVDATA
{
    IP330_ID	pcard;
    int		funcflag;
} IP330_DEVDATA;


/* This function will be called by all device support */
/* The memory for IP330_DEVDATA will be malloced inside */
static int IP330_DevData_Init(dbCommon * precord, char * ioString)
{
    int		count;
    int		loop;

    char	cardname[MAX_CA_STRING_SIZE];
    IP330_ID	pcard;
    char	param[MAX_CA_STRING_SIZE];
    int		funcflag = 0;

    IP330_DEVDATA *   pdevdata;

    /* param check */
    if(precord == NULL || ioString == NULL)
    {
        if(!precord) errlogPrintf("No legal record pointer!\n");
        if(!ioString) errlogPrintf("No INP/OUT field for record %s!\n", precord->name);
        return -1;
    }

    /* analyze INP/OUT string */
    count = sscanf(ioString, "%[^:]:%[^:]", cardname, param);
    if (count != 2)
    {
        errlogPrintf("Record %s INP/OUT string %s format is illegal!\n", precord->name, ioString);
        return -1;
    }

    pcard = ip330GetByName(cardname);
    if( !pcard )
    {
        errlogPrintf("Record %s IP330 %s is not registered!\n", precord->name, cardname);
        return -1;
    }

    for(loop=0; loop<N_PARAM_MAP; loop++)
    {
        if( 0 == strcmp(param_map[loop].param, param) )
        {
            funcflag = param_map[loop].funcflag;
            break;
        }
    }
    if(loop >= N_PARAM_MAP)
    {
        errlogPrintf("Record %s param %s is illegal!\n", precord->name, param);
        return -1;
    }

    pdevdata = (IP330_DEVDATA *)callocMustSucceed(1, sizeof(IP330_DEVDATA), "Init record for IP330");

    pdevdata->pcard = pcard;
    pdevdata->funcflag = funcflag;

    precord->dpvt = (void *)pdevdata;
    return 0;
}


static long init_bo( struct boRecord * pbo)
{
    IP330_DEVDATA * pdevdata;

    pbo->dpvt = NULL;

    if (pbo->out.type!=INST_IO)
    {
        recGblRecordError(S_db_badField, (void *)pbo, "devBoIP330 Init_record, Illegal INP");
        pbo->pact=TRUE;
        return (S_db_badField);
    }

    if(IP330_DevData_Init((dbCommon *) pbo, pbo->out.value.instio.string) != 0)
    {
        errlogPrintf("Fail to init devdata for record %s!\n", pbo->name);
        recGblRecordError(S_db_badField, (void *) pbo, "Init devdata Error");
        pbo->pact = TRUE;
        return (S_db_badField);
    }

    pdevdata = (IP330_DEVDATA *)(pbo->dpvt);

    return 2;
}

static long write_bo(struct boRecord *pbo)
{
    IP330_DEVDATA * pdevdata = (IP330_DEVDATA *)(pbo->dpvt);

    int status=-1;

    switch(pdevdata->funcflag)
    {
    case IP330_START_CONVERT:
        if (pbo->val) ip330StartConvert(pdevdata->pcard);
        status = 0;
        break;
    }

    if(status)
    {
        recGblSetSevr(pbo, WRITE_ALARM, INVALID_ALARM);
        return -1;
    }

    return 0;
}

struct IP330_DEV_SUP_SET
{
    long            number;
    DEVSUPFUN       report;
    DEVSUPFUN       init;
    DEVSUPFUN       init_record;
    DEVSUPFUN       get_ioint_info;
    DEVSUPFUN       write_bo;
} devBoIP330 = {5, NULL, NULL, init_bo, NULL, write_bo};

#if EPICS_VERSION>=3 && EPICS_REVISION>=14
epicsExportAddress(dset, devBoIP330);
#endif

