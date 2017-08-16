/*******************************************************************************

Project:
    Gemini Multi-Conjugate Adaptive Optics Project

File:
    drvXy9660.c

Description:
    EPICS Driver for the XVME-9660 Industrial I/O Pack
    VMEbus 6U Non-Intelligent Carrier Board

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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "xipIo.h"

/* Only used within this file */
static char *strdupn( const char *ct, size_t n );

int xipIoParse( char *str, xipIo_t *ptr, char flag )
{
  char *name;
  char *end;

  if( str == NULL || ptr == NULL )
    return 1;

  while( !isalnum(*str) )
  {
    if( *str++ == '\0')
      return 1;
  }

  name = str;

  str = strpbrk(str, " ");
  if( str == NULL || *str == '\0' )
    return 1;

  ptr->name = strdupn(name, str - name);
  if( ptr->name == NULL )
    return 1;

  if( flag == 'A' )
  {
    end = strchr(str,'C');
    if( end )
    {
      str = end + 1;
      sscanf(str,"%d", &ptr->channel);
    }
    else
      return 1;
  }
  else if( flag == 'B' )
  {
    end = strchr(str,'P');
    if( end )
    {
      str = end + 1;
      sscanf(str,"%d", &ptr->port);
      end = strchr(str,'B');
      if( end )
      {
        str = end + 1;
        sscanf(str,"%d", &ptr->bit);
      }
      else
        return 1;
    }
    else
      return 1;
  }
  return(0);
}


static char *strdupn( const char *ct, size_t n )
{
  char *duplicate;

  duplicate = (char *)malloc(n+1);
  if( !duplicate )
    return NULL;

  memcpy(duplicate, ct, n);
  duplicate[n] = '\0';

  return duplicate;
}
