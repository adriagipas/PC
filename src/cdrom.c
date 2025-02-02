/*
 * Copyright 2023-2025 Adrià Giménez Pastor.
 *
 * This file is part of adriagipas/PC.
 *
 * adriagipas/PC is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * adriagipas/PC is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with adriagipas/PC.  If not, see <https://www.gnu.org/licenses/>.
 */
/*
 *  cdrom.c - Implementa el tipus PC_CDRom.
 *
 */

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "PC.h"




/**********************/
/* FUNCIONS PÚBLIQUES */
/**********************/

PC_CDRom *
PC_cdrom_new (void)
{
  
  PC_CDRom *ret;
  
  
  ret= (PC_CDRom *) malloc ( sizeof(PC_CDRom) );
  if ( ret == NULL ) return NULL;
  ret->info= NULL;
  ret->current= NULL;
  
  return ret;
  
} // end PC_cdrom_new


void
PC_cdrom_free (
               PC_CDRom *cdrom
               )
{
  
  if ( cdrom->info != NULL ) CD_info_free ( cdrom->info );
  if ( cdrom->current != NULL ) CD_disc_free ( cdrom->current );
  free ( cdrom );
  
} // end PC_cdrom_free


bool
PC_cdrom_insert_disc (
                      PC_CDRom    *cdrom,
                      const char  *file_name,
                      char       **err
                      )
{
  
  CD_Disc *disc;
  

  // Intenta carregar.
  if ( file_name != NULL )
    {
      disc= CD_disc_new ( file_name, err );
      if ( disc == NULL ) return false;
    }
  else disc= NULL;
      
  // Intercànvia.
  if ( cdrom->info != NULL ) CD_info_free ( cdrom->info );
  if ( cdrom->current != NULL ) CD_disc_free ( cdrom->current );
  cdrom->current= disc;
  cdrom->info= disc!=NULL ? CD_disc_get_info ( disc ) : NULL;
  
  return true;
  
} // end PC_cdrom_insert_disc
