/*
 * Copyright 2021-2025 Adrià Giménez Pastor.
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
 *  files.c - Implementa diversos tipus de PC_File.
 *
 */

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "PC.h"




/*********/
/* TIPUS */
/*********/

typedef struct
{
  
  PC_FILE_CLASS;
  FILE *fd;
  long  offset;
  
} file_t;




/***********/
/* MÈTODES */
/***********/

static int
file_seek (
           PC_File *f,
           long     offset
           )
{
  
  file_t *self;
  int ret;
  
  
  self= (file_t *) f;
  if ( offset >= self->nbytes )
    return -1;
  ret= fseek ( self->fd, offset, SEEK_SET );
  if ( ret != -1 )
    self->offset= offset;
  
  return ret;
  
} // end file_seek


static long
file_tell (
           PC_File *f
           )
{
  return ((file_t *) f)->offset;
} // end file_tell


static int
file_read (
           PC_File *f,
           void    *dst,
           long     nbytes
           )
{

  file_t *self;
  long tmp;
  int ret;

  
  self= (file_t *) f;
  tmp= self->offset + nbytes;
  if ( nbytes == 0 ) return -1;
  if ( tmp < nbytes ) return -1;
  if ( tmp > self->nbytes ) return -1;
  ret= (int) fread ( dst, (size_t) nbytes, 1, self->fd );
  if ( ret==1 ) self->offset+= nbytes;
  
  return ret==1 ? 0 : -1;
  
} // end file_read


static int
file_write (
            PC_File *f,
            void    *src,
            long     nbytes
            )
{

  file_t *self;
  long tmp;
  int ret;
  
  
  self= (file_t *) f;
  tmp= self->offset + nbytes;
  if ( nbytes == 0 ) return -1;
  if ( tmp < nbytes ) return -1;
  if ( tmp > self->nbytes ) return -1;
  ret= (int) fwrite ( src, (size_t) nbytes, 1, self->fd );
  if ( ret==1 )
    {
      fflush ( self->fd );
      self->offset+= nbytes;
    }
  
  return ret==1 ? 0 : -1;
  
} // end file_write


static void
file_free (
           PC_File *f
           )

{

  file_t *self;


  self= (file_t *) f;
  if ( self->fd != NULL ) fclose ( self->fd );
  free ( self );
  
} // end file_free




/**********************/
/* FUNCIONS PÚBLIQUES */
/**********************/

PC_File *
PC_file_new_from_file (
                       const char *file_name,
                       const bool  read_only
                       )
{

  file_t *ret;
  long size;
  

  // Prepara.
  ret= (file_t *) malloc ( sizeof(file_t) );
  if ( ret == NULL ) return NULL;
  ret->fd= NULL;
  ret->read_only= read_only;
  ret->seek= file_seek;
  ret->tell= file_tell;
  ret->read= file_read;
  ret->write= file_write;
  ret->free= file_free;
  
  // Obri fitxer.
  ret->fd= fopen ( file_name, read_only ? "rb" : "r+b" );
  if ( ret->fd == NULL ) goto error;

  // Comprova grandària.
  if ( fseek ( ret->fd, 0, SEEK_END ) == -1 ) goto error;
  size= ftell ( ret->fd );
  if ( size == -1 || size == 0 ) goto error;
  ret->nbytes= size;
  rewind ( ret->fd );
  ret->offset= 0;
  
  return PC_FILE(ret);
  
 error:
  PC_file_free ( PC_FILE(ret) );
  return NULL;
  
} // end PC_file_new_from_file
