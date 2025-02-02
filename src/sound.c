/*
 * Copyright 2022-2025 Adrià Giménez Pastor.
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
 *  sound.c - Gestiona la generació del so.
 *
 */

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "PC.h"




/**********/
/* MACROS */
/**********/

#define READY 0x3




/*********/
/* ESTAT */
/*********/

// Callbacks
static PC_Warning *_warning;
static PC_PlaySound *_play_sound;
static void *_udata;

// Estat
static uint8_t _active_sources;

static int16_t _out[PC_AUDIO_BUFFER_SIZE*2];




/**********************/
/* FUNCIONS PÚBLIQUES */
/**********************/

void
PC_sound_init (
               PC_Warning   *warning,
               PC_PlaySound *play_sound,
               void         *udata
               )
{

  // Callbacks
  _warning= warning;
  _play_sound= play_sound;
  _udata= udata;

  // Estat.
  _active_sources= 0;
  
} // end PC_sound_init


void
PC_sound_set (
              const int16_t samples[PC_AUDIO_BUFFER_SIZE*2],
              const int     source_id
              )
{
  
  uint8_t mask;
  int32_t tmp;
  int i;
  
  mask= 0x1<<source_id;
  if ( (mask&_active_sources) == 0 )
    {
      
      // Copia
      if ( _active_sources == 0 )
        memcpy ( _out, samples, sizeof(int16_t)*PC_AUDIO_BUFFER_SIZE*2 );
      else
        for ( i= 0; i < PC_AUDIO_BUFFER_SIZE*2; ++i )
          {
            // Satura.
            tmp= ((int32_t) _out[i]) + ((int32_t) samples[i]);
            if      ( tmp > 32767 )  tmp= 32767;
            else if ( tmp < -32768 ) tmp= -32768;
            _out[i]= (int16_t) tmp;
          }
      _active_sources|= mask;
      
      // Crida
      if ( _active_sources == READY )
        {
          _play_sound ( _out, _udata );
          _active_sources= 0x00;
        }
      
    }
  
} // end PC_sound_set
