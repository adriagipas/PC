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
 *  speaker.c - Implementa el "PC speaker".
 *
 */


#include <assert.h>
#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "PC.h"




/**********/
/* MACROS */
/**********/

// Un poc menys de (2**16)/4 --> /2 pel signe /2 per poder sumar
#define MAX_AMP 16383




/*********/
/* ESTAT */
/*********/

// Callback.
static PC_Warning *_warning;
static void *_udata;

// Estat del buffer.
static struct
{
  int16_t buf[PC_AUDIO_BUFFER_SIZE*2];
  int     N;
  bool    out;
  bool    timer_enabled;
  bool    enabled;
  long    sample_cc;
  long    sample_1s;
} _s;

// Gestionar cicles
static struct
{

  int cc_used;
  int cc;
  int cctoEvent;

  // Per a passar a mostres a 44100Hz (cc*(cc_mul))/cc_div
  long cc_mul;
  long cc_div;
  
} _timing;




/*********************/
/* FUNCIONS PRIVADES */
/*********************/

static int
calc_cc_to_fill_buffer (void)
{

  long tmpcc;
  int ret;
  

  tmpcc= (PC_AUDIO_BUFFER_SIZE-(_s.N/2))*_timing.cc_div;
  tmpcc-= _s.sample_cc;
  ret= (int) (tmpcc / _timing.cc_mul) + ((tmpcc%_timing.cc_mul)!=0);

  return ret;
  
} // end calc_cc_to_fill_buffer


static void
update_cc_to_event (void)
{

  int cc,tmp;

  
  // Per defecte 1s
  _timing.cctoEvent= PC_ClockFreq;
  // --> Un nou buffer
  tmp= calc_cc_to_fill_buffer ();
  assert ( tmp > 0 );
  if ( tmp < _timing.cctoEvent ) _timing.cctoEvent= tmp;
  
  // Actualitza PC_NextEventCC
  cc= PC_speaker_next_event_cc ();
  cc+= PC_Clock; // Medim sempre des de que PC_Clock és 0
  if ( cc < PC_NextEventCC )
    PC_NextEventCC= cc;
  
} // end update_cc_to_event


static void
run_sample_cc (
               long cc
               )
{

  bool out;
  long tmp;
  int16_t sample;
  

  out= _s.timer_enabled ? (_s.enabled&&_s.out) : _s.enabled;
  while ( cc > 0 )
    {
      tmp= _timing.cc_div-_s.sample_cc; // Cal per a acavar
      if ( cc >= tmp ) // Una mostra nova
        {
          // Genere mostra
          if ( out ) _s.sample_1s+= tmp;
          if ( _s.sample_1s == 0 ) sample= 0;
          else if ( _s.sample_1s == _timing.cc_div ) sample= MAX_AMP;
          else // convolució
            sample=
              (int16_t) ((_s.sample_1s/(double) _timing.cc_div)*MAX_AMP + 0.5);
          _s.buf[_s.N++]= sample;
          _s.buf[_s.N++]= sample;
          if ( _s.N == PC_AUDIO_BUFFER_SIZE*2 )
            {
              PC_sound_set ( _s.buf, PC_SOUND_SOURCE_SPEAKER );
              _s.N= 0;
            }
          // Actualitze cicles
          cc-= tmp;
          _s.sample_cc= 0;
          _s.sample_1s= 0;
        }
      else
        {
          _s.sample_cc+= cc;
          if ( out ) _s.sample_1s+= cc;
          cc= 0;
        }
    }
  
} // end run_sample_cc


static void
clock (void)
{

  int cc;
  long sample_cc;
  
  
  // Processa cicles
  cc= PC_Clock-_timing.cc_used;
  if ( cc > 0 ) { _timing.cc+= cc; _timing.cc_used+= cc; }
  
  // Processa cicles
  sample_cc= ((long) _timing.cc)*_timing.cc_mul;
  _timing.cc= 0;
  if ( sample_cc > 0 ) run_sample_cc ( sample_cc );
  
  // Actualitza cctoEvent
  update_cc_to_event ();
  
} // end clock




/**********************/
/* FUNCIONS PÚBLIQUES */
/**********************/

void
PC_speaker_init (
                 PC_Warning *warning,
                 void       *udata
                 )
{

  static const int DIVS[4]= {3,3,7,7};
  int i;

  
  // Callbacks.
  _warning= warning;
  _udata= udata;

  // Estat.
  memset ( _s.buf, 0, sizeof(_s.buf) );
  _s.N= 0;
  _s.enabled= false;
  _s.out= false;
  _s.timer_enabled= false;
  _s.sample_cc= 0;
  _s.sample_1s= 0;
  
  // Timing.
  _timing.cc_used= 0;
  _timing.cc= 0;
  _timing.cctoEvent= 0;
  assert ( PC_ClockFreq%100 == 0 );
  _timing.cc_div= (long) (PC_ClockFreq/100);
  _timing.cc_mul= 441; // 44.1Khz/100
  // --> Intenta ajustar un poc
  for ( i= 0; i < 4; ++i )
    if ( _timing.cc_div%DIVS[i] == 0 )
      {
        _timing.cc_div/= DIVS[i];
        _timing.cc_mul/= DIVS[i];
      }
  
  update_cc_to_event ();
  
} // end PC_speaker_init


int
PC_speaker_next_event_cc (void)
{

  int tmp;
  
  
  tmp= _timing.cctoEvent - _timing.cc;
  assert ( tmp > 0 );

  return tmp;
  
} // end PC_speaker_next_event_cc


void
PC_speaker_end_iter (void)
{

  int cc;
  
  
  cc= PC_Clock-_timing.cc_used;
  if ( cc > 0 )
    {
      _timing.cc+= cc;
      _timing.cc_used+= cc;
      if ( _timing.cctoEvent && _timing.cc >= _timing.cctoEvent )
        clock ();
    }
  _timing.cc_used= 0;
  
} // end PC_speaker_end_iter


void
PC_speaker_reset (void)
{

  // ATENCIÓ!! Per a no fer glitches raros en la generació del so, no
  // vaig a resetejar els timings ni l'estat del buffer. Únicament
  // reseteje l'estat de l'eixida.
  clock ();
  
  _s.out= false;
  _s.timer_enabled= false;
  _s.enabled= false;
  
} // end PC_speaker_reset


void
PC_speaker_set_out (
                    const bool val
                    )
{

  clock ();
  _s.out= val;
  
} // end PC_speaker_set_out


void
PC_speaker_enable_timer (
                         const bool val
                         )
{

  clock ();
  _s.timer_enabled= val;
  
} // end PC_speaker_enable_timer


void
PC_speaker_data_enable (
                        const bool enabled
                        )
{

  clock ();
  _s.enabled= enabled;
  
} // end PC_speaker_data_enable


bool
PC_speaker_get_enabled (void)
{
  return _s.enabled;
} // end PC_speaker_get_enabled
