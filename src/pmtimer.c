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
 *  pmtimer.c - Implementa el power management timer del chipset 430TX.
 *
 */


#include <assert.h>
#include <limits.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "PC.h"




/**********/
/* MACROS */
/**********/

#define PMTHZ 3579545




/*********/
/* ESTAT */
/*********/

// Callbacks.
static PC_Warning *_warning;
static void *_udata;

// Timing.
static struct
{
  int     cc_used;
  int     cc; // Cicles acumulats.
  int64_t cc_remain; // Cicles que resten després de multiplicar per 
  int     cctoEvent;
} _timing;

// Comptador.
static uint32_t _counter;




/*********************/
/* FUNCIONS PRIVADES */
/*********************/

static void
update_cc_to_event (void)
{

  int cc;

  
  // Per defecte 1s
  _timing.cctoEvent= PC_ClockFreq;
  // NOTA!!!! Pot ser calga en el futur implementar una interrupció!!!

  // Actualitza PC_NextEventCC
  cc= PC_pmtimer_next_event_cc ();
  cc+= PC_Clock; // Medim sempre des de que PC_Clock és 0
  if ( cc < PC_NextEventCC )
    PC_NextEventCC= cc;
  
} // end update_cc_to_event


static void
clock (
       const bool update_cc2event
       )
{

  int cc;
  int64_t tmp,clocks;

  
  // Processa cicles.
  cc= PC_Clock-_timing.cc_used;
  _timing.cc+= cc;
  _timing.cc_used+= cc;

  // Transforma a clocks.
  tmp= ((int64_t) _timing.cc)*((int64_t) PMTHZ) + _timing.cc_remain;
  clocks= tmp/((int64_t) PC_ClockFreq);
  _timing.cc_remain= tmp%((int64_t) PC_ClockFreq);
  _timing.cc= 0;

  // Actualitza comptador.
  // int64_t loops= clocks/((int64_t) 0x1000000);
  clocks%= (int64_t) 0x1000000;
  _counter+= (uint32_t) clocks;
  _counter&= 0xFFFFFF; // 24bit

  // Actualitza cctoEvent
  if ( update_cc2event )
    update_cc_to_event ();
  
} // end clock




/**********************/
/* FUNCIONS PÚBLIQUES */
/**********************/

void
PC_pmtimer_init (
                 PC_Warning *warning,
                 void       *udata
                 )
{

  // Callbacks.
  _warning= warning;
  _udata= udata;

  // Comptador.
  _counter= 0;

  // Temporitzadors.
  _timing.cc= 0;
  _timing.cc_used= 0;
  _timing.cc_remain= 0;
  update_cc_to_event ();
  
} // end PC_pmtimer_init


int
PC_pmtimer_next_event_cc (void)
{
  return _timing.cctoEvent - _timing.cc;
} // end PC_pmtimer_next_event_cc


void
PC_pmtimer_end_iter (void)
{

  int cc;

  
  cc= PC_Clock-_timing.cc_used;
  if ( cc > 0 )
    {
      _timing.cc+= cc;
      _timing.cc_used+= cc;
      if ( _timing.cctoEvent && _timing.cc >= _timing.cctoEvent )
        clock ( true );
    }
  _timing.cc_used= 0;
  
} // end PC_pmtimer_end_iter



uint32_t
PC_pmtimer_get (void)
{

  clock ( true );

  return _counter;
  
} // end PC_pmtimer_get
