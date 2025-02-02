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
 *  timers.c - Implementa els "timers/counters" del chipset 430TX.
 *
 */


#include <assert.h>
#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "PC.h"



/*********/
/* ESTAT */
/*********/

// Callbacks.
static PC_Warning *_warning;
static PC_TimerOutChanged *_timer_out_changed;
static void *_udata;
static bool _trace_enabled;

// Estat
static struct
{

  bool     gate; 
  bool     out;
  int      mode;
  bool     is_bcd;
  enum {
    RW_LSB,
    RW_MSB,
    RW_LSB_MSB
  }        rw_mode;
  uint16_t count;
  uint16_t init_count;
  uint16_t latched_count;
  uint8_t  status;
  bool     waiting_write_first_byte;
  bool     waiting_read_first_byte;
  bool     waiting_init_count;
  bool     load_count; // Indica que en el següent cicle cal carregar
                       // el comptador
  bool     latched_value;
  bool     latched_status;
  
} _s[3];

// Timing
static struct
{

  int  cc_used;
  long tcc; // Clocks de timer 1.193 MHz
  long num; // Cada cicle de timer en realitat es NUM*cc/1193
  long tcctoEvent;
  
} _timing;

// Refresh request
static bool _refresh_request_toggle;




/*********************/
/* FUNCIONS PRIVADES */
/*********************/

static void
out_changed (
             const int id
             )
{

  if ( id == 0 ) PC_ic_irq ( 0, _s[id].out );
  else if ( id == 1 )
    {
      if ( _s[id].out )
        _refresh_request_toggle= !_refresh_request_toggle;
    }
  else if ( id == 2 ) PC_speaker_set_out ( _s[id].out );
  
  if ( _trace_enabled && _timer_out_changed != NULL )
    _timer_out_changed ( id, _s[id].out );
  
} // end out_changed


static long
mode0_tcctoevent (
                  const int id
                  )
{
  
  uint16_t count;
  int cc;
  long ret;


  // NOTA!!! En mode 0 el comptador deixa de generar events quan out
  // és 1. Per tant torna -1 per a indicar que no s'ha de tindre en
  // compte
  if ( _s[id].out ) return -1; // Ja està en high
  
  // Tin en compte el load_count
  if ( _s[id].load_count )
    {
      count= _s[id].init_count;
      cc= 1;
    }
  else
    {
      cc= 0;
      count= _s[id].count;
    }
  
  // Afegeix clocks.
  if ( count > 0 )
    {
      cc+= count;
      ret= ((long) cc) * _timing.num;
    }
  else ret= -1;
  
  return ret;
  
} // end mode0_tcctoevent


static long
mode2_tcctoevent (
                  const int id
                  )
{

  uint16_t count;
  int cc;
  long ret;

  
  // Tin en compte el load_count
  if ( _s[id].load_count )
    {
      count= _s[id].init_count;
      cc= 1;
    }
  else
    {
      cc= 0;
      count= _s[id].count;
    }

  // Afegeix clocks.
  if ( count > 1 ) cc+= count-1;
  else if ( count == 1 ) ++cc;
  else // count == 0
    cc+= 0xFFFF;

  // Passa a tcc i resta cicles que ja tenim
  ret= ((long) cc) * _timing.num;
  
  return ret;
  
} // end mode2_tcctoevent


static long
mode3_tcctoevent (
                  const int id
                  )
{

  uint16_t count;
  int cc;
  long ret;

  
  // Tin en compte el load_count
  if ( _s[id].load_count )
    {
      count= (_s[id].init_count&0xFFFE);
      cc= 1;
    }
  else
    {
      cc= 0;
      count= _s[id].count;
    }

  // Afegeix clocks.
  if ( count > 2 ) cc+= (count-2)/2;
  else if ( count == 2 ) ++cc;
  else // count == 0
    cc+= 0xFFFE/2;
  
  // Passa a tcc i resta cicles que ja tenim
  ret= ((long) cc) * _timing.num;
  
  return ret;
  
} // end mode3_tcctoevent


static void
update_tcc_to_event (void)
{

  int i,cc;
  long tmp;
  

  // Per defecte 1s
  _timing.tcctoEvent= (long) PC_ClockFreq * (long)1193;
  // Calcula ttcctoEvent
  for ( i= 0; i < 3; ++i )
    if ( _s[i].gate )
      {
        switch ( _s[i].mode )
          {
          case 0: tmp= mode0_tcctoevent ( i ); break;
            
          case 2: tmp= mode2_tcctoevent ( i ); break;
          case 3: tmp= mode3_tcctoevent ( i ); break;
          case 6: tmp= mode2_tcctoevent ( i ); break;
          case 7: tmp= mode3_tcctoevent ( i ); break;
          default:
            printf("timers - clock: TIMER MODE %d NOT IMPLEMENTED !! \n",
                   _s[i].mode );
            exit ( EXIT_FAILURE );
          }
        // NOTA!! tmp==-1 indica que no s'espera event.
        if ( tmp != -1 && tmp < _timing.tcctoEvent )
          _timing.tcctoEvent= tmp;
      }
  
  // Actualitza PC_NextEventCC
  cc= PC_timers_next_event_cc ();
  cc+= PC_Clock; // Medim sempre des de que PC_Clock és 0
  if ( cc < PC_NextEventCC )
    PC_NextEventCC= cc;
  
} // end update_tcc_to_event


static void
run_mode0 (
           const int id,
           int       clocks
           )
{

  assert ( clocks > 0 );
  if ( _s[id].load_count )
    {
      _s[id].count= _s[id].init_count;
      _s[id].load_count= false;
      --clocks;
    }

  if ( !_s[id].gate ) return;
  
  while ( clocks > 0 )
    {
      if ( _s[id].count > 0 )
        {
          if ( clocks >= (int) (uint32_t) _s[id].count )
            {
              clocks-= (int) (uint32_t) _s[id].count;
              _s[id].count= 0;
              if ( !_s[id].out )
                {
                  _s[id].out= true;
                  out_changed ( id );
                }
            }
          else
            {
              _s[id].count-= (uint16_t) clocks;
              clocks= 0;
            }
        }
      else if ( _s[id].count == 0 )
        {
          _s[id].count= 0xFFFF;
          --clocks;
        }
    }
  
} // end run_mode0


static void
run_mode2 (
           const int id,
           int       clocks
           )
{

  assert ( clocks > 0 );
  if ( _s[id].load_count )
    {
      _s[id].count= _s[id].init_count;
      _s[id].load_count= false;
      --clocks;
    }

  if ( !_s[id].gate ) return;
  
  while ( clocks > 0 )
    {
      if ( _s[id].count > 1 )
        {
          if ( clocks >= (int) (uint32_t) (_s[id].count-1) )
            {
              clocks-= (int) (uint32_t) (_s[id].count-1);
              _s[id].count= 1;
              _s[id].out= false;
              out_changed ( id );
            }
          else
            {
              _s[id].count-= (uint16_t) clocks;
              clocks= 0;
            }
        }
      else if ( _s[id].count == 1 )
        {
          _s[id].count= _s[id].init_count;
          --clocks;
          _s[id].out= true;
          out_changed ( id );
        }
      else // == 0
        {
          _s[id].count= 0xFFFF;
          --clocks;
        }
    }
  
} // end run_mode2


static void
run_mode3 (
           const int id,
           int       clocks
           )
{

  int tmp;

  
  assert ( clocks > 0 );
  if ( _s[id].load_count )
    {
      _s[id].count= (_s[id].init_count&0xFFFE);
      _s[id].load_count= false;
      --clocks;
    }
  
  if ( !_s[id].gate ) return;
  
  while ( clocks > 0 )
    {
      if ( _s[id].count > 2 )
        {
          tmp= 2*clocks;
          if ( tmp >= (int) ((uint32_t) (_s[id].count-2)) )
            {
              clocks-= ((int) ((uint32_t) (_s[id].count-2)))/2;
              _s[id].count= 2;
            }
          else
            {
              _s[id].count-= tmp;
              clocks= 0;
            }
        }
      else if ( _s[id].count == 2 )
        {
          _s[id].count= (_s[id].init_count&0xFFFE);
          --clocks;
          _s[id].out= !_s[id].out;
          out_changed ( id );
        }
      else // 0
        {
          _s[id].count= 0xFFFE;
          --clocks;
        }
    }
  
} // end run_mode3


static void
clock (
       const bool update_tcc2event
       )
{

  int cc,i,clocks;
  

  // Processa cicles.
  cc= PC_Clock-_timing.cc_used;
  if ( cc > 0 ) { _timing.tcc+= (long)1193*(long)cc; _timing.cc_used+= cc; }
  
  // Transforma a cicles del timer.
  clocks= (int) (_timing.tcc/_timing.num);
  if ( clocks == 0 ) return;
  _timing.tcc%= _timing.num;
  
  // Processa clocks
  for ( i= 0; i < 3; ++i )
    switch ( _s[i].mode )
      {
      case 0: run_mode0 ( i, clocks ); break;
        
      case 2: run_mode2 ( i, clocks ); break;
      case 3: run_mode3 ( i, clocks ); break;
        
      case 6: run_mode2 ( i, clocks ); break;
      case 7: run_mode3 ( i, clocks ); break;
      default:
        printf("timers - clock: TIMER MODE %d NOT IMPLEMENTED !! \n",
               _s[i].mode );
        exit ( EXIT_FAILURE );
      }

  // Actualitza cctoEvent
  if ( update_tcc2event )
    update_tcc_to_event ();
  
} // end clock




/**********************/
/* FUNCIONS PÚBLIQUES */
/**********************/

void
PC_timers_init (
                PC_Warning         *warning,
                PC_TimerOutChanged *timer_out_changed,
                void               *udata,
                const PC_Config    *config
                )
{

  // Callbacks.
  _warning= warning;
  _timer_out_changed= timer_out_changed;
  _udata= udata;
  _trace_enabled= false;

  // Inicialitza.
  memset ( &_s, 0, sizeof(_s) );
  _s[0].gate= true;
  _s[1].gate= true;
  _s[2].gate= false; // Únic gate configurable
  // Fique mode 2!!! Ja implemente més modes, però per tradició.
  _s[0].mode= _s[1].mode= _s[2].mode= 2;
  // NOTA!!! LA BIOS NO PROGRAMA EL REFRESHREQUEST!!! Així que el vaig
  // a programar jo en el reset per defecte al que s'espera (request
  // for refresh about every 15 µs)
  _s[1].count= _s[1].init_count= 18;

  // Refres toggle
  _refresh_request_toggle= false;
  
  // Timers
  _timing.cc_used= 0;
  _timing.tcc= 0;
  assert ( PC_ClockFreq%1000 == 0 );
  _timing.num= PC_ClockFreq/1000;
  _timing.tcctoEvent= 0;
  update_tcc_to_event ();
  
} // end PC_timers_init


void
PC_timers_reset (void)
{

  clock ( false );

  // Inicialitza.
  memset ( &_s, 0, sizeof(_s) );
  _s[0].gate= true;
  _s[1].gate= true;
  _s[2].gate= false; // Únic gate configurable
  // Fique mode 2!!! Ja implemente més modes però per tradició.
  _s[0].mode= _s[1].mode= _s[2].mode= 2;
  // NOTA!!! LA BIOS NO PROGRAMA EL REFRESHREQUEST!!! Així que el vaig
  // a programar jo en el reset per defecte al que s'espera (request
  // for refresh about every 15 µs)
  _s[1].count= _s[1].init_count= 18;

  // Refres toggle
  _refresh_request_toggle= false;
  
  // Timers
  _timing.tcc= 0;
  _timing.tcctoEvent= 0;
  
  update_tcc_to_event ();
  
} // end PC_timers_reset


void
PC_timers_control_write (
                         const uint8_t data
                         )
{
  
  int timer,i;
  uint8_t mask,tmp;
  bool old_val;

  
  clock ( false );

  // Read Back Command
  //
  // Açò és un súper comandament que fa moltes coses de colp.
  if ( (data&0xC0) == 0xC0 )
    {
      // Latch status (ho faig abans del latch counter a propòsit)
      if ( (data&0x10) == 0 )
        {
          mask= 0x02;
          for ( i= 0; i < 3; ++i )
            {
              if ( (data&mask)!=0 && !_s[i].latched_status )
                {
                  tmp= 0x00;
                  if ( _s[i].out ) tmp|= 0x80;
                  // ATENCIÓ!!! No vaig a emular el Count Register
                  // Status. Vaig a deixar-lo sempre a 0, que
                  // significa: Count has been transferred from CR to
                  // CE and is available for reading.
                  if ( !_s[i].latched_value )
                    switch ( _s[i].rw_mode )
                      {
                      case RW_LSB: tmp|= 0x10; break;
                      case RW_MSB: tmp|= 0x20; break;
                      case RW_LSB_MSB: tmp|= 0x30; break;
                      }
                  tmp|= (uint8_t) (_s[i].mode<<1);
                  if ( _s[i].is_bcd ) tmp|= 0x01;
                  _s[i].status= tmp;
                  _s[i].latched_status= true;
                }
              mask<<= 1;
            }
        }
      // Latch counters
      if ( (data&0x20) == 0 )
        {
          mask= 0x02;
          for ( i= 0; i < 3; ++i )
            {
              if ( (data&mask)!=0 && !_s[i].latched_value )
                {
                  _s[i].latched_count= _s[i].count;
                  _s[i].latched_value= true;
                  _s[i].waiting_read_first_byte= true;
                }
              mask<<= 1;
            }
        }
    }
  
  else
    {
      
      // Identificador timer
      timer= (data>>6)&0x3;

      // Counter Latch Command
      if ( (data&0x30) == 0x00 )
        {
          if ( !_s[timer].latched_value )
            {
              _s[timer].latched_count= _s[timer].count;
              _s[timer].latched_value= true;
              _s[timer].waiting_read_first_byte= true;
            }
        }
      
      // Read/Write Mode (Configure values)
      else
        {
          switch ( (data>>4)&0x3 )
            {
            case 1: _s[timer].rw_mode= RW_LSB; break;
            case 2: _s[timer].rw_mode= RW_MSB; break;
            case 3: _s[timer].rw_mode= RW_LSB_MSB; break;
            default: printf ( "WTF !!!\n"); break;
            }
          _s[timer].is_bcd= (data&0x1)==1;
          _s[timer].mode= (data>>1)&0x7;
          switch ( _s[timer].mode )
            {
            case 0:
              old_val= _s[timer].out;
              _s[timer].out= false;
              if ( old_val ) out_changed ( timer );
              _s[timer].waiting_init_count= true;
              _s[timer].waiting_write_first_byte= true;
              _s[timer].waiting_read_first_byte= true;
              _s[timer].load_count= false; // Important reiniciar procés
              break;
            case 2: // Mode 2 i 3 coincideixen
            case 3:
            case 6:
            case 7:
              old_val= _s[timer].out;
              _s[timer].out= true;
              if ( !old_val ) out_changed ( timer );
              _s[timer].waiting_init_count= true;
              _s[timer].waiting_write_first_byte= true;
              _s[timer].waiting_read_first_byte= true;
              _s[timer].load_count= false; // Important reiniciar procés
              break;
            default:
              printf("TIMER MODE %d NOT SUPPORTED !!!!\n",_s[timer].mode);
              exit(EXIT_FAILURE);
            }
        }
      
    }

  update_tcc_to_event ();
  
} // end PC_timers_control_write


void
PC_timers_data_write (
                      const int     id,
                      const uint8_t data
                      )
{

  bool init;

  
  clock ( false );

  init= false;
  switch ( _s[id].rw_mode )
    {
    case RW_LSB:
      _s[id].init_count= (_s[id].init_count&0xFF00) | ((uint16_t) data);
      init= true;
      break;
    case RW_MSB:
      _s[id].init_count= (_s[id].init_count&0xFF) | (((uint16_t) data)<<8);
      init= true;
      break;
    case RW_LSB_MSB:
      if ( _s[id].waiting_write_first_byte )
        {
          _s[id].init_count= (_s[id].init_count&0xFF00) | ((uint16_t) data);
          _s[id].waiting_write_first_byte= !_s[id].waiting_write_first_byte;
          // NOTA!! Pareix que en mode 0 ací es deuria ficar ja a 0
          // OUT, però és un poc estrany i redundant. Crec que per
          // simplificar ho faré en l'init.
        }
      else
        {
          _s[id].init_count= (_s[id].init_count&0xFF) | (((uint16_t) data)<<8);
          _s[id].waiting_write_first_byte= !_s[id].waiting_write_first_byte;
          init= true;
        }
      break;
    }

  // Accions inicials si toca
  if ( init )
    {
      
      switch ( _s[id].mode )
        {
        case 0:
          _s[id].load_count= true;
          if ( _s[id].out )
            {
              _s[id].out= false;
              out_changed ( id );
            }
          break;
        case 2: // Mode 2 i 3 coincideixen
        case 3:
        case 6:
        case 7:
          if ( _s[id].waiting_init_count )
            _s[id].load_count= true;
          break;
        default:
          printf("TIMER_DATA_WRITE MODE %d NOT SUPPORTED !!!!\n",_s[id].mode);
          exit(EXIT_FAILURE);
        }
      _s[id].waiting_init_count= false; // Pot ser ja fora false
    }

  update_tcc_to_event ();
  
} // end PC_timers_data_write


uint8_t
PC_timers_data_read (
                     const int id
                     )
{

  uint8_t ret;

  
  clock ( true );

  ret= 0xFF;

  // Latched status
  if ( _s[id].latched_status )
    {
      ret= _s[id].status;
      _s[id].latched_status= false;
    }
  
  // Latched value
  else if ( _s[id].latched_value )
    switch ( _s[id].rw_mode )
      {
      case RW_LSB:
        ret= (uint8_t) (_s[id].latched_count&0xFF);
        _s[id].latched_value= false;
        break;
      case RW_MSB:
        ret= (uint8_t) (_s[id].latched_count>>8);
        _s[id].latched_value= false;          
        break;
      case RW_LSB_MSB:
        if ( _s[id].waiting_read_first_byte )
          {
            ret= (uint8_t) (_s[id].latched_count&0xFF);
            _s[id].waiting_read_first_byte= false;
          }
        else
          {
            ret= (uint8_t) (_s[id].latched_count>>8);
            _s[id].waiting_read_first_byte= true;
            _s[id].latched_value= false;
          }
        break;
      }

  // Lectura directa
  else
    {
      /* ES FA MOLT. NO CAL IMPRIMIR RES
      if ( _s[id].gate )
        _warning ( _udata, "s'ha intentat fer una lectura directa"
                   " del Timer.%d amb GATE=1", id );
      */
      switch ( _s[id].rw_mode )
        {
        case RW_LSB:
          ret= (uint8_t) (_s[id].count&0xFF);
          break;
        case RW_MSB:
          ret= (uint8_t) (_s[id].count>>8);
          break;
        case RW_LSB_MSB:
          if ( _s[id].waiting_read_first_byte )
            {
              ret= (uint8_t) (_s[id].count&0xFF);
              _s[id].waiting_read_first_byte= false;
            }
          else
            {
              ret= (uint8_t) (_s[id].count>>8);
              _s[id].waiting_read_first_byte= true;
            }
          break;
        }
    }
  
  return ret;
  
} // end PC_timers_data_read


void
PC_timers_gate2_set (
                     const bool val
                     )
{

  bool old_val;
  
  
  clock ( false );
  old_val= _s[2].gate;
  _s[2].gate= val;
  switch ( _s[2].mode )
    {
    case 0: break; // En mode 0 simplement para de contar
    case 2: // Mode 2 i 3 coincideixen.
    case 3:
    case 6:
    case 7:
      if ( old_val && !val ) 
        {
          if ( !_s[2].out )
            {
              _s[2].out= true;
              out_changed ( 2 );
              _s[2].waiting_init_count= true;
            }
        }
      break;
    default:
      printf("timers - PC_timers_gate2_set: TIMER MODE %d NOT IMPLEMENTED !!\n",
             _s[2].mode );
      exit ( EXIT_FAILURE );
    }
  update_tcc_to_event ();
  
} // end PC_timers_gate2_set


bool
PC_timers_gate2_get (void)
{

  clock ( true );
  
  return _s[2].gate;
  
} // end PC_timers_gate2_get


int
PC_timers_next_event_cc (void)
{

  long tmp;


  tmp= _timing.tcctoEvent - _timing.tcc;
  assert ( tmp > 0 );
  tmp= tmp/1193 + ((tmp%1193)!=0 ? 1 : 0);

  return (int) tmp;
  
} // end PC_timers_next_event_cc


void
PC_timers_end_iter (void)
{

  int cc;

  
  cc= PC_Clock-_timing.cc_used;
  if ( cc > 0 )
    {
      _timing.tcc+= ((long) cc)*((long) 1193);
      _timing.cc_used+= cc;
      if ( _timing.tcctoEvent && _timing.tcc >= _timing.tcctoEvent )
        clock ( true );
    }
  _timing.cc_used= 0;
  
} // end PC_timers_end_iter


void
PC_timers_set_mode_trace (
                          const bool val
                          )
{
  _trace_enabled= val;
} // end PC_timers_set_mode_trace


bool
PC_timers_out2_get (void)
{
  
  clock ( true );
  
  return _s[2].out;
  
} // end PC_timers_out2_get


bool
PC_timers_get_refresh_request_toggle (void)
{

  clock ( true );

  return _refresh_request_toggle;
  
} // end PC_timers_get_refresh_request_toggle
