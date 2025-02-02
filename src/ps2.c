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
 *  ps2.c - Implementació del controlador 8042 PS/2.
 *
 */


#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <float.h>

#include "PC.h"




/**********/
/* MACROS */
/**********/

#define BSIZE 100

#define KBD_ACK 0xfa
#define KBD_NACK 0xfe

#define MOUSE_ACK 0xfa
#define MOUSE_NACK 0xfe




/*********/
/* TIPUS */
/*********/

typedef struct
{
  uint8_t v[BSIZE];
  int     N;
  int     p;
} buffer_t;




/*********/
/* ESTAT */
/*********/

// Callbacks.
static PC_Warning *_warning;
static void *_udata;
static PC_HostMouse _host_mouse;

// Estat del controlador.
static struct
{
  
  uint8_t inbuff;
  uint8_t outbuff;
  bool    inbuff_full;
  bool    outbuff_full;
  int     outbuff_source;
  bool    data_is_command;
  bool    system_flag;
  struct
  {
    bool enabled;
    bool irq_enabled;
    bool clock_enabled;
    bool translation_enabled;
  }       ports[2];
  enum {
    WAITING_NONE,
    WAITING_NEXT_COMMAND_BYTE,
    WAITING_BYTE_PORT2,
    WAITING_BYTE_OUTPUT_PORT
  }       waiting;
  uint8_t pending_cmd;
  
} _controller;

// Keyboard
static struct
{
  buffer_t buf;
  enum {
    KBD_WAIT_CMD,
    KBD_WAIT_ARG__SETGET_SCANCODE_SET,
    KBD_WAIT_ARG__SET_LEDS
  }        state;
  bool     scan_enabled;
  uint8_t  repeat_delay_reg;
  uint8_t  scancode_set;
  int      repeat_rate_cc; // APROXIMAT No importa molt que siga exacte
  int      delay_cc; // APROXIMAT
  struct
  {
    struct
    {
      enum {
        KEY_RELEASED=0,
        KEY_WAIT_FIRST_REPEAT,
        KEY_WAIT_REPEAT
      }   state;
      int pos_as; // Posició en vector actius
      int cc;
    }   v[PC_SCANCODE_ALL];  // Info sobre estat tecles
    int as[PC_SCANCODE_ALL]; // Tecles presionades ara mateix
    int N;                   // Número de tecles en as
  }        keys;
} _kbd;

// Mouse
static struct
{
  buffer_t buf;
  enum {
    MOUSE_WAIT_CMD,
    MOUSE_WAIT_ARG__SET_MOUSE_RESOLUTION,
    MOUSE_WAIT_ARG__SET_MOUSE_SAMPLE_RATE
  }        state;
  struct
  {
    /* // Possibles futures millores
    int32_t old_dx; // En píxels
    int32_t old_dy; // En píxels
    int32_t thrx; // Píxels de sensitivitat que falten
    int32_t thry; // Píxels de sensitivitat que falten
    */
    float dx; // En milímitres
    float dy; // En milímitres
  }        motion;
  uint8_t  buttons;
  uint8_t  last_buttons;
  uint8_t  sample_rate;
  bool     stream_mode;
  int      resolution;
} _mouse;

// Timing
static struct
{

  int cc_used;
  int cc; // Cicles pendents de processar
  int ccperbit;
  int bit_counter; // -1 indica no s'està enviant res.
  int cc_bit_counter_pendent;
  int cctoEvent;
  // Açò medix els 1/3 de cicles de CPU necessàris per a fer un cicle
  // de 1200Hz. El motiu és que 1200 és múltiple de totes les
  // freqüències de mostreig. I el tema dels 1/3 és que puc assumir
  // que la freqüència de la CPU siga múltiple de 400 pero no de 1200.
  int cc_mouse_duration;
  // 1/3 de cicles de CPU necessàris per a tindre una mostra.
  int cc_mouse_sample_rate;
  // 1/3 de cicles pendents de processar.
  int cc_mouse;
  
} _timing;




/*********************/
/* FUNCIONS PRIVADES */
/*********************/

// Definicions
static void
kbd_buffer_add_scancode (
                         const PC_Scancode sc,
                         const bool        break_code
                         );

static void
update_cc_to_event (void)
{

  int cc,tmp,n;
  
  
  // Per defecte 1s
  _timing.cctoEvent= PC_ClockFreq;
  // Interrupció següent byte transerit
  if ( _timing.bit_counter != -1 )
    {
      tmp= (8-_timing.bit_counter)*_timing.ccperbit;
      assert ( tmp >= 0 );
      tmp-= _timing.cc_bit_counter_pendent;
      assert ( tmp > 0 );
      if ( tmp < _timing.cctoEvent ) _timing.cctoEvent= tmp;
    }
  // Tecles que es repeteixen
  for ( n= 0; n < _kbd.keys.N; ++n )
    {
      tmp= _kbd.keys.v[_kbd.keys.as[n]].cc;
      assert ( tmp > 0 );
      if ( tmp < _timing.cctoEvent ) _timing.cctoEvent= tmp;
    }
  // Mostreig del ratolí.
  if ( _mouse.stream_mode )
    {
      tmp= _timing.cc_mouse_sample_rate - _timing.cc_mouse;
      assert ( tmp > 0 );
      tmp= tmp/3 + ((tmp%3)!=0);
      if ( tmp < _timing.cctoEvent ) _timing.cctoEvent= tmp;
    }
  
  // Actualitza PC_NextEventCC
  cc= PC_ps2_next_event_cc ();
  cc+= PC_Clock; // Medim sempre des de que PC_Clock és 0
  if ( cc < PC_NextEventCC )
    PC_NextEventCC= cc;
  
} // end update_cc_to_event


static void
buffer_clear (
              buffer_t *b
              )
{

  b->p= 0;
  b->N= 0;
  
} // end buffer_clear


// Torna cert si tot ha anat bé
static bool
buffer_add (
            buffer_t       *b,
            const uint8_t  data
            )
{

  if ( b->N == BSIZE ) return false;

  b->v[(b->p+b->N)%BSIZE]= data;
  ++(b->N);
  if ( _timing.bit_counter == -1 ) // Prepara transmissió.
    _timing.bit_counter= 0;
  
  return true;
  
} // end buffer_add


// S'assumeix que no està buit.
static uint8_t
buffer_read (
             buffer_t *b
             )
{

  uint8_t ret;


  ret= b->v[b->p];
  b->p= (b->p+1)%BSIZE;
  --(b->N);

  return ret;
  
} // end buffer_read


static void
kbd_recalc_repeat_delay_cc (void)
{

  int B,D;
  double tmp;

  
  // Delay.
  switch ( (_kbd.repeat_delay_reg>>5)&0x3 )
    {
    case 0: _kbd.delay_cc= PC_ClockFreq/4; break;
    case 1: _kbd.delay_cc= PC_ClockFreq/2; break;
    case 2: _kbd.delay_cc= (3*PC_ClockFreq)/4; break;
    case 3: _kbd.delay_cc= PC_ClockFreq; break;
    }

  // Repeat rate
  B= (_kbd.repeat_delay_reg>>3)&0x3;
  D= _kbd.repeat_delay_reg&0x7;
  tmp= ((1<<B)*(D+8))/240.0;
  _kbd.repeat_rate_cc= (int) (tmp*PC_ClockFreq + 0.5);
  
} // end kbd_recalc_repeat_delay_cc


static void
kbd_set_defaults (void)
{

  buffer_clear ( &_kbd.buf );
  _kbd.repeat_delay_reg= (0x1<<5) | 0x0b; // 500 ms i 10.9
  kbd_recalc_repeat_delay_cc ();
  _kbd.state= KBD_WAIT_CMD;
  _kbd.scancode_set= 0x2; // NOTA !!! De moment sols suporte scancode_set 2
  
} // end kbd_set_defaults


static void
kbd_clear_keys (void)
{

  int n,key;


  for ( n= 0; n < _kbd.keys.N; ++n )
    {
      key= _kbd.keys.as[n];
      _kbd.keys.v[key].state= KEY_RELEASED;
      if ( key != PC_SCANCODE_E1_14_77_E1_F0_14_F0_77 )
        kbd_buffer_add_scancode ( key, true );
    }
  _kbd.keys.N= 0;
  
} // end kbd_clear_keys


static void
kbd_reset (void)
{
  
  _kbd.scan_enabled= false;
  kbd_clear_keys ();
  kbd_set_defaults ();
  
} // end kbd_reset


static void
kbd_init (void)
{

  int n;
  
  
  for ( n= 0; n < PC_SCANCODE_ALL; ++n )
    _kbd.keys.v[n].state= KEY_RELEASED;
  _kbd.keys.N= 0;
  kbd_reset ();
  
} // end kbd_init


static void
kbd_buffer_add (
                const uint8_t data
                )
{
  
  if ( !buffer_add ( &_kbd.buf, data ) )
    _warning ( _udata, "PS/2: buffer teclat ple " );
  
} // end kbd_buffer_add


// NOTA!!! De moment els scancodes són del SET 2
static void
kbd_buffer_add_scancodes (
                          const uint8_t v[],
                          const int     N
                          )
{

  static const uint8_t TABLE[256]= {
    // 0x00
    0xff,0x43,0x41,0x3f,0x3d,0x3b,0x3c,0x58,
    0x64,0x44,0x42,0x40,0x3e,0x0f,0x29,0x59,
    // 0x10
    0x65,0x38,0x2a,0x70,0x1d,0x10,0x02,0x5a,
    0x66,0x71,0x2c,0x1f,0x1e,0x11,0x03,0x5b,
    // 0x20
    0x67,0x2e,0x2d,0x20,0x12,0x05,0x04,0x5c,
    0x68,0x39,0x2f,0x21,0x14,0x13,0x06,0x5d,
    // 0x30
    0x69,0x31,0x30,0x23,0x22,0x15,0x07,0x5e,
    0x6a,0x72,0x32,0x24,0x16,0x08,0x09,0x5f,
    // 0x40
    0x6b,0x33,0x25,0x17,0x18,0x0b,0x0a,0x60,
    0x6c,0x34,0x35,0x26,0x27,0x19,0x0c,0x61,
    // 0x50
    0x6d,0x73,0x28,0x74,0x1a,0x0d,0x62,0x6e,
    0x3a,0x36,0x1c,0x1b,0x75,0x2b,0x63,0x76,
    // 0x60
    0x55,0x56,0x77,0x78,0x79,0x7a,0x0e,0x7b,
    0x7c,0x4f,0x7d,0x4b,0x47,0x7e,0x7f,0x6f,
    // 0x70
    0x52,0x53,0x50,0x4c,0x4d,0x48,0x01,0x45,
    0x57,0x4e,0x51,0x4a,0x37,0x49,0x46,0x54,
    // 0x80
    0x80,0x81,0x82,0x41,0x54,0x85,0x86,0x87,
    0x88,0x89,0x8a,0x8b,0x8c,0x8d,0x8e,0x8f,
    // 0x90
    0x90,0x91,0x92,0x93,0x94,0x95,0x96,0x97,
    0x98,0x99,0x9a,0x9b,0x9c,0x9d,0x9e,0x9f,
    // 0xa0
    0xa0,0xa1,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,
    0xa8,0xa9,0xaa,0xab,0xac,0xad,0xae,0xaf,
    // 0xb0
    0xb0,0xb1,0xb2,0xb3,0xb4,0xb5,0xb6,0xb7,
    0xb8,0xb9,0xba,0xbb,0xbc,0xbd,0xbe,0xbf,
    // 0xc0
    0xc0,0xc1,0xc2,0xc3,0xc4,0xc5,0xc6,0xc7,
    0xc8,0xc9,0xca,0xcb,0xcc,0xcd,0xce,0xcf,
    // 0xd0
    0xd0,0xd1,0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,
    0xd8,0xd9,0xda,0xdb,0xdc,0xdd,0xde,0xdf,
    // 0xe0
    0xe0,0xe1,0xe2,0xe3,0xe4,0xe5,0xe6,0xe7,
    0xe8,0xe9,0xea,0xeb,0xec,0xed,0xee,0xef,
    // 0xf0
    0x00,0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,
    0xf8,0xf9,0xfa,0xfb,0xfc,0xfd,0xfe,0xff
  };
  
  int n;
  uint8_t next;
  
  
  if ( _controller.ports[0].translation_enabled )
    {
      next= 0x00;
      for ( n= 0; n < N; ++n )
        {
          if ( v[n] == 0xf0 ) next= 0x80;
          else
            {
              kbd_buffer_add ( TABLE[v[n]] | next );
              next= 0x00;
            }
        }
    }
  else
    for ( n= 0; n < N; ++n )
      kbd_buffer_add ( v[n] );
  
} // end kbd_buffer_add_scancodes


// NOTA!!! De moment els scancodes són del SET 2
static void
kbd_buffer_add_scancode (
                         const PC_Scancode sc,
                         const bool        break_code
                         )
{

  uint8_t b[8];
  int N;

  
  switch ( sc )
    {
    case PC_SCANCODE_76: b[0]= 0x76; N= 1; break;
    case PC_SCANCODE_05: b[0]= 0x05; N= 1; break;
    case PC_SCANCODE_06: b[0]= 0x06; N= 1; break;
    case PC_SCANCODE_04: b[0]= 0x04; N= 1; break;
    case PC_SCANCODE_0C: b[0]= 0x0c; N= 1; break;
    case PC_SCANCODE_03: b[0]= 0x03; N= 1; break;
    case PC_SCANCODE_0B: b[0]= 0x0b; N= 1; break;
    case PC_SCANCODE_83: b[0]= 0x83; N= 1; break;
    case PC_SCANCODE_0A: b[0]= 0x0a; N= 1; break;
    case PC_SCANCODE_01: b[0]= 0x01; N= 1; break;
    case PC_SCANCODE_09: b[0]= 0x09; N= 1; break;
    case PC_SCANCODE_78: b[0]= 0x78; N= 1; break;
    case PC_SCANCODE_07: b[0]= 0x07; N= 1; break;
    case PC_SCANCODE_E0_12_E0_7C:
      if ( !break_code )
        {
          b[0]= 0xe0; b[1]= 0x12; b[2]= 0xe0; b[3]= 0x7c;
          N= 4;
        }
      else
        {
          b[0]= 0xe0; b[1]= 0xf0; b[2]= 0x12; b[3]= 0xe0; b[4]= 0xf0; b[5]=0x7c;
          N= 6;
        }
      break;
    case PC_SCANCODE_E0_7C: b[0]= 0xe0; b[1]= 0x7c; N= 2; break;
    case PC_SCANCODE_7F: b[0]= 0x7f; N= 1; break;
    case PC_SCANCODE_7E: b[0]= 0x7e; N= 1; break;
    case PC_SCANCODE_E1_14_77_E1_F0_14_F0_77:
      b[0]= 0xe1; b[1]= 0x14; b[2]= 0x77;
      b[3]= 0xe1; b[4]= 0xf0; b[5]= 0x14;
      b[6]= 0xf0; b[7]= 0x77;
      N= 8;
      break;
    case PC_SCANCODE_E0_7E_E0_C6:
      if ( !break_code )
        {
          b[0]= 0xe0; b[1]= 0x7e; b[2]= 0xe0; b[3]= 0xc6;
          N= 4;
        }
      else
        {
          b[0]= 0xe0; b[1]= 0xf0; b[2]= 0x7e; b[3]= 0xe0; b[4]= 0xf0; b[5]=0xc6;
          N= 6;
        }
      break;
    case PC_SCANCODE_0E: b[0]= 0x0e; N= 1; break;
    case PC_SCANCODE_16: b[0]= 0x16; N= 1; break;
    case PC_SCANCODE_1E: b[0]= 0x1e; N= 1; break;
    case PC_SCANCODE_26: b[0]= 0x26; N= 1; break;
    case PC_SCANCODE_25: b[0]= 0x25; N= 1; break;
    case PC_SCANCODE_2E: b[0]= 0x2e; N= 1; break;
    case PC_SCANCODE_36: b[0]= 0x36; N= 1; break;
    case PC_SCANCODE_3D: b[0]= 0x3d; N= 1; break;
    case PC_SCANCODE_3E: b[0]= 0x3e; N= 1; break;
    case PC_SCANCODE_46: b[0]= 0x46; N= 1; break;
    case PC_SCANCODE_45: b[0]= 0x45; N= 1; break;
    case PC_SCANCODE_4E: b[0]= 0x4e; N= 1; break;
    case PC_SCANCODE_55: b[0]= 0x55; N= 1; break;
    case PC_SCANCODE_66: b[0]= 0x66; N= 1; break;
    case PC_SCANCODE_0D: b[0]= 0x0d; N= 1; break;
    case PC_SCANCODE_15: b[0]= 0x15; N= 1; break;
    case PC_SCANCODE_1D: b[0]= 0x1d; N= 1; break;
    case PC_SCANCODE_24: b[0]= 0x24; N= 1; break;
    case PC_SCANCODE_2D: b[0]= 0x2d; N= 1; break;
    case PC_SCANCODE_2C: b[0]= 0x2c; N= 1; break;
    case PC_SCANCODE_35: b[0]= 0x35; N= 1; break;
    case PC_SCANCODE_3C: b[0]= 0x3c; N= 1; break;
    case PC_SCANCODE_43: b[0]= 0x43; N= 1; break;
    case PC_SCANCODE_44: b[0]= 0x44; N= 1; break;
    case PC_SCANCODE_4D: b[0]= 0x4d; N= 1; break;
    case PC_SCANCODE_54: b[0]= 0x54; N= 1; break;
    case PC_SCANCODE_5B: b[0]= 0x5b; N= 1; break;
    case PC_SCANCODE_5A: b[0]= 0x5a; N= 1; break;
    case PC_SCANCODE_58: b[0]= 0x58; N= 1; break;
    case PC_SCANCODE_1C: b[0]= 0x1c; N= 1; break;
    case PC_SCANCODE_1B: b[0]= 0x1b; N= 1; break;
    case PC_SCANCODE_23: b[0]= 0x23; N= 1; break;
    case PC_SCANCODE_2B: b[0]= 0x2b; N= 1; break;
    case PC_SCANCODE_34: b[0]= 0x34; N= 1; break;
    case PC_SCANCODE_33: b[0]= 0x33; N= 1; break;
    case PC_SCANCODE_3B: b[0]= 0x3b; N= 1; break;
    case PC_SCANCODE_42: b[0]= 0x42; N= 1; break;
    case PC_SCANCODE_4B: b[0]= 0x4b; N= 1; break;
    case PC_SCANCODE_4C: b[0]= 0x4c; N= 1; break;
    case PC_SCANCODE_52: b[0]= 0x52; N= 1; break;
    case PC_SCANCODE_5D: b[0]= 0x5d; N= 1; break;
    case PC_SCANCODE_12: b[0]= 0x12; N= 1; break;
    case PC_SCANCODE_61: b[0]= 0x61; N= 1; break;
    case PC_SCANCODE_1A: b[0]= 0x1a; N= 1; break;
    case PC_SCANCODE_22: b[0]= 0x22; N= 1; break;
    case PC_SCANCODE_21: b[0]= 0x21; N= 1; break;
    case PC_SCANCODE_2A: b[0]= 0x2a; N= 1; break;
    case PC_SCANCODE_32: b[0]= 0x32; N= 1; break;
    case PC_SCANCODE_31: b[0]= 0x31; N= 1; break;
    case PC_SCANCODE_3A: b[0]= 0x3a; N= 1; break;
    case PC_SCANCODE_41: b[0]= 0x41; N= 1; break;
    case PC_SCANCODE_49: b[0]= 0x49; N= 1; break;
    case PC_SCANCODE_4A: b[0]= 0x4a; N= 1; break;
    case PC_SCANCODE_59: b[0]= 0x59; N= 1; break;
    case PC_SCANCODE_14: b[0]= 0x14; N= 1; break;
    case PC_SCANCODE_E0_1F: b[0]= 0xe0; b[1]= 0x1f; N= 2; break;
    case PC_SCANCODE_11: b[0]= 0x11; N= 1; break;
    case PC_SCANCODE_29: b[0]= 0x29; N= 1; break;
    case PC_SCANCODE_E0_11: b[0]= 0xe0; b[1]= 0x11; N= 2; break;
    case PC_SCANCODE_E0_27: b[0]= 0xe0; b[1]= 0x27; N= 2; break;
    case PC_SCANCODE_E0_2F: b[0]= 0xe0; b[1]= 0x2f; N= 2; break;
    case PC_SCANCODE_E0_14: b[0]= 0xe0; b[1]= 0x14; N= 2; break;
    case PC_SCANCODE_E0_70: b[0]= 0xe0; b[1]= 0x70; N= 2; break;
    case PC_SCANCODE_E0_6C: b[0]= 0xe0; b[1]= 0x6c; N= 2; break;
    case PC_SCANCODE_E0_7D: b[0]= 0xe0; b[1]= 0x7d; N= 2; break;
    case PC_SCANCODE_E0_71: b[0]= 0xe0; b[1]= 0x71; N= 2; break;
    case PC_SCANCODE_E0_69: b[0]= 0xe0; b[1]= 0x69; N= 2; break;
    case PC_SCANCODE_E0_7A: b[0]= 0xe0; b[1]= 0x7a; N= 2; break;
    case PC_SCANCODE_E0_75: b[0]= 0xe0; b[1]= 0x75; N= 2; break;
    case PC_SCANCODE_E0_6B: b[0]= 0xe0; b[1]= 0x6b; N= 2; break;
    case PC_SCANCODE_E0_72: b[0]= 0xe0; b[1]= 0x72; N= 2; break;
    case PC_SCANCODE_E0_74: b[0]= 0xe0; b[1]= 0x74; N= 2; break;
    case PC_SCANCODE_77: b[0]= 0x77; N= 1; break;
    case PC_SCANCODE_E0_4A: b[0]= 0xe0; b[1]= 0x4a; N= 2; break;
    case PC_SCANCODE_7C: b[0]= 0x7c; N= 1; break;
    case PC_SCANCODE_7B: b[0]= 0x7b; N= 1; break;
    case PC_SCANCODE_6C: b[0]= 0x6c; N= 1; break;
    case PC_SCANCODE_75: b[0]= 0x75; N= 1; break;
    case PC_SCANCODE_7D: b[0]= 0x7d; N= 1; break;
    case PC_SCANCODE_79: b[0]= 0x79; N= 1; break;
    case PC_SCANCODE_6B: b[0]= 0x6b; N= 1; break;
    case PC_SCANCODE_73: b[0]= 0x73; N= 1; break;
    case PC_SCANCODE_74: b[0]= 0x74; N= 1; break;
    case PC_SCANCODE_69: b[0]= 0x69; N= 1; break;
    case PC_SCANCODE_72: b[0]= 0x72; N= 1; break;
    case PC_SCANCODE_7A: b[0]= 0x7a; N= 1; break;
    case PC_SCANCODE_70: b[0]= 0x70; N= 1; break;
    case PC_SCANCODE_71: b[0]= 0x71; N= 1; break;
    case PC_SCANCODE_E0_5A: b[0]= 0xe0; b[1]= 0x5a; N= 2; break;
    default:
      N= 0;
      printf ( "[EE] kbd_buffer_add_scancode - WTF (%d)!!!!\n", sc );
      exit ( EXIT_FAILURE );
    }
  if ( break_code && N <= 2 )
    {
      b[N]= b[N-1];
      b[N-1]= 0xf0;
      ++N;
    }
  kbd_buffer_add_scancodes ( b, N );
    
} // end kbd_buffer_add_scancode


static void
kbd_data_write (
                const uint8_t data
                )
{

  uint8_t tmp[2];


  // No sé molt bé com interpretar aquest apunt que he trobat:
  //
  //  (PS/2) Command 0xad: Disable keyboard
  //
  //    Disable the keyboard clock line and set bit 4 of the Command
  //    byte. Any keyboard command enables the keyboard again.
  //
  if ( !_controller.ports[0].enabled )
    {
      if ( _kbd.state == KBD_WAIT_CMD ) _controller.ports[0].enabled= true;
      else
        _warning ( _udata,
                   "PS/2: s'ha enviat el byte %02X al teclat"
                   " metre este estava desactivat",
                   data );
    }
  
  switch ( _kbd.state )
    {
      
      // Esperant comandament
    case KBD_WAIT_CMD:
      switch ( data )
        {

          // Set LEDs
        case 0xed:
          _kbd.state= KBD_WAIT_ARG__SET_LEDS;
          kbd_buffer_add ( KBD_ACK );
          break;
          
          // Set/Get scancode set
        case 0xf0:
          _kbd.state= KBD_WAIT_ARG__SETGET_SCANCODE_SET;
          kbd_buffer_add ( KBD_ACK );
          break;

          // Read keyboard ID
        case 0xf2:
          kbd_buffer_add ( KBD_ACK );
          // NOTA!!!! No em queda clar el model de teclat i si este
          // està relacionat en si és espanyol o no, però vaig a
          // apostar per un model MF2 AT
          tmp[0]= 0xab;
          tmp[1]= 0x83;
          kbd_buffer_add_scancodes ( tmp, 2 );
          break;
          
          // Keyboard enable
        case 0xf4:
          buffer_clear ( &_kbd.buf );
          _kbd.scan_enabled= true;
          kbd_buffer_add ( KBD_ACK );
          break;
          
          // Set defaults and disable keyboard
        case 0xf5:
          kbd_set_defaults ();
          _kbd.scan_enabled= false;
          kbd_buffer_add ( KBD_ACK );
          break;
          
          // Reset and start self-test
        case 0xff:
          kbd_reset ();
          kbd_buffer_add ( KBD_ACK );
          kbd_buffer_add ( 0xaa ); // self-test passed
          break;
        default:
          printf("[EE] PS2 - kbd_data_write - no implementat byte: %02X\n",
                 data);
          exit(EXIT_FAILURE);
        }
      break;

      // Set/Get scancode set
    case KBD_WAIT_ARG__SETGET_SCANCODE_SET:
      kbd_buffer_add ( KBD_ACK );
      if ( data == 0x00 )
        {
          tmp[0]= _kbd.scancode_set;
          kbd_buffer_add_scancodes ( tmp, 1 );
        }
      else if ( data <= 0x03 )
        {
          _kbd.scancode_set= data;
          if ( _kbd.scancode_set != 0x2 )
            {
              printf("[EE] ps2.c kbd_data_write - scanode_set: %d no suportat\n!!!",_kbd.scancode_set);
              exit(EXIT_FAILURE);
            }
        }
      else
        {
          _warning ( _udata, "ps2 - kbd_data_write - unknown command F0 %02X",
                     data );
          kbd_buffer_add ( KBD_NACK );
        }
      _kbd.state= KBD_WAIT_CMD;
      break;

      // Set LEDs
    case KBD_WAIT_ARG__SET_LEDS:
      kbd_buffer_add ( KBD_ACK );
      _warning ( _udata,
                 "ps2 - kbd_data_write - LED.ScrollLock:%d"
                 " LED.NumberLock:%d LED.CapsLock:%d",
                 data&0x1, (data&02)!=0, (data&0x4)!=0 );
      _kbd.state= KBD_WAIT_CMD;
      break;
      
    default:
      kbd_buffer_add ( KBD_NACK );
      printf("[EE] WTF - kbd_data_write !!!!\n");
      exit(EXIT_FAILURE);
    }
  
} // end kbd_data_write


static void
mouse_set_sampling_rate (
                         const uint8_t srate
                         )
{

  if ( srate == 10 || srate == 20 || srate == 40 || srate == 80 ||
       srate == 100 || srate == 200 )
    {
      _timing.cc_mouse_sample_rate=
        _timing.cc_mouse_duration*(1200/srate);
      // Reajusta cicles
      if ( _timing.cc_mouse >= _timing.cc_mouse_sample_rate )
        _timing.cc_mouse%= _timing.cc_mouse_sample_rate;
    }
  else _warning ( _udata, "[PS2::MOUSE] invalid sampling rate: %d", srate );
  
} // end mouse_set_sampling_rate


static void
mouse_reset (void)
{

  buffer_clear ( &_mouse.buf );
  _mouse.stream_mode= false;
  _mouse.state= MOUSE_WAIT_CMD;
  _mouse.resolution= 4;
  mouse_set_sampling_rate ( 100 );
  
} // end mouse_reset


static void
mouse_init (void)
{

  mouse_reset ();
  /* // Possibles futures millores
  _mouse.motion.old_dx= 0;
  _mouse.motion.old_dy= 0;
  _mouse.motion.thrx= _host_mouse.sensitivity;
  _mouse.motion.thry= _host_mouse.sensitivity;
  */
  _mouse.motion.dx= 0.0;
  _mouse.motion.dy= 0.0;
  _mouse.buttons= 0x00;
  _mouse.last_buttons= 0x00;
  
} // end mouse_init


static void
mouse_buffer_add (
                  const uint8_t data
                  )
{
  
  if ( !buffer_add ( &_mouse.buf, data ) )
    _warning ( _udata, "PS/2: buffer ratolí ple " );
  
} // end mouse_buffer_add


static void
mouse_data_write (
                  const uint8_t data
                  )
{

  //uint8_t tmp[2];

  
  // No sé molt bé com interpretar el enabled. Faig el mateix que en el teclat.
  if ( !_controller.ports[1].enabled )
    {
      if ( _mouse.state == MOUSE_WAIT_CMD ) _controller.ports[1].enabled= true;
      else
        _warning ( _udata,
                   "PS/2: s'ha enviat el byte %02X al ratolí"
                   " metre este estava desactivat",
                   data );
    }
  
  switch ( _mouse.state )
    {
      
      // Esperant comandament
    case MOUSE_WAIT_CMD:
      switch ( data )
        {
          
          // Set mouse resolution
        case 0xe8:
          _mouse.state= MOUSE_WAIT_ARG__SET_MOUSE_RESOLUTION;
          mouse_buffer_add ( MOUSE_ACK );
          break;

          // Read mouse ID
        case 0xf2:
          mouse_buffer_add ( MOUSE_ACK );
          mouse_buffer_add ( 0x00 );
          /*
          mouse_buffer_add ( 0x03 ); // <-- Intellimouse / Mouse with
                                     //     scroll wheel
                                     */
          
          break;

          // Set mouse sample rate
        case 0xf3:
          _mouse.state= MOUSE_WAIT_ARG__SET_MOUSE_SAMPLE_RATE;
          mouse_buffer_add ( MOUSE_ACK );
          break;
          // Mouse enable
        case 0xf4:
          mouse_buffer_add ( MOUSE_ACK );
          _mouse.stream_mode= true;
          break;
          
          // Mouse disable
        case 0xf5:
          mouse_buffer_add ( MOUSE_ACK );
          _mouse.stream_mode= false;
          break;
          
          // Reset and start self-test
        case 0xff:
          mouse_reset ();
          mouse_buffer_add ( MOUSE_ACK );
          mouse_buffer_add ( 0xaa );
          mouse_buffer_add ( 0x00 );
          break;
          
        default:
          printf("[EE] PS2 - mouse_data_write - no implementat byte: %02X\n",
                 data);
          exit(EXIT_FAILURE);
        }
      break;

      //  Set mouse resolution
    case MOUSE_WAIT_ARG__SET_MOUSE_RESOLUTION:
      mouse_buffer_add ( MOUSE_ACK );
      switch ( data )
        {
        case 0: _mouse.resolution= 1; break;
        case 1: _mouse.resolution= 2; break;
        case 2: _mouse.resolution= 4; break;
        case 3: _mouse.resolution= 8; break;
        default:
          _warning ( _udata,
                     "ps2 - mouse_data_write - Set Mouse Resolution -"
                     " valor no suportat %02X", data );
        }
      _mouse.state= MOUSE_WAIT_CMD;
      break;

      // Set mouse sample rate
    case MOUSE_WAIT_ARG__SET_MOUSE_SAMPLE_RATE:
      mouse_buffer_add ( MOUSE_ACK );
      mouse_set_sampling_rate ( data );
      _mouse.state= MOUSE_WAIT_CMD;
      break;
      
    default:
      mouse_buffer_add ( MOUSE_NACK );
      printf("[EE] WTF - kbd_data_write !!!!\n");
      exit(EXIT_FAILURE);
    }
  
} // end mouse_data_write


static void
mouse_get_sample (void)
{

  uint8_t b;
  int16_t x,y;
  float tmp;
  bool ox,oy;

  
  // Prepara x
  tmp= _mouse.motion.dx*_mouse.resolution;
  if ( _mouse.motion.dx >= 0 )
    {
      if ( tmp < _mouse.motion.dx || tmp > 255.0 )
        { x= 0; ox= true; _mouse.motion.dx= 0.0; }
      else
        {
          x= (int16_t) tmp;
          ox= false;
          _mouse.motion.dx-= ((float) x)/_mouse.resolution;
        }
    }
  else
    {
      if ( tmp > _mouse.motion.dx || tmp < -256.0 )
        { x= 0; ox= true; _mouse.motion.dx= 0.0; }
      else
        {
          x= (int16_t) tmp;
          ox= false;
          _mouse.motion.dx-= ((float) x)/_mouse.resolution;
        }
    }
  
  // Prepara y
  tmp= _mouse.motion.dy*_mouse.resolution;
  if ( _mouse.motion.dy >= 0 )
    {
      if ( tmp < _mouse.motion.dy || tmp > 255.0 )
        { y= 0; oy= true; _mouse.motion.dy= 0.0; }
      else
        {
          y= (int16_t) tmp;
          oy= false;
          _mouse.motion.dy-= ((float) y)/_mouse.resolution;
        }
    }
  else
    {
      if ( tmp > _mouse.motion.dy || tmp < -256.0 )
        { y= 0; oy= true; _mouse.motion.dy= 0.0; }
      else
        {
          y= (int16_t) tmp;
          oy= false;
          _mouse.motion.dy-= ((float) y)/_mouse.resolution;
        }
    }
  
  if ( _mouse.last_buttons != _mouse.buttons ||
       x > 0 || x < 0 || y > 0 || y < 0 )
    {
      // Byte inicial.
      b=
        (oy ? 0x80 : 0x00) | // Y overflow
        (ox ? 0x40 : 0x00) | // X overflow
        (y<0 ? 0x20 : 0x00) | // bit 9 y
        (x<0 ? 0x10 : 0x00) | // bit 9 x
        0x08 |
        _mouse.buttons;
      mouse_buffer_add ( b );
      // x
      mouse_buffer_add ( (uint8_t) (x>=0 ? x : 256+x) );
      // y
      mouse_buffer_add ( (uint8_t) (y>=0 ? y : 256+y) );
      
    }
  
  // Prepara següent.
  _mouse.last_buttons= _mouse.buttons;
  
} // end mouse_get_sample


static void
run_pending_command (void)
{

  uint8_t data;


  // NOTA!!! El controlador (depenent del firmware) podria generar un
  // IRQ quan dese un valor en el outbuff. Per simplificat he decidit
  // no generar-lo.
  
  data= _controller.inbuff;
  switch ( _controller.pending_cmd )
    {
      
      // PS/2 Controller Configuration Byte
    case 0x60:
      _controller.ports[0].irq_enabled= ((data&0x01)!=0);
      _controller.ports[1].irq_enabled= ((data&0x02)!=0);
      _controller.system_flag= ((data&0x04)!=0);
      _controller.ports[0].clock_enabled= ((data&0x10)==0);
      if(_controller.ports[0].clock_enabled)
        PC_MSG("PS/2 - First PS/2 port clock");
      _controller.ports[1].clock_enabled= ((data&0x20)==0);
      if(_controller.ports[1].clock_enabled)
        PC_MSG("PS/2 - Second PS/2 port clock");
      _controller.ports[0].translation_enabled= ((data&0x40)!=0);
      _controller.waiting= WAITING_NONE;
      break;
      
    default:
      printf("[EE] PS2 - run_pending_command - unknown command: %X next: %X\n",
             _controller.pending_cmd,_controller.inbuff);
      exit(EXIT_FAILURE);
    }
  _controller.inbuff_full= false;
  
} // run_pending_command


// Torna cert si ha conseguit ficar un byte
static bool
fill_outbuff (void)
{

  bool ret;


  // Si està ple espera a que es buide.
  // IMPORTAT!!! Açò és fonamental per a un bon funcionament. És a
  // dir, el controlador no permet mai que es fique un altre byte en
  // el buffer si no s'ha fet abans una lectura o s'ha buidad. Per
  // tant no es farà una IRQ fins que no es llisca.
  if ( _controller.outbuff_full ) return false;
  
  ret= false;
  //prev_state= _controller.outbuff_full;
  
  // Primer intenta del teclat
  if ( _kbd.buf.N > 0 )
    {
      _controller.outbuff_full= true;
      _controller.outbuff= buffer_read ( &_kbd.buf );
      _controller.outbuff_source= 0;
      ret= true;
    }

  // Segon el dispositiu
  else if ( _mouse.buf.N > 0 )
    {
      _controller.outbuff_full= true;
      _controller.outbuff= buffer_read ( &_mouse.buf );
      _controller.outbuff_source= 1;
      ret= true;
    }

  /*
  if (  prev_state && ret )
    _warning ( _udata,
               "PS/2: s'ha sobreescrit un byte en el outbuff!!" );
  */
  
  return ret;
  
} // end fill_outbuff


static void
clock (
       const bool update_cc2event
       )
{
  
  int cc,tmp,clocks,n,key;
  bool irq_kbd,irq_mouse;
  
  
  // Processa cicles
  cc= PC_Clock-_timing.cc_used;
  if ( cc > 0 ) { _timing.cc+= cc; _timing.cc_used+= cc; }
  
  // Processa clocks
  clocks= _timing.cc;
  _timing.cc= 0;
  tmp= clocks + _timing.cc_bit_counter_pendent;
  if ( _timing.bit_counter != -1 )
    _timing.bit_counter+= tmp/_timing.ccperbit;
  _timing.cc_bit_counter_pendent= tmp%_timing.ccperbit;
  _timing.cc_mouse+= clocks*3; // 1/3 de cicles de CPU

  // Abans de fer res, sampleja el ratolí.
  if ( _timing.cc_mouse >= _timing.cc_mouse_sample_rate )
    {
      _timing.cc_mouse%= _timing.cc_mouse_sample_rate;
      // Sampleja mostra
      if ( _mouse.stream_mode ) mouse_get_sample ();
    }
  
  // Transmiteix bits a outbuff
  // NOTA!! no sé molt bé com gestionar el tema de les interrupcions
  // ací. Vaig a forçar un pols
  irq_kbd= irq_mouse= false;
  if ( _timing.bit_counter >= 8 && (_kbd.buf.N > 0 || _mouse.buf.N > 0) )
    {
      _timing.bit_counter-= 8;
      if ( fill_outbuff () )
        {
          if ( _controller.outbuff_source == 0 )
            {
              if ( _controller.ports[0].irq_enabled )
                {
                  if ( !irq_kbd )
                    { PC_ic_irq ( 1, true );
                      PC_ic_irq ( 1, false ); }
                  irq_kbd= true;
                }
            }
          else
            {
              if ( _controller.ports[1].irq_enabled )
                {
                  if ( !irq_mouse )
                    { PC_ic_irq ( 12, true );
                      PC_ic_irq ( 12, false ); }
                  irq_mouse= true;
                }
            }
        }
      if ( _kbd.buf.N == 0 && _mouse.buf.N == 0 )
        _timing.bit_counter= -1;
    }
  assert ( _timing.bit_counter < 8 );

  // Repetició de tecles
  for ( n= 0; n < _kbd.keys.N; ++n )
    {
      key= _kbd.keys.as[n];
      _kbd.keys.v[key].cc-= clocks;
      while ( _kbd.keys.v[key].cc <= 0 )
        {
          _kbd.keys.v[key].cc+= _kbd.repeat_rate_cc;
          _kbd.keys.v[key].state= KEY_WAIT_REPEAT;
          kbd_buffer_add_scancode ( key, false );
        }
    }
  
  // Actualitza cctoEvent
  if ( update_cc2event )
    update_cc_to_event ();
  
} // end clock




/**********************/
/* FUNCIONS PÚBLIQUES */
/**********************/

void
PC_ps2_init (
             PC_Warning      *warning,
             void            *udata,
             const PC_Config *config
             )
{

  int i;

  
  // Callbacks.
  _warning= warning;
  _udata= udata;
  _host_mouse= config->host_mouse;
  if ( _host_mouse.resolution <= 0.0 )
    {
      _host_mouse.resolution= 1.0;
      _warning ( _udata, "Host mouse resolution set to %f",
                 _host_mouse.resolution );
    }
  /* // Possibles futures millores.
  if ( _host_mouse.sensitivity < 0 )
    {
      _host_mouse.resolution= 0;
      _warning ( _udata, "Host mouse sensitivity set to %d",
                 _host_mouse.sensitivity );
    }
  if ( _host_mouse.acceleration <= 0.0 )
    {
      _host_mouse.acceleration= 1.0;
      _warning ( _udata, "Host mouse acceleration set to %f",
                 _host_mouse.acceleration );
    }
  */
  
  // Inicialitza el controlador.
  _controller.inbuff= 0x00;
  _controller.outbuff= 0x00;
  _controller.inbuff_full= false;
  _controller.outbuff_full= false;
  _controller.outbuff_source= 0;
  _controller.data_is_command= false;
  _controller.system_flag= false;
  for ( i= 0; i < 2; ++i )
    {
      _controller.ports[i].enabled= false;
      _controller.ports[i].irq_enabled= false;
      _controller.ports[i].clock_enabled= false;
      _controller.ports[i].translation_enabled= false;
    }
  _controller.waiting= WAITING_NONE;
  _controller.pending_cmd= 0x00;
  
  // Keyboard.
  kbd_init ();
  
  // Timing.
  _timing.cc_used= 0;
  assert ( PC_ClockFreq%10000 == 0 );
  _timing.ccperbit= PC_ClockFreq/10000;
  _timing.cc= 0;
  _timing.cctoEvent= 0;
  _timing.bit_counter= -1;
  _timing.cc_bit_counter_pendent= 0;
  assert ( PC_ClockFreq%400 == 0 );
  _timing.cc_mouse_duration= PC_ClockFreq/400;
  _timing.cc_mouse= 0;

  // Mouse. Important fer-ho després d'inicialitzar cc_mouse_duration.
  mouse_init ();
  
  update_cc_to_event ();
  
} // end PC_ps2_init


void
PC_ps2_reset (void)
{

  int i;

  
  clock ( false );
  
  // Inicialitza el controlador.
  _controller.inbuff= 0x00;
  _controller.outbuff= 0x00;
  _controller.inbuff_full= false;
  _controller.outbuff_full= false;
  _controller.outbuff_source= 0;
  _controller.data_is_command= false;
  _controller.system_flag= false;
  for ( i= 0; i < 2; ++i )
    {
      _controller.ports[i].enabled= false;
      _controller.ports[i].irq_enabled= false;
      _controller.ports[i].clock_enabled= false;
      _controller.ports[i].translation_enabled= false;
    }
  _controller.waiting= WAITING_NONE;
  _controller.pending_cmd= 0x00;
  
  // Keyboard.
  kbd_init ();
  
  // Mouse
  mouse_init ();
  
  // Timing.
  _timing.cctoEvent= 0;
  _timing.bit_counter= -1;
  _timing.cc_bit_counter_pendent= 0;
  
  update_cc_to_event ();
  
} // end PC_ps2_reset


int
PC_ps2_next_event_cc (void)
{

  int tmp;


  tmp= _timing.cctoEvent - _timing.cc;
  assert ( tmp > 0 );

  return tmp;
  
} // end PC_ps2_next_event_cc


void
PC_ps2_end_iter (void)
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
  
} // end PC_ps2_end_iter


void
PC_ps2_data_write (
                   const uint8_t data
                   )
{
  
  clock ( false );
  
  _controller.inbuff= data;
  _controller.inbuff_full= true;
  _controller.data_is_command= false;
  switch ( _controller.waiting )
    {
    case WAITING_NEXT_COMMAND_BYTE:
      _controller.data_is_command= true;
      run_pending_command ();
      break;
    case WAITING_BYTE_PORT2:
      _controller.waiting= WAITING_NONE;
      mouse_data_write ( data );
      _controller.inbuff_full= false; // Preferisc fer-ho en plan
                                      // redundant perquè quede clar
      break;
    case WAITING_BYTE_OUTPUT_PORT:
      _controller.waiting= WAITING_NONE;
      PC_MSGF("ps2 - PC_ps2_data_write:"
              " WRITE CONTROLLER OUTPUT PORT: %X",data);
      _controller.inbuff_full= false; // Preferisc fer-ho en plan
                                      // redundant perquè quede clar
      break;
    case WAITING_NONE:
    default:
      kbd_data_write ( data );
      _controller.inbuff_full= false; // Preferisc fer-ho en plan
                                      // redundant perquè quede clar
    }
  update_cc_to_event ();
  
} // end PC_ps2_data_write


uint8_t
PC_ps2_data_read (void)
{

  uint8_t ret;


  clock ( false );

  // NOTA!!! Vaig a fer que sempre torne l'últim byte del buffer. No
  // s'esborra
  ret= _controller.outbuff;
  _controller.outbuff_full= false;
  /*
  if ( _controller.outbuff_full )
    {
      ret= _controller.outbuff;
      _controller.outbuff_full= false;
    }
  else
    {
      ret= 0xff;
      _warning ( _udata,
                 "PC_ps2_data_read - s'ha intentat llegir un byte"
                 " del buffer però està buit " );
    }
  */
  update_cc_to_event ();
  
  return ret;
  
} // end PC_ps2_data_read


uint8_t
PC_ps2_status (void)
{

  uint8_t ret;

  
  clock ( true );

  // NOTA!!! Parity error i time-out error són errors de hardware que
  // no vaig a permetre en el meu simulador. Així que sempre estan a
  // 0.
  
  ret=
    0x00 |
    // 7 - Parity error (0 = no error, 1 = parity error)
    // 6 - Time-out error (0 = no error, 1 = time-out error)
    // 5 - Auxiliary output buffer full
    (_controller.outbuff_source==1 ? 0x20 : 0x00) |
    // 4 - Unknown (chipset specific)
    // 3 - Command/data
    (_controller.data_is_command ? 0x08 : 0x00) |
    // 2 - System Flag
    (_controller.system_flag ? 0x04 : 0x00) |
    // 1 - Input buffer status
    (_controller.inbuff_full ? 0x02 : 0x00) |
    // 0 - Output buffer status
    (_controller.outbuff_full ? 0x01 : 0x00)
    ;
  
  return ret;
  
} // PC_ps2_status


void
PC_ps2_command (
                const uint8_t data
                )
{
  
  clock ( false );

  // NOTA!!! El controlador (depenent del firmware) podria generar un
  // IRQ quan dese un valor en el outbuff. Per simplificat he decidit
  // no generar-lo.

  _controller.waiting= WAITING_NONE;
  switch ( data )
    {

      // Write next byte to "byte 0" of internal RAM (Controller
      // Configuration Byte, see below)
    case 0x60:
      _controller.waiting= WAITING_NEXT_COMMAND_BYTE;
      _controller.pending_cmd= data;
      break;
      
      // Disable second PS/2 port
    case 0xa7:
      _controller.ports[1].enabled= false;
      break;

      // Test PS/2 Controller
    case 0xaa:
      _controller.outbuff= 0x55; // Test passed
      _controller.outbuff_full= true;
      break;
      // Test first PS/2 port
    case 0xab:
      _controller.outbuff= 0x00; // Test passed
      _controller.outbuff_full= true;
      break;
      
      // Disable first PS/2 port 
    case 0xad:
      _controller.ports[0].enabled= false;
      break;
      // Enable first PS/2 port 
    case 0xae:
      _controller.ports[0].enabled= true;
      break;

      // Write next byte to Controller Output Port
    case 0xd1:
      _controller.waiting= WAITING_BYTE_OUTPUT_PORT;
      break;
      
      // Write next byte to second PS/2 port input buffer.
    case 0xd4:
      _controller.waiting= WAITING_BYTE_PORT2;
      break;
      
    default:
      printf("[EE] PS2 - PC_ps2_command - unknown command: %X\n",data);
      exit(EXIT_FAILURE);
    }

  update_cc_to_event ();
  
} // PC_ps2_command


void
PC_kbd_press (
              const PC_Scancode key
              )
{
  
  clock ( false );

  if ( _kbd.scan_enabled &&
       key != PC_SCANCODE_ALL &&
       _kbd.keys.v[key].state == KEY_RELEASED )
    {
      _kbd.keys.as[_kbd.keys.N]= key;
      _kbd.keys.v[key].pos_as= _kbd.keys.N++;
      _kbd.keys.v[key].state= KEY_WAIT_FIRST_REPEAT;
      _kbd.keys.v[key].cc= _kbd.delay_cc;
      kbd_buffer_add_scancode ( key, false );
    }
  
  update_cc_to_event ();
  
} // end PC_kbd_press


void
PC_kbd_release (
                const PC_Scancode key
                )
{

  int last_akey;

  
  clock ( false );
  
  if ( _kbd.scan_enabled &&
       key != PC_SCANCODE_ALL &&
       _kbd.keys.v[key].state != KEY_RELEASED )
    {
      assert ( _kbd.keys.N > 0 );
      // --> Fique l'últim element de la llista en la posició de key
      last_akey= _kbd.keys.as[--_kbd.keys.N];
      _kbd.keys.v[last_akey].pos_as= _kbd.keys.v[key].pos_as;
      _kbd.keys.as[_kbd.keys.v[key].pos_as]= last_akey;
      // --> Canvie estat de key
      _kbd.keys.v[key].state= KEY_RELEASED;
      // --> Inserta amb break_code
      if ( key != PC_SCANCODE_E1_14_77_E1_F0_14_F0_77 )
        kbd_buffer_add_scancode ( key, true );
    }
  
  update_cc_to_event ();
  
} // end PC_kbd_release


void
PC_kbd_clear (void)
{

  clock ( false );
  kbd_clear_keys ();
  update_cc_to_event ();
  
} // PC_kbd_clear


void
PC_mouse_motion (
                 int32_t deltax,
                 int32_t deltay
                 )
{
  
  float tmp,dx,dy;
  //bool is_neg;

  
  clock ( false );

  /* // Possibles futures millores
  // deltax -> dx
  // --> Comprova canvi sentit
  if ( deltax == 0 ||
       (deltax > 0 && _mouse.motion.old_dx < 0) ||
       (deltax < 0 && _mouse.motion.old_dx > 0 ) )
    _mouse.motion.thrx= _host_mouse.sensitivity;
  _mouse.motion.old_dx= deltax;
  // --> Passa a positiu
  if ( deltax >= 0 ) is_neg= false;
  else              { is_neg= true; deltax= -deltax; }
  // --> Passa a mm
  if ( deltax > _mouse.motion.thrx )
    {
      dx=
        (_mouse.motion.thrx +
         (deltax-_mouse.motion.thrx)/_host_mouse.acceleration) /
        _host_mouse.resolution;
      _mouse.motion.thrx= 0;
    }
  else
    {
      dx= deltax/_host_mouse.resolution;
      _mouse.motion.thrx-= deltax;
    }
  // --> canvia signe
  if ( is_neg ) dx= -dx;

  // deltay -> dy
  // --> Comprova canvi sentit
  if ( deltay == 0 ||
       (deltay > 0 && _mouse.motion.old_dy < 0) ||
       (deltay < 0 && _mouse.motion.old_dy > 0 ) )
    _mouse.motion.thry= _host_mouse.sensitivity;
  _mouse.motion.old_dy= deltay;
  // --> Passa a positiu
  if ( deltay >= 0 ) is_neg= false;
  else              { is_neg= true; deltay= -deltay; }
  // --> Passa a mm
  if ( deltay > _mouse.motion.thry )
    {
      dy=
        (_mouse.motion.thry +
         (deltay-_mouse.motion.thry)/_host_mouse.acceleration) /
        _host_mouse.resolution;
      _mouse.motion.thry= 0;
    }
  else
    {
      dy= deltay/_host_mouse.resolution;
      _mouse.motion.thry-= deltay;
    }
  // --> canvia signe
  if ( !is_neg ) dy= -dy; // Al inrevés
  */

  // Desta -> d
  dx= deltax/_host_mouse.resolution;
  dy= -deltay/_host_mouse.resolution;
  
  // Actualitza dx
  tmp= _mouse.motion.dx;
  _mouse.motion.dx+= dx;
  if ( dx < 0 && _mouse.motion.dx > tmp )
    _mouse.motion.dx= -FLT_MAX;
  else if ( dx > 0 && _mouse.motion.dx < tmp )
    _mouse.motion.dx= FLT_MAX;
  
  // Actualitza dy
  tmp= _mouse.motion.dy;
  _mouse.motion.dy+= dy;
  if ( dy < 0 && _mouse.motion.dy > tmp )
    _mouse.motion.dy= -FLT_MAX;
  else if ( dy > 0 && _mouse.motion.dy < tmp )
    _mouse.motion.dy= FLT_MAX;
  
  update_cc_to_event ();
  
} // end PC_mouse_motion


void
PC_mouse_button_press (
                       const PC_MouseButton but
                       )
{

  clock ( false );
  _mouse.buttons|= but;
  update_cc_to_event ();
  
} // end PC_mouse_button_press


void
PC_mouse_button_release (
                         const PC_MouseButton but
                         )
{
  
  clock ( false );
  _mouse.buttons&= ~but;
  update_cc_to_event ();
  
} // end PC_mouse_button_release


void
PC_mouse_motion_clear (void)
{

  clock ( false );
  /* // Possibles futures millores
  _mouse.motion.old_dx= 0;
  _mouse.motion.old_dy= 0;
  _mouse.motion.thrx= _host_mouse.sensitivity;
  _mouse.motion.thry= _host_mouse.sensitivity;
  */
  _mouse.motion.dx= 0;
  _mouse.motion.dy= 0;
  _mouse.buttons= 0x00;
  _mouse.last_buttons= 0x00;
  update_cc_to_event ();
  
} // end PC_mouse_motion_clear

void
PC_set_host_mouse (
                   PC_HostMouse host_mouse
                   )
{

  clock ( false );

  _host_mouse= host_mouse;
  update_cc_to_event ();
  
} // end PC_set_host_mouse
