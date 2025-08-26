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
 *  ic.c - Implementació del Interrupt Controller.
 *
 */


#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "PC.h"




/*********/
/* ESTAT */
/*********/

// Callbacks.
static PC_Warning *_warning;
static PC_InterruptionServiced *_int_serviced;
static void *_udata;
static bool _trace_enabled;

// Estat controladors
static struct
{
  enum {
    WAIT_ICW1,
    WAIT_ICW2,
    WAIT_ICW3,
    WAIT_ICW4,
    INITIALIZED
  } step;
  uint8_t IRR; // Interrupt Request Register
  uint8_t ISR; // In-Service Register
  uint8_t IMR; // Interrupt Mask Register
  uint8_t input;
  uint8_t vector_base;
  int     priority[8]; // L'índex és la prioritat i el valor el IRQ.
  int     last_irq;
  bool    out;
  bool    rotate_in_auto_eoi; // NO IMPLEMENTAT AUTO EOI. Ara no fa res.
  bool    special_mask_mode;
  enum {
    READ_CMD_IRR= 0,
    READ_CMD_ISR
  }       read_cmd;
} _s[2];

// Estat PCI programmable interrupts
static struct
{
  uint8_t reg;
  bool    enabled;
  int     irq; // -1 s'ignora
} _pci[4];

// Registres ELCR, controlen si els IRQ són per nivell o edge. 1-Level
// Triggered; 0-Edge Triggered
static uint8_t _elcr[2];




/*********************/
/* FUNCIONS PRIVADES */
/*********************/

static void
init_pci (void)
{

  int i;


  for ( i= 0; i < 4; ++i )
    {
      _pci[i].reg= 0x80;
      _pci[i].enabled= false;
      _pci[i].irq= -1;
    }
  
} // end init_pci


// Pot ser IRQ2
// NOTA!! Sols afecta a IRR en cas de edge_triggered mode i a input.
static void
input_changed (
               const int  irq,
               const bool in
               )
{
  
  uint8_t mask;
  int id;


  // Activa el IRR corresponent
  id= irq/8;
  mask= (0x1<<(irq%8));
  if ( in )
    {
      // NOTA!! Açò no és del tot redundant amb el que es fa en
      // update_state. Ací és l'únic lloc on es pot detectar el
      // edge/triggered. Mentre que en update_state es detecten tots
      // els leveled.
      // O no és edge/triggered o hi ha un edge/triggered
      if ( (_elcr[id]&mask)!=0x00 || (_s[id].input&mask)==0x00 )
        _s[id].IRR|= mask;
      _s[id].input|= mask;
    }
  else _s[id].input&= ~mask;
  
} // end input_changed


static void
nonspecific_eoi_fully_nested (
                              const int  icid,
                              const bool rotate
                              )
{

  int j,k,irq;
  uint8_t mask;

  
  // Sols fully nested i sense automatic
  if ( _s[icid].ISR != 0 )
    {
      for ( j= 0; j < 8; ++j )
        {
          irq= _s[icid].priority[j];
          mask= 0x01<<irq;
          if ( (_s[icid].ISR&mask)!=0x00 )
            {
              _s[icid].ISR&= ~mask;
              if ( rotate )
                for ( k= 0; k < 8; ++k )
                  {
                    _s[icid].priority[k]= (irq+1+k)%8;
                    PC_MSG("ROTATE FET PERÒ NO FICAT EN MARXA, ESTE PRINT ES PER COMPROVAR QUE VA BÉ!!");
                    //printf("%d -> %d\n",k,_s[icid].priority[k]);
                    exit(EXIT_FAILURE);
                  }
              return;
            }
        }
    }
  
} // end nonspecific_eoi_fully_nested


static void
nonspecific_eoi (
                 const int  icid,
                 const bool rotate
                 )
{
  // Sols fully nested i sense automatic
  nonspecific_eoi_fully_nested ( icid, rotate );
} // end nonspecific_eoi


static void
specific_eoi (
              const int  icid,
              const int  irq,
              const bool rotate
              )
{

  int k;
  uint8_t mask;

  
  mask= 0x01<<irq;
  if ( (_s[icid].ISR&mask)!=0x00 )
    {
      _s[icid].ISR&= ~mask;
      if ( rotate )
        for ( k= 0; k < 8; ++k )
          {
            _s[icid].priority[k]= (irq+1+k)%8;
            PC_MSG("ROTATE (B) FET PERÒ NO FICAT EN MARXA, ESTE PRINT ES PER COMPROVAR QUE VA BÉ!!");
            //printf("%d -> %d\n",k,_s[icid].priority[k]);
            exit(EXIT_FAILURE);
          }
    }
  
} // end specific_eoi


// Actualitza el valor d'eixida d'un xip i es desa l'irq que està
// provocant l'eixida a 1 si és el cas.
// NOTA!! Special fully nested mode no implementat !!!
// NOTA!! Automatic End of Interruption no implementat !!!
static void
update_out_fully_nested (
                         const int id
                         )
{

  int i,irq;
  uint8_t mask;
  bool out;
  

  // Torna a comprovar els leveled. Té sentit perquè igual ha canviat
  // el elcr.
  mask= 0x01;
  for ( i= 0; i < 8; ++i )
    {
      if ( (_elcr[id]&mask)!=0x00 && (_s[id].input&mask)!=0x00 )
        _s[id].IRR|= mask;
      mask<<= 1;
    }
  
  // Intenta generar interrupció.
  out= false;
  for ( i= 0; i < 8 && !out; ++i )
    {
      irq= _s[id].priority[i];
      mask= 0x1<<irq;
      if ( (_s[id].ISR&mask) != 0x00 )
        {
          if ( _s[id].special_mask_mode ) continue;
          else                            break;
        }
      if ( ((_s[id].IRR&(~_s[id].IMR))&mask) != 0x00 )
        {
          out= true;
          _s[id].last_irq= irq;
        }
    }
  _s[id].out= out;
  
} // end update_out_fully_nested


static void
update_fully_nested (void)
{

  // NOTA!! Automatic End of Interruption no implementat !!!
  
  // Comprova xips per ordre: slave -> master
  update_out_fully_nested ( 1 );
  input_changed ( 2, _s[1].out );
  update_out_fully_nested ( 0 );
  IA32_set_intr ( &PC_CPU, _s[0].out );
  IA32_jit_set_intr ( PC_CPU_JIT, _s[0].out );
  
} // end update_fully_nested


static void
update_state (void)
{
  // Sols fully nested i sense automatic
  update_fully_nested ();
} // end update_state


static uint8_t
ack_intr_fully_nested (void)
{

  // NOTA!! Automatic End of Interruption no implementat !!!
  
  uint8_t vec,mask;
  int irq;
  
  
  // Assumim que si estem ací és perquè ha ocorregut una excepció.
  if ( _s[0].last_irq == 2 ) // Slave
    {
      // ACK IRQ2 master
      mask= 0x1<<(2);
      _s[0].IRR&= ~mask;
      _s[0].ISR|= mask;
      // ACK IRQ slave
      irq= _s[1].last_irq;
      mask= 0x1<<(irq);
      _s[1].IRR&= ~mask;
      _s[1].ISR|= mask;
      // Calcula vector
      vec= _s[1].vector_base | (uint8_t) irq;
      if ( _trace_enabled && _int_serviced != NULL )
        _int_serviced ( irq + 8, vec );
    }
  else // Master
    {
      // ACK IRQ
      irq= _s[0].last_irq;
      mask= 0x1<<(irq);
      _s[0].IRR&= ~mask;
      _s[0].ISR|= mask;
      // Calcula vector
      vec= _s[0].vector_base | (uint8_t) irq;
      if ( _trace_enabled && _int_serviced != NULL )
        _int_serviced ( irq, vec );
    }

  // Actualitza state de intr
  update_state ();
  
  return vec;
  
} // ack_intr_fully_nested




/**********************/
/* FUNCIONS PÚBLIQUES */
/**********************/

void
PC_ic_init (
            PC_Warning              *warning,
            PC_InterruptionServiced *int_serviced,
            void                    *udata,
            const PC_Config         *config
            )
{

  // Callbacks.
  _warning= warning;
  _int_serviced= int_serviced;
  _udata= udata;
  _trace_enabled= false;

  // Inicialitza.
  PC_ic_reset ();
  
} // end PC_ic_init


void
PC_ic_reset (void)
{
  
  int i,j;
  
  
  memset ( &_s, 0, sizeof(_s) );
  _s[0].step= WAIT_ICW1;
  _s[1].step= WAIT_ICW1;
  _s[0].input= 0x00;
  _s[1].input= 0x00;
  _s[0].out= false;
  _s[1].out= false;
  _s[0].read_cmd= READ_CMD_IRR;
  _s[1].read_cmd= READ_CMD_IRR;
  _s[0].rotate_in_auto_eoi= false;
  _s[1].rotate_in_auto_eoi= false;
  _s[0].special_mask_mode= false;
  _s[1].special_mask_mode= false;
  for ( j= 0; j < 2; ++j )
    for ( i= 0; i < 8; ++i )
      _s[j].priority[i]= i;
  init_pci ();
  _elcr[0]= _elcr[1]= 0x00; // Tots edge triggered
  update_state ();
  
} // end PC_ic_reset


void
PC_ic_cmd_write (
                 const int     icid,
                 const uint8_t data
                 )
{
  
  // ICW1
  if ( data&0x10 )
    {
      if ( _s[icid].step != WAIT_ICW1 )
      _warning ( _udata,
                 "reinicialitzant procés de configuració"
                 " d'Interrupt Controller %d", icid+1 );
      _s[icid].step= WAIT_ICW2;
      if ( (data&0xE3) != 0x01 )
        _warning ( _udata, "IC%d.ICW1= %02X no és una configuració suportada",
                   icid+1, data );
    }
  
  // Comandaments
  else
    {
      if ( _s[icid].step != INITIALIZED )
        {
          _warning ( _udata,
                     "s'ha intentat escriure un comandament sense"
                     " inicialitzar Interrupt Controller %d",
                     icid+1 );
          return;
        }
      switch ( (data>>3)&0x3 )
        {
        case 0: // OCW2—Operational Control Word 2 Register (IO)
          switch ( (data>>5)&0x7 )
            {
            case 0: _s[icid].rotate_in_auto_eoi= false; break;
            case 1: nonspecific_eoi ( icid, false ); break;
            case 2: break;
            case 3: specific_eoi ( icid, data&0x7, false ); break;
            case 4: _s[icid].rotate_in_auto_eoi= true; break;
            case 5: nonspecific_eoi ( icid, true ); break;
              
            default:
              PC_MSGF ( "(OCW2 - Operational Control Word"
                        " 2 Register) IC%d.OCW2=%02X\n",
                        icid+1, data );
              exit(EXIT_FAILURE);
            }
          break;

        case 1: // OCW3—Operational Control Word 3 Register (IO)
          // Special Mask Mode (SMM)
          _s[icid].special_mask_mode= ((data&0x60)==0x60);
          // Register Read Command.
          if ( data&0x02 )
            _s[icid].read_cmd= (data&0x01)!=0 ? READ_CMD_ISR : READ_CMD_IRR;
          break;
          
        default:
          _warning ( _udata,
                     "s'ha intentat escriure %02X en el port de comandaments"
                     " (OCW2/OCW3) del Interrupt Controller %d però el "
                     "el comandament no és ni un OCW2 ni un OCW3",
                     data, icid+1);
        }
      update_state ();
    }
  
} // end PC_ic_cmd_write


void
PC_ic_data_write (
                  const int     icid,
                  const uint8_t data
                  )
{

  int i;

  
  switch ( _s[icid].step )
    {
      // Vaig a assumir que escriure quan està en mode
      // WAIT_ICW2/WAIT_ICW3/WAIT_ICW4 reinicialitza.
    case WAIT_ICW1:
      _warning ( _udata,
                 "s'ha intentat escriure %02X en el port DATA del"
                 " Interrupt Controller %d sense haver inicialitzat"
                 " el procés de configuració", data, icid+1);
      break;
    case WAIT_ICW2:
      _s[icid].vector_base= data&0xF8;
      if ( (data&0x07) != 0x00 )
        _warning ( _udata, "IC%d.ICW2= %02X no és una configuració suportada."
                   " Interrupt Request Level ha de ser 0", icid+1, data );
      _s[icid].step= WAIT_ICW3;
      break;
    case WAIT_ICW3:
      if ( (icid == 0 && data != 0x04 ) ||
           (icid == 1 && data != 0x02 ) )
        _warning ( _udata,
                   "IC%d.ICW3= %02X no és una configuració suportada",
                   icid+1, data );
      _s[icid].step= WAIT_ICW4;
      break;
    case WAIT_ICW4:
      if ( data&0x10 )
        {
          PC_MSGF ( "(IC%d.ICW4) No s'ha implementat Special"
                    " Fully Nested Mode", icid+1 );
          exit ( EXIT_FAILURE );
        }
      if ( data&0x02 )
        {
          PC_MSGF ( "(IC%d.ICW4) No s'ha implementat Automatic"
                    " End of Interrupt", icid+1 );
          exit ( EXIT_FAILURE );
        }
      if ( (data&0xED) != 0x01 )
        _warning ( _udata, "IC%d.ICW4= %02X no és una configuració suportada",
                   icid+1, data );
      // Altres inicialitzacions que no queden clares si es fan ací o no.
      // --> 1. The Interrupt Mask register is cleared.
      _s[icid].IMR= 0x00;
      // --> 2. IRQ7 input is assigned priority 7.
      for ( i= 0; i < 8; ++i )
        _s[icid].priority[i]= i;
      // --> 3. The slave mode address is set to 7 ????
      // --> 4. Special Mask Mode is cleared and Status Read is set to IRR. ???
      _s[icid].read_cmd= READ_CMD_IRR;
      // --> 5. If IC4 was set to 0, ... ???
      _s[icid].step= INITIALIZED;
      update_state ();
      break;
    case INITIALIZED: // OCW1
      _s[icid].IMR= data;
      update_state ();
      break;
    }
  
} // end PC_ic_data_write


uint8_t
PC_ic_cmd_read (
                const int icid
                )
{

  uint8_t ret;

  
  ret= _s[icid].read_cmd==READ_CMD_IRR ? _s[icid].IRR : _s[icid].ISR;
  
  return ret;
  
} // end PC_ic_cmd_read


uint8_t
PC_ic_data_read (
                 const int icid
                 )
{
  
  uint8_t ret;


  ret= _s[icid].IMR;
  
  return ret;
  
} // end PC_ic_data_read


uint8_t
PC_ic_pirqrc_read (
                   const int line
                   )
{
  return _pci[line].reg;
} // end PC_ic_pirqrc_read


void
PC_ic_pirqrc_write (
                    const int     line,
                    const uint8_t data
                    )
{

  int tmp;

  
  _pci[line].reg= data&0x8F;
  _pci[line].enabled= ((data&0x80)==0);
  _pci[line].irq= tmp= data&0xF;
  if ( tmp <= 2 || tmp == 8 || tmp == 13 )
    {
      _pci[line].irq= -1;
      if ( _pci[line].enabled )
        _warning ( _udata, "PIRQRC%d = %02X, s'ha itentat"
                   " redirigir la senyal PCI a IRQ%d",
                   line, data, tmp );
    }
  update_state ();
  
} // end PC_ic_pirqrc_write


uint8_t
PC_ic_elcr_read (
                 const int reg
                 )
{
  return _elcr[reg];
} // end PC_ic_elcr_read


void
PC_ic_elcr_write (
                  const int     reg,
                  const uint8_t data
                  )
{

  uint8_t mask;


  mask= reg==0 ? 0x03 : 0x21;
  if ( data&mask )
    _warning ( _udata, "ELCR%d = %02X, s'ha intentat ficar en mode "
               "\"level triggered\" canals IRQ que han d'estar sempre en"
               " mode \"edge triggered\"", reg+1, data );
  _elcr[reg]= data&(~mask);
  update_state ();
  
} // end PC_ic_elcr_write


void
PC_ic_irq (
           const int  irq,
           const bool in
           )
{
  
  if ( irq == 2 )
    {
      _warning ( _udata,
                 "PC_ic_irq - s'ha intentat modificar l'entrada de IRQ2" );
      return;
    }
  input_changed ( irq, in );
  update_state ();
  
} // end PC_ic_irq


uint8_t
IA32_ack_intr (void)
{
  
  uint8_t ret;

  
  // Sols fully nested i sense automatic
  ret= ack_intr_fully_nested ();
  
  return ret;
  
} // end IA32_ack_intr


void
PC_ic_set_mode_trace (
                      const bool val
                      )
{
  _trace_enabled= val;
} // end PC_ic_set_mode_trace
