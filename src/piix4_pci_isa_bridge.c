/*
 * Copyright 2020-2025 Adrià Giménez Pastor.
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
 *  piix4_pci_isa_bridge.c - Implementa la part del PIIX4 que
 *                           s'encarrega de implementar el pont de PCI
 *                           a ISA.
 *
 */


#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "PC.h"




/*************/
/* CONSTANTS */
/*************/

static const uint16_t VID= 0x8086;
static const uint16_t DID= 0x7110;
static const uint8_t RID= 0x00;

// CLASSC
static const uint8_t BASEC= 0x06; // Bridge device.
static const uint8_t SCC= 0x01; // Host Bridge.
static const uint8_t PI= 0x00; // Hardwired as a Host-to-PCI Bridge.

static const uint8_t HEDT= 0x80;




/**********/
/* MACROS */
/**********/

#define XBCS_RESERVED             0xF800
#define XBCS_COP_ERR_FUNC_ENABLED 0x0020

#define XBCS_IMPLEMENTED (XBCS_RESERVED|                \
                          XBCS_COP_ERR_FUNC_ENABLED)




/*********/
/* ESTAT */
/*********/

// Callbacks.
static PC_Warning *_warning;
static void *_udata;

// Registres PCI
static struct
{
  uint16_t pcicmd;
  uint16_t xbcs;
} _pci_regs;

// Reset control
static uint8_t _rc;




/*******/
/* PCI */
/*******/

static void
pci_set_xbcs (
              const uint16_t val
              )
{

  if ( (val&(~XBCS_IMPLEMENTED)) != (0x0003&(~XBCS_IMPLEMENTED)) )
    {
      printf("[EE] piix4_pci_isa_bridge - pci_set_xbcs - S'ha "
             "modificat bits de XBCS no implementats: %04X\n",val);
      exit(EXIT_FAILURE);
    }
  _pci_regs.xbcs= val;
  
} // end pci_set_xbcs


static uint8_t
pci_read8 (
           const uint8_t addr
           )
{

  uint8_t ret;

  
  switch ( addr )
    {

      // VID
    case 0x00 ... 0x01: ret= (uint8_t) ((VID>>(addr&0x1))&0xFF); break;
      // DID
    case 0x02 ... 0x03: ret= (uint8_t) ((DID>>(addr&0x1))&0xFF); break;

      // RID
    case 0x08: ret= RID; break;
      // PI
    case 0x09: ret= PI; break;
      // SCC
    case 0x0a: ret= SCC; break;
      // BASEC
    case 0x0b: ret= BASEC; break;

      // HEDT
    case 0x0e: ret= HEDT; break;
      // Reserved
    case 0x0f ... 0x4b: ret= 0x00; break;

      // PIRQX ROUTE CONTROL REGISTERS
    case 0x60 ... 0x63: ret= PC_ic_pirqrc_read ( addr&0x3 ); break;
      
    default:
      printf ( "[EE] PCI:PIIX4_PCI/ISA.read8 - addreça no implementada %02X\n",
               addr );
      exit ( EXIT_FAILURE );
      /*
      _warning ( _udata,
                 "PCI:PIIX4_PCI/ISA.read8 - addreça no implementada %02X\n",
                 addr );
      ret= 0xff;
      */
    }

  return ret;
  
} // end pci_read8


static uint16_t
pci_read16 (
            const uint8_t addr
            )
{

  uint16_t ret;

  
  switch ( addr )
    {

      // VID
    case 0x00: ret= VID; break;
      // DID
    case 0x01: ret= DID; break;
      // PCICMD
    case 0x02: ret= _pci_regs.pcicmd; break;
      
      // RID i PI
    case 0x04: ret= (((uint16_t) PI)<<8) | ((uint16_t) RID); break;
      // SCC i BASEC;
    case 0x05: ret= (((uint16_t) BASEC)<<8) | ((uint16_t) SCC); break;

      // Reserved
    case 0x08 ... 0x25: ret= 0x0000; break;
      
    default: goto warn;
    }
  
  return ret;
  
 warn:
  printf ( "[EE] PCI:PIIX4_PCI/ISA.read16 - addreça no implementada %02X\n",
           addr );
  exit ( EXIT_FAILURE );
  /*
  _warning ( _udata,
             "PCI:PIIX4_PCI/ISA.read16 - addreça no implementada %02X\n",
             addr );
  return 0xffff;
  */
  
} // end pci_read16


static uint32_t
pci_read32 (
            const uint8_t addr
            )
{

  uint32_t ret;
  
  
  switch ( addr )
    {

      // VID/DID
    case 0x00: ret= (((uint32_t) DID)<<16) | VID; break;

      // RID/PI/SCC/BASEC
    case 0x02:
      ret=
        ((uint32_t) RID) |
        (((uint32_t) PI)<<8) |
        (((uint32_t) SCC)<<16) |
        (((uint32_t) BASEC)<<24);
      break;

      // Reserved
    case 0x04 ... 0x12: ret= 0x00000000; break;
      
    default:
      printf ( "[EE] PCI:PIIX4_PCI/ISA.read32 - addreça no implementada %02X\n",
               addr );
      exit ( EXIT_FAILURE );
      /*
      _warning ( _udata,
                 "PCI:PIIX4_PCI/ISA.read32 - addreça no implementada %02X\n",
                 addr );
      ret= 0xffffffff;
      */
    }

  return ret;
  
} // end pci_read32


static void
pci_write8 (
            const uint8_t addr,
            const uint8_t data
            )
{

  switch ( addr )
    {

      // VID
    case 0x00 ... 0x01: break;
      // DID
    case 0x02 ... 0x03: break;

      // PI
    case 0x09: break;
      // SCC
    case 0x0a: break;
      // BASEC
    case 0x0b: break;

      // HEDT
    case 0x0e: break;
      // Reserved
    case 0x0f ... 0x4b: break;

      // PIRQX ROUTE CONTROL REGISTERS
    case 0x60 ... 0x63: PC_ic_pirqrc_write ( addr&0x3, data ); break;
      
    default:
      printf ( "[EE] PCI:PIIX4_PCI/ISA.write8 - addreça no implementada %02X\n",
               addr );
      exit ( EXIT_FAILURE );
      /*
      _warning ( _udata,
                 "PCI:PIIX4_PCI/ISA.write8 - addreça no implementada %02X\n",
                 addr );
      */
    }
  
} // end pci_write8


static void
pci_write16 (
             const uint8_t  addr,
             const uint16_t data
             )
{

  switch ( addr )
    {

      // VID
    case 0x00: break;
      // DID
    case 0x01: break;
      //PCICMD
    case 0x02:
      _pci_regs.pcicmd= (data&0x18)|0x07;
      if ( !(data&0x01) )
        _warning ( _udata,
                   "pci_write16 (PIIX4 PCI/ISA) - s'ha intentat deshabilitar"
                   " el I/O Space Access Enable, però no està implementat" );
      if ( !(data&0x02) )
        _warning ( _udata,
                   "pci_write16 (PIIX4 PCI/ISA) - s'ha intentat deshabilitar"
                   " el Memory Access Enable (MAE), però no està implementat" );
      if ( !(data&0x04) )
        _warning ( _udata,
                   "pci_write16 (PIIX4 PCI/ISA) - s'ha intentat deshabilitar"
                   " el Bus Master Enable, però no està implementat" );
      if ( data&0x08 )
        PC_MSG ( "pci_write16 (PIIX4 PCI/ISA) -"
                 " Special Cycle Enable (SCE)" );
      if ( data&0x100 )
        PC_MSG ( "pci_write16 (PIIX4 PCI/ISA) -"
                 " SERR# Enable (SERRE)" );
      break;
      
      // SCC i BASEC;
    case 0x05: break;

      // Reserved
    case 0x08 ... 0x25: break;

      // XBCS
    case 0x27: pci_set_xbcs ( data ); break;
      
    default:
      printf ( "[EE] PCI:PIIX4_PCI/ISA.write16 - addreça"
               " no implementada %02X\n",
               addr );
      exit ( EXIT_FAILURE );
      /*
      _warning ( _udata,
                 "PCI:PIIX4_PCI/ISA.write16 - addreça no implementada %02X\n",
                 addr );
      */
    }
  
} // end pci_write16


static void
pci_write32 (
             const uint8_t  addr,
             const uint32_t data
             )
{

  switch ( addr )
    {

    case 0x00: break;

      // Reserved
    case 0x04 ... 0x12: break;
      
    default:
      printf ( "[EE] PCI:PIIX4_PCI/ISA.write32 - addreça"
               " no implementada %02X\n",
               addr );
      exit ( EXIT_FAILURE );
      /*
      _warning ( _udata,
                 "PCI:PIIX4_PCI/ISA.write32 - addreça no implementada %02X\n",
                 addr );
      */
    }
  
} // end pci_write32


const PC_PCIFunction PC_PIIX4_PCIFunction_pci_isa_bridge=
  {
    pci_read8,
    pci_read16,
    pci_read32,
    pci_write8,
    pci_write16,
    pci_write32,
    "82371AB (PIIX4) - PCI/ISA Bridge"
  };

static void
init_pci_regs (void)
{
  
  _pci_regs.pcicmd= 0x0007;
  _pci_regs.xbcs= 0x0003;
  
} // end init_pci_regs




/**********************/
/* FUNCIONS PÚBLIQUES */
/**********************/

void
PC_piix4_pci_isa_bridge_init (
                              PC_Warning *warning,
                              void       *udata
                              )
{

  _warning= warning;
  _udata= udata;

  PC_piix4_pci_isa_bridge_reset ();
  
} // end PC_piix4_pci_isa_bridge_init


void
PC_piix4_pci_isa_bridge_reset (void)
{
  
  init_pci_regs ();
  _rc= 0x00;
  
} // end PC_piix4_pci_isa_bridge_reset


void
PC_piix4_pci_isa_bridge_reset_control_write (
                                             const uint8_t data,
                                             const bool    use_jit
                                             )
{
  
  bool old_bit;

  
  old_bit= (_rc&0x04)!=0;
  _rc= data;
  if ( (_rc&0x04)!=0 && !old_bit )
    {
      if ( (_rc&0x02)!=0 )
        {
          PC_dma_reset ();
          PC_fd_reset ();
          PC_ic_reset ();
          PC_io_reset ();
          PC_mtxc_reset ( use_jit );
          PC_piix4_reset ();
          PC_ps2_reset ();
          PC_timers_reset ();
          PC_speaker_reset ();
          PC_sb16_reset ();
          PC_cpu_reset (); // <-- Últim !!!
          // El PMTIMER DE MOMENT NO EL RESETEJE
          // TAMPOC RTC/CMOS (crec que és millor que es mantinga igual)
          
          // IMPORTANT FER-HO DESPRÉS DE CPU_RESET
          if ( use_jit ) PC_CPU_JIT->_stop_after_port_write= true;
        }
      else
        {
          PC_MSG("SOFT_RESET");
          exit(EXIT_FAILURE);
        }
    }
  
} // end PC_piix4_pci_isa_bridge_reset_control_write


bool
PC_piix4_pci_isa_bridge_port_write8 (
                                     const uint16_t port,
                                     const uint8_t  data
                                     )
{

  bool ret;


  ret= false;
  switch ( port )
    {
    case 0x00f0: // CERR Coprocessor Error Register
      /*
       * NOTA!!! Mentre no s'intente activar el bit corresponent de
       * XBCS no acaba de tindre trellat emular açò. Quan s'active
       * caldrà emular FERR# (com un callback de la CPU???) i IGNNE#
       * (com una funció de la cpu???)
       */
      if ( _pci_regs.xbcs&XBCS_COP_ERR_FUNC_ENABLED )
        {
          PC_MSG("piix4_pci_isa_bridge - "
                 "PC_piix4_pci_isa_bridge_port_write8 - PORT 00F0");
          exit(EXIT_FAILURE);
        }
      ret= true;
      break;
    }
  
  return ret;
  
} // end PC_piix4_pci_isa_bridge_port_write8
