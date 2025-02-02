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
 *  piix4_power_management.c - Implementa la part del PIIX4 que s'encarrega de
 *                             implementar el gestor d'energia.
 *
 */


#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "PC.h"




/**********/
/* MACROS */
/**********/

#define PCICMD_IOSE 0x0001




/*************/
/* CONSTANTS */
/*************/

static const uint16_t VID= 0x8086;
static const uint16_t DID= 0x7113;
static const uint8_t RID= 0x00;

// CLASSC
static const uint8_t BASEC= 0x06; // Bridge device
static const uint8_t SCC= 0x80; // Other Bridge device
static const uint8_t PI= 0x00; // No specific register level programming defined

static const uint8_t HEDT= 0x00;

static const uint8_t INTPN= 0x01;




/*********/
/* ESTAT */
/*********/

// Callbacks.
static PC_Warning *_warning;
static void *_udata;

// Registres pci
static struct
{
  
  uint16_t pcicmd;
  uint8_t  intln; // No fa res
  uint16_t pmba; // El bit 1 està hardcodejat a 1 però perque es puga
                 // gastar directament com offset ho faig en el read
                 // (no en el init!!!). (són 32 bits però sols em
                 // guarde els 16)
  bool     pmiose; // Bit.1 de PMREGMISC
  uint16_t smbba; // El bit 1 està hardcodejat a 1 però perque es puga
                  // gastar directament com offset ho faig en el read
                  // (no en el init!!!). (són 32 bits però sols em
                  // guarde els 16)
  struct
  {
    uint8_t reg;
    bool    enabled;
    bool    smi_int;
    bool    irq9_int;
  }        smbhstcfg; // SMBUS host configuration
} _pci_regs;




/*******/
/* PCI */
/*******/

static void
set_smbhstcfg (
               const uint8_t data
               )
{
  
  _pci_regs.smbhstcfg.reg= data;
  _pci_regs.smbhstcfg.enabled= ((data&0x1)!=0);
  _pci_regs.smbhstcfg.smi_int= ((data&0xE)==0);
  _pci_regs.smbhstcfg.irq9_int= ((data&0xE)==0x8);
  
} // end set_smbhstcfg


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
    case 0x0f ... 0x3b: ret= 0x00; break;
      // INTLN - INTERRUPT LINE REGISTER
    case 0x3c: ret= _pci_regs.intln; break;
      // INTPN - INTERRUPT PIN
    case 0x3d: ret= INTPN; break;

      // PMREGMISC - MISCELLANEOUS POWER MANAGEMENT
    case 0x80: ret= _pci_regs.pmiose ? 0x01 : 0x00; break;

      // SMBHSTCFG - SMBUS HOST CONFIGURATION
    case 0xd2: ret= _pci_regs.smbhstcfg.reg; break;
      
    default:
      _warning ( _udata,
                 "PCI:PIIX4_PWM.read8 - addreça no implementada %02X\n",
                 addr );
      ret= 0xff;
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
    case 0x08 ... 0x1d: ret= 0x0000; break;
      
    default: goto warn;
    }
  
  return ret;
  
 warn:
  _warning ( _udata,
             "PCI:PIIX4_PWM.read16 - addreça no implementada %02X\n",
             addr );
  return 0xffff;
  
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
    case 0x04 ... 0x0e: ret= 0x00000000; break;

      // PMBA - POWER MANAGEMENT BASE ADDRESS
    case 0x10: ret= (uint32_t) (_pci_regs.pmba|0x1); break;

      // SMBBA - SMBUS BASE ADDRESS
    case 0x24: ret= (uint32_t) (_pci_regs.smbba|0x1); break;
      
    default:
      _warning ( _udata,
                 "PCI:PIIX4_PWM.read32 - addreça no implementada %02X\n",
                 addr );
      ret= 0xffffffff;
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
    case 0x0f ... 0x3b: break;
      // INTLN - INTERRUPT LINE REGISTER
    case 0x3c: _pci_regs.intln= data; break;
      // INTPN - INTERRUPT PIN
    case 0x3d: break;

      // PMREGMISC - MISCELLANEOUS POWER MANAGEMENT
    case 0x80: _pci_regs.pmiose= ((data&0x01)!=0); break;

      // SMBHSTCFG - SMBUS HOST CONFIGURATION
    case 0xd2: set_smbhstcfg ( data ); break;
      
    default:
      _warning ( _udata,
                 "PCI:PIIX4_PWM.write8 - addreça no implementada %02X\n",
                 addr );
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
      _pci_regs.pcicmd= data&0x021F;
      break;
      
      // SCC i BASEC;
    case 0x05: break;

      // Reserved
    case 0x08 ... 0x1d: break;
      
    default:
      _warning ( _udata,
                 "PCI:PIIX4_PWM.write16 - addreça no implementada %02X\n",
                 addr );
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
    case 0x04 ... 0x0e: break;

      // PMBA - POWER MANAGEMENT BASE ADDRESS
    case 0x10: _pci_regs.pmba= (uint16_t) (data&0x0000FFC0); break;

      // SMBBA - SMBUS BASE ADDRESS
    case 0x24: _pci_regs.smbba= (uint16_t) (data&0x0000FFF0); break;
      
    default:
      _warning ( _udata,
                 "PCI:PIIX4_PWM.write32 - addreça no implementada %02X\n",
                 addr );
    }
  
} // end pci_write32


const PC_PCIFunction PC_PIIX4_PCIFunction_power_management=
  {
    pci_read8,
    pci_read16,
    pci_read32,
    pci_write8,
    pci_write16,
    pci_write32,
    "82371AB (PIIX4) - Power management"
  };

static void
init_pci_regs (void)
{

  _pci_regs.pcicmd= 0x0000;
  _pci_regs.intln= 0x00;
  _pci_regs.pmba= 0x0000;
  _pci_regs.pmiose= false;
  set_smbhstcfg ( 0x00 );
  
} // end init_pci_regs




/**********************/
/* FUNCIONS PÚBLIQUES */
/**********************/

void
PC_piix4_power_management_init (
                                PC_Warning *warning,
                                void       *udata
                                )
{
  
  _warning= warning;
  _udata= udata;

  PC_piix4_power_management_reset ();
  
} // end PC_piix4_power_management_init


void
PC_piix4_power_management_reset (void)
{
  init_pci_regs ();
} // end PC_piix4_power_management_reset


bool
PC_piix4_power_management_port_read8 (
                                      const uint16_t  port,
                                      uint8_t        *data
                                      )
{
  
  uint16_t base,iport;
  bool ret;


  ret= false;
  if ( _pci_regs.pmiose )
    {
      base= _pci_regs.pmba;
      if ( port >= base && port < (base+0x38) )
        {
          iport= port-base;
          switch ( iport )
            {
            default:
              _warning (  _udata,
                          "PC_piix4_power_management_port_read8 ->"
                          " unknown port %04X (%04X)",
                          port, iport );
            }
          ret= true;
        }
    }
  if ( ret ) return ret;
  
  // SMBUS
  if ( _pci_regs.pcicmd&PCICMD_IOSE )
    {
      base= _pci_regs.smbba;
      if ( port >= base && port < (base+0x0e) )
        {
          iport= port-base;
          switch ( iport )
            {
            default:
              _warning (  _udata,
                          "PC_piix4_power_management_port_read8 ->"
                          " unknown port %04X (%04X)",
                          port, iport );
            }
          ret= true;
        }
    }
  
  return ret;
  
} // end PC_piix4_power_management_port_read8


bool
PC_piix4_power_management_port_read16 (
                                       const uint16_t  port,
                                       uint16_t       *data
                                       )
{
  
  uint16_t base,iport;
  bool ret;


  ret= false;
  if ( _pci_regs.pmiose )
    {
      base= _pci_regs.pmba;
      if ( port >= base && port < (base+0x38) )
        {
          iport= port-base;
          switch ( iport )
            {
            default:
              _warning (  _udata,
                          "PC_piix4_power_management_port_read16 ->"
                          " unknown port %04X (%04X)",
                          port, iport );
            }
          ret= true;
        }
    }
  if ( ret ) return ret;
  
  // SMBUS
  if ( _pci_regs.pcicmd&PCICMD_IOSE )
    {
      base= _pci_regs.smbba;
      if ( port >= base && port < (base+0x0e) )
        {
          iport= port-base;
          switch ( iport )
            {
            default:
              _warning (  _udata,
                          "PC_piix4_power_management_port_read16 ->"
                          " unknown port %04X (%04X)",
                          port, iport );
            }
          ret= true;
        }
    }

  return ret;
  
} // end PC_piix4_power_management_port_read16


bool
PC_piix4_power_management_port_read32 (
                                       const uint16_t  port,
                                       uint32_t       *data
                                       )
{
  
  uint16_t base,iport;
  bool ret;


  ret= false;
  if ( _pci_regs.pmiose )
    {
      base= _pci_regs.pmba;
      if ( port >= base && port < (base+0x38) )
        {
          iport= port-base;
          switch ( iport )
            {
              
            case 0x08: // PMTMR - POWER MANAGEMENT TIMER REGISTER
              *data= PC_pmtimer_get ();
              break;
              
            default:
              _warning (  _udata,
                          "PC_piix4_power_management_port_read32 ->"
                          " unknown port %04X (%04X)",
                          port, iport );
            }
          ret= true;
        }
    }
  if ( ret ) return ret;
  
  // SMBUS
  if ( _pci_regs.pcicmd&PCICMD_IOSE )
    {
      base= _pci_regs.smbba;
      if ( port >= base && port < (base+0x0e) )
        {
          iport= port-base;
          switch ( iport )
            {
            default:
              _warning (  _udata,
                          "PC_piix4_power_management_port_read32 ->"
                          " unknown port %04X (%04X)",
                          port, iport );
            }
          ret= true;
        }
    }
  
  return ret;

} // end PC_piix4_power_management_port_read32


bool
PC_piix4_power_management_port_write8 (
                                       const uint16_t port,
                                       const uint8_t  data
                                       )
{
  
  uint16_t base,iport;
  bool ret;


  ret= false;
  if ( _pci_regs.pmiose )
    {
      base= _pci_regs.pmba;
      if ( port >= base && port < (base+0x38) )
        {
          iport= port-base;
          switch ( iport )
            {
            default:
              _warning (  _udata,
                          "PC_piix4_power_management_port_write8 ->"
                          " unknown port %04X (%04X)",
                          port, iport );
            }
          ret= true;
        }
    }
  if ( ret ) return ret;
  
  // SMBUS
  if ( _pci_regs.pcicmd&PCICMD_IOSE )
    {
      base= _pci_regs.smbba;
      if ( port >= base && port < (base+0x0e) )
        {
          iport= port-base;
          switch ( iport )
            {
            default:
              _warning (  _udata,
                          "PC_piix4_power_management_port_write8 ->"
                          " unknown port %04X (%04X)",
                          port, iport );
            }
          ret= true;
        }
    }

  return ret;

} // end PC_piix4_power_management_port_write8


bool
PC_piix4_power_management_port_write16 (
                                        const uint16_t port,
                                        const uint16_t data
                                        )
{
  
  uint16_t base,iport;
  bool ret;


  ret= false;
  if ( _pci_regs.pmiose )
    {
      base= _pci_regs.pmba;
      if ( port >= base && port < (base+0x38) )
        {
          iport= port-base;
          switch ( iport )
            {
            default:
              _warning (  _udata,
                          "PC_piix4_power_management_port_write16 ->"
                          " unknown port %04X (%04X)",
                          port, iport );
            }
          ret= true;
        }
    }
  if ( ret ) return ret;
  
  // SMBUS
  if ( _pci_regs.pcicmd&PCICMD_IOSE )
    {
      base= _pci_regs.smbba;
      if ( port >= base && port < (base+0x0e) )
        {
          iport= port-base;
          switch ( iport )
            {
            default:
              _warning (  _udata,
                          "PC_piix4_power_management_port_write16 ->"
                          " unknown port %04X (%04X)",
                          port, iport );
            }
          ret= true;
        }
    }

  return ret;
  
} // end PC_piix4_power_management_port_write16


bool
PC_piix4_power_management_port_write32 (
                                        const uint16_t port,
                                        const uint32_t data
                                        )
{
    
  uint16_t base,iport;
  bool ret;


  ret= false;
  if ( _pci_regs.pmiose )
    {
      base= _pci_regs.pmba;
      if ( port >= base && port < (base+0x38) )
        {
          iport= port-base;
          switch ( iport )
            {
            default:
              _warning (  _udata,
                          "PC_piix4_power_management_port_write32 ->"
                          " unknown port %04X (%04X)",
                          port, iport );
            }
          ret= true;
        }
    }
  if ( ret ) return ret;
  
  // SMBUS
  if ( _pci_regs.pcicmd&PCICMD_IOSE )
    {
      base= _pci_regs.smbba;
      if ( port >= base && port < (base+0x0e) )
        {
          iport= port-base;
          switch ( iport )
            {
            default:
              _warning (  _udata,
                          "PC_piix4_power_management_port_write32 ->"
                          " unknown port %04X (%04X)",
                          port, iport );
            }
          ret= true;
        }
    }
  
  return ret;

} // end PC_piix4_power_management_port_write32
