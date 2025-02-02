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
 *  piix4_usb.c - Implementa la part del PIIX4 que s'encarrega de
 *                implementar el controlador USB.
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
static const uint16_t DID= 0x7112;
static const uint8_t RID= 0x00;

// CLASSC
static const uint8_t BASEC= 0x0C; // Serial Bus controller
static const uint8_t SCC= 0x03; // Universal Serial Bus Host Controller
static const uint8_t PI= 0x00; // Universal Host Controller Interface

static const uint8_t HEDT= 0x00;

static const uint8_t INTPN= 0x04;




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
  uint32_t usbba;
} _pci_regs;




/*******/
/* PCI */
/*******/

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
    case 0x0f ... 0x1f: ret= 0x00; break;

      // Reserved
    case 0x24 ... 0x3b: ret= 0x00; break;
      // INTLN - INTERRUPT LINE REGISTER
    case 0x3c: ret= _pci_regs.intln; break;
      // INTPN - INTERRUPT PIN
    case 0x3d: ret= INTPN; break;
      
    default:
      _warning ( _udata,
                 "PCI:PIIX4_USB.read8 - addreça no implementada %02X\n",
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
    case 0x08 ... 0x0f: ret= 0x0000; break;

      // Reserved
    case 0x12 ... 0x1d: ret= 0x0000; break;
      
    default: goto warn;
    }
  
  return ret;
  
 warn:
  _warning ( _udata,
             "PCI:PIIX4_USB.read16 - addreça no implementada %02X\n",
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
    case 0x04 ... 0x07: ret= 0x00000000; break;
      // USBBA
    case 0x08: ret= _pci_regs.usbba; break;
      // Reserved
    case 0x09 ... 0x0e: ret= 0x00000000; break;
      
    default:
      _warning ( _udata,
                 "PCI:PIIX4_USB.read32 - addreça no implementada %02X\n",
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
    case 0x0f ... 0x1f: break;

      // Reserved
    case 0x24 ... 0x3b: break;
      // INTLN - INTERRUPT LINE REGISTER
    case 0x3c: _pci_regs.intln= data; break;
      // INTPN - INTERRUPT PIN
    case 0x3d: break;
      
    default:
      _warning ( _udata,
                 "PCI:PIIX4_USB.write8 - addreça no implementada %02X\n",
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
      if ( data&0x04 )
        _warning ( _udata,
                   "pci_write16 (PIIX4 USB) - s'ha intentat habilitar"
                   " el Bus Master Function Enable, però no està implementat" );
      break;

      // SCC i BASEC;
    case 0x05: break;

      // Reserved
    case 0x08 ... 0x0f: break;
      
      // Reserved
    case 0x12 ... 0x1d: break;

      // LEGSUP - LEGACY SUPPORT REGISTER
    case 0x60:
      PC_MSGF("PIIX4 USB - PCI - W16 LEGSUP (%04X)",data);
      break;
      
    default:
      _warning ( _udata,
                 "PCI:PIIX4_USB.write16 - addreça no implementada %02X\n",
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
    case 0x04 ... 0x07: break;
      // BMIBA
    case 0x08:
      _pci_regs.usbba= (data&0xFFFFFFE0)|0x1;
      break;
      // Reserved
    case 0x09 ... 0x0e: break;
      
    default:
      _warning ( _udata,
                 "PCI:PIIX4_USB.write32 - addreça no implementada %02X\n",
                 addr );
    }
  
} // end pci_write32


const PC_PCIFunction PC_PIIX4_PCIFunction_usb=
  {
    pci_read8,
    pci_read16,
    pci_read32,
    pci_write8,
    pci_write16,
    pci_write32,
    "82371AB (PIIX4) - USB Controller"
  };

static void
init_pci_regs (void)
{

  _pci_regs.pcicmd= 0x0000;
  _pci_regs.intln= 0x00;
  _pci_regs.usbba= 0x00000001;
  
} // end init_pci_regs




/**********************/
/* FUNCIONS PÚBLIQUES */
/**********************/

void
PC_piix4_usb_init (
                   PC_Warning *warning,
                   void       *udata
                   )
{
  
  _warning= warning;
  _udata= udata;

  PC_piix4_usb_reset ();
  
} // end PC_piix4_usb_init


void
PC_piix4_usb_reset (void)
{
  init_pci_regs ();
} // end PC_piix4_usb_reset


bool
PC_piix4_usb_port_read8 (
                         const uint16_t  port,
                         uint8_t        *data
                         )
{
  
  uint16_t base,iport;
  bool ret;
  

  if ( !(_pci_regs.pcicmd&PCICMD_IOSE) ) return false;
  
  base= (_pci_regs.usbba&0x0000FFE0);
  if ( port >= base && port < base+20 )
    {
      iport= port-base;
      switch ( iport )
        {
        default:
          _warning (  _udata,
                      "PC_piix4_usb_port_read8 -> unknown port %04X (%04X)",
                      port, iport );
          *data= 0xFF;
          
        }
      ret= true;
    }
  else ret= false;

  return ret;
  
} // end PC_piix4_usb_port_read8


bool
PC_piix4_usb_port_read16 (
                          const uint16_t  port,
                          uint16_t       *data
                          )
{

  uint16_t base,iport;
  bool ret;
  
  static uint16_t TEMP_06= 0x0000;
  
  if ( !(_pci_regs.pcicmd&PCICMD_IOSE) ) return false;
  
  base= (_pci_regs.usbba&0x0000FFE0);
  if ( port >= base && port < base+20 )
    {
      iport= port-base;
      switch ( iport )
        {
        case 0x6: // FRNUM - FRAME NUMBER REGISTER
          PC_MSG("PIIX4 USB - IO - R16 FRNUM");
          // NOTA!!!! perquè la BIOS funcione cal fer que torne valors
          // diferents cada vegada. Vaig a tornar el PC_Clocks.
          *data= TEMP_06;
          ++TEMP_06;
          break;
          
        case 0x10: // PORTSC0 - PORT STATUS AND CONTROL REGISTER
          PC_MSG("PIIX4 USB - IO - R16 PORTSC0");
          // NOTA!!!! De moment fique el bit0 a 0 perquè pense que no
          // té cap dispositiu.
          *data= 0xFFFE;
          break;
        case 0x12: // PORTSC1 - PORT STATUS AND CONTROL REGISTER
          PC_MSG("PIIX4 USB - IO - R16 PORTSC1");
          // NOTA!!!! De moment fique el bit0 a 0 perquè pense que no
          // té cap dispositiu.
          *data= 0xFFFE;
          break;          
        default:
          _warning (  _udata,
                      "PC_piix4_usb_port_read16 -> unknown port %04X (%04X)",
                      port, iport );
          *data= 0xFFFF;
          
        }
      ret= true;
    }
  else ret= false;

  return ret;
  
} // end PC_piix4_usb_port_read16


bool
PC_piix4_usb_port_read32 (
                          const uint16_t  port,
                          uint32_t       *data
                          )
{
  
  uint16_t base,iport;
  bool ret;
  

  if ( !(_pci_regs.pcicmd&PCICMD_IOSE) ) return false;
  
  base= (_pci_regs.usbba&0x0000FFE0);
  if ( port >= base && port < base+20 )
    {
      iport= port-base;
      switch ( iport )
        {
        default:
          _warning (  _udata,
                      "PC_piix4_usb_port_read32 -> unknown port %04X (%04X)",
                      port, iport );
          *data= 0xFFFFFFFF;
          
        }
      ret= true;
    }
  else ret= false;

  return ret;

} // end PC_piix4_usb_port_read16


bool
PC_piix4_usb_port_write8 (
                          const uint16_t port,
                          const uint8_t  data
                          )
{

  uint16_t base,iport;
  bool ret;
  

  if ( !(_pci_regs.pcicmd&PCICMD_IOSE) ) return false;
  
  base= (_pci_regs.usbba&0x0000FFE0);
  if ( port >= base && port < base+20 )
    {
      iport= port-base;
      switch ( iport )
        {
        case 0xc: // SOFMOD - START OF FRAME (SOF) MODIFY REGISTER
          PC_MSGF("PIIX4 USB - IO - W8 SOFMOD (%02X)",data);
          break;
        default:
          _warning (  _udata,
                      "PC_piix4_usb_port_write8 -> unknown port %04X (%04X)",
                      port, iport );
        }
      ret= true;
    }
  else ret= false;

  return ret;
  
} // end PC_piix4_usb_port_write8


bool
PC_piix4_usb_port_write16 (
                           const uint16_t port,
                           const uint16_t data
                           )
{
  
  uint16_t base,iport;
  bool ret;
  

  if ( !(_pci_regs.pcicmd&PCICMD_IOSE) ) return false;
  
  base= (_pci_regs.usbba&0x0000FFE0);
  if ( port >= base && port < base+20 )
    {
      iport= port-base;
      switch ( iport )
        {
        case 0x0: // USBCMD - USB COMMAND REGISTER (IO)
          PC_MSGF("PIIX4 USB - IO - W16 USBCMD (%04X)",data);
          break;
        case 0x4: // USBINTR - USB INTERRUPT ENABLE REGISTER
          PC_MSGF("PIIX4 USB - IO - W16 USBINTR (%04X)",data);
          break;
        case 0x6: // FRNUM - FRAME NUMBER REGISTER
          PC_MSGF("PIIX4 USB - IO - W16 FRNUM (%04X)",data);
          break;
        default:
          _warning (  _udata,
                      "PC_piix4_usb_port_write16 -> unknown port %04X (%04X)",
                      port, iport );
        }
      ret= true;
    }
  else ret= false;

  return ret;
  
} // end PC_piix4_usb_port_write16


bool
PC_piix4_usb_port_write32 (
                           const uint16_t port,
                           const uint32_t data
                           )
{

  uint16_t base,iport;
  bool ret;
  

  if ( !(_pci_regs.pcicmd&PCICMD_IOSE) ) return false;
  
  base= (_pci_regs.usbba&0x0000FFE0);
  if ( port >= base && port < base+20 )
    {
      iport= port-base;
      switch ( iport )
        {
        case 0x8: // FLBASEADD - FRAME LIST BASE ADDRESS REGISTER
          PC_MSGF("PIIX4 USB - IO - W32 FLBASEADD (%04X)",data);
          break;
        default:
          _warning (  _udata,
                      "PC_piix4_usb_port_write32 -> unknown port %04X (%04X)",
                      port, iport );
        }
      ret= true;
    }
  else ret= false;
  
  return ret;
  
} // end PC_piix4_usb_port_write32
