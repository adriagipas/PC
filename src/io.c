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
 *  io.c - Implementació del mapa d'entrada/eixida.
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

#define QEMU_DEBUG_READBACK 0xE9




/*********/
/* ESTAT */
/*********/

// Callbacks.
static PC_Warning *_warning;
static PC_WriteSeaBiosDebugPort *_write_sb_dbg_port;
static PC_PortAccess *_port_access;
static void *_udata;

// Pci devs.
static const PC_PCICallbacks *_pci_devs[PC_PCI_DEVICE_NULL+1];

// Config.
static const PC_Config *_config;

// I/O regs
static struct
{

  // Port 0x92 (Ignore la funcionalitat)
  struct
  {
    bool fast_A20;
  } port_92;
  // NMI TEMPORAL !!!!
  bool nmi_enabled;
  
} _io;

// Game port
static struct
{
  PC_GamePort *func;
  uint8_t      data; // Açò sol ser basura
} _game_port;

// ISA delay.
static int _delay_ISA;




/*********************/
/* FUNCIONS PRIVADES */
/*********************/

static bool
pci_port_read8 (
                const uint16_t  port,
                uint8_t        *data
                )
{

  int i;


  for ( i= 0; _pci_devs[i] != NULL; ++i )
    if ( _pci_devs[i]->ports != NULL )
      if ( _pci_devs[i]->ports->read8 ( port, data ) )
        return true;
  
  return false;
  
} // end pci_port_read8


static bool
pci_port_read16 (
                 const uint16_t  port,
                 uint16_t       *data
                 )
{

  int i;


  for ( i= 0; _pci_devs[i] != NULL; ++i )
    if ( _pci_devs[i]->ports != NULL )
      if ( _pci_devs[i]->ports->read16 ( port, data ) )
        return true;
  
  return false;
  
} // end pci_port_read16


static bool
pci_port_read32 (
                 const uint16_t  port,
                 uint32_t       *data
                 )
{

  int i;


  for ( i= 0; _pci_devs[i] != NULL; ++i )
    if ( _pci_devs[i]->ports != NULL )
      if ( _pci_devs[i]->ports->read32 ( port, data ) )
        return true;
  
  return false;
  
} // end pci_port_read32


static bool
pci_port_write8 (
                 const uint16_t port,
                 const uint8_t  data
                 )
{

  int i;


  for ( i= 0; _pci_devs[i] != NULL; ++i )
    if ( _pci_devs[i]->ports != NULL )
      if ( _pci_devs[i]->ports->write8 ( port, data ) )
        return true;
  
  return false;
  
} // end pci_port_write8


static bool
pci_port_write16 (
                  const uint16_t port,
                  const uint16_t  data
                  )
{

  int i;


  for ( i= 0; _pci_devs[i] != NULL; ++i )
    if ( _pci_devs[i]->ports != NULL )
      if ( _pci_devs[i]->ports->write16 ( port, data ) )
        return true;
  
  return false;
  
} // end pci_port_write16


static bool
pci_port_write32 (
                  const uint16_t port,
                  const uint32_t  data
                  )
{

  int i;


  for ( i= 0; _pci_devs[i] != NULL; ++i )
    if ( _pci_devs[i]->ports != NULL )
      if ( _pci_devs[i]->ports->write32 ( port, data ) )
        return true;
  
  return false;
  
} // end pci_port_write32


static void
init_io (void)
{

  // Port 0x92
  // --> FAST_A20
  _io.port_92.fast_A20= false;
  // Temporal !!!
  _io.nmi_enabled= true;
  
} // end init_io


static void
write_port_92 (
               const uint8_t data
               )
{

  _io.port_92.fast_A20= (data&0x02)!=0;
  if ( _io.port_92.fast_A20 )
    {
      PC_MSG("P92 A20M# Asserted. En realitat en la implementació"
             " actual s'assumeix que A20M# estarà sempre asserted");
    }
  if ( data&0x1 )
    {
      PC_MSG("P92.FAST_INIT= 1. No està implementat P92.FAST_INIT");
    }
  
} // end write_port_92


static uint8_t
read_port_92 (void)
{
  return (_io.port_92.fast_A20 ? 0x02 : 0x00);
} // end read_port_92


static uint8_t
port_read8 (
            void           *udata,
            const uint16_t  port
            )
{

  uint8_t ret;

  
  switch ( port )
    {

      // DMA1 Registers
    case 0x0000:
    case 0x0010: ret= PC_dma_dbaddr_read ( 0 ); break;
    case 0x0001:
    case 0x0011: ret= PC_dma_dbcnt_read ( 0 ); break;
    case 0x0002:
    case 0x0012: ret= PC_dma_dbaddr_read ( 1 ); break;
    case 0x0003:
    case 0x0013: ret= PC_dma_dbcnt_read ( 1 ); break;
    case 0x0004:
    case 0x0014: ret= PC_dma_dbaddr_read ( 2 ); break;
    case 0x0005:
    case 0x0015: ret= PC_dma_dbcnt_read ( 2 ); break;
    case 0x0006:
    case 0x0016: ret= PC_dma_dbaddr_read ( 3 ); break;
    case 0x0007:
    case 0x0017: ret= PC_dma_dbcnt_read ( 3 ); break;
    case 0x0008:
    case 0x0018: ret= PC_dma_status ( 0 ); break;

    case 0x000a:
    case 0x000c:
    case 0x000d:
    case 0x000e:
      _warning ( _udata, "port_read8 - port %04X sols escriptura", port );
      ret= 0xff;
      break;
      
      // Interrupt Controller 1 Registers
    case 0x0020:
    case 0x0024:
    case 0x0028:
    case 0x002c:
    case 0x0030:
    case 0x0034:
    case 0x0038:
    case 0x003c:
      ret= PC_ic_cmd_read ( 0 );
      PC_Clock+= _delay_ISA;
      break;
    case 0x0021:
    case 0x0025:
    case 0x0029:
    case 0x002d:
    case 0x0031:
    case 0x0035:
    case 0x0039:
    case 0x003d:
      ret= PC_ic_data_read ( 0 );
      PC_Clock+= _delay_ISA;
      break;

      // Counter/Timer Registers
    case 0x0040:
      ret= PC_timers_data_read ( 0 );
      PC_Clock+= _delay_ISA;
      break;
    case 0x0041:
      ret= PC_timers_data_read ( 1 );
      PC_Clock+= _delay_ISA;
      break;
    case 0x0042:
      ret= PC_timers_data_read ( 2 );
      PC_Clock+= _delay_ISA;
      break;
      
      // NMI - PS/2 Registers.
    case 0x0060: ret= PC_ps2_data_read (); break;
    case 0x0061:
      PC_MSG("R NMISC - SERR# NMI Source Status");
      PC_MSG("R NMISC - IOCHK# NMI Source Status");
      PC_MSG("R NMISC - IOCHK# NMI Enable");
      PC_MSG("R NMISC - PCI SERR# Enable");
      ret=
        (PC_timers_out2_get () ? 0x20 : 0x00) |
        (PC_timers_get_refresh_request_toggle () ? 0x10 : 0x00) |
        (PC_speaker_get_enabled () ? 0x02 : 0x00) |
        (PC_timers_gate2_get () ? 0x01 : 0x00)
        ;
      PC_Clock+= _delay_ISA;
      break;
      
    case 0x0064: ret= PC_ps2_status (); break;
      
      // Real Time Clock Registers
    case 0x0070: ret= PC_rtc_rtci_read (); break;
    case 0x0071: ret= PC_rtc_rtcd_read (); break;

      // DMA registers
    case 0x0080: ret= 0x00; break; // DMA1 Page (Reserved) ????
    case 0x0081:
    case 0x0091: ret= PC_dma_dlpage_read ( 2 ); break;
    case 0x0082: ret= PC_dma_dlpage_read ( 3 ); break;
    case 0x0083:
    case 0x0093: ret= PC_dma_dlpage_read ( 1 ); break;
      
    case 0x0087:
    case 0x0097: ret= PC_dma_dlpage_read ( 0 ); break;
      
    case 0x0089:
    case 0x0099: ret= PC_dma_dlpage_read ( 6 ); break;
    case 0x008a:
    case 0x009a: ret= PC_dma_dlpage_read ( 7 ); break;
    case 0x008b:
    case 0x009b: ret= PC_dma_dlpage_read ( 5 ); break;
      
      // X-Bus, coprocessor, and reset registers
    case 0x0092: ret= read_port_92 (); break;

      // Interrupt Controller 2 Registers
    case 0x00a0:
    case 0x00a4:
    case 0x00a8:
    case 0x00ac:
    case 0x00b0:
    case 0x00b4:
    case 0x00b8:
    case 0x00bc: ret= PC_ic_cmd_read ( 1 ); break;
    case 0x00a1:
    case 0x00a5:
    case 0x00a9:
    case 0x00ad:
    case 0x00b1:
    case 0x00b5:
    case 0x00b9:
    case 0x00bd: ret= PC_ic_data_read ( 1 ); break;

      // DMA2 Registers
    case 0x00c0:
    case 0x00c1: ret= PC_dma_dbaddr_read ( 4 ); break;
    case 0x00c2:
    case 0x00c3: ret= PC_dma_dbcnt_read ( 4 ); break;
    case 0x00c4:
    case 0x00c5: ret= PC_dma_dbaddr_read ( 5 ); break;
    case 0x00c6:
    case 0x00c7: ret= PC_dma_dbcnt_read ( 5 ); break;
    case 0x00c8:
    case 0x00c9: ret= PC_dma_dbaddr_read ( 6 ); break;
    case 0x00ca:
    case 0x00cb: ret= PC_dma_dbcnt_read ( 6 ); break;
    case 0x00cc:
    case 0x00cd: ret= PC_dma_dbaddr_read ( 7 ); break;
    case 0x00ce:
    case 0x00cf: ret= PC_dma_dbcnt_read ( 7 ); break;
    case 0x00d0:
    case 0x00d1: ret= PC_dma_status ( 1 ); break;
      
      // GamePort
    case 0x0201 ... 0x0207:
      if ( _game_port.func != NULL )
        ret= _game_port.func ( _game_port.data, _udata );
      else ret= 0xff;
      break;
      // ??? Algo de GamePort???
    case 0x0208 ... 0x020f: ret= 0xff; break;

      // Unused SoundBlaster. Alguns programes van provant
    case 0x0215:
    case 0x0216:
    case 0x021a:
    case 0x021c: 
    case 0x021e: ret= 0xff; break;
      
      // Sound blaster 16 (A)
    case 0x0220:
      ret= PC_sb16_fm_status ();
      PC_Clock+= _delay_ISA;
      break;
    case 0x0221:
      _warning ( _udata, "port_read8 - port %04X sols escriptura", port );
      ret= 0xff;
      PC_Clock+= _delay_ISA;
      break;
    case 0x0222:
      // IMPORTANT!!! No sé què ha de fer açò. Dubte entre ficar alias
      // de fm_status o tornar 00.
      /*
      ret= 0x00;
      */
      ret= PC_sb16_fm_status ();
      PC_Clock+= _delay_ISA;
      break;
      
    case 0x0225:
      ret= PC_sb16_mixer_read_data ();
      PC_Clock+= _delay_ISA;
      break;
    case 0x0226:
      _warning ( _udata, "port_read8 - port %04X sols escriptura", port );
      ret= 0xff;
      PC_Clock+= _delay_ISA;
      break;
      
    case 0x0228:
      ret= PC_sb16_fm_status ();
      PC_Clock+= _delay_ISA;
      break;
      
    case 0x022a:
      ret= PC_sb16_dsp_read_data ();
      PC_Clock+= _delay_ISA;
      break;
    case 0x022b:
      _warning ( _udata, "port_read8 - port %04X desconegut", port );
      ret= 0xFF;
      break;
    case 0x022c:
      ret= PC_sb16_dsp_write_buffer_status ();
      PC_Clock+= _delay_ISA;
      break;
      
    case 0x022e:
      ret= PC_sb16_dsp_read_buffer_status ();
      PC_Clock+= _delay_ISA;
      break;
    case 0x022f:
      ret= PC_sb16_dsp_ack_dma16_irq ();
      PC_Clock+= _delay_ISA;
      break;
      
      // ¿¿???
    case 0x0236:
    case 0x023a:
    case 0x023b:
    case 0x0246:
    case 0x0248:
    case 0x024a:
    case 0x024b:
    case 0x0256:
    case 0x025a:
    case 0x026b:
    case 0x0273:
      _warning ( _udata, "port_read8 - port %04X desconegut", port );
      ret= 0xFF;
      break;
      
      // LPT2
    case 0x0278:
      PC_MSG("LPT2 - DATA REGISTER R.8");
      ret= 0xFF;
      break;
    case 0x0279:
      PC_MSG("LPT2 - STATUS REGISTER R.8");
      ret= 0xFF;
      break;
    case 0x027a:
      PC_MSG("LPT2 - CONTROL REGISTER R.8");
      ret= 0xFF;
      break;
    case 0x027b:
      _warning ( _udata, "port_read8 - port %04X desconegut", port );
      ret= 0xFF;
      break;

      // Sound blaster 16 - CONF 280h
    case 0x0282:
    case 0x0288:
      _warning ( _udata, "port_read8 - port %04X desconegut", port );
      ret= 0xFF;
      break;

      // ???
    case 0x02a2:
    case 0x02c8:
      _warning ( _udata, "port_read8 - port %04X desconegut", port );
      ret= 0xFF;
      break;
      
      // PORT_SERIAL4
    case 0x02e9:
      PC_MSG("PORT_SERIAL4 - IER R.8");
      ret= 0xFF;
      break;
      // PORT_SERIAL2
    case 0x02f9:
      PC_MSG("PORT_SERIAL2 - IER R.8");
      ret= 0xFF;
      break;

    case 0x02fb:
      PC_MSG("PORT_SERIAL2 - LCR R.8");
      ret= 0xFF;
      break;

    case 0x0300: // ??
    case 0x0327:
      _warning ( _udata, "port_read8 - port %04X desconegut", port );
      ret= 0xFF;
      break;

      // Sound Blaster 16 (MIDI)
    case 0x0330:
      PC_MSG("port_read8 - MIDI DATA (0330) no implementat");
      ret= 0xFF;
      PC_Clock+= _delay_ISA;
      break;
    case 0x0331:
      PC_MSG("port_read8 - MIDI STATUS (0331) no implementat");
      ret= 0xFF;
      PC_Clock+= _delay_ISA;
      break;
      
      // LPT1
    case 0x0378:
      PC_MSG("LPT1 - DATA REGISTER R.8");
      ret= 0xFF;
      break;
    case 0x0379:
      PC_MSG("LPT1 - STATUS REGISTER R.8");
      ret= 0xFF;
      break;
    case 0x037a:
      PC_MSG("LPT1 - CONTROL REGISTER R.8");
      ret= 0xFF;
      break;

      // ¿¿???
    case 0x0380:
      _warning ( _udata, "port_read8 - port %04X desconegut", port );
      ret= 0xFF;
      break;
      
      // Sound blaster 16 (B)
    case 0x0388:
      ret= PC_sb16_fm_status ();
      PC_Clock+= _delay_ISA;
      break;
    case 0x0389:
      _warning ( _udata, "port_read8 - port %04X sols escriptura", port );
      ret= 0xff;
      PC_Clock+= _delay_ISA;
      break;
      
      // ¿¿???
    case 0x038a:
    case 0x038b:
    case 0x03cd:
    case 0x03df:
      _warning ( _udata, "port_read8 - port %04X desconegut", port );
      ret= 0xFF;
      break;
      
      // PORT_SERIAL3
    case 0x03e9:
      PC_MSG("PORT_SERIAL3 - IER R.8");
      ret= 0xFF;
      break;
      
      // Primary Floppy Disk Controller
    case 0x03f4: ret= PC_fd_msr_read (); break;
    case 0x03f5: ret= PC_fd_fifo_read (); break;
      
      // PORT_SERIAL1
    case 0x03f9:
      PC_MSG("PORT_SERIAL - IER R.8");
      ret= 0xFF;
      break;

    case 0x03fb:
      PC_MSG("PORT_SERIAL - LCR R.8");
      ret= 0xFF;
      break;
      
    case 0x03fd:
      PC_MSG("PORT_SERIAL - LSR R.8");
      ret= 0x00;
      break;
      
      // SeaBIOS debug port.
    case 0x0402:
      if ( (_config->flags&PC_CFG_QEMU_COMPATIBLE) &&
           _write_sb_dbg_port != NULL )
        {
          ret= QEMU_DEBUG_READBACK;
        }
      else
        {
          ret= 0xFF;
          printf ( "[EE] port_read8 -> unknown port %04X\n", port );
          exit ( EXIT_FAILURE );
        }
      break;
      
      // ELCR* - Edge/Level Control Registers (IO)
    case 0x04d0 ... 0x04d1: ret= PC_ic_elcr_read ( port&0x1 ); break;
      
      // QEMU fw cfg - FW_CFG_PORT_DATA
    case 0x0511:
      if ( _config->flags&PC_CFG_QEMU_COMPATIBLE )
        {
          PC_MSG("port_read8 - No s'ha implementat QEMU fw cfg"
                 " - FW_CFG_PORT_DATA");
          ret= 0xFF;
        }
      else ret= 0xFF;
      break;

      // ¿¿???
    case 0x0533: // Gravis Ultra Sound???
    case 0x0534:
    case 0x0607:
    case 0x0608:
      _warning ( _udata, "port_read8 - port %04X desconegut", port );
      ret= 0xFF;
      break;
      
    case 0x0A20: // Token Ring (adapter 1) ???
      _warning ( _udata, "port_read8 - port %04X desconegut", port );
      ret= 0xff;
      break;

    case 0x0A24: // Token Ring (adapter 2) ???
      _warning ( _udata, "port_read8 - port %04X desconegut", port );
      ret= 0xff;
      break;
      
      // CONFDATA -  CONFIGURATION DATA REGISTER
    case 0x0CFC ... 0x0CFF: ret= PC_mtxc_confdata_read8 ( port&0x3 ); break;

      // ¿¿??????
    case 0x007A: // PC radio by CoZet Info Systems???
    case 0x00E2:
    case 0x00E6:
    case 0x00EA:
    case 0x0B8B:
    case 0x0E83:
    case 0x0E84:
    case 0x0F43:
    case 0x0F44:
    case 0x0F8D:
    case 0x0F8F:
    case 0x1EE0:
    case 0x56E0:
    case 0xAAE0:
    case 0xE2E0: // ¿¿ GPIB (General Purpose Interface Bus, IEEE 488
                 // interface) ??
      _warning ( _udata, "port_read8 - port %04X desconegut", port );
      ret= 0xff;
      break;
      
    default:
      // Prova ports configurables.
      if ( !PC_piix4_ide_port_read8 ( port, &ret ) &&
           !PC_piix4_usb_port_read8 ( port, &ret ) &&
           !PC_piix4_power_management_port_read8 ( port, &ret ) &&
           !pci_port_read8 ( port, &ret ) )
        {
          printf ( "[EE] port_read8 -> unknown port %04X\n", port );
          ret= 0xFF;
          exit ( EXIT_FAILURE );
        }
    }

  return ret;
  
} // end port_read8


static uint16_t
port_read16 (
             void           *udata,
             const uint16_t  port
             )
{

  uint16_t ret;

  
  switch ( port )
    {

      // DMA1 Registers
    case 0x0000:
    case 0x0010:
      ret=
        ((uint16_t) PC_dma_dbaddr_read ( 0 )) |
        (((uint16_t) PC_dma_dbcnt_read ( 0 ))<<8)
        ;
      break;
      
      // DMA registres
    case 0x0080:
      ret=
        0x0000 | // Part baixa DMA1 Page (Reserved) ???
        (((uint16_t) PC_dma_dlpage_read ( 2 ))<<8)
        ;
      break;

      // Sound blaster 16 (A)
    case 0x0224:
      ret= 0x00ff | (((uint16_t) PC_sb16_mixer_read_data ())<<8);
      PC_Clock+= _delay_ISA;
      break;

      // Sound blaster 16 - CONF 280h
    case 0x0282:
    case 0x0292:
      _warning ( _udata, "port_read16 - port %04X desconegut", port );
      ret= 0xffff;
      break;

      // ¿¿¿?????
    case 0x02a2:
    case 0x02b2:
      _warning ( _udata, "port_read16 - port %04X desconegut", port );
      ret= 0xffff;
      break;
      
      // CONFDATA -  CONFIGURATION DATA REGISTER
    case 0x0CFC:
    case 0x0CFE: ret= PC_mtxc_confdata_read16 ( (port>>1)&0x1 ); break;
      
      // ¿¿¿????????
    case 0x92E8: // ¿¿ 8514/A and compatible video cards ??
      _warning ( _udata, "port_read16 - port %04X desconegut", port );
      ret= 0xffff;
      break;
      
    default:
      // Prova ports configurables.
      if ( !PC_piix4_ide_port_read16 ( port, &ret ) &&
           !PC_piix4_usb_port_read16 ( port, &ret ) &&
           !PC_piix4_power_management_port_read16 ( port, &ret ) &&
           !pci_port_read16 ( port, &ret ) )
        {
          printf ( "[EE] port_read16 -> unknown port %04X\n", port );
          ret= 0xFFFF;
          exit ( EXIT_FAILURE );
        }
    }

  return ret;
  
} // end port_read16


static uint32_t
port_read32 (
             void           *udata,
             const uint16_t  port
             )
{

  uint32_t ret;

  
  switch ( port )
    {

      // CONFADD - CONFIGURATION ADDRESS REGISTER
    case 0x0CF8: ret= PC_mtxc_confadd_read (); break;
      
      // CONFDATA -  CONFIGURATION DATA REGISTER
    case 0x0CFC: ret= PC_mtxc_confdata_read32 (); break;

      // QEMU - ACPI Support for hotplug
    case 0xAE00 ... 0xAE10:
      if ( (_config->flags&PC_CFG_QEMU_COMPATIBLE) &&
           _write_sb_dbg_port != NULL )
        {
          ret= 0;
          PC_MSGF("port_read32 (QEMU ACPI Support for hotplug"
                  " not implementend) [port %04X]",port);
        }
      else
        {
          ret= 0xFFFFFFFF;
          printf ( "[EE] port_read32 -> unknown port %04X\n", port );
          exit ( EXIT_FAILURE );
        }
      break;
      
    default:
      if ( !PC_piix4_ide_port_read32 ( port, &ret ) &&
           !PC_piix4_usb_port_read32 ( port, &ret ) &&
           !PC_piix4_power_management_port_read32 ( port, &ret ) &&
           !pci_port_read32 ( port, &ret ) )
        {
          printf ( "[EE] port_read32 -> unknown port %04X\n", port );
          ret= 0xFFFFFFFF;
          exit ( EXIT_FAILURE );
        }
    }

  return ret;
  
} // end port_read32


static void
port_write8_base (
                  void           *udata,
                  const uint16_t  port,
                  const uint8_t   data,
                  const bool      use_jit
                  )
{

  switch ( port )
    {

      // DMA1 Registers
    case 0x0000:
    case 0x0010: PC_dma_dbaddr_write ( 0, data ); break;
    case 0x0001:
    case 0x0011: PC_dma_dbcnt_write ( 0, data ); break;
    case 0x0002:
    case 0x0012: PC_dma_dbaddr_write ( 1, data ); break;
    case 0x0003:
    case 0x0013: PC_dma_dbcnt_write ( 1, data ); break;
    case 0x0004:
    case 0x0014: PC_dma_dbaddr_write ( 2, data ); break;
    case 0x0005:
    case 0x0015: PC_dma_dbcnt_write ( 2, data ); break;
    case 0x0006:
    case 0x0016: PC_dma_dbaddr_write ( 3, data ); break;
    case 0x0007:
    case 0x0017: PC_dma_dbcnt_write ( 3, data ); break;
    case 0x0008:
    case 0x0018: PC_dma_dcom_write ( 0, data ); break;
    case 0x0009:
    case 0x0019: PC_dma_dr_write ( 0, data ); break;
    case 0x000a:
    case 0x001a: PC_dma_wsmb_write ( 0, data ); break;
    case 0x000b:
    case 0x001b: PC_dma_dcm_write ( 0, data ); break;
    case 0x000c:
    case 0x001c: PC_dma_dcbp_write ( 0 ); break;
    case 0x000d:
    case 0x001d: PC_dma_dmc_write ( 0 ); break;
    case 0x000e:
    case 0x001e: PC_dma_dclm_write ( 0 ); break;

      // Interrupt Controller 1 Registers
    case 0x0020:
    case 0x0024:
    case 0x0028:
    case 0x002c:
    case 0x0030:
    case 0x0034:
    case 0x0038:
    case 0x003c:
      PC_ic_cmd_write ( 0, data );
      PC_Clock+= _delay_ISA;
      break;
    case 0x0021:
    case 0x0025:
    case 0x0029:
    case 0x002d:
    case 0x0031:
    case 0x0035:
    case 0x0039:
    case 0x003d:
      PC_ic_data_write ( 0, data );
      PC_Clock+= _delay_ISA;
      break;

      // Counter/Timer Registers
    case 0x0040:
      PC_timers_data_write ( 0, data );
      PC_Clock+= _delay_ISA;
      break;
    case 0x0041:
      PC_timers_data_write ( 1, data );
      PC_Clock+= _delay_ISA;
      break;
    case 0x0042:
      PC_timers_data_write ( 2, data );
      PC_Clock+= _delay_ISA;
      break;
    case 0x0043:
      PC_timers_control_write ( data );
      PC_Clock+= _delay_ISA;
      break;

      // NMI - PS/2 Registers.
    case 0x0060: PC_ps2_data_write ( data ); break;
    case 0x0061:
      PC_MSG("W NMISC - IOCHK# NMI Enable");
      PC_MSG("W NMISC - PCI SERR# Enable");
      PC_speaker_data_enable ( (data&0x2)!=0 );
      PC_timers_gate2_set ( (data&0x1)!=0 );
      PC_speaker_enable_timer ( (data&0x1)!=0 );
      PC_Clock+= _delay_ISA;
      break;
      
    case 0x0064: PC_ps2_command ( data ); break;
      
      // Real Time Clock Registers
    case 0x0070:
      if ( !(data&0x80) && _io.nmi_enabled )
        {
          _io.nmi_enabled= false;
          PC_MSGF ( "IO port 0x70 <- %02X: S'ha deshabilitat"
                    " el bit NMI!!! No implementat !!!",
                    data );
        }
      else if ( (data&0x80) && !_io.nmi_enabled ) 
        {
          _io.nmi_enabled= true;
          PC_MSGF ( "IO port 0x70 <- %02X: S'ha habilitat el"
                    " bit NMI!!! No implementat !!!",
                    data );
        }
      PC_rtc_write_rtci ( data );
      break;
    case 0x0071: PC_rtc_rtcd_write ( data ); break;

      // DMA registres
    case 0x0081:
    case 0x0091:
      PC_dma_dlpage_write ( 2, data );
      PC_Clock+= _delay_ISA;
      break;
    case 0x0082:
      PC_dma_dlpage_write ( 3, data );
      PC_Clock+= _delay_ISA;
      break;
    case 0x0083:
    case 0x0093:
      PC_dma_dlpage_write ( 1, data );
      PC_Clock+= _delay_ISA;
      break;
      
    case 0x0087:
    case 0x0097:
      PC_dma_dlpage_write ( 0, data );
      PC_Clock+= _delay_ISA;
      break;
      
    case 0x0089:
    case 0x0099:
      PC_dma_dlpage_write ( 6, data );
      PC_Clock+= _delay_ISA;
      break;
    case 0x008a:
    case 0x009a:
      PC_dma_dlpage_write ( 7, data );
      PC_Clock+= _delay_ISA;
      break;
    case 0x008b:
    case 0x009b:
      PC_dma_dlpage_write ( 5, data );
      PC_Clock+= _delay_ISA;
      break;
      
      // X-Bus, coprocessor, and reset registers
    case 0x0092: write_port_92 ( data ); break;
      
      // Interrupt Controller 2 Registers
    case 0x00a0:
    case 0x00a4:
    case 0x00a8:
    case 0x00ac:
    case 0x00b0:
    case 0x00b4:
    case 0x00b8:
    case 0x00bc: PC_ic_cmd_write ( 1, data ); break;
    case 0x00a1:
    case 0x00a5:
    case 0x00a9:
    case 0x00ad:
    case 0x00b1:
    case 0x00b5:
    case 0x00b9:
    case 0x00bd: PC_ic_data_write ( 1, data ); break;
      
      // DMA2 Registers
    case 0x00c0:
    case 0x00c1: PC_dma_dbaddr_write ( 4, data ); break;
    case 0x00c2:
    case 0x00c3: PC_dma_dbcnt_write ( 4, data ); break;
    case 0x00c4:
    case 0x00c5: PC_dma_dbaddr_write ( 5, data ); break;
    case 0x00c6:
    case 0x00c7: PC_dma_dbcnt_write ( 5, data ); break;
    case 0x00c8:
    case 0x00c9: PC_dma_dbaddr_write ( 6, data ); break;
    case 0x00ca:
    case 0x00cb: PC_dma_dbcnt_write ( 6, data ); break;
    case 0x00cc:
    case 0x00cd: PC_dma_dbaddr_write ( 7, data ); break;
    case 0x00ce:
    case 0x00cf: PC_dma_dbcnt_write ( 7, data ); break;
    case 0x00d0:
    case 0x00d1: PC_dma_dcom_write ( 1, data ); break;
    case 0x00d2:
    case 0x00d3: PC_dma_dr_write ( 1, data ); break;
    case 0x00d4:
    case 0x00d5: PC_dma_wsmb_write ( 1, data ); break;
    case 0x00d6:
    case 0x00d7: PC_dma_dcm_write ( 1, data ); break;
    case 0x00d8:
    case 0x00d9: PC_dma_dcbp_write ( 1 ); break;
    case 0x00da:
    case 0x00db: PC_dma_dmc_write ( 1 ); break;
    case 0x00dc:
    case 0x00dd: PC_dma_dclm_write ( 1 ); break;
      
      // GamePort
    case 0x0201 ... 0x0207: _game_port.data= data; break;

      // Sound Blaster unused. Alguns programes proven ports.
    case 0x0214:
    case 0x0215:
    case 0x0216:
    case 0x021c:
      break;
      
      // Sound blaster 16 (A)
    case 0x0220:
      PC_sb16_fm_set_addr ( data, 0 );
      PC_Clock+= _delay_ISA;
      break;
    case 0x0221:
      PC_sb16_fm_write_data ( data, 0 );
      PC_Clock+= _delay_ISA;
      break;
    case 0x0222:
      PC_sb16_fm_set_addr ( data, 1 ); // Advanced és array 1??!!!
      PC_Clock+= _delay_ISA;
      break;
    case 0x0223:
      PC_sb16_fm_write_data ( data, 1 );
      PC_Clock+= _delay_ISA;
      break;
    case 0x0224:
      PC_sb16_mixer_set_addr ( data );
      PC_Clock+= _delay_ISA;
      break;
    case 0x0225:
      PC_sb16_mixer_write_data ( data );
      PC_Clock+= _delay_ISA;
      break;
    case 0x0226:
      PC_sb16_dsp_reset ( data );
      PC_Clock+= _delay_ISA;
      break;
    case 0x0227:
      _warning ( _udata, "port_write8 - port %04X desconegut (data: %02X)",
                 port, data);
      break;
    case 0x0228:
      PC_sb16_fm_set_addr ( data, 0 );
      PC_Clock+= _delay_ISA;
      break;
    case 0x0229:
      PC_sb16_fm_write_data ( data, 0 );
      PC_Clock+= _delay_ISA;
      break;
    case 0x022a: // READ DATA PORT???
      _warning ( _udata, "port_write8 - port %04X sols lectura (data: %02X)",
                 port, data);
      break;
    case 0x022b: // ???
      _warning ( _udata, "port_write8 - port %04X desconegut (data: %02X)",
                 port, data);
      break;
    case 0x022c:
      PC_sb16_dsp_write ( data );
      PC_Clock+= _delay_ISA;
      break;

    case 0x022e: // DSP Read-Buffer Status???
      _warning ( _udata, "port_write8 - port %04X sols lectura (data: %02X)",
                 port, data);
      break;
      
      // ????
    case 0x0236:
    case 0x0237:
    case 0x023a:
    case 0x0246:
    case 0x0247:
    case 0x024b:
    case 0x0256:
    case 0x0267:
    case 0x0277:
      _warning ( _udata, "port_write8 - port %04X desconegut (data: %02X)",
                 port, data);
      break;
      
      // LPT2
    case 0x0278:
      PC_MSGF("LPT2 - DATA REGISTER W.8: %X",data);
      break;
    case 0x0279:
      PC_MSGF("LPT2 - STATUS REGISTER W.8: %X",data);
      break;
    case 0x027a:
      PC_MSGF("LPT2 - CONTROL REGISTER W.8: %X",data);
      break;

      // ????
    case 0x028b:
    case 0x02cb:
      _warning ( _udata, "port_write8 - port %04X desconegut (data: %02X)",
                 port, data);
      break;
      
      // PORT_SERIAL4
    case 0x02e9:
      PC_MSGF("PORT_SERIAL4 - IER W.8: %X",data);
      break;

      // ¿¿¿????????
    case 0x02f2:
    case 0x02f3:
    case 0x02f4:
    case 0x02f5:
    case 0x02f6:
    case 0x02f7:
      _warning ( _udata, "port_write8 - port %04X desconegut (data: %02X)",
                 port, data);
      break;
      
      // PORT_SERIAL2
    case 0x02f9:
      PC_MSGF("PORT_SERIAL2 - IER W.8: %X",data);
      break;
    case 0x02fa:
      PC_MSGF("PORT_SERIAL2 - IIR W.8: %X",data);
      break;
    case 0x02fb:
      PC_MSGF("PORT_SERIAL2 - LCR W.8: %X",data);
      break;

      // ¿¿¿???
    case 0x0323:
    case 0x0325:
    case 0x0327:
      _warning ( _udata, "port_write8 - port %04X desconegut (data: %02X)",
                 port, data);
      break;
      
      // LPT1
    case 0x0378:
      PC_MSGF("LPT1 - DATA REGISTER W.8: %X",data);
      break;
    case 0x0379:
      PC_MSGF("LPT1 - STATUS REGISTER W.8: %X",data);
      break;
    case 0x037a:
      PC_MSGF("LPT1 - CONTROL REGISTER W.8: %X",data);
      break;

      // ¿¿???
    case 0x0380:
    case 0x0381:
      _warning ( _udata, "port_write8 - port %04X desconegut (data: %02X)",
                 port, data);
      break;
      
      // Sound blaster 16 (B)
    case 0x0388:
      PC_sb16_fm_set_addr ( data, 0 );
      PC_Clock+= _delay_ISA;
      break;
    case 0x0389:
      PC_sb16_fm_write_data ( data, 0 );
      PC_Clock+= _delay_ISA;
      break;
      // ¿¿???
    case 0x038a:
    case 0x038b:
    case 0x03cd:
    case 0x03de:
      _warning ( _udata, "port_write8 - port %04X desconegut (data: %02X)",
                 port, data);
      break;
      
      // PORT_SERIAL3
    case 0x03e9:
      PC_MSGF("PORT_SERIAL3 - IER W.8: %X",data);
      break;

      // Primary Floppy Disk Controller
    case 0x03f2: PC_fd_dor_write ( data ); break;

    case 0x03f5: PC_fd_fifo_write ( data ); break;

    case 0x03f7: PC_fd_ccr_write ( data ); break;
      
      // PORT_SERIAL1
    case 0x03f9:
      PC_MSGF("PORT_SERIAL1 - IER W.8: %X",data);
      break;
    case 0x03fa:
      PC_MSGF("PORT_SERIAL1 - IIR W.8: %X",data);
      break;
    case 0x03fb:
      PC_MSGF("PORT_SERIAL1 - LCR W.8: %X",data);
      break;
      
    case 0x03fe:
      PC_MSGF("PORT_SERIAL1 - LCR W.8: %X",data);
      break;
      
      // SeaBIOS debug port.
    case 0x0402:
      if ( _write_sb_dbg_port != NULL ) _write_sb_dbg_port ( (char) data,
                                                             _udata );
      break;

      // ELCR* - Edge/Level Control Registers (IO)
    case 0x04d0 ... 0x04d1: PC_ic_elcr_write ( port&0x1, data ); break;

      // ¿¿¿????????
    case 0x00e2:
    case 0x06f2:
    case 0x06f3:
    case 0x06f4:
    case 0x06f5:
    case 0x06f6:
    case 0x06f7:
    case 0x0b8b:
    case 0x0a79:
    case 0x0f8d:
    case 0x0f8f:
      _warning ( _udata, "port_write8 - port %04X desconegut (data: %02X)",
                 port, data);
      break;

      // RC - Reset Control Register
    case 0x0CF9:
      PC_piix4_pci_isa_bridge_reset_control_write ( data, use_jit );
      break;
      
      // CONFDATA -  CONFIGURATION DATA REGISTER
    case 0x0CFC ... 0x0CFF:
      PC_mtxc_confdata_write8 ( port&0x3, data );
      break;
      
    default:
      if ( !PC_piix4_ide_port_write8 ( port, data ) &&
           !PC_piix4_usb_port_write8 ( port, data ) &&
           !PC_piix4_power_management_port_write8 ( port, data ) &&
           !PC_piix4_pci_isa_bridge_port_write8 ( port, data ) &&
           !pci_port_write8 ( port, data ) )
        {
          printf ( "[EE] port_write8 -> unknown port %04X (DATA: %02X)\n",
                   port, data );
          exit ( EXIT_FAILURE );
        }
    }
  
} // end port_write8_base


static void
port_write8 (
             void           *udata,
             const uint16_t  port,
             const uint8_t   data
             )
{
  port_write8_base ( udata, port, data, false );
} // end port_write8


static void
port_jit_write8 (
                 void           *udata,
                 const uint16_t  port,
                 const uint8_t   data
                 )
{
  port_write8_base ( udata, port, data, true );
} // end port_jit_write8


static void
port_write16 (
              void           *udata,
              const uint16_t  port,
              const uint16_t  data
              )
{

  switch ( port )
    {

      // DMA1 Registers
    case 0x0008:
    case 0x0018:
      PC_dma_dcom_write ( 0, (uint8_t) (data&0xFF) );
      PC_dma_dr_write ( 0, (uint8_t) (data>>8) );
      break;
      
      // Sound blaster 16 (A)
    case 0x0224:
      PC_sb16_mixer_set_addr ( (uint8_t) (data&0xFF) );
      PC_Clock+= _delay_ISA;
      PC_sb16_mixer_write_data ( (uint8_t) (data>>8) );
      PC_Clock+= _delay_ISA;
      break;
      
      // ???
    case 0x0324:
    case 0x03de:
      _warning ( _udata, "port_write16 - port %04X desconegut (data: %04X)",
                 port, data);
      break;
      
      // QEMU fw cfg - FW_CFG_PORT_SEL
    case 0x0510:
      if ( _config->flags&PC_CFG_QEMU_COMPATIBLE )
        PC_MSG("port_write16 - No s'ha implementat QEMU fw cfg"
               " - FW_CFG_PORT_SEL");
      break;

      // CONFDATA -  CONFIGURATION DATA REGISTER
    case 0x0CFC:
    case 0x0CFE:
      PC_mtxc_confdata_write16 ( (port>>1)&0x1, data );
      break;
      
      // ¿¿¿????????
    case 0x92e8: // ¿¿ 8514/A and compatible video cards ??
      _warning ( _udata, "port_write16 - port %04X desconegut (data: %04X)",
                 port, data);
      break;
      
    default:
      if ( !PC_piix4_ide_port_write16 ( port, data ) &&
           !PC_piix4_usb_port_write16 ( port, data ) &&
           !PC_piix4_power_management_port_write16 ( port, data ) &&
           !pci_port_write16 ( port, data ) )
        {
          printf ( "[EE] port_write16 -> unknown port %04X\n", port );
          exit ( EXIT_FAILURE );
        }
    }
  
} // end port_write16


static void
port_write32_base (
                   void           *udata,
                   const uint16_t  port,
                   const uint32_t  data,
                   const bool      use_jit
                   )
{

  switch ( port )
    {
      
      // Interrupt Controller 1 Registers
    case 0x0020:
    case 0x0024:
    case 0x0028:
    case 0x002c:
    case 0x0030:
    case 0x0034:
    case 0x0038:
    case 0x003c:
      PC_ic_cmd_write ( 0, (uint8_t) (data&0xFF) );
      PC_ic_data_write ( 0, (uint8_t) ((data>>8)&0xFF) );
      // No sé què fa  023 i 024
      break;
      
      // CONFADD - CONFIGURATION ADDRESS REGISTER
    case 0x0CF8: PC_mtxc_confadd_write ( data, use_jit ); break;
  
      // CONFDATA -  CONFIGURATION DATA REGISTER
    case 0x0CFC: PC_mtxc_confdata_write32 ( data ); break;
      
    default:
      if ( !PC_piix4_ide_port_write32 ( port, data ) &&
           !PC_piix4_usb_port_write32 ( port, data ) &&
           !PC_piix4_power_management_port_write32 ( port, data ) &&
           !pci_port_write32 ( port, data ) )
        {
          printf ( "[EE] port_write32 -> unknown port %04X\n", port );
          exit ( EXIT_FAILURE );
        }
    }
  
} // end port_write32_base


static void
port_write32 (
              void           *udata,
              const uint16_t  port,
              const uint32_t  data
              )
{
  port_write32_base ( udata,  port, data, false );  
} // end port_write32


static void
port_jit_write32 (
                  void           *udata,
                  const uint16_t  port,
                  const uint32_t  data
                  )
{
  port_write32_base ( udata,  port, data, true );  
} // end port_jit_write32


static uint8_t
port_read8_trace (
                  void           *udata,
                  const uint16_t  port
                  )
{

  uint8_t ret;
  

  ret= port_read8 ( udata, port );
  _port_access ( PC_READ8, port, (uint32_t) ret, _udata );

  return ret;
  
} // end port_read8_trace


static uint16_t
port_read16_trace (
                   void           *udata,
                   const uint16_t  port
                   )
{

  uint16_t ret;
  

  ret= port_read16 ( udata, port );
  _port_access ( PC_READ16, port, (uint32_t) ret, _udata );

  return ret;
  
} // end port_read16_trace


static uint32_t
port_read32_trace (
                   void           *udata,
                   const uint16_t  port
                   )
{

  uint32_t ret;
  

  ret= port_read32 ( udata, port );
  _port_access ( PC_READ32, port, ret, _udata );

  return ret;
  
} // end port_read32_trace


static void
port_write8_trace (
                   void           *udata,
                   const uint16_t  port,
                   const uint8_t   data
                   )
{

  port_write8 ( udata, port, data );
  _port_access ( PC_WRITE8, port, (uint32_t) data, _udata );
  
} // end port_write8_trace


static void
port_jit_write8_trace (
                       void           *udata,
                       const uint16_t  port,
                       const uint8_t   data
                       )
{

  port_jit_write8 ( udata, port, data );
  _port_access ( PC_WRITE8, port, (uint32_t) data, _udata );
  
} // end port_jit_write8_trace


static void
port_write16_trace (
                    void           *udata,
                    const uint16_t  port,
                    const uint16_t  data
                    )
{

  port_write16 ( udata, port, data );
  _port_access ( PC_WRITE16, port, (uint32_t) data, _udata );
  
} // end port_write16_trace


static void
port_write32_trace (
                    void           *udata,
                    const uint16_t  port,
                    const uint32_t  data
                    )
{

  port_write32 ( udata, port, data );
  _port_access ( PC_WRITE32, port, data, _udata );
  
} // end port_write32_trace


static void
port_jit_write32_trace (
                        void           *udata,
                        const uint16_t  port,
                        const uint32_t  data
                        )
{

  port_jit_write32 ( udata, port, data );
  _port_access ( PC_WRITE32, port, data, _udata );
  
} // end port_jit_write32_trace




/**********************/
/* FUNCIONS PÚBLIQUES */
/**********************/

void
PC_io_init (
            PC_Warning               *warning,
            PC_WriteSeaBiosDebugPort *write_sb_dbg_port,
            PC_PortAccess            *port_access,
            const PC_PCICallbacks    *pci_devs[], // Acava en NULL
            void                     *udata,
            const PC_Config          *config
            )
{
  
  int i;
  
  
  // Callbacks.
  _warning= warning;
  _write_sb_dbg_port= write_sb_dbg_port;
  _port_access= port_access;
  _udata= udata;

  // Config.
  _config= config;

  // Dispositius PCI.
  for ( i= 0; pci_devs[i] != NULL; ++i )
    _pci_devs[i]= pci_devs[i];
  for ( ; i < PC_PCI_DEVICE_NULL; ++i )
    _pci_devs[i]= NULL;
  
  // Callbacks ports I/O.
  PC_CPU.port_read8= port_read8;
  PC_CPU.port_read16= port_read16;
  PC_CPU.port_read32= port_read32;
  PC_CPU.port_write8= port_write8;
  PC_CPU.port_write16= port_write16;
  PC_CPU.port_write32= port_write32;
  PC_CPU_JIT->port_read8= port_read8;
  PC_CPU_JIT->port_read16= port_read16;
  PC_CPU_JIT->port_read32= port_read32;
  PC_CPU_JIT->port_write8= port_jit_write8;
  PC_CPU_JIT->port_write16= port_write16;
  PC_CPU_JIT->port_write32= port_jit_write32;

  // Game port
  _game_port.func= NULL;
  
  // Delay ISA (Assumiré 8.33 MB/s).
  _delay_ISA= PC_ClockFreq/(8330000/8);
  
  // Altres
  PC_io_reset ();
  
} // end PC_io_init


void
PC_io_reset (void)
{

  _game_port.data= 0x00;
  init_io ();
  
} // end PC_io_reset


void
PC_io_set_mode_trace (
                      const bool val
                      )
{
  
  // Ports I/O
  if ( _port_access && val )
    {
      PC_CPU.port_read8= port_read8_trace;
      PC_CPU.port_read16= port_read16_trace;
      PC_CPU.port_read32= port_read32_trace;
      PC_CPU.port_write8= port_write8_trace;
      PC_CPU.port_write16= port_write16_trace;
      PC_CPU.port_write32= port_write32_trace;
      PC_CPU_JIT->port_read8= port_read8_trace;
      PC_CPU_JIT->port_read16= port_read16_trace;
      PC_CPU_JIT->port_read32= port_read32_trace;
      PC_CPU_JIT->port_write8= port_jit_write8_trace;
      PC_CPU_JIT->port_write16= port_write16_trace;
      PC_CPU_JIT->port_write32= port_jit_write32_trace;
    }
  else
    {
      PC_CPU.port_read8= port_read8;
      PC_CPU.port_read16= port_read16;
      PC_CPU.port_read32= port_read32;
      PC_CPU.port_write8= port_write8;
      PC_CPU.port_write16= port_write16;
      PC_CPU.port_write32= port_write32;
      PC_CPU_JIT->port_read8= port_read8;
      PC_CPU_JIT->port_read16= port_read16;
      PC_CPU_JIT->port_read32= port_read32;
      PC_CPU_JIT->port_write8= port_jit_write8;
      PC_CPU_JIT->port_write16= port_write16;
      PC_CPU_JIT->port_write32= port_jit_write32;
    }
  
} // end PC_io_set_mode_trace


void
PC_connect_game_port (
                      PC_GamePort *game_port
                      )
{
  _game_port.func= game_port;
} // end PC_connect_game_port
