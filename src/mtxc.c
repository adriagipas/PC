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
 *  mtxc.c - Implementa el xip 82439TX (MTXC). És el north-bridge.
 *
 */


#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "PC.h"




/**********/
/* MACROS */
/**********/

#ifdef PC_LE
#define SWAPU16(DATA) (DATA)
#define SWAPU32(DATA) (DATA)
#else
#define SWAPU16(DATA) (((DATA)>>8)|((DATA)<<8))
#define SWAPU32(DATA)                                                   \
  (((DATA)>>24)|(((DATA)>>8)&0x0000FF00)|(((DATA)<<8)&0x00FF0000)|((DATA)<<24))
#endif

#define RAM_READ8(ADDR) (_ram.v[ADDR])
#define RAM_READ16(ADDR)                                                \
  SWAPU16((((const uint16_t *) (_ram.v+((ADDR)&0x1)))[ADDR>>1]))
#define RAM_READ32(ADDR)                                        \
  SWAPU32(((const uint32_t *) (_ram.v+((ADDR)&0x3)))[ADDR>>2])
#define RAM_WRITE8(ADDR,DATA) (_ram.v[ADDR]= (DATA))
#define RAM_WRITE16(ADDR,DATA)                                          \
  (((uint16_t *) (_ram.v+((ADDR)&0x1)))[ADDR>>1]= SWAPU16(DATA))
#define RAM_WRITE32(ADDR,DATA)                                          \
  (((uint32_t *) (_ram.v+((ADDR)&0x3)))[ADDR>>2]= SWAPU32(DATA))

#define PAGE_CODE_BITS 4




/*************/
/* CONSTANTS */
/*************/

static const uint16_t VID= 0x8086;
static const uint16_t DID= 0x7100;
static const uint8_t RID= 0x01;

// CLASSC
static const uint8_t BASEC= 0x00;
static const uint8_t SCC= 0x00;
static const uint8_t PI= 0x00;

static const uint8_t HEDT= 0x00;

// Flags específiques per a QEMU
static const uint16_t QEMU_SUBSYSTEM_VENDOR_ID= 0x1AF4; // Read Hat, Inc
static const uint16_t QEMU_SUBSYSTEM_ID= 0x1100; // Qemu virtual machine




/*********/
/* ESTAT */
/*********/

// Callbacks.
static PC_Warning *_warning;
static PC_MemAccess *_mem_access;
static PC_PCIRegAccess *_pci_reg_access;
static void *_udata;

// Pci funcs.
static const PC_PCICallbacks *_pci_devs[PC_PCI_DEVICE_NULL+1];

// Config.
static const PC_Config *_config;

// API PCI
static struct
{

  // Conexió actual.
  const PC_PCIFunction *func;
  uint8_t               reg;

  // Valor actual de confadd
  uint32_t confadd;
  
} _pci_api;

// RAM
static struct
{
  uint8_t  *v;
  bool     *pages_code; // Sols es gasta amb JIT
  int       npages;
  uint64_t  size;
  uint64_t  size_1;
  uint64_t  size_3;
  struct
  {
    uint8_t reg;
    struct
    {
      bool read_enabled;
      bool write_enabled;
    } flags[2];
    // FALTEN COSES DE CACHE !!!
  } pam[7];
} _ram;

// Registres PCI MTXC
static struct
{
  uint16_t pcicmd;
} _pci_regs;

// Access a confdata
static uint8_t (*_confdata_read8) (const uint8_t low_addr);
static uint16_t (*_confdata_read16) (const uint8_t low_addr);
static uint32_t (*_confdata_read32) (void);
static void (*_confdata_write8) (const uint8_t low_addr,const uint8_t data);
static void (*_confdata_write16) (const uint8_t  low_addr,const uint16_t data);
static void (*_confdata_write32) (const uint32_t data);




/*******/
/* PCI */
/*******/

static void
pam_reg_write (
               const int     reg,
               const uint8_t val
               )
{
  
  _ram.pam[reg].reg= val;
  _ram.pam[reg].flags[0].read_enabled= (val&0x01)!=0;
  _ram.pam[reg].flags[0].write_enabled= (val&0x02)!=0;
  _ram.pam[reg].flags[1].read_enabled= (val&0x10)!=0;
  _ram.pam[reg].flags[1].write_enabled= (val&0x20)!=0;
  
} // end pam_reg_write


static void
jit_area_remapped (
                   const uint32_t begin,
                   const uint32_t end
                   )
{

  int pb,pe,p;

  
  IA32_jit_area_remapped ( PC_CPU_JIT, begin, end );
  pb= begin>>PAGE_CODE_BITS;
  pe= (end+1)>>PAGE_CODE_BITS;
  for ( p= pb; p < pe; ++p  )
    _ram.pages_code[p]= false;
  
} // end jit_area_remapped


static void
pam_reg_jit_write (
                   const int     reg,
                   const uint8_t val
                   )
{

  bool new_val;

  
  _ram.pam[reg].reg= val;
  _ram.pam[reg].flags[0].write_enabled= (val&0x02)!=0;
  _ram.pam[reg].flags[1].write_enabled= (val&0x20)!=0;
  new_val= (val&0x01)!=0;
  if ( _ram.pam[reg].flags[0].read_enabled != new_val )
    {
      switch ( reg )
        {
        case 0: break; // ESTE NO FA RES
        case 1: jit_area_remapped ( 0xC0000, 0xC3FFF ); break;
        case 2: jit_area_remapped ( 0xC8000, 0xCBFFF ); break;
        case 3: jit_area_remapped ( 0xD0000, 0xD3FFF ); break;
        case 4: jit_area_remapped ( 0xD8000, 0xDBFFF ); break;
        case 5: jit_area_remapped ( 0xE0000, 0xE3FFF ); break;
        case 6: jit_area_remapped ( 0xE8000, 0xEBFFF ); break;
        default: break;
        }
    }
  _ram.pam[reg].flags[0].read_enabled= new_val;
  new_val= (val&0x10)!=0;
  if ( _ram.pam[reg].flags[1].read_enabled != new_val )
    {
      switch ( reg )
        {
        case 0: jit_area_remapped ( 0xF0000, 0xFFFFF ); break;
        case 1: jit_area_remapped ( 0xC4000, 0xC7FFF ); break;
        case 2: jit_area_remapped ( 0xCC000, 0xCFFFF ); break;
        case 3: jit_area_remapped ( 0xD4000, 0xD7FFF ); break;
        case 4: jit_area_remapped ( 0xDC000, 0xDFFFF ); break;
        case 5: jit_area_remapped ( 0xE4000, 0xE7FFF ); break;
        case 6: jit_area_remapped ( 0xEC000, 0xEFFFF ); break;
        default: break;
        }
    }
  _ram.pam[reg].flags[1].read_enabled= new_val;
  
} // end pam_reg_jit_write


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
    case 0x10 ... 0x3f: ret= 0x00; break;
      
      // PAM
    case 0x59 ... 0x5f: ret= _ram.pam[addr-0x59].reg; break;
      
    default:
      _warning ( _udata, "PCI:MTXC.read8 - addreça no implementada %02X\n",
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
    case 0x08 ... 0x15: ret= 0x0000; break;
      // PCI_SUBSYSTEM_VENDOR_ID (Sols per QEMU)
    case 0x16:
      if ( _config->flags&PC_CFG_QEMU_COMPATIBLE )
        ret= QEMU_SUBSYSTEM_VENDOR_ID;
      else
        goto warn;
      break;
      // PCI_SUBSYSTEM_ID (Sols per QEMU)
    case 0x17:
      if ( _config->flags&PC_CFG_QEMU_COMPATIBLE )
        ret= QEMU_SUBSYSTEM_ID;
      else
        goto warn;
      break;
      // Reserved
    case 0x18 ... 0x1f: ret= 0x0000; break;
      
      // PAM
    case 0x2c:
      ret=
        (((uint16_t) _ram.pam[0].reg)<<8) |
        0x00;
      PC_MSG("PCI:MTXC.read16 - ADDR 0x2c falta un registre!!!!");
      break;
    case 0x2d ... 0x2f:
      ret=
        ((uint16_t) _ram.pam[2*(addr-0x2c)-1].reg) |
        (((uint16_t) _ram.pam[2*(addr-0x2c)].reg)<<8);
      break;
      
    default: goto warn;
    }
  
  return ret;
  
 warn:
  _warning ( _udata, "PCI:MTXC.read16 - addreça no implementada %02X\n",
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
    case 0x04 ... 0x0f: ret= 0x00000000; break;
      
      // PAM
    case 0x16:
      PC_MSG("PCI:MTXC.read32 - ADDR 0x16 falta un registre!!!!");
      ret=
        0x00 |
        (((uint32_t) _ram.pam[0].reg)<<8) |
        (((uint32_t) _ram.pam[1].reg)<<16) |
        (((uint32_t) _ram.pam[2].reg)<<24);
      break;
    case 0x17:
      ret=
        ((uint32_t) _ram.pam[3].reg) |
        (((uint32_t) _ram.pam[4].reg)<<8) |
        (((uint32_t) _ram.pam[5].reg)<<16) |
        (((uint32_t) _ram.pam[6].reg)<<24);
      break;
      
    default:
      _warning ( _udata, "PCI:MTXC.read32 - addreça no implementada %02X\n",
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
    case 0x10 ... 0x3f: break;
      
      // PAM
    case 0x59 ... 0x5f: pam_reg_write ( addr-0x59, data ); break;
      
    default:
      _warning ( _udata, "PCI:MTXC.write8 - addreça no implementada %02X\n",
                 addr );
    }
  
} // end pci_write8


static void
pci_jit_write8 (
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
    case 0x10 ... 0x3f: break;
      
      // PAM
    case 0x59 ... 0x5f: pam_reg_jit_write ( addr-0x59, data ); break;
      
    default:
      _warning ( _udata, "PCI:MTXC.write8 - addreça no implementada %02X\n",
                 addr );
    }
  
} // end pci_jit_write8


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
      _pci_regs.pcicmd= (data&0x02)|0x04;
      if ( !(data&0x02) )
        _warning ( _udata,
                   "pci_write16 (MTXC) - s'ha intentat deshabilitar"
                   " el Memory Access Enable (MAE), però no està implementat" );
      break;
      
      // SCC i BASEC;
    case 0x05: break;
      
      // Reserved
    case 0x08 ... 0x1f: break;
      
      // PAM
    case 0x2c:
      pam_reg_write ( 0, (uint8_t) (data>>8) );
      PC_MSG("PCI:MTXC.write16 - ADDR 0x2c falta un registre!!!!");
      break;
    case 0x2d ... 0x2f:
      pam_reg_write ( 2*(addr-0x2c)-1, (uint8_t) (data&0xFF) );
      pam_reg_write ( 2*(addr-0x2c), (uint8_t) (data>>8) );
      break;
      
    default:
      _warning ( _udata, "PCI:MTXC.write16 - addreça no implementada %02X\n",
                 addr );
    }
  
} // end pci_write16


static void
pci_jit_write16 (
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
      _pci_regs.pcicmd= (data&0x02)|0x04;
      if ( !(data&0x02) )
        _warning ( _udata,
                   "pci_write16 (MTXC) - s'ha intentat deshabilitar"
                   " el Memory Access Enable (MAE), però no està implementat" );
      break;
      
      // SCC i BASEC;
    case 0x05: break;
      
      // Reserved
    case 0x08 ... 0x1f: break;
      
      // PAM
    case 0x2c:
      pam_reg_jit_write ( 0, (uint8_t) (data>>8) );
      PC_MSG("PCI:MTXC.write16 - ADDR 0x2c falta un registre!!!!");
      break;
    case 0x2d ... 0x2f:
      pam_reg_jit_write ( 2*(addr-0x2c)-1, (uint8_t) (data&0xFF) );
      pam_reg_jit_write ( 2*(addr-0x2c), (uint8_t) (data>>8) );
      break;
      
    default:
      _warning ( _udata, "PCI:MTXC.write16 - addreça no implementada %02X\n",
                 addr );
    }
  
} // end pci_jit_write16


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
    case 0x04 ... 0x0f: break;
      
      // PAM
    case 0x16:
      PC_MSG("PCI:MTXC.write32 - ADDR 0x16 falta un registre!!!!");
      pam_reg_write ( 0, (uint8_t) ((data>>8)&0xFF) );
      pam_reg_write ( 1, (uint8_t) ((data>>16)&0xFF) );
      pam_reg_write ( 2, (uint8_t) (data>>24) );
      break;
    case 0x17:
      pam_reg_write ( 3, (uint8_t) (data&0xFF) );
      pam_reg_write ( 4, (uint8_t) ((data>>8)&0xFF) );
      pam_reg_write ( 5, (uint8_t) ((data>>16)&0xFF) );
      pam_reg_write ( 6, (uint8_t) (data>>24) );
      break;
      
    default:
      _warning ( _udata, "PCI:MTXC.write32 - addreça no implementada %02X\n",
                 addr );
    }
  
} // end pci_write32


static void
pci_jit_write32 (
                 const uint8_t  addr,
                 const uint32_t data
                 )
{

  switch ( addr )
    {

    case 0x00: break;
      
      // Reserved
    case 0x04 ... 0x0f: break;
      
      // PAM
    case 0x16:
      PC_MSG("PCI:MTXC.write32 - ADDR 0x16 falta un registre!!!!");
      pam_reg_jit_write ( 0, (uint8_t) ((data>>8)&0xFF) );
      pam_reg_jit_write ( 1, (uint8_t) ((data>>16)&0xFF) );
      pam_reg_jit_write ( 2, (uint8_t) (data>>24) );
      break;
    case 0x17:
      pam_reg_jit_write ( 3, (uint8_t) (data&0xFF) );
      pam_reg_jit_write ( 4, (uint8_t) ((data>>8)&0xFF) );
      pam_reg_jit_write ( 5, (uint8_t) ((data>>16)&0xFF) );
      pam_reg_jit_write ( 6, (uint8_t) (data>>24) );
      break;
      
    default:
      _warning ( _udata, "PCI:MTXC.write32 - addreça no implementada %02X\n",
                 addr );
    }
  
} // end pci_jit_write32


static const PC_PCIFunction MTXC_PCIFunction=
  {
    pci_read8,
    pci_read16,
    pci_read32,
    pci_write8,
    pci_write16,
    pci_write32,
    "82439TX (MTXC)"
  };

static const PC_PCIFunction MTXC_JIT_PCIFunction=
  {
    pci_read8,
    pci_read16,
    pci_read32,
    pci_jit_write8,
    pci_jit_write16,
    pci_jit_write32,
    "82439TX (MTXC - JIT)"
  };

static void
init_pci_regs (void)
{
  _pci_regs.pcicmd= 0x0006;
} // end init_pci_regs




/*********************/
/* FUNCIONS PRIVADES */
/*********************/

// MEMORY /////////////////////////////////////////////////////////////////////
static void
init_ram (
          const PC_Config *config
          )
{

  static const uint32_t RAM_SIZE_MB[PC_RAM_SIZE_SENTINEL]=
    { 4, 8, 16, 24, 32, 48, 64, 96, 128, 192, 256 };

  int i,page_size;
  

  // Reserva memòria.
  _ram.size= RAM_SIZE_MB[config->ram_size]*1024*1024;
  _ram.size_1= _ram.size-1;
  _ram.size_3= _ram.size-3;
  _ram.v= (uint8_t *) malloc ( _ram.size );
  if ( _ram.v == NULL )
    {
      fprintf ( stderr, "[EE] cannot allocate memory\n" );
      exit ( EXIT_FAILURE );
    }
  page_size= 1<<PAGE_CODE_BITS;
  memset ( _ram.v, 0, _ram.size );

  // Reserva memòria pàgines codi per al JIT.
  assert ( _ram.size>(unsigned int)page_size && _ram.size%page_size==0 );
  _ram.npages= _ram.size/page_size;
  _ram.pages_code= (bool *) malloc ( _ram.npages );
  if ( _ram.pages_code == NULL )
    {
      fprintf ( stderr, "[EE] cannot allocate memory\n" );
      exit ( EXIT_FAILURE );
    }
  for ( i= 0; i < _ram.npages; ++i )
    _ram.pages_code[i]= false;
  
  // Registres pam.
  for ( i= 0; i < 7; ++i )
    pam_reg_write ( i, 0x00 );
  
} // end init_ram


static void
close_ram (void)
{
  
  free ( _ram.pages_code );
  free ( _ram.v );
  
} // end close_ram


static uint8_t
pci_mem_read8 (
               const uint64_t addr
               )
{

  int i;
  uint8_t ret;

  
  if ( !PC_piix4_mem_read8 ( addr, &ret ) )
    {
      ret= 0xFF;
      for ( i= 0; _pci_devs[i] != NULL; ++i )
        if ( _pci_devs[i]->mem != NULL )
          if ( _pci_devs[i]->mem->read8 ( addr, &ret ) )
            break;
    }
  
  return ret;
  
} // end pci_mem_read8


static uint8_t
mem_read8 (
           void           *udata,
           const uint64_t  addr
           )
{
  
  uint8_t ret;
  uint64_t tmp;
  

  ret= 0xFF;
  
  if ( addr < 0x000A0000 ) ret= RAM_READ8(addr);

  // Video Buffer Area (A0000h–BFFFFh) 
  else if ( addr < 0x000C0000 ) ret= pci_mem_read8 ( addr );
  
  // Secció configurable via PAM
  else if ( addr < 0x00100000 )
    {
      if ( addr < 0x000F0000 )
        {
          tmp= addr&0x3FFFF;
          ret= _ram.pam[(tmp>>15)+1].flags[(tmp>>14)&0x1].read_enabled ?
            RAM_READ8(addr) : pci_mem_read8 ( addr );
        }
      else
        {
          ret= _ram.pam[0].flags[1].read_enabled ?
            RAM_READ8(addr) : pci_mem_read8 ( addr );
        }
    }
  
  // Més RAM
  else if ( addr < _ram.size ) ret= RAM_READ8(addr);

  // PCI
  else ret= pci_mem_read8 ( addr );
  
  return ret;
  /*
 error:
  fprintf ( stderr, "[EE] MEM_READ8 ADDR: %016lX\n", addr );
  exit ( EXIT_FAILURE );
  */
} // end mem_read8


static uint8_t
mem_jit_read8 (
               void           *udata,
               const uint64_t  addr,
               const bool      reading_data
               )
{
  
  uint8_t ret;
  uint64_t tmp;
  

  ret= 0xFF;
  
  if ( addr < 0x000A0000 )
    {
      ret= RAM_READ8(addr);
      if ( !reading_data ) _ram.pages_code[addr>>PAGE_CODE_BITS]= true;
    }

  // Video Buffer Area (A0000h–BFFFFh) 
  else if ( addr < 0x000C0000 ) ret= pci_mem_read8 ( addr );
  
  // Secció configurable via PAM
  else if ( addr < 0x00100000 )
    {
      if ( addr < 0x000F0000 )
        {
          tmp= addr&0x3FFFF;
          if ( _ram.pam[(tmp>>15)+1].flags[(tmp>>14)&0x1].read_enabled )
            {
              ret= RAM_READ8(addr);
              if ( !reading_data )
                _ram.pages_code[addr>>PAGE_CODE_BITS]= true;
            }
          else ret= pci_mem_read8 ( addr );
        }
      else
        {
          if ( _ram.pam[0].flags[1].read_enabled )
            {
              ret= RAM_READ8(addr);
              if ( !reading_data )
                _ram.pages_code[addr>>PAGE_CODE_BITS]= true;
            }
          else ret= pci_mem_read8 ( addr );
        }
    }
  
  // Més RAM
  else if ( addr < _ram.size )
    {
      ret= RAM_READ8(addr);
      if ( !reading_data )
        _ram.pages_code[addr>>PAGE_CODE_BITS]= true;
    }

  // PCI
  else ret= pci_mem_read8 ( addr );
  
  return ret;
  /*
 error:
  fprintf ( stderr, "[EE] MEM_READ8 ADDR: %016lX\n", addr );
  exit ( EXIT_FAILURE );
  */
} // end mem_jit_read8


static uint16_t
pci_mem_read16 (
                const uint64_t addr
                )
{

  int i;
  uint16_t ret;
  

  if ( !PC_piix4_mem_read16 ( addr, &ret ) )
    {
      ret= 0xFFFF;
      for ( i= 0; _pci_devs[i] != NULL; ++i )
        if ( _pci_devs[i]->mem != NULL )
          if ( _pci_devs[i]->mem->read16 ( addr, &ret ) )
            break;
    }
  
  return ret;
  
} // end pci_mem_read16


// Les seccions 'borderline' les llegisc cridant a funcions inferiors
// encara que siga més costós.
static uint16_t
mem_read16_bl (
               void           *udata,
               const uint64_t  addr,
               const bool      use_jit
               )
{

  uint16_t ret;


  if ( use_jit )
    ret=
      ((uint16_t) mem_jit_read8 ( udata, addr, true )) |
      (((uint16_t) mem_jit_read8 ( udata, addr+1, true ))<<8)
      ;
  else
    ret=
      ((uint16_t) mem_read8 ( udata, addr )) |
      (((uint16_t) mem_read8 ( udata, addr+1 ))<<8)
      ;

  return ret;
  
} // end mem_read16_bl


static uint16_t
mem_read16_base (
                 void           *udata,
                 const uint64_t  addr,
                 const bool      use_jit
                 )
{
  
  uint16_t ret;
  uint64_t tmp;
  
  
  ret= 0xFFFF;
  
  if ( addr < 0x000A0000 )
    ret= addr!=0x0009FFFF ?
      RAM_READ16(addr) : mem_read16_bl ( udata, addr, use_jit );
  
  // Video Buffer Area (A0000h–BFFFFh) 
  else if ( addr < 0x000C0000 )
    ret= addr!=0x000BFFFF ?
      pci_mem_read16 ( addr ) : mem_read16_bl ( udata, addr, use_jit );
  
  // Secció configurable via PAM
  else if ( addr < 0x00100000 )
    {
      if ( addr < 0x000F0000 )
        {
          if ( (addr&0x3FFF) != 0x3FFF )
            {
              tmp= addr&0x3FFFF;
              ret= _ram.pam[(tmp>>15)+1].flags[(tmp>>14)&0x1].read_enabled ?
                RAM_READ16(addr) : pci_mem_read16 ( addr );
            }
          else ret= mem_read16_bl ( udata, addr, use_jit );
        }
      else if ( addr != 0x000FFFFF )
        {
          ret= _ram.pam[0].flags[1].read_enabled ?
            RAM_READ16(addr) : pci_mem_read16 ( addr );
        }
      else ret= mem_read16_bl ( udata, addr, use_jit );
    }
  
  // Més RAM
  else if ( addr < _ram.size )
    ret= addr!=_ram.size_1 ?
      RAM_READ16(addr) : mem_read16_bl ( udata, addr, use_jit );
  
  // PCI
  else ret= pci_mem_read16 ( addr );
  
  return ret;
  /*
 error:
  fprintf ( stderr, "[EE] MEM_READ16 ADDR: %016lX\n", addr );
  exit ( EXIT_FAILURE );
  */
} // end mem_read16_base


static uint16_t
mem_read16 (
            void           *udata,
            const uint64_t  addr
            )
{
  return mem_read16_base ( udata, addr, false );
} // end mem_read16


static uint16_t
mem_jit_read16 (
                void           *udata,
                const uint64_t  addr
                )
{
  return mem_read16_base ( udata, addr, true );
} // end mem_jit_read16


static uint32_t
pci_mem_read32 (
                const uint64_t addr
                )
{
  
  int i;
  uint32_t ret;
  
  
  if ( !PC_piix4_mem_read32 ( addr, &ret ) )
    {
      ret= 0xFFFFFFFF;
      for ( i= 0; _pci_devs[i] != NULL; ++i )
        if ( _pci_devs[i]->mem != NULL )
          if ( _pci_devs[i]->mem->read32 ( addr, &ret ) )
            break;
    }
  
  return ret;
  
} // end pci_mem_read32


// Les seccions 'borderline' les llegisc cridant a funcions inferiors
// encara que siga més costós.
static uint32_t
mem_read32_bl (
               void           *udata,
               const uint64_t  addr,
               const bool      use_jit
               )
{

  uint16_t ret;


  if ( use_jit )
    ret=
    ((uint32_t) mem_jit_read16 ( udata, addr )) |
    (((uint32_t) mem_jit_read16 ( udata, addr+2 ))<<16)
    ;
  else
    ret=
      ((uint32_t) mem_read16 ( udata, addr )) |
      (((uint32_t) mem_read16 ( udata, addr+2 ))<<16)
      ;
  
  return ret;
  
} // end mem_read32_bl


static uint32_t
mem_read32_base (
                 void           *udata,
                 const uint64_t  addr,
                 const bool      use_jit
                 )
{
    
  uint32_t ret;
  uint64_t tmp;
  
  
  ret= 0xFFFFFFFF;
  
  if ( addr < 0x000A0000 )
    ret= addr<0x0009FFFD ?
      RAM_READ32(addr) : mem_read32_bl ( udata, addr, use_jit );
  
  // Video Buffer Area (A0000h–BFFFFh) 
  else if ( addr < 0x000C0000 )
    ret= addr<0x000BFFFD ?
      pci_mem_read32 ( addr ) : mem_read32_bl ( udata, addr, use_jit );
  
  // Secció configurable via PAM
  else if ( addr < 0x00100000 )
    {
      if ( addr < 0x000F0000 )
        {
          if ( (addr&0x3FFF) < 0x3FFD )
            {
              tmp= addr&0x3FFFF;
              ret= _ram.pam[(tmp>>15)+1].flags[(tmp>>14)&0x1].read_enabled ?
                RAM_READ32(addr) : pci_mem_read32 ( addr );
            }
          else ret= mem_read32_bl ( udata, addr, use_jit );
        }
      else if ( addr < 0x000FFFFD )
        {
          ret= _ram.pam[0].flags[1].read_enabled ?
            RAM_READ32(addr) : pci_mem_read32 ( addr );
        }
      else ret= mem_read32_bl ( udata, addr, use_jit );
    }
  
  // Més RAM
  else if ( addr < _ram.size )
    ret= addr<_ram.size_3  ?
      RAM_READ32(addr) : mem_read32_bl ( udata, addr, use_jit );
  
  // PCI
  else ret= pci_mem_read32 ( addr );
  
  return ret;
  /*
 error:
  fprintf ( stderr, "[EE] MEM_READ32 ADDR: %016lX\n", addr );
  exit ( EXIT_FAILURE );
  */
} // end mem_read32_base


static uint32_t
mem_read32 (
            void           *udata,
            const uint64_t  addr
            )
{
  return mem_read32_base ( udata, addr, false );
} // end mem_read32


static uint32_t
mem_jit_read32 (
                void           *udata,
                const uint64_t  addr
                )
{
  return mem_read32_base ( udata, addr, true );
} // end mem_jit_read32


static uint64_t
pci_mem_read64 (
                const uint64_t addr
                )
{

  int i;
  uint64_t ret;
  

  ret= 0xFFFFFFFFFFFFFFFF;
  for ( i= 0; _pci_devs[i] != NULL; ++i )
    if ( _pci_devs[i]->mem != NULL )
      if ( _pci_devs[i]->mem->read64 ( addr, &ret ) )
        break;
  
  return ret;
  
} // end pci_mem_read64


static uint64_t
mem_read64 (
            void           *udata,
            const uint64_t  addr
            )
{
  
  uint64_t ret;


  ret= pci_mem_read64 ( addr );
  //goto error;
  
  return ret;

  /*
 error:
  fprintf ( stderr, "[EE] MEM_READ64 ADDR: %016lX\n", addr );
  exit ( EXIT_FAILURE );
  */
  
} // end mem_read64


static void
pci_mem_write8 (
                const uint64_t addr,
                const uint8_t  data
                )
{

  int i;
  
  
  if ( !PC_piix4_mem_write8 ( addr, data ) )
    for ( i= 0; _pci_devs[i] != NULL; ++i )
      if ( _pci_devs[i]->mem != NULL )
        if ( _pci_devs[i]->mem->write8 ( addr, data ) )
          break;
  
} // end pci_mem_write8


static void
mem_write8 (
            void           *udata,
            const uint64_t  addr,
            const uint8_t   data
            )
{
  
  uint64_t tmp;
  
  
  if ( addr < 0x000A0000 ) RAM_WRITE8(addr,data);
  
  // Video Buffer Area (A0000h–BFFFFh) 
  else if ( addr < 0x000C0000 ) pci_mem_write8 ( addr, data );
  
  // Secció configurable via PAM
  else if ( addr < 0x00100000 )
    {
      if ( addr < 0x000F0000 )
        {
          tmp= addr&0x3FFFF;
          if ( _ram.pam[(tmp>>15)+1].flags[(tmp>>14)&0x1].write_enabled )
            RAM_WRITE8(addr,data);
          else pci_mem_write8 ( addr, data );
        }
      else
        {
          if ( _ram.pam[0].flags[1].write_enabled )
            RAM_WRITE8(addr,data);
          else pci_mem_write8 ( addr, data );
        }
    }
  
  // Més RAM
  else if ( addr < _ram.size ) RAM_WRITE8(addr,data);

  // PCI
  else pci_mem_write8 ( addr, data );
  
} // end mem_write8


static void
page_code_changed (
                   const uint64_t addr
                   )
{

  int pb,pe,p;

  
  // Si la pàgina s'ha modificat completament es modifica el flag a
  // fals. Es tornarà a ficar a true si es torna a llegir instruccions
  // d'eixa pàgina.
  //
  // ATENCIÓ!!! Pot ser que a l'esborrar una pàgina s'esborre una
  // altra que estiguera marcada com a codi i no es desmarque. No
  // passa res, simplement eixa pàgina es tornarà a comprovar. Estar
  // marcat sols s'utilitza per a saber si es crida o no a
  // jit_addr_changed.
  if ( IA32_jit_addr_changed ( PC_CPU_JIT, addr ) )
    {
      pb= ((addr>>PC_JIT_BITS_PAGE)<<(PC_JIT_BITS_PAGE-PAGE_CODE_BITS));
      pe= pb + (1<<(PC_JIT_BITS_PAGE-PAGE_CODE_BITS));
      for ( p= pb; p < pe; ++p )
        _ram.pages_code[p]= false;
    }
  
} // end page_code_changed


static void
mem_jit_write8 (
                void           *udata,
                const uint64_t  addr,
                const uint8_t   data
                )
{
  
  uint64_t tmp;
  int i,j,page;
  
  
  if ( addr < 0x000A0000 )
    {
      page= addr>>PAGE_CODE_BITS;
      if ( _ram.pages_code[page] ) page_code_changed ( addr );
      RAM_WRITE8(addr,data);
    }

  // NOTA!! Teoricament es pot llegir codi d'aquesta àrea però:
  //   1) Si passa segurament no tinga ningun sentit, siga algun bug.
  //   2) El que s'escriu i es llig moltes vegades és diferent
  // per tant vaig a passar de gestionar esta àrea.
  // Video Buffer Area (A0000h–BFFFFh) 
  else if ( addr < 0x000C0000 ) pci_mem_write8 ( addr, data );
  
  // Secció configurable via PAM
  else if ( addr < 0x00100000 )
    {
      if ( addr < 0x000F0000 )
        {
          tmp= addr&0x3FFFF;
          i= (tmp>>15)+1;
          j= (tmp>>14)&0x1;
          if ( _ram.pam[i].flags[j].write_enabled )
            {
              if ( _ram.pam[i].flags[j].read_enabled )
                {
                  page= addr>>PAGE_CODE_BITS;
                  if ( _ram.pages_code[page] ) page_code_changed ( addr );
                }
              RAM_WRITE8(addr,data);
            }
          else pci_mem_write8 ( addr, data );
        }
      else
        {
          if ( _ram.pam[0].flags[1].write_enabled )
            {
              if ( _ram.pam[0].flags[1].read_enabled )
                {
                  page= addr>>PAGE_CODE_BITS;
                  if ( _ram.pages_code[page] ) page_code_changed ( addr );
                }
              RAM_WRITE8(addr,data);
            }
          else pci_mem_write8 ( addr, data );
        }
    }
  
  // Més RAM
  else if ( addr < _ram.size )
    {
      page= addr>>PAGE_CODE_BITS;
      if ( _ram.pages_code[page] ) page_code_changed ( addr );
      RAM_WRITE8(addr,data);
    }

  // PCI
  else pci_mem_write8 ( addr, data );
  
} // end mem_jit_write8


static void
pci_mem_write16 (
                 const uint64_t addr,
                 const uint16_t data
                 )
{

  int i;
  

  if ( !PC_piix4_mem_write16 ( addr, data ) )
    for ( i= 0; _pci_devs[i] != NULL; ++i )
      if ( _pci_devs[i]->mem != NULL )
        if ( _pci_devs[i]->mem->write16 ( addr, data ) )
          break;
  
} // end pci_mem_write16


static void
mem_write16_bl (
                void           *udata,
                const uint64_t  addr,
                const uint16_t  data
                )
{

  mem_write8 ( udata, addr, (uint8_t) (data&0xFF) );
  mem_write8 ( udata, addr+1, (uint8_t) (data>>8) );
  
} // end mem_write16_bl


static void
mem_write16 (
             void           *udata,
             const uint64_t  addr,
             const uint16_t  data
             )
{
  
  uint64_t tmp;
  
  
  if ( addr < 0x000A0000 )
    {
      if ( addr != 0x0009FFFF ) { RAM_WRITE16(addr,data); }
      else mem_write16_bl ( udata, addr, data );
    }
  
  // Video Buffer Area (A0000h–BFFFFh) 
  else if ( addr < 0x000C0000 )
    {
      if ( addr!=0x000BFFFF ) pci_mem_write16 ( addr, data );
      else mem_write16_bl ( udata, addr, data );
    }
  
  // Secció configurable via PAM
  else if ( addr < 0x00100000 )
    {
      if ( addr < 0x000F0000 )
        {
          if ( (addr&0x3FFF) != 0x3FFF )
            {
              tmp= addr&0x3FFFF;
              if ( _ram.pam[(tmp>>15)+1].flags[(tmp>>14)&0x1].write_enabled )
                { RAM_WRITE16(addr,data); }
              else pci_mem_write16 ( addr, data );
            }
          else mem_write16_bl ( udata, addr, data );
        }
      else if ( addr != 0x000FFFFF )
        {
          if ( _ram.pam[0].flags[1].write_enabled )
            { RAM_WRITE16(addr,data); }
          else pci_mem_write16 ( addr, data );
        }
      else mem_write16_bl ( udata, addr, data );
    }
  
  // Més RAM
  else if ( addr < _ram.size )
    {
      if ( addr != _ram.size_1 ) { RAM_WRITE16(addr,data); }
      else mem_write16_bl ( udata, addr, data );
    }

  // PCI
  else pci_mem_write16 ( addr, data );
  
} // end mem_write16


static void
mem_jit_write16_bl (
                    void           *udata,
                    const uint64_t  addr,
                    const uint16_t  data
                    )
{

  mem_jit_write8 ( udata, addr, (uint8_t) (data&0xFF) );
  mem_jit_write8 ( udata, addr+1, (uint8_t) (data>>8) );
  
} // end mem_jit_write16_bl


static void
mem_jit_write16 (
                 void           *udata,
                 const uint64_t  addr,
                 uint16_t        data
                 )
{
  
  uint64_t tmp;
  int i,j,page;
  
  
  if ( addr < 0x000A0000 )
    {
      if ( addr != 0x0009FFFF )
        {
          page= addr>>PAGE_CODE_BITS;
          if ( _ram.pages_code[page] ) page_code_changed ( addr );
          RAM_WRITE16(addr,data);
        }
      else mem_jit_write16_bl ( udata, addr, data );
    }

  // NOTA!! Teoricament es pot llegir codi d'aquesta àrea però:
  //   1) Si passa segurament no tinga ningun sentit, siga algun bug.
  //   2) El que s'escriu i es llig moltes vegades és diferent
  // per tant vaig a passar de gestionar esta àrea.
  // Video Buffer Area (A0000h–BFFFFh) 
  else if ( addr < 0x000C0000 )
    {
      if ( addr!=0x000BFFFF ) pci_mem_write16 ( addr, data );
      else mem_jit_write16_bl ( udata, addr, data );
    }
  
  // Secció configurable via PAM
  else if ( addr < 0x00100000 )
    {
      if ( addr < 0x000F0000 )
        {
          if ( (addr&0x3FFF) != 0x3FFF )
            {
              tmp= addr&0x3FFFF;
              i= (tmp>>15)+1;
              j= (tmp>>14)&0x1;
              if ( _ram.pam[i].flags[j].write_enabled )
                {
                  if ( _ram.pam[i].flags[j].read_enabled )
                    {
                      page= addr>>PAGE_CODE_BITS;
                      if ( _ram.pages_code[page] ) page_code_changed ( addr );
                    }
                  RAM_WRITE16(addr,data);
                }
              else pci_mem_write16 ( addr, data );
            }
          else mem_jit_write16_bl ( udata, addr, data );
        }
      else if ( addr != 0x000FFFFF )
        {
          if ( _ram.pam[0].flags[1].write_enabled )
            {
              if ( _ram.pam[0].flags[1].read_enabled )
                {
                  page= addr>>PAGE_CODE_BITS;
                  if ( _ram.pages_code[page] ) page_code_changed ( addr );
                }
              RAM_WRITE16(addr,data);
            }
          else pci_mem_write16 ( addr, data );
        }
      else mem_jit_write16_bl ( udata, addr, data );
    }
  
  // Més RAM
  else if ( addr < _ram.size )
    {
      if ( addr != _ram.size_1 )
        {
          page= addr>>PAGE_CODE_BITS;
          if ( _ram.pages_code[page] ) page_code_changed ( addr );
          RAM_WRITE16(addr,data);
        }
      else mem_jit_write16_bl ( udata, addr, data );
    }
  
  // PCI
  else pci_mem_write16 ( addr, data );
  
} // end mem_jit_write16


static void
pci_mem_write32 (
                 const uint64_t addr,
                 const uint32_t data
                 )
{

  
  int i;
  

  if ( !PC_piix4_mem_write32 ( addr, data ) )
    for ( i= 0; _pci_devs[i] != NULL; ++i )
      if ( _pci_devs[i]->mem != NULL )
        if ( _pci_devs[i]->mem->write32 ( addr, data ) )
          break;
  
} // end pci_mem_write32


static void
mem_write32_bl (
                void           *udata,
                const uint64_t  addr,
                const uint32_t  data
                )
{
  
  mem_write16 ( udata, addr, (uint16_t) (data&0xFFFF) );
  mem_write16 ( udata, addr+2, (uint16_t) (data>>16) );
  
} // end mem_write32_bl


static void
mem_write32 (
             void           *udata,
             const uint64_t  addr,
             const uint32_t  data
             )
{
  
  uint64_t tmp;
  
  
  if ( addr < 0x000A0000 )
    {
      if ( addr<0x0009FFFD ) { RAM_WRITE32(addr,data); }
      else mem_write32_bl ( udata, addr, data );
    }
  
  // Video Buffer Area (A0000h–BFFFFh) 
  else if ( addr < 0x000C0000 )
    {
      if ( addr<0x000BFFFD ) pci_mem_write32 ( addr, data );
      else mem_write32_bl ( udata, addr, data );
    }
  
  // Secció configurable via PAM
  else if ( addr < 0x00100000 )
    {
      if ( addr < 0x000F0000 )
        {
          if ( (addr&0x3FFF) < 0x3FFD )
            {
              tmp= addr&0x3FFFF;
              if ( _ram.pam[(tmp>>15)+1].flags[(tmp>>14)&0x1].write_enabled )
                { RAM_WRITE32(addr,data); }
              else pci_mem_write32 ( addr, data );
            }
          else mem_write32_bl ( udata, addr, data );
        }
      else if ( addr < 0x000FFFFD )
        {
          if ( _ram.pam[0].flags[1].write_enabled )
            { RAM_WRITE32(addr,data); }
          else pci_mem_write32 ( addr, data );
        }
      else mem_write32_bl ( udata, addr, data );
    }
  
  // Més RAM
  else if ( addr < _ram.size )
    {
      if ( addr<_ram.size_3 ) { RAM_WRITE32(addr,data); }
      else mem_write32_bl ( udata, addr, data );
    }

  // PCI
  else pci_mem_write32 ( addr, data );
  
} // end mem_write32


static void
mem_jit_write32_bl (
                    void           *udata,
                    const uint64_t  addr,
                    const uint32_t  data
                    )
{
  
  mem_jit_write16 ( udata, addr, (uint16_t) (data&0xFFFF) );
  mem_jit_write16 ( udata, addr+2, (uint16_t) (data>>16) );
  
} // end mem_jit_write32_bl


static void
mem_jit_write32 (
                 void           *udata,
                 const uint64_t  addr,
                 const uint32_t  data
                 )
{
  
  uint64_t tmp;
  int i,j,page;
  
  
  if ( addr < 0x000A0000 )
    {
      if ( addr < 0x0009FFFD )
        {
          page= addr>>PAGE_CODE_BITS;
          if ( _ram.pages_code[page] ) page_code_changed ( addr );
          RAM_WRITE32(addr,data);
        }
      else mem_jit_write32_bl ( udata, addr, data );
    }

  // NOTA!! Teoricament es pot llegir codi d'aquesta àrea però:
  //   1) Si passa segurament no tinga ningun sentit, siga algun bug.
  //   2) El que s'escriu i es llig moltes vegades és diferent
  // per tant vaig a passar de gestionar esta àrea.
  // Video Buffer Area (A0000h–BFFFFh) 
  else if ( addr < 0x000C0000 )
    {
      if ( addr<0x000BFFFD ) pci_mem_write32 ( addr, data );
      else mem_jit_write32_bl ( udata, addr, data );
    }
  
  // Secció configurable via PAM
  else if ( addr < 0x00100000 )
    {
      if ( addr < 0x000F0000 )
        {
          if ( (addr&0x3FFF) < 0x3FFD )
            {
              tmp= addr&0x3FFFF;
              i= (tmp>>15)+1;
              j= (tmp>>14)&0x1;
              if ( _ram.pam[i].flags[j].write_enabled )
                {
                  if ( _ram.pam[i].flags[j].read_enabled )
                    {
                      page= addr>>PAGE_CODE_BITS;
                      if ( _ram.pages_code[page] ) page_code_changed ( addr );
                    }
                  RAM_WRITE32(addr,data);
                }
              else pci_mem_write32 ( addr, data );
            }
          else mem_jit_write32_bl ( udata, addr, data );
        }
      else if ( addr < 0x000FFFFD )
        {
          if ( _ram.pam[0].flags[1].write_enabled )
            {
              if ( _ram.pam[0].flags[1].read_enabled )
                {
                  page= addr>>PAGE_CODE_BITS;
                  if ( _ram.pages_code[page] ) page_code_changed ( addr );
                }
              RAM_WRITE32(addr,data);
            }
          else pci_mem_write32 ( addr, data );
        }
      else mem_jit_write32_bl ( udata, addr, data );
    }
  
  // Més RAM
  else if ( addr < _ram.size )
    {
      if ( addr<_ram.size_3 )
        {
          page= addr>>PAGE_CODE_BITS;
          if ( _ram.pages_code[page] ) page_code_changed ( addr );
          RAM_WRITE32(addr,data);
        }
      else mem_jit_write32_bl ( udata, addr, data );
    }

  // PCI
  else pci_mem_write32 ( addr, data );
  
} // end mem_jit_write32


static uint8_t
mem_read8_trace (
                 void           *udata,
                 const uint64_t  addr
                 )
{

  uint8_t ret;
  
  
  ret= mem_read8 ( udata, addr );
  _mem_access ( PC_READ8, addr, (uint64_t) ret, _udata );

  return ret;
  
} // end mem_read8_trace


static uint8_t
mem_jit_read8_trace (
                     void           *udata,
                     const uint64_t  addr,
                     const bool      reading_data
                     )
{

  uint8_t ret;
  
  
  ret= mem_jit_read8 ( udata, addr, reading_data );
  _mem_access ( PC_READ8, addr, (uint64_t) ret, _udata );

  return ret;
  
} // end mem_jit_read8_trace


static uint16_t
mem_read16_trace (
                  void           *udata,
                  const uint64_t  addr
                  )
{

  uint16_t ret;


  ret= mem_read16 ( udata, addr );
  _mem_access ( PC_READ16, addr, (uint64_t) ret, _udata );

  return ret;
  
} // end mem_read16_trace


static uint16_t
mem_jit_read16_trace (
                      void           *udata,
                      const uint64_t  addr
                      )
{

  uint16_t ret;


  ret= mem_jit_read16 ( udata, addr );
  _mem_access ( PC_READ16, addr, (uint64_t) ret, _udata );

  return ret;
  
} // end mem_jit_read16_trace


static uint32_t
mem_read32_trace (
                  void           *udata,
                  const uint64_t  addr
                  )
{

  uint32_t ret;


  ret= mem_read32 ( udata, addr );
  _mem_access ( PC_READ32, addr, (uint64_t) ret, _udata );

  return ret;
  
} // end mem_read32_trace


static uint32_t
mem_jit_read32_trace (
                      void           *udata,
                      const uint64_t  addr
                      )
{

  uint32_t ret;


  ret= mem_jit_read32 ( udata, addr );
  _mem_access ( PC_READ32, addr, (uint64_t) ret, _udata );
  
  return ret;
  
} // end mem_jit_read32_trace


static uint64_t
mem_read64_trace (
                  void           *udata,
                  const uint64_t  addr
                  )
{

  uint64_t ret;


  ret= mem_read64 ( udata, addr );
  _mem_access ( PC_READ64, addr, ret, _udata );

  return ret;
  
} // end mem_read64_trace


static void
mem_write8_trace (
                  void           *udata,
                  const uint64_t  addr,
                  const uint8_t   data
                  )
{

  mem_write8 ( udata, addr, data );
  _mem_access ( PC_WRITE8, addr, (uint64_t) data, _udata );
  
} // end mem_write8_trace


static void
mem_jit_write8_trace (
                      void           *udata,
                      const uint64_t  addr,
                      const uint8_t   data
                      )
{

  mem_jit_write8 ( udata, addr, data );
  _mem_access ( PC_WRITE8, addr, (uint64_t) data, _udata );
  
} // end mem_jit_write8_trace


static void
mem_write16_trace (
                   void           *udata,
                   const uint64_t  addr,
                   const uint16_t  data
                   )
{

  mem_write16 ( udata, addr, data );
  _mem_access ( PC_WRITE16, addr, (uint64_t) data, _udata );
  
} // end mem_write16_trace


static void
mem_jit_write16_trace (
                       void           *udata,
                       const uint64_t  addr,
                       const uint16_t  data
                       )
{

  mem_jit_write16 ( udata, addr, data );
  _mem_access ( PC_WRITE16, addr, (uint64_t) data, _udata );
  
} // end mem_jit_write16_trace


static void
mem_write32_trace (
                   void           *udata,
                   const uint64_t  addr,
                   const uint32_t  data
                   )
{

  mem_write32 ( udata, addr, data );
  _mem_access ( PC_WRITE32, addr, (uint64_t) data, _udata );
  
} // end mem_write32_trace


static void
mem_jit_write32_trace (
                       void           *udata,
                       const uint64_t  addr,
                       const uint32_t  data
                       )
{

  mem_jit_write32 ( udata, addr, data );
  _mem_access ( PC_WRITE32, addr, (uint64_t) data, _udata );
  
} // end mem_jit_write32_trace


// CONFDATA ///////////////////////////////////////////////////////////////////
static uint8_t
confdata_read8 (
                const uint8_t low_addr
                )
{

  if ( _pci_api.func == NULL )
    {
      _warning ( _udata,
                 "s'ha intentat llegir 1 BYTE del bus PCI sense"
                 " establir una connexió" );
      return 0xff;
    }

  return _pci_api.func->read8 ( (_pci_api.reg<<2)|low_addr );
  
} // end confdata_read8


static uint16_t
confdata_read16 (
                 const uint8_t low_addr
                 )
{
  
  if ( _pci_api.func == NULL )
    {
      _warning ( _udata,
                 "s'ha intentat llegir 1 WORD del bus PCI sense"
                 " establir una connexió" );
      return 0xffff;
    }

  return _pci_api.func->read16 ( (_pci_api.reg<<1)|low_addr );
  
} // end confdata_read16


static uint32_t
confdata_read32 (void)
{
  
  if ( _pci_api.func == NULL )
    {
      _warning ( _udata,
                 "s'ha intentat llegir 1 DWORD del bus PCI sense"
                 " establir una connexió" );
      return 0xffff;
    }

  return _pci_api.func->read32 ( _pci_api.reg );
  
} // end confdata_read32


static void
confdata_write8 (
                 const uint8_t low_addr,
                 const uint8_t data
                 )
{

  if ( _pci_api.func == NULL )
    {
      _warning ( _udata,
                 "s'ha intentat escriure 1 BYTE en el bus PCI sense"
                 " establir una connexió" );
      return;
    }
  _pci_api.func->write8 ( (_pci_api.reg<<2)|low_addr, data );
  
} // end conf_data_write8


static void
confdata_write16 (
                  const uint8_t  low_addr,
                  const uint16_t data
                  )
{
  
  if ( _pci_api.func == NULL )
    {
      _warning ( _udata,
                 "s'ha intentat escriure 1 WORD en el bus PCI sense"
                 " establir una connexió" );
      return;
    }
  _pci_api.func->write16 ( (_pci_api.reg<<1)|low_addr, data );
  
} // end conf_data_write16


static void
confdata_write32 (
                  const uint32_t data
                  )
{
  
  if ( _pci_api.func == NULL )
    {
      _warning ( _udata,
                 "s'ha intentat escriure 1 DWORD en el bus PCI sense"
                 " establir una connexió" );
      return;
    }
  _pci_api.func->write32 ( _pci_api.reg, data );
  
} // end confdata_write32


static uint8_t
confdata_read8_trace (
                      const uint8_t low_addr
                      )
{
  
  uint8_t ret;
  
  
  ret= confdata_read8 ( low_addr );
  _pci_reg_access ( PC_PCI_READ8,
                    (_pci_api.reg<<2)|low_addr, (uint32_t) ret,
                    _pci_api.func!=NULL ? _pci_api.func->id : NULL,
                    _udata );
  
  return ret;
  
} // end confdata_read8_trace


static uint16_t
confdata_read16_trace (
                       const uint8_t low_addr
                       )
{

  uint16_t ret;
  
  
  ret= confdata_read16 ( low_addr );
  _pci_reg_access ( PC_PCI_READ16,
                    (_pci_api.reg<<2)|(low_addr<<1), (uint32_t) ret,
                    _pci_api.func!=NULL ? _pci_api.func->id : NULL,
                    _udata );
  
  return ret;
  
} // end confdata_read16_trace


static uint32_t
confdata_read32_trace (void)
{

  uint32_t ret;
  
  
  ret= confdata_read32 ();
  _pci_reg_access ( PC_PCI_READ32,
                    _pci_api.reg<<2, (uint32_t) ret,
                    _pci_api.func!=NULL ? _pci_api.func->id : NULL,
                    _udata );

  return ret;
  
} // end confdata_read32_trace


static void
confdata_write8_trace (
                       const uint8_t low_addr,
                       const uint8_t data
                       )
{
  confdata_write8 ( low_addr, data );
  _pci_reg_access ( PC_PCI_WRITE8,
                    (_pci_api.reg<<2)|low_addr, (uint32_t) data,
                    _pci_api.func!=NULL ? _pci_api.func->id : NULL,
                    _udata );  
} // end conf_data_write8_trace


static void
confdata_write16_trace (
                        const uint8_t  low_addr,
                        const uint16_t data
                        )
{
  confdata_write16 ( low_addr, data );
  _pci_reg_access ( PC_PCI_WRITE16,
                    (_pci_api.reg<<2)|(low_addr<<1), (uint32_t) data,
                    _pci_api.func!=NULL ? _pci_api.func->id : NULL,
                    _udata );
} // end conf_data_write16_trace


static void
confdata_write32_trace (
                        const uint32_t data
                        )
{
  confdata_write32 ( data );
  _pci_reg_access ( PC_PCI_WRITE32, _pci_api.reg<<2, data,
                    _pci_api.func!=NULL ? _pci_api.func->id : NULL,
                    _udata );  
} // end confdata_write32_trace




/**********************/
/* FUNCIONS PÚBLIQUES */
/**********************/

void
PC_mtxc_init (
              PC_Warning            *warning,
              PC_MemAccess          *mem_access,
              PC_PCIRegAccess       *pci_reg_access,
              const PC_PCICallbacks *pci_devs[], // Acava en NULL
              void                  *udata,
              const PC_Config       *config
              )
{

  int i;

  
  // Callbacks.
  _warning= warning;
  _mem_access= mem_access;
  _pci_reg_access= pci_reg_access;
  _udata= udata;

  // Config.
  _config= config;
  
  // Dispositius PCI.
  for ( i= 0; pci_devs[i] != NULL; ++i )
    _pci_devs[i]= pci_devs[i];
  for ( ; i < PC_PCI_DEVICE_NULL; ++i )
    _pci_devs[i]= NULL;
  
  // PCI API
  _pci_api.func= NULL;
  _pci_api.reg= 0;
  _pci_api.confadd= 0x0;

  // Trace confdata
  _confdata_read8= confdata_read8;
  _confdata_read16= confdata_read16;
  _confdata_read32= confdata_read32;
  _confdata_write8= confdata_write8;
  _confdata_write16= confdata_write16;
  _confdata_write32= confdata_write32;

  // Inicialitza memòria.
  init_ram ( config );
  init_pci_regs ();
  PC_CPU.mem_read8= mem_read8;
  PC_CPU.mem_read16= mem_read16;
  PC_CPU.mem_read32= mem_read32;
  PC_CPU.mem_read64= mem_read64;
  PC_CPU.mem_write8= mem_write8;
  PC_CPU.mem_write16= mem_write16;
  PC_CPU.mem_write32= mem_write32;
  PC_CPU_JIT->mem_read8= mem_jit_read8;
  PC_CPU_JIT->mem_read16= mem_jit_read16;
  PC_CPU_JIT->mem_read32= mem_jit_read32;
  PC_CPU_JIT->mem_read64= mem_read64;
  PC_CPU_JIT->mem_write8= mem_jit_write8;
  PC_CPU_JIT->mem_write16= mem_jit_write16;
  PC_CPU_JIT->mem_write32= mem_jit_write32;
  
} // end PC_mtxc_init


void
PC_mtxc_reset (
               const bool use_jit
               )
{

  int i;

  
  // PCI API
  _pci_api.func= NULL;
  _pci_api.reg= 0;
  _pci_api.confadd= 0x0;
  
  // Inicialitza memòria.
  memset ( _ram.v, 0, _ram.size );
  for ( i= 0; i < _ram.npages; ++i )
    _ram.pages_code[i]= false;
  // --> Registres pam.
  for ( i= 0; i < 7; ++i )
    pam_reg_write ( i, 0x00 );
  init_pci_regs ();

  // PCI
  for ( i= 0; _pci_devs[i] != NULL; ++i )
    if ( _pci_devs[i]->reset != NULL )
      _pci_devs[i]->reset ();

  // JIT
  if ( use_jit ) IA32_jit_clear_areas ( PC_CPU_JIT );
  
} // end PC_mtxc_reset


void
PC_mtxc_close (void)
{
  close_ram ();
} // end PC_mtxc_closes


uint8_t
PC_mtxc_confdata_read8 (
                        const uint8_t low_addr
                        )
{
  return _confdata_read8 ( low_addr );
} // end PC_mtxc_confdata_read8


uint16_t
PC_mtxc_confdata_read16 (
                         const uint8_t low_addr
                         )
{
  return _confdata_read16 ( low_addr );
} // end PC_mtxc_confdata_read16


uint32_t
PC_mtxc_confdata_read32 (void)
{
  return _confdata_read32 ();  
} // end PC_mtxc_confdata_read32


void
PC_mtxc_confdata_write8 (
                         const uint8_t low_addr,
                         const uint8_t data
                         )
{
  _confdata_write8 ( low_addr, data );  
} // end PC_mtxc_conf_data_write8


void
PC_mtxc_confdata_write16 (
                          const uint8_t  low_addr,
                          const uint16_t data
                          )
{
  _confdata_write16 ( low_addr, data );
} // end PC_mtxc_conf_data_write16


void
PC_mtxc_confdata_write32 (
                          const uint32_t data
                         )
{
  _confdata_write32 ( data );
} // end PC_mtxc_confdata_write32


uint32_t
PC_mtxc_confadd_read (void)
{
  return _pci_api.confadd;
} // end PC_mtxc_confadd_read


void
PC_mtxc_confadd_write (
                       const uint32_t data,
                       const bool     use_jit
                       )
{

  uint8_t bus,dev,func,reg,idev;

  
  _pci_api.confadd= data;
  
  // Deshabilita.
  if ( (data&0x80000000) == 0 )
    {
      _pci_api.func= NULL;
      return;
    }

  // Descodifica.
  bus= (data>>16)&0xFF;
  dev= (data>>11)&0x1F;
  func= (data>>8)&0x7;
  reg= (data>>2)&0x3F;

  // Actualitza api.
  _pci_api.reg= reg;
  // --> Busquem dispositiu en este bus. 
  if ( bus == 0 )
    {
      switch ( dev )
        {
          
        case 0: // MTXC
          if ( func == 0 )
            _pci_api.func= use_jit ? &MTXC_JIT_PCIFunction : &MTXC_PCIFunction;
          else
            {
              _pci_api.func= NULL;
              _warning ( _udata, "PCI:MTXC (Bus 0,Dev 0) - "
                         "funció desconeguda: %X", func );
            }
          break;

        case 1: // PIIX4
          switch ( func )
            {
            case 0: _pci_api.func= &PC_PIIX4_PCIFunction_pci_isa_bridge; break;
            case 1: _pci_api.func= &PC_PIIX4_PCIFunction_ide; break;
            case 2: _pci_api.func= &PC_PIIX4_PCIFunction_usb; break;
            case 3:
              _pci_api.func= &PC_PIIX4_PCIFunction_power_management;
              break;
            default:
              _pci_api.func= NULL;
              _warning ( _udata, "PCI:MTXC (Bus 0,Dev 1) - "
                         "funció desconeguda: %X", func );
            }
          break;
          
        default:
          idev= dev-2;
          if ( idev < PC_PCI_DEVICE_NULL &&
               _pci_devs[idev] != NULL )
            {
              if ( func < _pci_devs[idev]->N &&
                   _pci_devs[idev]->func[func] != NULL )
                _pci_api.func= _pci_devs[idev]->func[func];
              else
                {
                  _pci_api.func= NULL;
                  _warning ( _udata, "PCI:MTXC (Bus 0,Dev %d) - "
                             "funció desconeguda: %X", dev, func );
                }
            }
          else
            {
              _pci_api.func= NULL;
              _warning ( _udata, "PCI:MTXC (Bus 0) - dispositiu desconegut: %X",
                         dev );
            }
        }
    }
  
  // --> Busquem tots els "bridge" en aquest bus que puguen redirigir
  //     aquest missatge
  else
    {
      _pci_api.func= NULL;
      _warning ( _udata, "PCI:MTXC (Bus 0) - No s'ha pogut redirigir"
                 " el missatge (Bus:%X,Dev:%X,Func:%X,Reg:%x)",
                 bus, dev, func, reg );
    }
  
} // end PC_mtxc_confadd_write


void
PC_mtxc_set_mode_trace (
                        const bool val
                        )
{

  // Mem
  if ( val && _mem_access != NULL )
    {
      PC_CPU.mem_read8= mem_read8_trace;
      PC_CPU.mem_read16= mem_read16_trace;
      PC_CPU.mem_read32= mem_read32_trace;
      PC_CPU.mem_read64= mem_read64_trace;
      PC_CPU.mem_write8= mem_write8_trace;
      PC_CPU.mem_write16= mem_write16_trace;
      PC_CPU.mem_write32= mem_write32_trace;
      PC_CPU_JIT->mem_read8= mem_jit_read8_trace;
      PC_CPU_JIT->mem_read16= mem_jit_read16_trace;
      PC_CPU_JIT->mem_read32= mem_jit_read32_trace;
      PC_CPU_JIT->mem_read64= mem_read64_trace;
      PC_CPU_JIT->mem_write8= mem_jit_write8_trace;
      PC_CPU_JIT->mem_write16= mem_jit_write16_trace;
      PC_CPU_JIT->mem_write32= mem_jit_write32_trace;
    }
  else
    {
      PC_CPU.mem_read8= mem_read8;
      PC_CPU.mem_read16= mem_read16;
      PC_CPU.mem_read32= mem_read32;
      PC_CPU.mem_read64= mem_read64;
      PC_CPU.mem_write8= mem_write8;
      PC_CPU.mem_write16= mem_write16;
      PC_CPU.mem_write32= mem_write32;
      PC_CPU_JIT->mem_read8= mem_jit_read8;
      PC_CPU_JIT->mem_read16= mem_jit_read16;
      PC_CPU_JIT->mem_read32= mem_jit_read32;
      PC_CPU_JIT->mem_read64= mem_read64;
      PC_CPU_JIT->mem_write8= mem_jit_write8;
      PC_CPU_JIT->mem_write16= mem_jit_write16;
      PC_CPU_JIT->mem_write32= mem_jit_write32;
    }
  
  // PCI
  if ( val && _pci_reg_access != NULL )
    {
      _confdata_read8= confdata_read8_trace;
      _confdata_read16= confdata_read16_trace;
      _confdata_read32= confdata_read32_trace;
      _confdata_write8= confdata_write8_trace;
      _confdata_write16= confdata_write16_trace;
      _confdata_write32= confdata_write32_trace;
    }
  else
    {
      _confdata_read8= confdata_read8;
      _confdata_read16= confdata_read16;
      _confdata_read32= confdata_read32;
      _confdata_write8= confdata_write8;
      _confdata_write16= confdata_write16;
      _confdata_write32= confdata_write32;
    }
  
} // end PC_mtxc_set_mode_trace
