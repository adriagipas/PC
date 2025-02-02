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
 *  piix4.c - Implementa el mapa de memòria i alguns components del
 *            PIIX4. Alguns components s'implementen en mòduls
 *            separats.
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

#define BIOS_MASK_128K 0x1FFFF
#define BIOS_MASK_512K 0x7FFFF

#define BIOS_READ8(ADDR) (_bios.v8[ADDR])
#define BIOS_READ16(ADDR)                                                \
  PC_SWAP16((((const uint16_t *) (_bios.v8+((ADDR)&0x1)))[ADDR>>1]))
#define BIOS_READ32(ADDR)                                               \
  PC_SWAP32(((const uint32_t *) (_bios.v8+((ADDR)&0x3)))[ADDR>>2])
#define BIOS_READ64(ADDR)                                               \
  PC_SWAP64(((const uint64_t *) (_bios.v8+((ADDR)&0x7)))[ADDR>>3])




/*********/
/* ESTAT */
/*********/

// Callbacks.
static PC_Warning *_warning;
static void *_udata;

// Config.
static const PC_Config *_config;

// Bios.
static struct
{
  const uint8_t  *v8;
  const uint16_t *v16;
  const uint32_t *v32;
  const uint64_t *v64;
  size_t          size;
  uint64_t        off128;
  uint64_t        off512;
  uint64_t        addr_firstb_512; // En l'àrea de 512KB
} _bios;




/*********************/
/* FUNCIONS PRIVADES */
/*********************/

// Les seccions 'borderline' les llegisc cridant a funcions inferiors
// encara que siga més costós.
static bool
mem_read16_bl (
               const uint64_t  addr,
               uint16_t       *data
               )
{

  bool ret;
  uint8_t v0,v1;
  

  if ( PC_piix4_mem_read8 ( addr, &v0 ) &&
       PC_piix4_mem_read8 ( addr+1, &v1 ) )
    {
      ret= true;
      *data= ((uint16_t) v0) | (((uint16_t) v1)<<8);
    }
  else ret= false;
  
  return ret;
  
} // end mem_read16_bl


static bool
mem_read32_bl (
               const uint64_t  addr,
               uint32_t       *data
               )
{

  bool ret;
  uint16_t v0,v1;
  

  if ( PC_piix4_mem_read16 ( addr, &v0 ) &&
       PC_piix4_mem_read16 ( addr+2, &v1 ) )
    {
      ret= true;
      *data= ((uint32_t) v0) | (((uint32_t) v1)<<16);
    }
  else ret= false;
  
  return ret;
  
} // end mem_read32_bl


static bool
mem_read64_bl (
               const uint64_t  addr,
               uint64_t       *data
               )
{

  bool ret;
  uint32_t v0,v1;
  
  
  if ( PC_piix4_mem_read32 ( addr, &v0 ) &&
       PC_piix4_mem_read32 ( addr+4, &v1 ) )
    {
      ret= true;
      *data= ((uint64_t) v0) | (((uint64_t) v1)<<32);
    }
  else ret= false;
  
  return ret;
  
} // end mem_read64_bl




/**********************/
/* FUNCIONS PÚBLIQUES */
/**********************/

PC_Error
PC_piix4_init (
               const uint8_t   *bios,
               size_t           bios_size,
               PC_IDEDevice     ide_devices[2][2],
               PC_Warning      *warning,
               void            *udata,
               const PC_Config *config
               )
{

  PC_Error ret;
  
  
  // Callbacks.
  _warning= warning;
  _udata= udata;

  // Config.
  _config= config;
  
  // Bios. (Mínim 128KB, màxim 1MB)
  if ( bios_size < 128*1024 || bios_size > 1024*1024 ||
       bios_size%64 != 0 )
    return PC_BADBIOS;
  _bios.v8= (const uint8_t *) bios;
  _bios.size= bios_size;
  _bios.off128= bios_size - 128*1024;
  if ( bios_size >= 512*1024 )
    {
      _bios.off512= bios_size - 512*1024;
      _bios.addr_firstb_512= 0;
    }
  else
    {
      _bios.off512= 0;
      _bios.addr_firstb_512= 512*1024-bios_size;
    }

  // Inicialitza submoduls.
  PC_piix4_pci_isa_bridge_init ( warning, udata );
  ret= PC_piix4_ide_init ( ide_devices, warning, udata );
  if ( ret != PC_NOERROR ) return ret;
  PC_piix4_usb_init ( warning, udata );
  PC_piix4_power_management_init ( warning, udata );
  
  return PC_NOERROR;
  
} // end PC_piix4_init


void
PC_piix4_reset (void)
{

  PC_piix4_ide_reset ();
  PC_piix4_pci_isa_bridge_reset ();
  PC_piix4_power_management_reset ();
  PC_piix4_usb_reset ();
  
} // end PC_piix4_reset


bool
PC_piix4_mem_read8 (
                    const uint64_t  addr,
                    uint8_t        *data
                    )
{
  
  bool ret;
  

  // NOTA!!!! Mapa provisional, queden moltes coses que tocar. Es
  // poden configurar moltes coses.
  if ( addr < 0x000E0000 ) ret= false;

  // Lower 64KB BIOS. NOTA!!! Crec que es pot desactivar!!!!
  else if ( addr < 0x000F0000 )
    {
      *data= _bios.v8[_bios.off128+(addr&BIOS_MASK_128K)];
      ret= true;
    }
  
  // Top 64KB BIOS
  else if ( addr < 0x00100000 )
    {
      *data= _bios.v8[_bios.off128+(addr&BIOS_MASK_128K)];
      ret= true;
    }

  else if ( addr < 0xFFF80000 ) ret= false;

  // BIOS. Extended 384 KB
  else if ( addr < 0xFFFE0000 )
    {
      *data= (addr&BIOS_MASK_512K)>=_bios.addr_firstb_512 ?
        _bios.v8[_bios.off512+(addr&BIOS_MASK_512K)-_bios.addr_firstb_512] :
        0xFF;
      ret= true;
    }
  
  // Lower 64KB BIOS. Segurament es pot configurar...
  else if ( addr < 0xFFFF0000 )
    {
      *data= _bios.v8[_bios.off128+(addr&BIOS_MASK_128K)];
      ret= true;
    }
  
  // Top 64KB BIOS
  else if ( addr < 0x100000000 )
    {
      *data= _bios.v8[_bios.off128+(addr&BIOS_MASK_128K)];
      ret= true;
    }
  
  else ret= false;
  
  return ret;
  
} // end PC_piix4_mem_read8


bool
PC_piix4_mem_read16 (
                     const uint64_t  addr,
                     uint16_t       *data
                     )
{
  
  bool ret;
  uint64_t tmp_addr;
  
  
  // NOTA!!!! Mapa provisional, queden moltes coses que tocar. Es
  // poden configurar moltes coses.
  if ( addr < 0x000E0000 ) ret= false;

  // Lower 64KB BIOS. NOTA!!! Crec que es pot desactivar!!!!
  else if ( addr < 0x000F0000 )
    {
      if ( addr != 0x000EFFFF )
        {
          tmp_addr= _bios.off128+(addr&BIOS_MASK_128K);
          *data= BIOS_READ16(tmp_addr);
          ret= true;
        }
      else ret= mem_read16_bl ( addr, data );
    }
  
  // Top 64KB BIOS
  else if ( addr < 0x00100000 )
    {
      if ( addr != 0x000FFFFF )
        {
          tmp_addr= _bios.off128+(addr&BIOS_MASK_128K);
          *data= BIOS_READ16(tmp_addr);
          ret= true;
        }
      else ret= mem_read16_bl ( addr, data );
    }
  
  else if ( addr < 0xFFF80000 ) ret= false;

  // BIOS. Extended 384 KB
  else if ( addr < 0xFFFE0000 )
    {
      if ( addr != 0xFFFDFFFF )
        {
          tmp_addr= addr&BIOS_MASK_512K;
          if ( tmp_addr >= _bios.addr_firstb_512 )
            {
              tmp_addr= _bios.off512+tmp_addr-_bios.addr_firstb_512;
              *data= BIOS_READ16(tmp_addr);
            }
          else *data= 0xFFFF;
          ret= true;
        }
      else ret= mem_read16_bl ( addr, data );
    }
  
  // Lower 64KB BIOS. Segurament es pot configurar...
  else if ( addr < 0xFFFF0000 )
    {
      if ( addr != 0xFFFEFFFF )
        {
          tmp_addr= _bios.off128+(addr&BIOS_MASK_128K);
          *data= BIOS_READ16(tmp_addr);
          ret= true;
        }
      else ret= mem_read16_bl ( addr, data );
    }
  
  // Top 64KB BIOS
  else if ( addr < 0x100000000 )
    {
      if ( addr != 0x0FFFFFFFF )
        {
          tmp_addr= _bios.off128+(addr&BIOS_MASK_128K);
          *data= BIOS_READ16(tmp_addr);
          ret= true;
        }
      else ret= mem_read16_bl ( addr, data );
    }
  
  else ret= false;
  
  return ret;
  
} // end PC_piix4_mem_read16


bool
PC_piix4_mem_read32 (
                     const uint64_t  addr,
                     uint32_t       *data
                     )
{

  bool ret;
  uint64_t tmp_addr;
  
  
  // NOTA!!!! Mapa provisional, queden moltes coses que tocar. Es
  // poden configurar moltes coses.
  if ( addr < 0x000E0000 ) ret= false;
  
  // Lower 64KB BIOS. NOTA!!! Crec que es pot desactivar!!!!
  else if ( addr < 0x000F0000 )
    {
      if ( addr < 0x000EFFFD )
        {
          tmp_addr= _bios.off128+(addr&BIOS_MASK_128K);
          *data= BIOS_READ32(tmp_addr);
          ret= true;
        }
      else ret= mem_read32_bl ( addr, data );
    }
  
  // Top 64KB BIOS
  else if ( addr < 0x00100000 )
    {
      if ( addr < 0x000FFFFD )
        {
          tmp_addr= _bios.off128+(addr&BIOS_MASK_128K);
          *data= BIOS_READ32(tmp_addr);
          ret= true;
        }
      else ret= mem_read32_bl ( addr, data );
    }

  else if ( addr < 0xFFF80000 ) ret= false;

  // BIOS. Extended 384 KB
  else if ( addr < 0xFFFE0000 )
    {
      if ( addr < 0xFFFDFFFD )
        {
          tmp_addr= addr&BIOS_MASK_512K;
          if ( tmp_addr >= _bios.addr_firstb_512 )
            {
              tmp_addr= _bios.off512+tmp_addr-_bios.addr_firstb_512;
              *data= BIOS_READ32(tmp_addr);
            }
          else *data= 0xFFFFFFFF;
          ret= true;
        }
      else ret= mem_read32_bl ( addr, data );
    }
  
  // Lower 64KB BIOS. Segurament es pot configurar...
  else if ( addr < 0xFFFF0000 )
    {
      if ( addr < 0xFFFEFFFD )
        {
          tmp_addr= _bios.off128+(addr&BIOS_MASK_128K);
          *data= BIOS_READ32(tmp_addr);
          ret= true;
        }
      else ret= mem_read32_bl ( addr, data );
    }
  
  // Top 64KB BIOS
  else if ( addr < 0x100000000 )
    {
      if ( addr < 0x0FFFFFFFD )
        {
          tmp_addr= _bios.off128+(addr&BIOS_MASK_128K);
          *data= BIOS_READ32(tmp_addr);
          ret= true;
        }
      else ret= mem_read32_bl ( addr, data );
    }
  
  else ret= false;
  
  return ret;
  
} // end PC_piix4_mem_read32


bool
PC_piix4_mem_read64 (
                     const uint64_t  addr,
                     uint64_t       *data
                     )
{

  bool ret;
  uint64_t tmp_addr;
  

  // NOTA!!!! Mapa provisional, queden moltes coses que tocar. Es
  // poden configurar moltes coses.
  if ( addr < 0x000E0000 ) ret= false;

  // Lower 64KB BIOS. NOTA!!! Crec que es pot desactivar!!!!
  else if ( addr < 0x000F0000 )
    {
      if ( addr < 0x000EFFF9 )
        {
          tmp_addr= _bios.off128+(addr&BIOS_MASK_128K);
          *data= BIOS_READ64(tmp_addr);
          ret= true;
        }
      else ret= mem_read64_bl ( addr, data );
    }
  
  // Top 64KB BIOS
  else if ( addr < 0x00100000 )
    {
      if ( addr < 0x000FFFF9 )
        {
          tmp_addr= _bios.off128+(addr&BIOS_MASK_128K);
          *data= BIOS_READ64(tmp_addr);
          ret= true;
        }
      else ret= mem_read64_bl ( addr, data );
    }

  else if ( addr < 0xFFFE0000 ) ret= false;

  // Lower 64KB BIOS. Segurament es pot configurar...
  else if ( addr < 0xFFFF0000 )
    {
      if ( addr < 0xFFFEFFF9 )
        {
          tmp_addr= _bios.off128+(addr&BIOS_MASK_128K);
          *data= BIOS_READ64(tmp_addr);
          ret= true;
        }
      else ret= mem_read64_bl ( addr, data );
    }
  
  // Top 64KB BIOS
  else if ( addr < 0x100000000 )
    {
      if ( addr < 0x0FFFFFFF9 )
        {
          tmp_addr= _bios.off128+(addr&BIOS_MASK_128K);
          *data= BIOS_READ64(tmp_addr);
          ret= true;
        }
      else ret= mem_read64_bl ( addr, data );
    }
  
  else ret= false;
  
  return ret;
  
} // end PC_piix4_mem_read64


bool
PC_piix4_mem_write8 (
                     const uint64_t addr,
                     const uint8_t  data
                     )
{
  return false;
} // end PC_piix4_mem_write8


bool
PC_piix4_mem_write16 (
                      const uint64_t addr,
                      const uint16_t data
                      )
{
  return false;
} // end PC_piix4_mem_write16


bool
PC_piix4_mem_write32 (
                      const uint64_t addr,
                      const uint32_t data
                      )
{
  return false;
} // end PC_piix4_mem_write32
