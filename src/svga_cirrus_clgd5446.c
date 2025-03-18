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
 *  svga_cirrus_clgd5446.c - Implementació de targeta PCI SVGA de
 *                           Cirrus Logic CL-GD5446 (Revision B).
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

#define PCICMD_IO  0x0001
#define PCICMD_MEM 0x0002

#define PLANE_MASK ((64*1024)-1)

#define FB_WIDTH (256*9)
#define FB_HEIGHT (1024*2)

#define BIOS_READ8(ADDR) (_bios.v8[ADDR])
#define BIOS_READ16(ADDR)                                                \
  PC_SWAP16((((const uint16_t *) (_bios.v8+((ADDR)&0x1)))[ADDR>>1]))
#define BIOS_READ32(ADDR)                                               \
  PC_SWAP32(((const uint32_t *) (_bios.v8+((ADDR)&0x3)))[ADDR>>2])
#define BIOS_READ64(ADDR)                                               \
  PC_SWAP64(((const uint64_t *) (_bios.v8+((ADDR)&0x7)))[ADDR>>3])

#define VRAM_SIZE (4*1024*1024)
#define VRAM_MASK (VRAM_SIZE-1)




/***************/
/* DEFINICIONS */
/***************/

static void update_vclk (void);
static void update_cc_to_event (void);
static void clock ( const bool update_cc2event );
static void update_vga_mem (void);
static void misc_write (const uint8_t data,const bool update_clock,
                        const bool update_vclk);
static void SR_write (const uint8_t data,const bool update_clock,
                      const bool update_vclk);
static uint8_t SR_read (const bool update_clock);
static void CR_write (const uint8_t data,const bool update_clock);
static uint8_t CR_read (const bool update_clock);
static void GR_write (const uint8_t data,const bool update_clock);
static uint8_t GR_read (const bool update_clock);
static void AR_write (const uint8_t data,const bool update_clock);
static uint8_t AR_read (const bool update_clock);
static void pixel_mask_write (const uint8_t data,const bool update_clock);
static uint8_t pixel_mask_read (const bool update_clock);
static uint8_t stat_read (void);
static void dac_data_write (const uint8_t data);
static uint8_t dac_data_read (void);
static void dac_addr_w_write (const uint8_t data);
static void dac_addr_r_write (const uint8_t data);
static void vga_mem_write (const uint64_t addr,const uint8_t data);
static uint8_t vga_mem_read (const uint64_t addr);
static void init_pci_regs (void);
static void init_regs (void);




/*************/
/* CONSTANTS */
/*************/

static const uint16_t VID= 0x1013;
static const uint16_t DID= 0x00B8;
static const uint8_t RID= 0x00;

// CLASSC
static const uint8_t BASEC= 0x03;
static const uint8_t SCC= 0x00;
static const uint8_t PI= 0x00;

static const uint8_t HEDT= 0x00; // Inventada !!!


// NOTA !!! Aparentment el IRQ estava en EGA i alguna versió més, pero
// rarament s'utilitza. Segons un foro l'únic joc que ho necessita és
// Gauntlet d'EGA i aparentment no funciona en VGA modernes per altres
// motius.
// En el model de gràfica que estic simulant es pot activar/desactivar
// amb un jumper. Com que aparentment no és molt important de moment
// vaig a no implementar les interrupcions.
static const uint8_t INTPN= 0x00;
///static const uint8_t INTPN= 0x01; // Activat CAL IMPLEMENTAR !!!!




/*********/
/* ESTAT */
/*********/

// Callbacks.
static PC_Warning *_warning;
static PC_UpdateScreen *_update_screen;
static PC_VGAMemAccess *_vga_mem_access;
static PC_VGAMemLinearAccess *_vga_mem_linear_access;
static bool _trace_enabled;
static void *_udata;

// Registres PCI
static struct
{
  uint16_t pcicmd;
  uint32_t disp_mem_base_addr; // PCI10: PCI Display Memory Base Address
  uint32_t vga_bb_reg_base_addr; // PCI14: PCI VGA/BitBLT Register Base Address
  uint32_t erom; // PCI30: PCI Expansion ROM Base Address Enable. El
                 // bit EROM enable està també.
  uint8_t intln; // No fa res
} _pci_regs;

// Bios.
static struct
{
  const uint8_t  *v8;
  size_t          size;
  size_t          size_1;
  size_t          size_3;
  size_t          size_7;
  uint32_t        mask;
} _bios;

// VGA core registers
static struct
{

  uint8_t pixel_mask;
  struct
  {
    uint8_t val;
    int     counter;
    bool    mode_555_enabled;
    bool    all_ext_modes_enabled;
    bool    clocking_mode_is_1;
    bool    control_32k_color_enabled;
    uint8_t ext_mode;
  }       hdr;
  struct
  {
    uint8_t val;
    bool    vertical_sync_is_high;
    bool    horizontal_sync_is_high;
    bool    page_select_is_even;
    int     vlck_freq_ind;
    bool    display_mem_enabled;
    bool    crtc_io_addr_mode_color;
  } misc;
  struct
  {
    int     index;
    uint8_t reset;
    struct
    {
      uint8_t val;
      bool    full_bandwidth;
      bool    dot_clock_div2;
      bool    dot_clock_8_9;
      uint8_t shift_load;
    }       clocking_mode;
    uint8_t plane_mask;
    struct
    {
      uint8_t val;
      uint8_t primary_map;
      uint8_t secondary_map;
    }       char_map;
    struct
    {
      uint8_t val;
      bool    chain4;
      bool    odd_even_mode;
      bool    extended_memory;
    }       mem_mode;
    uint8_t r6_key;
    struct
    {
      uint8_t val;
      bool    linear_frame_buffer_enabled;
      int     srt;
      bool    extended_display_modes_enabled;
    }       r7;
    struct
    {
      uint8_t val;
      // DDC2B/EEPROM
    }       r8;
    uint8_t r10;
    struct
    {
      uint8_t val;
      bool    overscan_color_protect;
      bool    cursor_size_is_32x32;
      bool    allow_access_DAC_extended_colors;
      bool    cursor_enable;
    }       r12;
    uint8_t r13_cursor_pat_addr_off;
    struct
    {
      uint8_t val;
      bool    dram_bank_switch;
      bool    fast_page_detection_disabled;
      int     dram_data_bus_width;
    }       r15;
    struct
    {
      uint8_t val;
      bool    dram_bank_1MB;
      bool    mem_mapped_io_addr;
      bool    write_enable_PCI2C;
      bool    enable_mem_mapped_io;
      bool    enable_dram_bank_swap;
    }       r17;
    struct
    {
      uint8_t num;
      uint8_t den;
    }       vclk[4];
  } SR;
  struct
  {
    int     index;
    struct
    {
      uint8_t bg_colorb0; // Tot el byte
      uint8_t set_reset;
    }       r0;
    struct
    {
      uint8_t fg_colorb0; // Tot el byte
      uint8_t enable_sr;
    }       r1;
    uint8_t color_compare;
    struct
    {
      uint8_t val;
      uint8_t func;
      uint8_t count;
    }       data_rotate;
    uint8_t read_map_select;
    struct
    {
      uint8_t val;
      bool    color256;
      bool    shift_reg_mode_is_1;
      bool    odd_even_mode;
      bool    read_mode1;
      uint8_t write_mode;
    }       mode;
    struct
    {
      uint8_t val;
      uint8_t mem_map;
      bool    chain_odd_maps_to_event_is_1;
      bool    apa_mode;
    }       misc;
    uint8_t color_dont_care;
    uint8_t bit_mask;
    uint8_t offset0;
    uint8_t offset1;
    struct
    {
      uint8_t val;
      bool    offset_granularity;
      bool    enhanced_writes_16bit_enabled;
      bool    eightbyte_data_latches_enabled;
      bool    extended_write_modes_enabled;
      bool    by8_addr_enabled;
      bool    offset1_enabled;
    }       rb;
    struct
    {
      uint8_t val;
      bool    enable_autostart;
      bool    use_system_source_location;
      bool    pause;
      bool    blt_reset;
      bool    blt_start;
    }       r49;
  } GR;
  struct
  {
    int     index;
    uint8_t horizontal_total;
    uint8_t horizontal_display_end;
    uint8_t horizontal_blanking_start;
    struct
    {
      uint8_t val;
      bool    compatible_read;
      uint8_t display_enable_skew;
      uint8_t horizontal_blanking_end;
    }       horizontal_blanking_end;
    uint8_t horizontal_sync_start;
    struct
    {
      uint8_t val;
      uint8_t horizontal_blanking_end;
      uint8_t horizontal_sync_delay;
      uint8_t horizontal_sync_end;
    }       horizontal_sync_end;
    uint16_t vertical_total;
    struct
    {
      uint8_t  val;
      uint16_t vertical_retrace_start;
      uint16_t vertical_display_end;
      uint16_t vertical_total;
      uint16_t line_compare;
      uint16_t vertical_blanking_start;
    }       overflow;
    struct
    {
      uint8_t val;
      uint8_t byte_pan;
      uint8_t screen_a_prs;
    }       screen_a_prs;
    struct
    {
      uint8_t  val;
      bool     scan_double;
      uint16_t line_compare;
      uint16_t vertical_blank_start;
      uint16_t char_cell_height;
    }       char_cell_height;
    struct
    {
      uint8_t val;
      bool    text_cursor_disabled;
      uint8_t text_cursor_start;
    }       text_cursor_start;
    struct
    {
      uint8_t val;
      uint8_t text_cursor_skew;
      uint8_t text_cursor_end;
    }        text_cursor_end;
    uint16_t screen_start_a_addrH;
    uint16_t screen_start_a_addrL;
    uint16_t text_cursor_locH;
    uint16_t text_cursor_locL;
    uint16_t vertical_sync_start;
    struct
    {
      uint8_t val;
      bool    wprotect_cr0_7;
      bool    refresh_cycle_control_is_1;
      bool    disable_vint;
      uint8_t vertical_sync_end;
    }        vertical_sync_end;
    uint16_t vertical_display_end;
    uint8_t  offset;
    struct
    {
      uint8_t val;
      bool    double_word_mode;
      bool    count_by_four;
      uint8_t underline_scanline;
    }        underline_scanline;
    uint8_t  vertical_blank_start;
    uint8_t  vertical_blank_end;
    struct
    {
      uint8_t val;
      bool    timing_enabled;
      bool    byte_word_mode;
      bool    addr_wrap;
      bool    count_by_two;
      bool    vregs_by_two;
      bool    select_rsc_is_1;
      bool    compatibility_cga_mode;
    }        mode;
    uint8_t  line_compare;
    struct
    {
      uint8_t  val;
      uint16_t vblank_end;
      uint8_t  hblank_end;
      uint8_t  ovdac_mode_switch;
      bool     double_buff_display_start_addr;
      bool     interlace_enabled;
    } misc_ctrl;
    struct
    {
      uint8_t  val;
      bool     blank_end_extensions_enabled;
      bool     text_mode_fastpage_enabled;
      bool     blanking_control_is_1;
      uint16_t offset_overflow;
      uint32_t screen_start_a_addr;
      bool     ext_addr_wrap_enabled;
      uint32_t ext_disp_start_addr;
    } ext_disp_ctrl;
    struct
    {
      uint8_t  val;
      uint32_t screen_start_a_addr;
      bool     ov_timing_select_is_1;
      bool     color_chrome_select_is_1;
      bool     color_key_tag_enabled;
      bool     color_compare_width;
      uint8_t  dac_mode_switch;
    } ov_ext_ctrl;
    struct
    {
      uint8_t val;
      bool    occlusion_enabled;
      bool    error_difussion_enabled;
      bool    vertical_zoom_mode_enabled;
      uint8_t video_display_format;
      bool    video_window_master_enabled;
    } vid_win_master_ctrl;
    uint8_t vid_win_vend;
  } CR;
  struct
  {
    bool    mode_data;
    int     index;
    bool    display_enabled;
    uint8_t pal[16];
    struct
    {
      uint8_t val;
      bool    ar14_enabled;
      bool    pixel_double_clock;
      bool    pixel_panning_comp;
      bool    blink_enabled;
      bool    line_graphics_enabled;
      bool    display_type_is_1;
      bool    use_apa_mode;
    }       attr_ctrl_mode;
    uint8_t overscan_color;
    struct
    {
      uint8_t val;
      uint8_t video_status_mux;
      uint8_t enable;
    }       color_plane;
    uint8_t pixel_panning;
    uint8_t color_select;
  } AR;
  
} _regs;

// DAC
static struct
{
  uint8_t v[256][3]; // Valors R,G,B
  uint8_t addr_w;
  uint8_t addr_r;
  int     buffer_w_off;
  uint8_t buffer_w[3];
  int     buffer_r_off;
  uint8_t buffer_r[3];
} _dac;

// Video ram (4MB - SVGA)
static uint8_t _vram[VRAM_SIZE];

// Gestiona mapa memòria VGA estàndard
static struct
{
  
  uint64_t  begin;
  uint64_t  end;
  uint8_t  *p[4]; // Planols
  uint8_t   latch[4];
  
} _vga_mem;

static struct
{
  
  int cc_used;
  int cc;
  int cctoEvent;
  
  // Per a passar a clocks de VGA cal fer (cc*(cc_mul))/cc_div
  long cc_mul;
  long cc_div;
  long vcc_tmp; // Ací hi han cc*cc_mul que encara no han sigut
                // dividits, i típicament no apleguen a cc_div. Però
                // pot ser que siga més en un canvi.
  
} _timing;

static struct
{

  // Framebuffer
  PC_RGB fb[FB_WIDTH*FB_HEIGHT];
  
  // Comptadors
  int     H;
  int     V;
  int     char_dots;
  int     scanline;
  bool    in_hblank;
  bool    in_hretrace;
  bool    in_vblank;
  int     vblank_end;
  bool    in_vretrace;
  int     vretrace_end;
  bool    blink;
  int     blink_counter;
  uint8_t pixel_bus;
  int     start_addr;
  
} _render;




/*******/
/* PCI */
/*******/

static uint8_t
pci_read8 (
           const uint8_t addr
           )
{

  uint8_t ret;


  clock ( true );
  
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
      
      // INTLN - INTERRUPT LINE REGISTER
    case 0x3c: ret= _pci_regs.intln; break;
      // INTPN - INTERRUPT PIN
    case 0x3d: ret= INTPN; break;
      
    default:
      _warning ( _udata,
                 "PCI:CLGD5446.read8 - addreça no implementada %02X\n",
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
  

  clock ( true );
  
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
      
    default:
      _warning ( _udata,
                 "PCI:CLGD5446.read16 - addreça no implementada %02X\n",
                 addr );
      ret= 0xffff;
    }

  return ret;
  
} // end pci_read16


static uint32_t
pci_read32 (
            const uint8_t addr
            )
{

  uint32_t ret;
  

  clock ( true );
  
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

      // PCI10: PCI Display Memory Base Address
    case 0x04: ret= _pci_regs.disp_mem_base_addr; break;
      // PCI14: PCI VGA/BitBLT Register Base Address
    case 0x05: ret= _pci_regs.vga_bb_reg_base_addr; break;
      // PCI18: PCI GPIO Base Address
      // NOTA!! Assumim CF8 i CF4 a 1 (desactivat)
    case 0x06: ret= 0x00000000; break;
      // Reserved
    case 0x07 ... 0x0a: ret= 0x00000000; break;
      
      // PCI30: PCI Expansion ROM Base Address Enable
    case 0x0c: ret= _pci_regs.erom; break;
      
    default:
      _warning ( _udata,
                 "PCI:CLGD5446.read32 - addreça no implementada %02X\n",
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

  clock ( true );
  
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

      // INTLN - INTERRUPT LINE REGISTER
    case 0x3c: _pci_regs.intln= data; break;
      // INTPN - INTERRUPT PIN
    case 0x3d: break;
      
    default:
      _warning ( _udata,
                 "PCI:CLGD5446.write8 - addreça no implementada %02X\n",
                 addr );
    }
  
} // end pci_write8


static void
pci_write16 (
             const uint8_t  addr,
             const uint16_t data
             )
{

  clock ( true );
  
  switch ( addr )
    {

      // VID
    case 0x00: break;
      // DID
    case 0x01: break;
      //PCICMD
    case 0x02:
      _pci_regs.pcicmd= (data&0x0023);
      if ( data&0x0020 )
        _warning ( _udata,
                   "pci_write16 (SVGA CIRRUS CLGD5446) - s'ha intentat"
                   " habilitar el Enable DAC Shadowing, però no està"
                   " implementat" );
      break;
      
      // SCC i BASEC;
    case 0x05: break;
      
    default:
      _warning ( _udata,
                 "PCI:CLGD5446.write16 - addreça no implementada %02X\n",
                 addr );
    }
  
} // end pci_write16


static void
pci_write32 (
             const uint8_t  addr,
             const uint32_t data
             )
{

  clock ( true );
  
  switch ( addr )
    {

    case 0x00: break;

      // PCI10: PCI Display Memory Base Address
    case 0x04: _pci_regs.disp_mem_base_addr= data&0xFE000000; break;
      // PCI14: PCI VGA/BitBLT Register Base Address
    case 0x05: _pci_regs.vga_bb_reg_base_addr= data&0xFFFFF000; break;
      // PCI18: PCI GPIO Base Address
      // NOTA!! Assumim CF8 i CF4 a 1 (desactivat)
    case 0x06: break;
      // Reserved
    case 0x07 ... 0x0a: break;

      // PCI30: PCI Expansion ROM Base Address Enable
    case 0x0c:
      // ATENCIÓ !!!!!!!!
      // Segons la documentació la Base ADDR és 31:14
      // --> _pci_regs.erom= data&0xFFFFC001;
      // Però és incoherent amb que siga 32K!!! deuria ser 31:15
      /*
      _warning ( _udata,
                 "PCI:CLGD5446.write32 - EROM BASE: %08X !!!!! CAL REVISAR !!!",
                 data );
      */
      _pci_regs.erom= data&(_bios.mask|0x1);
      break;
      
    default:
      _warning ( _udata,
                 "PCI:CLGD5446.write32 - addreça no implementada %02X\n",
                 addr );
    }
  
} // end pci_write32


static const PC_PCIFunction FUNC=
  {
    pci_read8,
    pci_read16,
    pci_read32,
    pci_write8,
    pci_write16,
    pci_write32,
    "CL-GD5446 - Cirrus Logic SVGA"
  };



/*******/
/* I/O */
/*******/

static bool
port_read8 (
            const uint16_t  port,
            uint8_t        *data
            )
{

  bool ret;
  
  
  ret= true;
  switch ( port )
    {
    case 0x3b4:
      if ( !_regs.misc.crtc_io_addr_mode_color )
        *data= _regs.CR.index;
      else *data= 0xff;
      break;
    case 0x3b5:
      if ( !_regs.misc.crtc_io_addr_mode_color )
        *data= CR_read ( true );
      else *data= 0xff;
      break;
      
    case 0x3ba:
      if ( !_regs.misc.crtc_io_addr_mode_color )
        *data= stat_read ();
      else *data= 0xff;
      break;
      
    case 0x3c0:
    case 0x3c1: *data= AR_read ( true ); break;
      
    case 0x3c4: *data= _regs.SR.index; break;
    case 0x3c5: *data= SR_read ( true ); break;
    case 0x3c6: *data= pixel_mask_read ( true ); break;

    case 0x3c9: *data= dac_data_read (); break;
      
    case 0x3cc: /*clock ( true );*/ *data= _regs.misc.val; break;
      
    case 0x3ce: *data= _regs.GR.index; break;
    case 0x3cf: *data= GR_read ( true ); break;
      
    case 0x3d4:
      if ( _regs.misc.crtc_io_addr_mode_color )
        *data= _regs.CR.index;
      else *data= 0xff;
      break;
    case 0x3d5:
      if ( _regs.misc.crtc_io_addr_mode_color )
        *data= CR_read ( true );
      else *data= 0xff;
      break;

    case 0x3da:
      if ( _regs.misc.crtc_io_addr_mode_color )
        *data= stat_read ();
      else *data= 0xff;
      break;
      
    default: ret= false;
    }
  
  return ret;
  
} // end port_read8


static bool
port_read16 (
             const uint16_t  port,
             uint16_t       *data
             )
{
  return false;
} // end port_read16


static bool
port_read32 (
             const uint16_t  port,
             uint32_t       *data
             )
{
  return false;
} // end port_read32


static bool
port_write8 (
             const uint16_t port,
             const uint8_t  data
             )
{

  bool ret;
  
  
  ret= true;
  switch ( port )
    {
    case 0x3b4:
      if ( !_regs.misc.crtc_io_addr_mode_color )
        _regs.CR.index= ((uint8_t) data)&0x3F;
      break;
    case 0x3b5:
      if ( !_regs.misc.crtc_io_addr_mode_color )
        CR_write ( data, true );
      break;

    case 0x3c0:
    case 0x3c1: AR_write ( data, true ); break;
    case 0x3c2: misc_write ( data, true, true ); break;
      
    case 0x3c4: _regs.SR.index= ((uint8_t) data)&0x1F; break;
    case 0x3c5: SR_write ( data, true, true ); break;
    case 0x3c6: pixel_mask_write ( data, true ); break;
    case 0x3c7: dac_addr_r_write ( data ); break;
    case 0x3c8: dac_addr_w_write ( data ); break;
    case 0x3c9: dac_data_write ( data ); break;
      
    case 0x3ce: _regs.GR.index= ((uint8_t) data)&0x3F; break;
    case 0x3cf: GR_write ( data, true ); break;

    case 0x3d4:
      if ( _regs.misc.crtc_io_addr_mode_color )
        _regs.CR.index= ((uint8_t) data)&0x3F;
      break;
    case 0x3d5:
      if ( _regs.misc.crtc_io_addr_mode_color )
        CR_write ( data, true );
      break;

    case 0x3d9: // CGA palette register (S'ignora)
      break;
      
    default: ret= false;
    }
  
  return ret;
  
} // end port_write8


static bool
port_write16 (
              const uint16_t port,
              const uint16_t data
              )
{

  bool ret;


  ret= true;
  switch ( port>>1 )
    {
      
    case 0x3b4>>1:
      if ( !_regs.misc.crtc_io_addr_mode_color )
        {
          _regs.CR.index= ((uint8_t) data)&0x3F;
          CR_write ( (uint8_t) (data>>8), true );
        }
      break;
      
    case 0x3c4>>1:
      _regs.SR.index= ((uint8_t) data)&0x1F;
      SR_write ( (uint8_t) (data>>8), true, true );
      break;
      
    case 0x3ce>>1:
      _regs.GR.index= ((uint8_t) data)&0x3F;
      GR_write ( (uint8_t) (data>>8), true );
      break;
      
    case 0x3d4>>1:
      if ( _regs.misc.crtc_io_addr_mode_color )
        {
          _regs.CR.index= ((uint8_t) data)&0x3F;
          CR_write ( (uint8_t) (data>>8), true );
        }
      break;
      
    default: ret= false;
    }

  return ret;
  
} // end port_write16


static bool
port_write32 (
              const uint16_t port,
              const uint32_t data
              )
{
  return false;
} // end port_write32


static const PC_PCIPorts PORTS=
  {
    port_read8,
    port_read16,
    port_read32,
    port_write8,
    port_write16,
    port_write32
  };


/*******/
/* MEM */
/*******/

static bool
mem_read8 (
           const uint64_t  addr,
           uint8_t        *data
           )
{

  int aperture;
  bool ret;
  uint32_t tmp;
  

  // Comprova està habilitat.
  if ( (_pci_regs.pcicmd&PCICMD_MEM) == 0 ) ret= false;
  
  // Expansion ROM
  else if ( (_pci_regs.erom&0x1) &&
            ((uint32_t) (addr&_bios.mask)) == (_pci_regs.erom&_bios.mask) )
    {
      tmp= ((uint32_t) addr)&(~_bios.mask);
      if ( tmp < _bios.size )
        {
          *data= BIOS_READ8(tmp);
          ret= true;
        }
      else ret= false;
    }

  // Display memorya (VGA mode)
  else if ( _regs.misc.display_mem_enabled &&
            addr >= _vga_mem.begin && addr < _vga_mem.end )
    {
      *data= vga_mem_read ( addr );
      ret= true;
    }
  
  // Display memory
  else if ( _regs.misc.display_mem_enabled &&
            _regs.SR.r7.linear_frame_buffer_enabled &&
            ((uint32_t) (addr&0xFE000000)) == _pci_regs.disp_mem_base_addr )
    {
      aperture= (addr>>22)&0x3;
      switch ( aperture )
        {
        case 0: // No swap
          *data= _vram[addr&VRAM_MASK];
          break;
        default:
          PC_MSGF("MEM_READ8 (SVGA_CIRRUS_CLGD5446"
                  " - Display memory) addr: %016lX (aperture: %d)",
                  addr,aperture);
          exit(EXIT_FAILURE);
        }
      if ( _trace_enabled && _vga_mem_linear_access != NULL )
        _vga_mem_linear_access ( PC_READ8, aperture,
                                 (uint32_t) (addr&VRAM_MASK),
                                 (uint64_t) *data, _udata );
      ret= true;
    }
  
  // VGA I/O - BitBLT control registers
  else if ( ((uint32_t) (addr&0xFFFFF000)) == _pci_regs.vga_bb_reg_base_addr )
    {
      PC_MSGF("MEM_READ8 (SVGA_CIRRUS_CLGD5446"
              " - VGA I/O -- BitBLT control registers) addr: %016lX",addr);
      *data= 0xFF;
      ret= true;
      exit(EXIT_FAILURE);
    }
  
  else ret= false;
  
  return ret;
  
} // end mem_read8


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
  

  if ( mem_read8 ( addr, &v0 ) &&
       mem_read8 ( addr+1, &v1 ) )
    {
      ret= true;
      *data= ((uint16_t) v0) | (((uint16_t) v1)<<8);
    }
  else ret= false;
  
  return ret;
  
} // end mem_read16_bl


static bool
mem_read16 (
            const uint64_t  addr,
            uint16_t       *data
            )
{

  int aperture;
  bool ret;
  uint32_t tmp;
  

  // Comprova està habilitat.
  if ( (_pci_regs.pcicmd&PCICMD_MEM) == 0 ) ret= false;
  
  // Expansion ROM
  else if ( (_pci_regs.erom&0x1) &&
            ((uint32_t) (addr&_bios.mask)) == (_pci_regs.erom&_bios.mask) )
    {
      tmp= ((uint32_t) addr)&(~_bios.mask);
      if ( tmp < _bios.size_1 )
        {
          *data= BIOS_READ16(tmp);
          ret= true;
        }
      else if ( tmp == _bios.size_1 ) ret= mem_read16_bl ( addr, data );
      else ret= false;
    }

  // Display memorya (VGA mode)
  else if ( _regs.misc.display_mem_enabled &&
            addr >= _vga_mem.begin && addr < _vga_mem.end )
    {
      *data=
        ((uint16_t) vga_mem_read ( addr )) |
        (((uint16_t) vga_mem_read ( addr+1 ))<<8)
        ;
      ret= true;
    }
  
  // Display memory
  else if ( _regs.misc.display_mem_enabled &&
            _regs.SR.r7.linear_frame_buffer_enabled &&
            ((uint32_t) (addr&0xFE000000)) == _pci_regs.disp_mem_base_addr )
    {
      aperture= (addr>>22)&0x3;
      switch ( aperture )
        {
        case 0: // No swap
          *data= ((uint16_t *) _vram)[(addr&VRAM_MASK)>>1];
#if PC_BE
          *data= PC_SWAP16(*data);
#endif
          break;
        default:
          PC_MSGF("MEM_READ16 (SVGA_CIRRUS_CLGD5446"
                  " - Display memory) addr: %016lX (aperture: %d)",
                  addr,aperture);
          exit(EXIT_FAILURE);
        }
      if ( _trace_enabled && _vga_mem_linear_access != NULL )
        _vga_mem_linear_access ( PC_READ16, aperture,
                                 (uint32_t) (addr&VRAM_MASK),
                                 (uint64_t) *data, _udata );
      ret= true;
    }

  // VGA I/O - BitBLT control registers
  else if ( ((uint32_t) (addr&0xFFFFF000)) == _pci_regs.vga_bb_reg_base_addr )
    {
      PC_MSGF("MEM_READ16 (SVGA_CIRRUS_CLGD5446"
              " - VGA I/O -- BitBLT control registers) addr: %016lX",addr);
      *data= 0xFFFF;
      ret= true;
      exit(EXIT_FAILURE);
    }
  
  else ret= false;
  
  return ret;
  
} // end mem_read16


static bool
mem_read32_bl (
               const uint64_t  addr,
               uint32_t       *data
               )
{

  bool ret;
  uint16_t v0,v1;
  

  if ( mem_read16 ( addr, &v0 ) &&
       mem_read16 ( addr+2, &v1 ) )
    {
      ret= true;
      *data= ((uint32_t) v0) | (((uint32_t) v1)<<16);
    }
  else ret= false;
  
  return ret;
  
} // end mem_read32_bl


static bool
mem_read32 (
            const uint64_t  addr,
            uint32_t       *data
            )
{
  
  bool ret;
  int aperture;
  uint32_t tmp;
  

  // Comprova està habilitat.
  if ( (_pci_regs.pcicmd&PCICMD_MEM) == 0 ) ret= false;
  
  // Expansion ROM
  else if ( (_pci_regs.erom&0x1) &&
            ((uint32_t) (addr&_bios.mask)) == (_pci_regs.erom&_bios.mask) )
    {
      tmp= ((uint32_t) addr)&(~_bios.mask);
      if ( tmp < _bios.size_3 )
        {
          *data= BIOS_READ32(tmp);
          ret= true;
        }
      else if ( tmp < _bios.size ) ret= mem_read32_bl ( addr, data );
      else ret= false;
    }

  // Display memory (VGA mode)
  else if ( _regs.misc.display_mem_enabled &&
            addr >= _vga_mem.begin && addr < _vga_mem.end )
    {
      *data=
        ((uint32_t) vga_mem_read ( addr )) |
        (((uint32_t) vga_mem_read ( addr+1 ))<<8) |
        (((uint32_t) vga_mem_read ( addr+2 ))<<16) |
        (((uint32_t) vga_mem_read ( addr+3 ))<<24)
        ;
      ret= true;
    }
  
  // Display memory
  else if ( _regs.misc.display_mem_enabled &&
            _regs.SR.r7.linear_frame_buffer_enabled &&
            ((uint32_t) (addr&0xFE000000)) == _pci_regs.disp_mem_base_addr )
    {
      aperture= (addr>>22)&0x3;
      switch ( aperture )
        {
        case 0: // No swap
          *data= ((uint32_t *) _vram)[(addr&VRAM_MASK)>>2];
#if PC_BE
          *data= PC_SWAP32(*data);
#endif
          break;
        default:
          PC_MSGF("MEM_READ32 (SVGA_CIRRUS_CLGD5446"
                  " - Display memory) addr: %016lX (aperture: %d)",
                  addr,aperture);
          exit(EXIT_FAILURE);
        }
      if ( _trace_enabled && _vga_mem_linear_access != NULL )
        _vga_mem_linear_access ( PC_READ32, aperture,
                                 (uint32_t) (addr&VRAM_MASK),
                                 (uint64_t) *data, _udata );
      ret= true;
    }
  
  // VGA I/O - BitBLT control registers
  else if ( ((uint32_t) (addr&0xFFFFF000)) == _pci_regs.vga_bb_reg_base_addr )
    {
      PC_MSGF("MEM_READ32 (SVGA_CIRRUS_CLGD5446"
              " - VGA I/O -- BitBLT control registers) addr: %016lX",addr);
      *data= 0xFFFFFFFF;
      ret= true;
      exit(EXIT_FAILURE);
    }
  
  else ret= false;
  
  return ret;
  
} // end mem_read32


static bool
mem_read64_bl (
               const uint64_t  addr,
               uint64_t       *data
               )
{

  bool ret;
  uint32_t v0,v1;
  

  if ( mem_read32 ( addr, &v0 ) &&
       mem_read32 ( addr+4, &v1 ) )
    {
      ret= true;
      *data= ((uint64_t) v0) | (((uint64_t) v1)<<32);
    }
  else ret= false;
  
  return ret;
  
} // end mem_read64_bl


static bool
mem_read64 (
            const uint64_t  addr,
            uint64_t       *data
            )
{
  
  bool ret;
  uint32_t tmp;
  

  // Comprova està habilitat.
  if ( (_pci_regs.pcicmd&PCICMD_MEM) == 0 ) ret= false;
  
  // Expansion ROM
  else if ( (_pci_regs.erom&0x1) &&
            ((uint32_t) (addr&_bios.mask)) == (_pci_regs.erom&_bios.mask) )
    {
      tmp= ((uint32_t) addr)&(~_bios.mask);
      if ( tmp < _bios.size_7 )
        {
          *data= BIOS_READ64(tmp);
          ret= true;
        }
      else if ( tmp < _bios.size ) ret= mem_read64_bl ( addr, data );
      else ret= false;
    }

  // Display memorya (VGA mode)
  else if ( _regs.misc.display_mem_enabled &&
            addr >= _vga_mem.begin && addr < _vga_mem.end )
    {
      PC_MSGF("MEM_READ64 (SVGA_CIRRUS_CLGD5446"
              " - Display memory) addr: %016lX",addr);
      *data= 0xFFFFFFFFFFFFFFFF;
      ret= true;
    }
  
  // Display memory
  else if ( _regs.misc.display_mem_enabled &&
            _regs.SR.r7.linear_frame_buffer_enabled &&
            ((uint32_t) (addr&0xFE000000)) == _pci_regs.disp_mem_base_addr )
    {
      PC_MSGF("MEM_READ64 (SVGA_CIRRUS_CLGD5446"
              " - Display memory) addr: %016lX",addr);
      *data= 0xFFFFFFFFFFFFFFFF;
      ret= true;
    }

  // VGA I/O - BitBLT control registers
  else if ( ((uint32_t) (addr&0xFFFFF000)) == _pci_regs.vga_bb_reg_base_addr )
    {
      PC_MSGF("MEM_READ64 (SVGA_CIRRUS_CLGD5446"
              " - VGA I/O -- BitBLT control registers) addr: %016lX",addr);
      *data= 0xFFFFFFFFFFFFFFFF;
      ret= true;
    }
  
  else ret= false;
  
  return ret;
  
} // end mem_read64


static bool
mem_write8 (
            const uint64_t addr,
            const uint8_t  data
            )
{

  bool ret;
  int aperture;
  
  
  // Comprova està habilitat.
  if ( (_pci_regs.pcicmd&PCICMD_MEM) == 0 ) ret= false;
  
  // Expansion ROM
  else if ( (_pci_regs.erom&0x1) &&
            ((uint32_t) (addr&0xFFFFC000)) == (_pci_regs.erom&0xFFFFC000) )
    {
      PC_MSGF("MEM_WRITE8 (SVGA_CIRRUS_CLGD5446"
              " - EROM) addr: %016lX data: %02X",addr,data);
      ret= true;
    }

  // Display memory (VGA mode)
  else if ( _regs.misc.display_mem_enabled &&
            addr >= _vga_mem.begin && addr < _vga_mem.end )
    {
      vga_mem_write ( addr, data );
      ret= true;
    }

  // Display memory
  else if ( _regs.misc.display_mem_enabled &&
            _regs.SR.r7.linear_frame_buffer_enabled &&
            ((uint32_t) (addr&0xFE000000)) == _pci_regs.disp_mem_base_addr )
    {
      aperture= (addr>>22)&0x3;
      switch ( aperture )
        {
        case 0: // No swap
          _vram[addr&VRAM_MASK]= data;
          break;
        default:
          PC_MSGF("MEM_WRITE8 (SVGA_CIRRUS_CLGD5446"
                  " - Display memory) addr: %016lX data: %02X (aperture: %d)",
                  addr,data,aperture);
          exit(EXIT_FAILURE);
        }
      if ( _trace_enabled && _vga_mem_linear_access != NULL )
        _vga_mem_linear_access ( PC_WRITE8, aperture,
                                 (uint32_t) (addr&VRAM_MASK),
                                 (uint64_t) data, _udata );
      ret= true;
    }
  
  // VGA I/O - BitBLT control registers
  else if ( ((uint32_t) (addr&0xFFFFF000)) == _pci_regs.vga_bb_reg_base_addr )
    {
      PC_MSGF("MEM_WRITE8 (SVGA_CIRRUS_CLGD5446"
              " - VGA I/O -- BitBLT control registers)"
              " addr: %016lX data: %02X",addr,data);
      ret= true;
    }
  
  else ret= false;
  
  return ret;
  
} // end mem_write8


static bool
mem_write16 (
             const uint64_t addr,
             const uint16_t data
             )
{

  int aperture;
  bool ret;
#if PC_BE
  uint16_t data_tmp;
#endif

  // Comprova està habilitat.
  if ( (_pci_regs.pcicmd&PCICMD_MEM) == 0 ) ret= false;
  
  // Expansion ROM
  else if ( (_pci_regs.erom&0x1) &&
            ((uint32_t) (addr&0xFFFFC000)) == (_pci_regs.erom&0xFFFFC000) )
    {
      PC_MSGF("MEM_WRITE16 (SVGA_CIRRUS_CLGD5446"
              " - EROM) addr: %016lX data: %04X",addr,data);
      ret= true;
    }

  // Display memory (VGA mode)
  else if ( _regs.misc.display_mem_enabled &&
            addr >= _vga_mem.begin && addr < _vga_mem.end )
    {
      vga_mem_write ( addr, (uint8_t) (data&0xFF) );
      vga_mem_write ( addr+1, (uint8_t) ((data>>8)&0xFF) );
      ret= true;
    }

  // Display memory
  else if ( _regs.misc.display_mem_enabled &&
            _regs.SR.r7.linear_frame_buffer_enabled &&
            ((uint32_t) (addr&0xFE000000)) == _pci_regs.disp_mem_base_addr )
    {
      aperture= (addr>>22)&0x3;
      switch ( aperture )
        {
        case 0: // No swap
#if PC_BE
          data_tmp= PC_SWAP16(data);
          ((uint16_t *) _vram)[(addr&VRAM_MASK)>>1]= data_tmp;
#else
          ((uint16_t *) _vram)[(addr&VRAM_MASK)>>1]= data;
#endif
          break;
        default:
          PC_MSGF("MEM_WRITE16 (SVGA_CIRRUS_CLGD5446"
                  " - Display memory) addr: %016lX data: %04X (aperture: %d)",
                  addr,data,aperture);
          exit(EXIT_FAILURE);
        }
      if ( _trace_enabled && _vga_mem_linear_access != NULL )
        _vga_mem_linear_access ( PC_WRITE16, aperture,
                                 (uint32_t) (addr&VRAM_MASK),
                                 (uint64_t) data, _udata );
      ret= true;
    }
  
  // VGA I/O - BitBLT control registers
  else if ( ((uint32_t) (addr&0xFFFFF000)) == _pci_regs.vga_bb_reg_base_addr )
    {
      PC_MSGF("MEM_WRITE16 (SVGA_CIRRUS_CLGD5446"
              " - VGA I/O -- BitBLT control registers)"
              " addr: %016lX data: %04X",addr,data);
      ret= true;
    }
  
  else ret= false;
  
  return ret;
  
} // end mem_write16


static bool
mem_write32 (
             const uint64_t addr,
             const uint32_t data
             )
{

  bool ret;
  int aperture;
#if PC_BE
  uint32_t data_tmp;
#endif
  

  // Comprova està habilitat.
  if ( (_pci_regs.pcicmd&PCICMD_MEM) == 0 ) ret= false;
  
  // Expansion ROM
  else if ( (_pci_regs.erom&0x1) &&
            ((uint32_t) (addr&0xFFFFC000)) == (_pci_regs.erom&0xFFFFC000) )
    {
      PC_MSGF("MEM_WRITE32 (SVGA_CIRRUS_CLGD5446"
              " - EROM) addr: %016lX data: %08X",addr,data);
      ret= true;
    }

  // Display memory (VGA mode)
  else if ( _regs.misc.display_mem_enabled &&
            addr >= _vga_mem.begin && addr < _vga_mem.end )
    {
      vga_mem_write ( addr, (uint8_t) (data&0xFF) );
      vga_mem_write ( addr+1, (uint8_t) ((data>>8)&0xFF) );
      vga_mem_write ( addr+2, (uint8_t) ((data>>16)&0xFF) );
      vga_mem_write ( addr+3, (uint8_t) ((data>>24)&0xFF) );
      ret= true;
    }

  // Display memory
  else if ( _regs.misc.display_mem_enabled &&
            _regs.SR.r7.linear_frame_buffer_enabled &&
            ((uint32_t) (addr&0xFE000000)) == _pci_regs.disp_mem_base_addr )
    {
      aperture= (addr>>22)&0x3;
      switch ( aperture )
        {
        case 0: // No swap
#if PC_BE
          data_tmp= PC_SWAP32(data);
          ((uint32_t *) _vram)[(addr&VRAM_MASK)>>2]= data_tmp;
#else
          ((uint32_t *) _vram)[(addr&VRAM_MASK)>>2]= data;
#endif
          break;
        default:
          PC_MSGF("MEM_WRITE32 (SVGA_CIRRUS_CLGD5446"
                  " - Display memory) addr: %016lX data: %08X (aperture: %d)",
                  addr,data,aperture);
          exit(EXIT_FAILURE);
        }
      if ( _trace_enabled && _vga_mem_linear_access != NULL )
        _vga_mem_linear_access ( PC_WRITE32, aperture,
                                 (uint32_t) (addr&VRAM_MASK),
                                 (uint64_t) data, _udata );
      ret= true;
    }
  
  // VGA I/O - BitBLT control registers
  else if ( ((uint32_t) (addr&0xFFFFF000)) == _pci_regs.vga_bb_reg_base_addr )
    {
      PC_MSGF("MEM_WRITE32 (SVGA_CIRRUS_CLGD5446"
              " - VGA I/O -- BitBLT control registers)"
              " addr: %016lX data: %08X",addr,data);
      ret= true;
    }
  
  else ret= false;
  
  return ret;
  
} // end mem_write32

static const PC_PCIMem MEM=
  {
    mem_read8,
    mem_read16,
    mem_read32,
    mem_read64,
    mem_write8,
    mem_write16,
    mem_write32
  };




/*************/
/* PCI CLOCK */
/*************/

static int
next_event_cc (void)
{

  int tmp;
  
  
  tmp= _timing.cctoEvent - _timing.cc;
  assert ( tmp > 0 );

  return tmp;
  
} // end next_event_cc


static void
end_iter (void)
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
  
} // end end_iter


static void
set_mode_trace (
                const bool enable
                )
{
  _trace_enabled= enable;
} // end set_mode_trace


static void
reset (void)
{

  clock ( false );
  
  // Timing.
  _timing.cctoEvent= 0;
  _timing.vcc_tmp= 0;
  
  // Rendering
  memset ( _render.fb, 0, sizeof(_render.fb) );
  _render.H= 0;
  _render.V= 0;
  _render.char_dots= 0;
  _render.scanline= 0;
  _render.in_hblank= false;
  _render.in_hretrace= false;
  _render.in_vblank= false;
  _render.vblank_end= 0;
  _render.in_vretrace= false;
  _render.vretrace_end= 0;
  _render.blink= false;
  _render.blink_counter= 0;
  
  // Altres
  memset ( _vram, 0, sizeof(_vram) );
  init_pci_regs ();
  init_regs ();
  update_vclk ();
  
  update_cc_to_event ();
  
} // end reset


static const PC_PCIClock CLOCK=
  {
    next_event_cc,
    end_iter
  };

static const PC_PCIFunction *FUNCS[]= { &FUNC };

const PC_PCICallbacks PC_svga_cirrus_clgd5446=
  {
    .func= FUNCS,
    .N= 1,
    .ports= &PORTS,
    .mem= &MEM,
    .clock= &CLOCK,
    .set_mode_trace= set_mode_trace,
    .reset= reset
  };




/*********************/
/* FUNCIONS PRIVADES */
/*********************/

static void
update_vga_mem (void)
{
  
  switch ( _regs.GR.misc.mem_map )
    {
    case 0: // Extended
      _vga_mem.begin= 0xA0000;
      _vga_mem.end= _vga_mem.begin + 128*1024;
      break;
    case 1: // EGA/VGA
      _vga_mem.begin= 0xA0000;
      _vga_mem.end= _vga_mem.begin + 64*1024;
      break;
    case 2: // Hercules
      _vga_mem.begin= 0xB0000;
      _vga_mem.end= _vga_mem.begin + 32*1024;
      break;
    case 3: // CGA
      _vga_mem.begin= 0xB8000;
      _vga_mem.end= _vga_mem.begin + 32*1024;
      break;
    default:
      printf("WTF!!! update_vga_mem\n");
      exit ( EXIT_FAILURE );
    }
  
} // end update_vga_mem


static void
misc_write (
            const uint8_t data,
            const bool    update_clock,
            const bool    update_vclk_
            )
{

  if ( update_clock ) clock ( false );
  
  _regs.misc.val= data&0xEF;
  _regs.misc.vertical_sync_is_high= ((data&0x80)!=0);
  PC_MSG("SVGA - MISC : Vertical Sync Polarity");
  _regs.misc.horizontal_sync_is_high= ((data&0x40)!=0);
  PC_MSG("SVGA - MISC : Horizontal Sync Polarity");
  _regs.misc.page_select_is_even= ((data&0x20)!=0);
  PC_MSG("SVGA - MISC : Page Select");
  _regs.misc.vlck_freq_ind= (data>>2)&0x3;
  _regs.misc.display_mem_enabled= ((data&0x02)!=0);
  _regs.misc.crtc_io_addr_mode_color= ((data&0x01)!=0);
  
  if ( update_vclk_ ) update_vclk ();
  
  if ( update_clock ) update_cc_to_event ();
  
} // end misc_write


static void
SR_write (
          const uint8_t data,
          const bool    update_clock,
          const bool    update_vclk_
          )
{

  if ( update_clock ) clock ( false );
  
  switch ( _regs.SR.index )
    {
    case 0x00: // Sequencer Reset
      _regs.SR.reset= data&0x3;
      if ( (_regs.SR.reset&0x02) == 0 )
        PC_MSG("SVGA - SR0 : Synchronous Reset !!!");
      if ( (_regs.SR.reset&0x01) == 0 )
        PC_MSG("SVGA - SR0 : Asynchronous Reset !!!");
      break;
    case 0x01: // Sequencer Clocking Mode
      _regs.SR.clocking_mode.val= data&0x3D;
      _regs.SR.clocking_mode.full_bandwidth= ((data&0x20)!=0);
      if ( _regs.SR.clocking_mode.full_bandwidth )
        PC_MSG("SVGA - SR1 : Full Bandwidth");
      _regs.SR.clocking_mode.dot_clock_div2= ((data&0x08)!=0);
      _regs.SR.clocking_mode.dot_clock_8_9= ((data&0x01)!=0);
      _regs.SR.clocking_mode.shift_load= ((data>>3)&0x2) | ((data>>2)&0x1);
      PC_MSGF("SVGA - SR1 : Shift and Load: %X",
              _regs.SR.clocking_mode.shift_load);
      if ( update_vclk_ ) update_vclk ();
      break;
    case 0x02: // Sequencer Plane Mask
      _regs.SR.plane_mask= data&0x0F;
      break;
    case 0x03: // Sequencer Character Map Select
      _regs.SR.char_map.val= data&0x3F;
      _regs.SR.char_map.primary_map= ((data>>2)&0x04) | (data&0x3);
      _regs.SR.char_map.secondary_map= ((data>>3)&0x04) | ((data>>2)&0x3);
      break;
    case 0x04: // Sequencer memory mode
      _regs.SR.mem_mode.val= data&0x0E;
      _regs.SR.mem_mode.chain4= ((data&0x08)!=0);
      _regs.SR.mem_mode.odd_even_mode= ((data&0x04)==0);
      _regs.SR.mem_mode.extended_memory= ((data&0x02)!=0);
      break;
      
    case 0x06: // Key
      _regs.SR.r6_key= data&0x17;
      break;
    case 0x07: // Extended Sequencer Mode
      _regs.SR.r7.val= data;
      _regs.SR.r7.linear_frame_buffer_enabled= ((data&0xF0)!=0);
      _regs.SR.r7.srt= (data>>1)&0x7;
      _regs.SR.r7.extended_display_modes_enabled= ((data&0x1)!=0);
      break;
    case 0x08: // DDC2B/EEPROM Control
      _regs.SR.r8.val= data;
      if ( _regs.SR.r8.val&0x7F != 0 )
        PC_MSGF("SVGA - SR8: DDC2B/ERPROM CONTROL: %X\n",_regs.SR.r8.val);
      break;
      
    case 0x0a: // Scratch pad 1
      _regs.SR.r10= data;
      break;
    case 0x0b ... 0x0e: // VCLK Numerator
      _regs.SR.vclk[_regs.SR.index-0x0b].num= data;
      if ( update_vclk_ ) update_vclk ();
      break;

    case 0x12: // Graphics Cursor Attribute
      _regs.SR.r12.val= data;
      _regs.SR.r12.overscan_color_protect= ((data&0x80)!=0);
      if ( _regs.SR.r12.overscan_color_protect )
        PC_MSG("SVGA - SR12 : Overscan Color Protect");
      _regs.SR.r12.cursor_size_is_32x32= ((data&0x04)==0);
      _regs.SR.r12.allow_access_DAC_extended_colors= ((data&0x02)!=0);
      if ( _regs.SR.r12.allow_access_DAC_extended_colors )
        PC_MSG("SVGA - SR12 : Allow Access to DAC Extended Colors");
      _regs.SR.r12.cursor_enable= ((data&0x01)!=0);
      if ( _regs.SR.r12.cursor_enable )
        {
          PC_MSG("SVGA - SR12 : Graphics Cursor Enable");
          exit ( EXIT_FAILURE );
        }
      break;
    case 0x13: // Graphics Cursor Pattern Address Offset
      _regs.SR.r13_cursor_pat_addr_off= data;
      break;

    case 0x17: // Configuration Readback and Extended Control
      _regs.SR.r17.val= data;
      _regs.SR.r17.dram_bank_1MB= ((data&0x80)!=0);
      if ( _regs.SR.r17.dram_bank_1MB )
        {
          PC_MSG("SVGA - SR17 : DRAM Bank Size Select");
          exit ( EXIT_FAILURE );
        }
      _regs.SR.r17.mem_mapped_io_addr= ((data&0x40)!=0);
      PC_MSGF("SVGA - SR17 : Memory-Mapped I/O Address (%d)",_regs.SR.r17.mem_mapped_io_addr);
      _regs.SR.r17.write_enable_PCI2C= ((data&0x08)!=0);
      if ( _regs.SR.r17.write_enable_PCI2C )
        {
          PC_MSG("SVGA - SR17 : Write Enable PCI2C (Revision B only)");
          exit ( EXIT_FAILURE );
        }
      _regs.SR.r17.enable_mem_mapped_io= ((data&0x04)!=0);
      if ( _regs.SR.r17.enable_mem_mapped_io )
        {
          PC_MSG("SVGA - SR17 : Enable Memory-Mapped I/O");
          exit ( EXIT_FAILURE );
        }
      _regs.SR.r17.enable_dram_bank_swap= ((data&0x02)!=0);
      if ( _regs.SR.r17.enable_dram_bank_swap )
        {
          PC_MSG("SVGA - SR17 : Enable DRAM Bank Swap");
          exit ( EXIT_FAILURE );
        }
      break;
      
    case 0x1b ... 0x1e: // VCLK Numerator
      _regs.SR.vclk[_regs.SR.index-0x1b].den= data;
      if ( update_vclk_ ) update_vclk ();
      break;
      /*
    case 0x0f: // DRAM Control
      _regs.SR.r15.val= data&0xD8;
      _regs.SR.r15.dram_bank_switch= ((data&0x80)!=0);
      _regs.SR.r15.fast_page_detection_disabled= ((data&0x40)!=0);
      _regs.SR.r15.dram_data_bus_width= (data>>3)&0x3;
      printf("[CAL_IMPLEMENTAR] SR.15 - val:%X dram_bank_switch:%d fast_page_detection_disabled:%d dram_data_bus_width:%d\n",_regs.SR.r15.val,_regs.SR.r15.dram_bank_switch,_regs.SR.r15.fast_page_detection_disabled,_regs.SR.r15.dram_data_bus_width);
      break;
      */
    default:
      PC_MSGF ( "SVGA SR.%x=%X write falta implementar !!!!",
                _regs.SR.index,data );
      if(update_clock)exit(EXIT_FAILURE);
    }

  if ( update_clock ) update_cc_to_event ();
  
} // end SR_write


static uint8_t
SR_read (
         const bool update_clock
         )
{

  uint8_t ret;

  
  if ( update_clock ) clock ( true );

  switch ( _regs.SR.index )
    {
    case 0x00: // Sequencer Reset
      ret= _regs.SR.reset;
      break;
    case 0x01: // Sequencer Clocking Mode
      ret= _regs.SR.clocking_mode.val;
      break;
    case 0x02: // Sequencer Plane Mask
      ret= _regs.SR.plane_mask;
      break;
    case 0x03: // Sequencer Character Map Select
      ret= _regs.SR.char_map.val;
      break;
    case 0x04: // Sequencer memory mode
      ret= _regs.SR.mem_mode.val;
      break;

    case 0x06: // Key
      ret= _regs.SR.r6_key==0x12 ? 0x12 : 0xff;
      break;
    case 0x07: // Extended Sequencer Mode
      ret= _regs.SR.r7.val;
      break;

    case 0x0a: // Scratch Pad 1
      ret= _regs.SR.r10;
      break;
    case 0x0b ... 0x0e: // VCLK Numerator
      ret= _regs.SR.vclk[_regs.SR.index-0x0b].num;
      break;
    case 0x0f: // DRAM Control
      // NOTA!!!! No sé que valor ficar en RAS Timing: MD[57] (Read-only).
      // Fique 1 - Standard RAS#
      ret= _regs.SR.r15.val | 0x04;
      break;

    case 0x12: // Graphics Cursor Attribute
      ret= _regs.SR.r12.val;
      break;
    case 0x13: // Graphics Cursor Pattern Address Offset
      ret= _regs.SR.r13_cursor_pat_addr_off;
      break;

    case 0x17: // Configuration Readback and Extended Control
      ret= _regs.SR.r17.val;
      break;
      
    case 0x1b ... 0x1e: // VCLK Numerator
      ret= _regs.SR.vclk[_regs.SR.index-0x1b].den;
      break;
      
    default:
      ret= 0xff;
      PC_MSGF ( "SVGA SR.%x read falta implementar !!!!",
                _regs.SR.index );
    }
  
  return ret;
  
} // end SR_read


static void
CR_write (
          const uint8_t data,
          const bool    update_clock
          )
{

  if ( update_clock ) clock ( false );
  
  switch ( _regs.CR.index )
    {
    case 0x00: // CRTC Horizontal Total
      if ( !_regs.CR.vertical_sync_end.wprotect_cr0_7 )
        _regs.CR.horizontal_total= data;
      break;
    case 0x01: // CRTC Horizontal Display End
      if ( !_regs.CR.vertical_sync_end.wprotect_cr0_7 )
        _regs.CR.horizontal_display_end= data;
      break;
    case 0x02: // CRTC Horizontal Blanking Start
      if ( !_regs.CR.vertical_sync_end.wprotect_cr0_7 )
        _regs.CR.horizontal_blanking_start= data;
      break;
    case 0x03: // CRTC Horizontal Blanking End
      if ( !_regs.CR.vertical_sync_end.wprotect_cr0_7 )
        {
          _regs.CR.horizontal_blanking_end.val= data;
          _regs.CR.horizontal_blanking_end.compatible_read= ((data&0x80)!=0);
          _regs.CR.horizontal_blanking_end.display_enable_skew= (data>>5)&0x3;
          PC_MSGF("SVGA - CR3 : Display Enable Skew: %X",_regs.CR.horizontal_blanking_end.display_enable_skew);
          _regs.CR.horizontal_blanking_end.horizontal_blanking_end= data&0x1F;
        }
      break;
    case 0x04: // CRTC Horizontal Sync Start
      if ( !_regs.CR.vertical_sync_end.wprotect_cr0_7 )
        _regs.CR.horizontal_sync_start= data;
      break;
    case 0x05: // CRTC Horizontal Sync End
      if ( !_regs.CR.vertical_sync_end.wprotect_cr0_7 )
        {
          _regs.CR.horizontal_sync_end.val= data;
          _regs.CR.horizontal_sync_end.horizontal_blanking_end=
            ((data>>2)&0x20);
          _regs.CR.horizontal_sync_end.horizontal_sync_delay= (data>>5)&0x3;
          _regs.CR.horizontal_sync_end.horizontal_sync_end= data&0x1F;
        }
      break;
    case 0x06: // CRTC Vertical Total
      if ( !_regs.CR.vertical_sync_end.wprotect_cr0_7 )
        _regs.CR.vertical_total= (uint16_t) data;
      break;
    case 0x07: // CRTC Overflow
      if ( !_regs.CR.vertical_sync_end.wprotect_cr0_7 )
        {
          _regs.CR.overflow.val= data;
          _regs.CR.overflow.vertical_retrace_start=
            ((uint16_t) (((data&0x80)>>6) | ((data&0x04)>>2)))<<8;
          _regs.CR.overflow.vertical_display_end=
            ((uint16_t) (((data&0x40)>>5) | ((data&0x02)>>1)))<<8;
          _regs.CR.overflow.vertical_total=
            ((uint16_t) (((data&0x20)>>4) | (data&0x01)))<<8;
          _regs.CR.overflow.line_compare= ((data&0x10)!=0) ? 0x0100 : 0x0000;
          PC_MSGF("SVGA - CR7 : Line Compare: %X",_regs.CR.overflow.line_compare);
          _regs.CR.overflow.vertical_blanking_start=
            ((data&0x08)!=0) ? 0x0100 : 0x0000;
        }
      break;
    case 0x08: // CRTC Screen A Preset Row-Scan
      _regs.CR.screen_a_prs.val= data&0x7F;
      _regs.CR.screen_a_prs.byte_pan= (data>>5)&0x3;
      PC_MSGF("SVGA - CR8 : Byte Pan: %X",_regs.CR.screen_a_prs.byte_pan);
      _regs.CR.screen_a_prs.screen_a_prs= data&0x1F;
      PC_MSGF("SVGA - CR8 : Screen A Preset Row Scan: %X",_regs.CR.screen_a_prs.screen_a_prs);
      break;
    case 0x09: // CRTC Character Cell Height
      _regs.CR.char_cell_height.val= data;
      _regs.CR.char_cell_height.scan_double= ((data&0x80)!=0);
      _regs.CR.char_cell_height.line_compare=
        ((data&0x40)!=0) ? 0x0200 : 0x0000;
      PC_MSGF("SVGA - CR9 : Line Compare: %X",_regs.CR.char_cell_height.line_compare);
      _regs.CR.char_cell_height.vertical_blank_start=
        ((data&0x20)!=0) ? 0x0200 : 0x0000;
      _regs.CR.char_cell_height.char_cell_height= data&0x1F;
      break;
    case 0x0a: // CRTC Text Cursor Start
      _regs.CR.text_cursor_start.val= data&0x3F;
      _regs.CR.text_cursor_start.text_cursor_disabled= ((data&0x20)!=0);
      _regs.CR.text_cursor_start.text_cursor_start= data&0x1F;
      break;
    case 0x0b: // CRTC Text Cursor End
      _regs.CR.text_cursor_end.val= data&0x7F;
      _regs.CR.text_cursor_end.text_cursor_skew= (data>>5)&0x3;
      _regs.CR.text_cursor_end.text_cursor_end= data&0x1F;
      break;
    case 0x0c: // CRTC Screen Start Address High
      _regs.CR.screen_start_a_addrH= ((uint16_t) data)<<8;
      break;
    case 0x0d: // CRTC Screen Start Address Low
      _regs.CR.screen_start_a_addrL= (uint16_t) data;
      break;
    case 0x0e: // CRTC Text Cursor Location High
      _regs.CR.text_cursor_locH= ((uint16_t) data)<<8;
      break;
    case 0x0f: // CRTC Text Cursor Location Low
      _regs.CR.text_cursor_locL= (uint16_t) data;
      break;
    case 0x10: // CRTC Vertical Sync Start
      _regs.CR.vertical_sync_start= (uint16_t) data;
      break;
    case 0x11: // CRTC Vertical Sync End
      _regs.CR.vertical_sync_end.val= data;
      _regs.CR.vertical_sync_end.wprotect_cr0_7= ((data&0x80)!=0);
      _regs.CR.vertical_sync_end.refresh_cycle_control_is_1= ((data&0x40)!=0);
      PC_MSGF("SVGA - CR11 : Refresh Cycle Control: %d",_regs.CR.vertical_sync_end.refresh_cycle_control_is_1);
      _regs.CR.vertical_sync_end.disable_vint= ((data&0x20)!=0);
      PC_MSGF("SVGA - CR11 : Disable Vertical Interrupt: %d",_regs.CR.vertical_sync_end.disable_vint);
      PC_MSGF("SVGA - CR11 : Clear Vertical Interrupt: %d",((data&0x10)!=0));
      _regs.CR.vertical_sync_end.vertical_sync_end= data&0x0F;
      break;
    case 0x12: // CRTC Vertical Display End
      _regs.CR.vertical_display_end= data;
      break;
    case 0x13: // CRTC Offset (Pitch)
      _regs.CR.offset= data;
      break;
    case 0x14: // CRTC Underline Row Scanline
      _regs.CR.underline_scanline.val= data&0x7F;
      _regs.CR.underline_scanline.double_word_mode= ((data&0x40)!=0);
      _regs.CR.underline_scanline.count_by_four= ((data&0x20)!=0);
      if ( _regs.CR.underline_scanline.count_by_four )
        PC_MSG("SVGA - CR14 : Count by Four");
      _regs.CR.underline_scanline.underline_scanline= data&0x1F;
      break;
    case 0x15: // CRTC Vertical Blank Start
      _regs.CR.vertical_blank_start= data;
      break;
    case 0x16: // CRTC Vertical Blank End
      _regs.CR.vertical_blank_end= data;
      break;
    case 0x17: // CRTC Mode Control
      _regs.CR.mode.val= data&0xEF;
      _regs.CR.mode.timing_enabled= ((data&0x80)!=0);
      if ( !_regs.CR.mode.timing_enabled )
        PC_MSG("SVGA - CR17 : Timing Disabled");
      _regs.CR.mode.byte_word_mode= ((data&0x40)!=0);
      _regs.CR.mode.addr_wrap= ((data&0x20)!=0);
      _regs.CR.mode.count_by_two= ((data&0x08)!=0);
      if ( _regs.CR.mode.count_by_two )
        PC_MSG("SVGA - CR17 : Count by Two");
      _regs.CR.mode.vregs_by_two= ((data&0x04)!=0);
      _regs.CR.mode.select_rsc_is_1= ((data&0x02)!=0);
      if ( _regs.CR.mode.select_rsc_is_1 )
        PC_MSG("SVGA - CR17 : Select Row-Scan Counter");
      _regs.CR.mode.compatibility_cga_mode= ((data&0x01)==0);
      break;
    case 0x18: // CRTC Line Compare
      _regs.CR.line_compare= data;
      PC_MSGF("SVGA - CR18 : CRTC Line Compare: %X",_regs.CR.line_compare);
      break;
      
    case 0x1a: // Miscellaneous Control
      _regs.CR.misc_ctrl.val= data;
      _regs.CR.misc_ctrl.vblank_end= ((uint16_t) (data>>6))<<8;
      _regs.CR.misc_ctrl.hblank_end= (data>>4)&0x3;
      PC_MSGF("SVGA - CR1A : Horizontal Blank End Overflow: %x",_regs.CR.misc_ctrl.hblank_end);
      _regs.CR.misc_ctrl.ovdac_mode_switch= (data>>2)&0x3;
      PC_MSGF("SVGA - CR1A : Overlay/DAC Mode Switching Control: %x",_regs.CR.misc_ctrl.ovdac_mode_switch);
      _regs.CR.misc_ctrl.double_buff_display_start_addr= ((data&0x02)!=0);
      if ( _regs.CR.misc_ctrl.double_buff_display_start_addr )
        PC_MSG("SVGA - CR1A : Enable Double Buffered Display Start Address");
      _regs.CR.misc_ctrl.interlace_enabled= ((data&0x01)!=0);
      if ( _regs.CR.misc_ctrl.interlace_enabled )
        PC_MSG("SVGA - CR1A : Enable Interlaced");
      break;
    case 0x1b: // Extended Display Controls
      _regs.CR.ext_disp_ctrl.val= data;
      _regs.CR.ext_disp_ctrl.blank_end_extensions_enabled= ((data&0x80)!=0);
      if ( _regs.CR.ext_disp_ctrl.blank_end_extensions_enabled )
        PC_MSG("SVGA - CR1B : Enable Blank End Extensions");
      _regs.CR.ext_disp_ctrl.text_mode_fastpage_enabled= ((data&0x40)!=0);
      if ( _regs.CR.ext_disp_ctrl.text_mode_fastpage_enabled )
        PC_MSG("SVGA - CR1B : Enable Text Mode Fast-page");
      _regs.CR.ext_disp_ctrl.blanking_control_is_1= ((data&0x20)!=0);
      PC_MSGF("SVGA - CR1B : Blanking Control %d",
              _regs.CR.ext_disp_ctrl.blanking_control_is_1);
      _regs.CR.ext_disp_ctrl.offset_overflow= ((data&0x10)!=0) ? 0x100 : 0x00;
      _regs.CR.ext_disp_ctrl.screen_start_a_addr=
        ((uint32_t) ((data>>2)&0x3))<<17;
      _regs.CR.ext_disp_ctrl.ext_addr_wrap_enabled= ((data&0x02)!=0);
      if ( _regs.CR.ext_disp_ctrl.ext_addr_wrap_enabled )
        PC_MSG("SVGA - CR1B : Enable Extended Address Wrap");
      _regs.CR.ext_disp_ctrl.ext_disp_start_addr=
        ((data&0x01)!=0) ? 0x10000 : 0x00000;
      break;

    case 0x1d: // Overlay Extended Control
      _regs.CR.ov_ext_ctrl.val= data&0xFE;
      _regs.CR.ov_ext_ctrl.screen_start_a_addr=
        (data&0x80)!=0 ? ((uint32_t) 1)<<19 : 0x00;
      _regs.CR.ov_ext_ctrl.ov_timing_select_is_1= ((data&0x40)!=0);
      PC_MSGF("SVGA - CR1D : Overlay Timing Select %d",
              _regs.CR.ov_ext_ctrl.ov_timing_select_is_1);
      _regs.CR.ov_ext_ctrl.color_chrome_select_is_1= ((data&0x20)!=0);
      PC_MSGF("SVGA - CR1D : Color Key / Chroma Key Select %d",
              _regs.CR.ov_ext_ctrl.color_chrome_select_is_1);
      _regs.CR.ov_ext_ctrl.color_key_tag_enabled= ((data&0x10)!=0);
      if (_regs.CR.ov_ext_ctrl.color_key_tag_enabled)
        PC_MSG("SVGA - CR1D : Enable Color Key Tag");
      _regs.CR.ov_ext_ctrl.color_compare_width= ((data&0x08)!=0);
      if (_regs.CR.ov_ext_ctrl.color_compare_width)
        PC_MSG("SVGA - CR1D : Color Compare Width");
      _regs.CR.ov_ext_ctrl.dac_mode_switch= (data>>1)&0x3;
      if ( _regs.CR.ov_ext_ctrl.dac_mode_switch != 0 )
        PC_MSGF("SVGA - CR1D : DAC Mode Switching Control %X",
                _regs.CR.ov_ext_ctrl.dac_mode_switch);
      break;
      
    case 0x38: // Video Window Vertical End
      _regs.CR.vid_win_vend= data;
      break;
      
    case 0x3e: // Video Window Master Control
      _regs.CR.vid_win_master_ctrl.val= data;
      _regs.CR.vid_win_master_ctrl.occlusion_enabled= ((data&0x80)!=0);
      if ( _regs.CR.vid_win_master_ctrl.occlusion_enabled )
        {
          PC_MSG("SVGA - CR3E : Occlusion Enable");
          exit(EXIT_FAILURE);
        }
      _regs.CR.vid_win_master_ctrl.error_difussion_enabled= ((data&0x20)!=0);
      if ( _regs.CR.vid_win_master_ctrl.error_difussion_enabled )
        {
          PC_MSG("SVGA - CR3E : Error Diffusion Enable");
          exit(EXIT_FAILURE);
        }
      _regs.CR.vid_win_master_ctrl.vertical_zoom_mode_enabled= ((data&0x10)!=0);
      if ( _regs.CR.vid_win_master_ctrl.vertical_zoom_mode_enabled )
        {
          PC_MSG("SVGA - CR3E : Vertical Zoom Mode");
          exit(EXIT_FAILURE);
        }
      _regs.CR.vid_win_master_ctrl.video_display_format= (data>>1)&0x7;
      _regs.CR.vid_win_master_ctrl.video_window_master_enabled=
        ((data&0x01)!=0);
      break;
     default:
       PC_MSGF ( "SVGA CR.%x=%X write falta implementar !!!!",
                 _regs.CR.index,data );
      if(update_clock)exit(EXIT_FAILURE);
    }

  if ( update_clock ) update_cc_to_event ();
  
} // end CR_write


static uint8_t
CR_read (
         const bool update_clock
         )
{
  
  uint8_t ret;

  
  if ( update_clock ) clock ( true );

  switch ( _regs.CR.index )
    {
    case 0x00: // CRTC Horizontal Total
      ret= _regs.CR.horizontal_total;
      break;
    case 0x01: // CRTC Horizontal Display End
      ret= _regs.CR.horizontal_display_end;
      break;
    case 0x02: // CRTC Horizontal Blanking Start
      ret= _regs.CR.horizontal_blanking_start;
      break;
    case 0x03: // CRTC Horizontal Blanking End
      ret= _regs.CR.horizontal_blanking_end.val;
      break;
    case 0x04: // CRTC Horizontal Sync Start
      ret= _regs.CR.horizontal_sync_start;
      break;
    case 0x05: // CRTC Horizontal Sync End
      ret= _regs.CR.horizontal_sync_end.val;
      break;
    case 0x06: // CRTC Vertical Total
      ret= (uint8_t) _regs.CR.vertical_total;
      break;
    case 0x07: // CRTC Overflow
      ret= _regs.CR.overflow.val;
      break;
    case 0x08: // CRTC Screen A Preset Row-Scan
      ret= _regs.CR.screen_a_prs.val;
      break;
    case 0x09: // CRTC Character Cell Height
      ret= _regs.CR.char_cell_height.val;
      break;
    case 0x0a: // CRTC Text Cursor Start
      ret= _regs.CR.text_cursor_start.val;
      break;
    case 0x0b: // CRTC Text Cursor End
      ret= _regs.CR.text_cursor_end.val;
      break;
    case 0x0c: // CRTC Screen Start Address High
      ret= (uint8_t) (_regs.CR.screen_start_a_addrH>>8);
      break;
    case 0x0d: // CRTC Screen Start Address Low
      ret= (uint8_t) _regs.CR.screen_start_a_addrL;
      break;
    case 0x0e: // CRTC Text Cursor Location High
      ret= (uint8_t) (_regs.CR.text_cursor_locH>>8);
      break;
    case 0x0f: // CRTC Text Cursor Location Low
      ret= (uint8_t) _regs.CR.text_cursor_locL;
      break;
    case 0x10: // CRTC Vertical Sync Start
      ret= _regs.CR.horizontal_blanking_end.compatible_read ?
        ((uint8_t) _regs.CR.vertical_sync_start) : 0xff;
      break;
    case 0x11: // CRTC Vertical Sync End
      ret= _regs.CR.horizontal_blanking_end.compatible_read ?
        _regs.CR.vertical_sync_end.val : 0xff;
      break;
    case 0x12: // CRTC Vertical Display End
      ret= (uint8_t) _regs.CR.vertical_display_end;
      break;
    case 0x13: // CRTC Offset (Pitch)
      ret= _regs.CR.offset;
      break;
    case 0x14: // CRTC Underline Row Scanline
      ret= _regs.CR.underline_scanline.val;
      break;
    case 0x15: // CRTC Vertical Blank Start
      ret= _regs.CR.vertical_blank_start;
      break;
    case 0x16: // CRTC Vertical Blank End
      ret= _regs.CR.vertical_blank_end;
      break;
    case 0x17: // CRTC Mode Control
      ret= _regs.CR.mode.val;
      break;
    case 0x18: // CRTC Line Compare
      ret= _regs.CR.line_compare;
      break;
      
    case 0x1a: // Miscellaneous Control
      ret= _regs.CR.misc_ctrl.val;
      break;
    case 0x1b: // Extended Display Controls
      ret= _regs.CR.ext_disp_ctrl.val;
      break;

    case 0x1d: // Overlay Extended Control
      ret= _regs.CR.ov_ext_ctrl.val;
      break;

    case 0x1f: // Unused
      ret= 0xff;
      break;
      
    default:
      ret= 0xff;
      PC_MSGF ( "SVGA CR.%x read falta implementar !!!!",
                _regs.CR.index );
    }
  
  return ret;
  
} // end CR_read


static void
GR_write (
          const uint8_t data,
          const bool    update_clock
          )
{

  bool tmp;


  if ( update_clock ) clock ( false );
  
  switch ( _regs.GR.index )
    {
    case 0x00: // Set/Reset / Background Color Byte 0
      _regs.GR.r0.bg_colorb0= data;
      /*
      printf("[CAL_IMPLEMENTAR] SVGA - GR0 : Background Color Byte 0 %X\n",
               _regs.GR.r0.bg_colorb0 );
      */
      _regs.GR.r0.set_reset= data&0x0F;
      break;
    case 0x01: // Set/Reset Enable / Foreground Color Byte 0
      _regs.GR.r1.fg_colorb0= data;
      _regs.GR.r1.enable_sr= data&0x0F;
      break;
    case 0x02: // Color Compare
      _regs.GR.color_compare= data&0x0F;
      break;
    case 0x03: // Data Rotate
      _regs.GR.data_rotate.val= data&0x1F;
      _regs.GR.data_rotate.func= (data>>3)&0x3;
      _regs.GR.data_rotate.count= data&0x7;
      break;
    case 0x04: // Read Map Select
      _regs.GR.read_map_select= data&0x3;
      break;
    case 0x05: // Graphics Controller Mode
      _regs.GR.mode.val= data&0x7B;
      _regs.GR.mode.color256= ((data&0x40)!=0);
      _regs.GR.mode.shift_reg_mode_is_1= ((data&0x20)!=0);
      _regs.GR.mode.odd_even_mode= ((data&0x10)!=0);
      _regs.GR.mode.read_mode1= ((data&0x08)!=0);
      _regs.GR.mode.write_mode= data&0x03;
      break;
    case 0x06: // Miscellaneous
      _regs.GR.misc.val= data&0x0F;
      _regs.GR.misc.mem_map= (data>>2)&0x3;
      update_vga_mem ();
      _regs.GR.misc.chain_odd_maps_to_event_is_1= ((data&0x02)!=0);
      PC_MSGF("SVGA - GR6 : Chain Odd Maps to Even %d",
              _regs.GR.misc.chain_odd_maps_to_event_is_1 );
      _regs.GR.misc.apa_mode= ((data&0x01)!=0);
      break;
    case 0x07: // Color Don't Care
      _regs.GR.color_dont_care= data&0x0F;
      break;
    case 0x08: // Bit Mask
      _regs.GR.bit_mask= data;
      break;
    case 0x09: // Offset Register 0
      _regs.GR.offset0= data;
      break;
    case 0x0a: // Offset Register 1
      _regs.GR.offset1= data;
      break;
    case 0x0b: // Graphics Controller Mode Extensions
      _regs.GR.rb.val= data&0x3F;
      _regs.GR.rb.offset_granularity= ((data&0x20)!=0);
      _regs.GR.rb.enhanced_writes_16bit_enabled= ((data&0x10)!=0);
      if ( _regs.GR.rb.enhanced_writes_16bit_enabled )
        PC_MSG("SVGA - GRB : Enable Enhanced Writes for 16-bit pixels" );
      _regs.GR.rb.eightbyte_data_latches_enabled= ((data&0x08)!=0);
      if ( _regs.GR.rb.eightbyte_data_latches_enabled )
        PC_MSG("SVGA - GRB : Enable Eight-Byte Data Latches" );
      _regs.GR.rb.extended_write_modes_enabled= ((data&0x04)!=0);
      if ( _regs.GR.rb.extended_write_modes_enabled )
        PC_MSG("SVGA - GRB : Enable Extended Write Modes" );
      _regs.GR.rb.by8_addr_enabled= ((data&0x02)!=0);
      if ( _regs.GR.rb.by8_addr_enabled )
        PC_MSG("SVGA - GRB : Enable BY8 Addressing" );
      _regs.GR.rb.offset1_enabled= ((data&0x01)!=0);
      break;

    case 0x0f: // Aquest registre no està suportat, però el vaig a
               // emprar per a activar compatibilitat amb altres
               // targetes en alguns jocs concrets.
      _warning ( _udata,
                 "PCI:CLGD5446.GR_write: registre no suportat -"
                 " GR.f=%X. Activant mode compatibilitat altres targetes!!\n",
                 data );
      break;
      
    case 0x31: // BLST Start/Status
      _regs.GR.r49.val= data&0xE6;
      _regs.GR.r49.enable_autostart= ((data&0x80)!=0);
      if ( _regs.GR.r49.enable_autostart )
        PC_MSG("SVGA - GR31 : Enable Autostart");
      _regs.GR.r49.use_system_source_location= ((data&0x40)!=0);
      if ( _regs.GR.r49.use_system_source_location )
        PC_MSG("SVGA - GR31 : System Source Location");
      _regs.GR.r49.pause= ((data&0x20)!=0);
      if ( _regs.GR.r49.pause )
        PC_MSG("SVGA - GR31 : Pause");
      tmp= ((data&0x04)!=0);
      if ( !_regs.GR.r49.blt_reset && tmp )
        PC_MSG("SVGA - GR31 : BLT Reset !!!!!");
      _regs.GR.r49.blt_reset= tmp;
      tmp= ((data&0x02)!=0);
      if ( !_regs.GR.r49.blt_start && tmp )
        PC_MSG("SVGA - GR31 : BLT Start !!!!!");
      _regs.GR.r49.blt_start= tmp;
      break;
      
    default:
      PC_MSGF ( "SVGA GR.%x=%X write falta implementar !!!!",
                _regs.GR.index,data );
      if(update_clock)exit(EXIT_FAILURE);
    }

  if ( update_clock ) update_cc_to_event ();
  
} // end GR_write


static uint8_t
GR_read (
         const bool update_clock
         )
{
  
  uint8_t ret;

  
  if ( update_clock ) clock ( true );

  switch ( _regs.GR.index )
    {
    case 0x00: // Set/Reset / Background Color Byte 0
      ret= _regs.GR.r0.bg_colorb0;
      break;
    case 0x01: // Set/Reset Enable / Foreground Color Byte 0
      ret= _regs.GR.r1.fg_colorb0;
      break;
    case 0x02: // Color Compare
      ret= _regs.GR.color_compare;
      break;
    case 0x03: // Data Rotate
      ret= _regs.GR.data_rotate.val;
      break;
    case 0x04: // Read Map Select
      ret= _regs.GR.read_map_select;
      break;
    case 0x05: // Graphics Controller Mode
      ret= _regs.GR.mode.val;
      break;
    case 0x06: // Miscellaneous
      ret= _regs.GR.misc.val;
      break;
    case 0x07: // Color Don't Care
      ret= _regs.GR.color_dont_care;
      break;
    case 0x08: // Bit Mask
      ret= _regs.GR.bit_mask;
      break;
    case 0x09: // Offset Register 0
      ret= _regs.GR.offset0;
      break;
    case 0x0a: // Offset Register 1
      ret= _regs.GR.offset1;
      break;
    case 0x0b: // Graphics Controller Mode Extensions
      ret= _regs.GR.rb.val;
      break;
      
    case 0x31: // BLT Start/Status
      ret= _regs.GR.r49.val;
      PC_MSG("BLT Start/Status - Falta Buffered Register Status,BLT Status,BLT Start");
      break;
      
    default:
      ret= 0xff;
      PC_MSGF ( "SVGA GR.%x read falta implementar !!!!",
                _regs.GR.index );
    }
  
  return ret;
  
} // end GR_read


static void
AR_write (
          const uint8_t data,
          const bool    update_clock
          )
{

  if ( update_clock ) clock ( false );
  
  if ( _regs.AR.mode_data )
    {
      _regs.AR.index= data&0x1F;
      _regs.AR.display_enabled= ((data&0x20)!=0);
    }
  else
    {
      switch ( _regs.AR.index )
        {
        case 0x00 ... 0x0F: // Attribute Controller Palette
          _regs.AR.pal[_regs.AR.index]= data&0x3F;
          break;
        case 0x10: // Attribute Controller Mode
          _regs.AR.attr_ctrl_mode.val= data&0xEF;
          _regs.AR.attr_ctrl_mode.ar14_enabled= ((data&0x80)!=0);
          if ( _regs.AR.attr_ctrl_mode.ar14_enabled )
            PC_MSG("SVGA - AR10 : AR14 Graphics Source Enable");
          _regs.AR.attr_ctrl_mode.pixel_double_clock= ((data&0x40)!=0);
          _regs.AR.attr_ctrl_mode.pixel_panning_comp= ((data&0x20)!=0);
          if ( _regs.AR.attr_ctrl_mode.pixel_panning_comp )
            PC_MSG("SVGA - AR10 : Pixel Panning Compatibility");
          _regs.AR.attr_ctrl_mode.blink_enabled= ((data&0x08)!=0);
          _regs.AR.attr_ctrl_mode.line_graphics_enabled= ((data&0x04)!=0);
          _regs.AR.attr_ctrl_mode.display_type_is_1= ((data&0x02)!=0);
          PC_MSGF("SVGA - AR10 : Display Type %d",
                  _regs.AR.attr_ctrl_mode.display_type_is_1);
          _regs.AR.attr_ctrl_mode.use_apa_mode= ((data&0x01)!=0);
          if ( _regs.AR.attr_ctrl_mode.use_apa_mode )
            PC_MSG("SVGA - AR10 : Graphics Mode");
          break;
        case 0x11: // Overscan (Border) Color
          _regs.AR.overscan_color= data&0x3F;
          break;
        case 0x12: // Color Plane Enable
          _regs.AR.color_plane.val= data&0x3F;
          _regs.AR.color_plane.video_status_mux= (data>>4)&0x3;
          _regs.AR.color_plane.enable= data&0x0F;
          break;
        case 0x13: // Pixel Panning
          _regs.AR.pixel_panning= data&0x0F;
          break;
        case 0x14: // Color Select
          _regs.AR.color_select= data&0x0F;
          PC_MSGF("SVGA - AR14 : Color Bit C : %X",_regs.AR.color_select);
          break;

          /*
        case 0x16: // ??
          _warning ( _udata, "SVGA AR.%x=%X - registre desconegut",
                     _regs.AR.index, data );
          break;
          */
        default:
          PC_MSGF ( "SVGA AR.%x=%X write falta"
                    " implementar !!!!",
                    _regs.AR.index,data );
          if(update_clock)exit(EXIT_FAILURE);
        }
    }
  _regs.AR.mode_data= !_regs.AR.mode_data;

  if ( update_clock ) update_cc_to_event ();
  
} // end AR_write


static uint8_t
AR_read (
         const bool update_clock
         )
{
  
  uint8_t ret;

  
  if ( update_clock ) clock ( true );
  
  if ( _regs.AR.mode_data )
    ret= _regs.AR.index | (_regs.AR.display_enabled ? 0x20 : 0x00);
  else
    {
      switch ( _regs.AR.index )
        {
        case 0x00 ... 0x0F: // Attribute Controller Palette
          ret= _regs.AR.pal[_regs.AR.index];
          break;
        case 0x10: // Attribute Controller Mode
          ret= _regs.AR.attr_ctrl_mode.val;
          break;
        case 0x11: // Overscan (Border) Color
          ret= _regs.AR.overscan_color;
          break;
        case 0x12: // Color Plane Enable
          ret= _regs.AR.color_plane.val;
          break;
        case 0x13: // Pixel Panning
          ret= _regs.AR.pixel_panning;
          break;
        case 0x14: // Color Select
          ret= _regs.AR.color_select;
          break;
        default:
          ret= 0xff;
          PC_MSGF ( "SVGA AR.%x read falta implementar !!!!",
                    _regs.AR.index );
        }
      
    }

  return ret;
  
} // end AR_read


static void
pixel_mask_write (
                  const uint8_t data,
                  const bool    update_clock
                  )
{

  if ( update_clock ) clock ( false );

  if ( _regs.hdr.counter == 4 )
    {
      _regs.hdr.counter= 0;
      _regs.hdr.val= data;
      _regs.hdr.mode_555_enabled= ((data&0x80)!=0);
      _regs.hdr.all_ext_modes_enabled= ((data&0x40)!=0);
      _regs.hdr.clocking_mode_is_1= ((data&0x20)!=0);
      _regs.hdr.control_32k_color_enabled= ((data&0x10)!=0);
      _regs.hdr.ext_mode= data&0xf;
    }
  else
    {
      _regs.pixel_mask= data;
      _regs.hdr.counter= 0;
    }

  if ( update_clock ) update_cc_to_event ();
  
} // end pixel_mask_write


static uint8_t
pixel_mask_read (
                 const bool update_clock
                 )
{

  uint8_t ret;

  
  if ( update_clock ) clock ( true );

  if ( _regs.hdr.counter == 4 )
    {
      ret= _regs.hdr.val;
      _regs.hdr.counter= 0;
    }
  else
    {
      ret= _regs.pixel_mask;
      ++_regs.hdr.counter;
    }

  return ret;
  
} // end pixel_mask_read


static uint8_t
stat_read (void)
{

  uint8_t ret,mux;
  
  
  clock ( true );

  switch ( _regs.AR.color_plane.video_status_mux )
    {
    case 0: mux= (_render.pixel_bus&0x1)|((_render.pixel_bus>>1)&0x2); break;
    case 1: mux= (_render.pixel_bus>>4)&0x3; break;
    case 2:
      mux= ((_render.pixel_bus>>1)&0x1)|((_render.pixel_bus>>2)&0x2);
      break;
    case 3: mux= (_render.pixel_bus>>6)&0x3; break;
    default:
      fprintf ( stderr, "WTF - stat_read\n" );
      exit ( EXIT_FAILURE );
    }
  ret=
    (mux<<4) | // Diagnostic
    (_render.in_vretrace ? 0x08 : 0x00) | // Vertical Retrace
    ((_render.in_hblank || _render.in_vblank) ? 0x01 : 0x00) // Display Enable
    ;
  
  // NOTA!!! Açò ho he deduit de la BIOS però on està clar!!!
  _regs.AR.mode_data= true;
  
  return ret;
  
} // end stat_read


static void
dac_data_write (
                const uint8_t data
                )
{

  int i;
  uint8_t tmp;
  

  clock ( false );
  _dac.buffer_w[_dac.buffer_w_off++]= data;
  if ( _dac.buffer_w_off == 3 )
    {

      // NOTA!!! No entenc si el pixel_mask funciona així !!!!!
      for ( i= 0; i < 3; ++i )
        {
          tmp= _dac.v[_dac.addr_w][i];
          _dac.v[_dac.addr_w][i]=
            (tmp&(~_regs.pixel_mask)) | (_dac.buffer_w[i]&_regs.pixel_mask);
        }
      _dac.buffer_w_off= 0;
      ++_dac.addr_w;
    }

  update_cc_to_event ();
  
} // end dac_data_write


static uint8_t
dac_data_read (void)
{

  int i;
  uint8_t ret;
  

  clock ( true );
  
  ret= _dac.buffer_r[_dac.buffer_r_off++];
  if ( _dac.buffer_r_off == 3 )
    {
      // NOTA!!! No entenc si el pixel_mask funciona així !!!!!
      ++_dac.addr_r;
      for ( i= 0; i < 3; ++i )
        _dac.buffer_r[i]= _dac.v[_dac.addr_r][i]&_regs.pixel_mask;
      _dac.buffer_r_off= 0;
    }

  return ret;
  
} // end dac_data_read


static void
dac_addr_w_write (
                  const uint8_t data
                  )
{
  
  clock ( false );
  
  _dac.buffer_w_off= 0;
  _dac.addr_w= data;

  update_cc_to_event ();
  
} // end dac_addr_w_write


static void
dac_addr_r_write (
                  const uint8_t data
                  )
{

  int i;
  
  
  clock ( false );
  
  _dac.buffer_r_off= 0;
  _dac.addr_r= data;
  // NOTA!! no sé si pixel_mask funciona així.
  for ( i= 0; i < 3; ++i )
    _dac.buffer_r[i]= _dac.v[_dac.addr_r][i]&_regs.pixel_mask;

  update_cc_to_event ();
  
} // end dac_addr_r_write


static uint32_t
mem_addr2xma (
              const uint64_t mem_addr
              )
{

  uint32_t xa,xma;

  
  // Calcula XA
  xa= mem_addr-_vga_mem.begin; // Correcte ???
  if ( _regs.GR.misc.mem_map == 1 && !_regs.GR.rb.offset1_enabled )
    xa&= 0xFFFF;
  else if ( _regs.GR.misc.mem_map == 1 || _regs.GR.rb.offset1_enabled )
    xa&= 0x7FFF;
  // ??? Que passa si no és cap dels dos?? Es deixa estar?

  // Calcula XMA
  if ( _regs.GR.rb.offset_granularity )
    {
      if ( !_regs.GR.rb.offset1_enabled || (mem_addr&0x8000)==0 )
        xma= xa + (((uint32_t) (_regs.GR.offset0&0x7f))<<14);
      else
        xma= xa + (((uint32_t) (_regs.GR.offset1&0x7f))<<14);
    }
  else
    {
      if ( !_regs.GR.rb.offset1_enabled || (mem_addr&0x8000)==0 )
        xma= xa + (((uint32_t) _regs.GR.offset0)<<12);
      else
        xma= xa + (((uint32_t) _regs.GR.offset1)<<12);
    }

  return xma;
  
} // mem_addr2xma


static void
vga_mem_write_mode0 (
                     const uint16_t offset,
                     const uint8_t  plane_sel,
                     const uint8_t  data
                     )
{

  uint8_t val,planes,sr_mask,sr_val,tmp_val;
  int shift,i;
  
  
  // Rota el valor
  if ( _regs.GR.data_rotate.count > 0 )
    {
      shift= _regs.GR.data_rotate.count;
      val= (data>>shift) | (data<<(8-shift));
    }
  else val= data;

  // Intenta desar en tots els plans
  planes= plane_sel&_regs.SR.plane_mask;
  sr_val= _regs.GR.r0.set_reset;
  sr_mask= _regs.GR.r1.enable_sr;
  for ( i= 0; i < 4; ++i )
    {
      if ( planes&0x1 )
        {
          if ( sr_mask&0x1 ) tmp_val= sr_val&0x1 ? 0xff : 0x00;
          else               tmp_val= val;
          switch ( _regs.GR.data_rotate.func )
            {
            case 0: break;
            case 1: tmp_val&= _vga_mem.latch[i]; break;
            case 2: tmp_val|= _vga_mem.latch[i]; break;
            case 3: tmp_val^= _vga_mem.latch[i]; break;
            default:
              printf("[EE] vga_mem_write_mode0 - WTF!!!\n");
              exit(EXIT_FAILURE);
            }
          tmp_val=
            (tmp_val&_regs.GR.bit_mask) |
            (_vga_mem.latch[i]&(~_regs.GR.bit_mask));
          _vga_mem.p[i][offset]= tmp_val;
          if ( _trace_enabled && _vga_mem_access != NULL )
            _vga_mem_access ( false, i, (uint32_t) offset, tmp_val, _udata );
        }
      planes>>= 1;
      sr_mask>>= 1; sr_val>>= 1;
    }
  
} // end vga_mem_write_mode0


static void
vga_mem_write_mode1 (
                     const uint16_t offset,
                     const uint8_t  plane_sel,
                     const uint8_t  data
                     )
{
  
  uint8_t planes,tmp_val;
  int i;
  
  
  // Intenta desar en tots els plans
  planes= plane_sel&_regs.SR.plane_mask;
  for ( i= 0; i < 4; ++i )
    {
      if ( planes&0x1 )
        {
          tmp_val= _vga_mem.latch[i];
          _vga_mem.p[i][offset]= tmp_val;
          if ( _trace_enabled && _vga_mem_access != NULL )
            _vga_mem_access ( false, i, (uint32_t) offset, tmp_val, _udata );
        }
      planes>>= 1;
    }
  
} // end vga_mem_write_mode1


static void
vga_mem_write_mode2 (
                     const uint16_t offset,
                     const uint8_t  plane_sel,
                     const uint8_t  data
                     )
{
  
  uint8_t planes,tmp_val,val;
  int i;
  
  
  // Intenta desar en tots els plans
  planes= plane_sel&_regs.SR.plane_mask;
  val= data;
  for ( i= 0; i < 4; ++i )
    {
      if ( planes&0x1 )
        {
          tmp_val= (val&0x1)!=0 ? 0xff: 0x00;
          switch ( _regs.GR.data_rotate.func )
            {
            case 0: break;
            case 1: tmp_val&= _vga_mem.latch[i]; break;
            case 2: tmp_val|= _vga_mem.latch[i]; break;
            case 3: tmp_val^= _vga_mem.latch[i]; break;
            default:
              printf("[EE] vga_mem_write_mode2 - WTF!!!\n");
              exit(EXIT_FAILURE);
            }
          tmp_val=
            (tmp_val&_regs.GR.bit_mask) |
            (_vga_mem.latch[i]&(~_regs.GR.bit_mask));
          _vga_mem.p[i][offset]= tmp_val;
          if ( _trace_enabled && _vga_mem_access != NULL )
            _vga_mem_access ( false, i, (uint32_t) offset, tmp_val, _udata );
        }
      planes>>= 1;
      val>>= 1;
    }
  
} // end vga_mem_write_mode2


static void
vga_mem_write_mode3 (
                     const uint16_t offset,
                     const uint8_t  plane_sel,
                     const uint8_t  data
                     )
{
  
  uint8_t val,bit_mask,sr_val,planes,tmp_val;
  int shift,i;
  
  
  // Rota el valor i calcula bitmask
  if ( _regs.GR.data_rotate.count > 0 )
    {
      shift= _regs.GR.data_rotate.count;
      val= (data>>shift) | (data<<(8-shift));
    }
  else val= data;
  bit_mask= val&_regs.GR.bit_mask;

  // Intenta desar en tots els plans
  planes= plane_sel&_regs.SR.plane_mask;
  sr_val= _regs.GR.r0.set_reset;
  for ( i= 0; i < 4; ++i )
    {
      if ( planes&0x1 )
        {
          tmp_val= sr_val&0x1 ? 0xff : 0x00;
          tmp_val=
            (tmp_val&bit_mask) |
            (_vga_mem.latch[i]&(~bit_mask));
          _vga_mem.p[i][offset]= tmp_val;
          if ( _trace_enabled && _vga_mem_access != NULL )
            _vga_mem_access ( false, i, (uint32_t) offset, tmp_val, _udata );
        }
      planes>>= 1;
      sr_val>>= 1;
    }
  
} // end vga_mem_write_mode3


static void
vga_mem_write_basic (
                     const uint64_t addr,
                     const uint8_t  data
                     )
{

  uint64_t tmp;
  uint16_t offset;
  uint8_t plane_sel;
  
  
  // Descodifica adreça.
  tmp= addr-_vga_mem.begin;
  // --> Chain4
  if ( _regs.SR.mem_mode.chain4 )
    {
      offset= (uint16_t) (tmp&0xFFFC);
      plane_sel= (uint8_t) (1<<(tmp&0x3));
    }
  // --> Odd/Even - Pareix que han d'estar els dos
  else if ( _regs.SR.mem_mode.odd_even_mode &&
            _regs.GR.mode.odd_even_mode )
    {
      offset= (uint16_t) (tmp&0xFFFE);
      plane_sel= tmp&0x1 ? 0x0A : 0x05;
    }
  // --> Normal
  else
    {
      offset= (uint16_t) (tmp&0xFFFF);
      plane_sel= 0xF; // Seleccione tots????
    }
  
  // Escriu
  switch ( _regs.GR.mode.write_mode )
    {
    case 0: vga_mem_write_mode0 ( offset, plane_sel, data ); break;
    case 1: vga_mem_write_mode1 ( offset, plane_sel, data ); break;
    case 2: vga_mem_write_mode2 ( offset, plane_sel, data ); break;
    case 3: vga_mem_write_mode3 ( offset, plane_sel, data ); break;
    default:
      printf("[EE] vga_mem_write - unknown write_mode: %X\n",
             _regs.GR.mode.write_mode);
      exit(EXIT_FAILURE);
    }
  
} // end vga_mem_write_basic


static void
vga_mem_write_extended (
                        const uint64_t mem_addr,
                        const uint8_t  data
                        )
{
  
  uint32_t xma;
  
  
  // NOTA!! Açò caldria fer-ho també si la memòria extenguda no està
  // activada?
  xma= mem_addr2xma ( mem_addr );
  _vram[xma&VRAM_MASK]= data;
  if ( _trace_enabled && _vga_mem_access != NULL )
    _vga_mem_access ( false, -1, xma&VRAM_MASK, data, _udata );
  
} // end vga_mem_write_extended


static void
vga_mem_write (
               const uint64_t addr,
               const uint8_t  data
               )
{
  
  if ( _regs.SR.r7.extended_display_modes_enabled )
    {
      if ( _regs.SR.mem_mode.extended_memory )
        vga_mem_write_extended ( addr, data );
      else
        {
          PC_MSG("vga_mem_write - MODE NO EXTENDED");
          exit(EXIT_FAILURE);
        }
    }
  else
    vga_mem_write_basic ( addr, data );
  
} // end vga_mem_write


static uint8_t
vga_mem_read_basic (
                    const uint64_t addr
                    )
{

  int i,plane;
  uint64_t tmp;
  uint16_t offset;
  //uint8_t plane_sel;
  uint8_t ret,tmp8;
  
  
  // NOTA!!! No està gens clar com es selecciona el plane en modo
  // lectura. Els manuals diuen que sempre és el planol de
  // READ_MAP_SELECT, però és fals. El que sí que pareix és que sempre
  // elegix un únic planol. El que vaig a fer és en els casos amb més
  // d'un planol elegir el més menut.
  
  // Descodifica adreça.
  tmp= addr-_vga_mem.begin;
  // --> Chain4
  if ( _regs.SR.mem_mode.chain4 )
    {
      offset= (uint16_t) (tmp&0xFFFC);
      plane= tmp&0x3; // ¿¿?? ¿¿Pareix que funciona????
      //plane_sel= (uint8_t) (1<<(tmp&0x3));
    }
  // --> Odd/Even - Pareix que han d'estar els dos.
  else if ( _regs.SR.mem_mode.odd_even_mode &&
            _regs.GR.mode.odd_even_mode )
    {
      offset= (uint16_t) (tmp&0xFFFE);
      //plane_sel= tmp&0x1 ? 0x0A : 0x05;
      plane= tmp&0x1 ? 1 : 0;
    }
  // --> Normal
  else
    {
      offset= (uint16_t) (tmp&0xFFFF);
      //plane_sel= (uint8_t) (1<<_regs.GR.read_map_select);
      plane= _regs.GR.read_map_select;
    }

  // NOTA!!! No tinc gens clar si el latch registre com funciona. De
  // moment interprete que de tots els planols es llig un byte.
  for ( i= 0; i < 4; ++i )
    _vga_mem.latch[i]= _vga_mem.p[i][offset];
  
  // Llig
  if ( _regs.GR.mode.read_mode1 )
    {
      ret= 0;
      for ( i= 0; i < 8; ++i )
        {
          ret<<= 1;
          tmp8=
            (((_vga_mem.latch[0]<<i)&0x80)>>7) |
            (((_vga_mem.latch[1]<<i)&0x80)>>6) |
            (((_vga_mem.latch[2]<<i)&0x80)>>5) |
            (((_vga_mem.latch[3]<<i)&0x80)>>4);
          ret|=
            ((tmp8&_regs.GR.color_dont_care) ==
             (_regs.GR.color_compare&_regs.GR.color_dont_care));
        }
      if ( _trace_enabled && _vga_mem_access != NULL )
        _vga_mem_access ( true, 0xff, (uint32_t) offset, ret, _udata );
    }
  else // MODE 0
    {
      // NOTA!!! En mode0 no es gasta el plane_sel. Es gasta read_map_select
      ret= _vga_mem.latch[plane];
      if ( _trace_enabled && _vga_mem_access != NULL )
        _vga_mem_access ( true, plane, (uint32_t) offset, ret, _udata );
    }
  
  return ret;
  
} // end vga_mem_read_basic


static uint8_t
vga_mem_read_extended (
                       const uint64_t addr
                       )
{

  uint8_t ret;
  uint32_t xma;
  
  
  // NOTA!! Açò caldria fer-ho també si la memòria extenguda no està
  // activada?
  xma= mem_addr2xma ( addr );
  ret= _vram[xma&VRAM_MASK];
  if ( _trace_enabled && _vga_mem_access != NULL )
    _vga_mem_access ( true, -1, xma&VRAM_MASK, ret, _udata );
  
  return ret;
  
} // end vga_mem_read_extended


static uint8_t
vga_mem_read (
              const uint64_t addr
              )
{

  uint8_t ret;
  

  if ( _regs.SR.r7.extended_display_modes_enabled )
    {
      if ( _regs.SR.mem_mode.extended_memory )
        ret= vga_mem_read_extended ( addr );
      else
        {
          PC_MSG("vga_mem_read - MODE NO EXTENDED");
          exit(EXIT_FAILURE);
        }
    }
  else
    ret= vga_mem_read_basic ( addr );
  
  return ret;
  
} // end vga_mem_read


static void
init_pci_regs (void)
{

  _pci_regs.pcicmd= 0x0000;
  _pci_regs.disp_mem_base_addr= 0x00000000;
  _pci_regs.vga_bb_reg_base_addr= 0x00000000;
  _pci_regs.erom= 0x00000000;
  _pci_regs.intln= 0x00;
  
} // end init_pci_regs


static void
init_dac (void)
{
  
  memset ( _dac.v, 0, sizeof(_dac.v) );
  _dac.addr_w= 0;
  _dac.addr_r= 0;
  _dac.buffer_w_off= 0;
  memset ( _dac.buffer_w, 0, sizeof(_dac.buffer_w) );
    _dac.buffer_r_off= 0;
  memset ( _dac.buffer_r, 0, sizeof(_dac.buffer_r) );
  
} // end init_dac


static void
init_vga_mem (void)
{

  int i;


  for ( i= 0; i < 4; ++i )
    _vga_mem.p[i]= &_vram[i*64*1024];
  memset ( _vga_mem.latch, 0, sizeof(_vga_mem.latch) );
  update_vga_mem ();
  
} // end init_vga_mem


static void
init_regs (void)
{

  init_dac ();
  misc_write ( 0x00, false, false );
  pixel_mask_write ( 0xff, false );
  pixel_mask_read ( false );
  pixel_mask_read ( false );
  pixel_mask_read ( false );
  pixel_mask_read ( false );
  pixel_mask_write ( 0x00, false );
  _regs.SR.index= 0; SR_write ( 0x03, false, false );
  _regs.SR.index= 1; SR_write ( 0x00, false, false );
  _regs.SR.index= 2; SR_write ( 0x00, false, false );
  _regs.SR.index= 3; SR_write ( 0x00, false, false );
  _regs.SR.index= 4; SR_write ( 0x00, false, false );
  _regs.SR.index= 6; SR_write ( 0x0f, false, false );
  _regs.SR.index= 7; SR_write ( 0x00, false, false );
  _regs.SR.index= 8; SR_write ( 0x00, false, false );
  _regs.SR.index= 0xa; SR_write ( 0x00, false, false );
  _regs.SR.index= 0xb; SR_write ( 0x66, false, false );
  _regs.SR.index= 0xc; SR_write ( 0x5b, false, false );
  _regs.SR.index= 0xd; SR_write ( 0x45, false, false );
  _regs.SR.index= 0xe; SR_write ( 0x7e, false, false );
  _regs.SR.index= 0xf; SR_write ( 0x00, false, false );
  _regs.SR.index= 0x12; SR_write ( 0x00, false, false ); // Default 0x00 ???
  _regs.SR.index= 0x13; SR_write ( 0x00, false, false ); // Default 0x00 ???
  _regs.SR.index= 0x1b; SR_write ( 0x3b, false, false );
  _regs.SR.index= 0x1c; SR_write ( 0x2f, false, false );
  _regs.SR.index= 0x1d; SR_write ( 0x30, false, false );
  _regs.SR.index= 0x1e; SR_write ( 0x33, false, false );
  _regs.SR.index= 0;
  _regs.GR.index= 0x00; GR_write ( 0x00, false );
  _regs.GR.index= 0x01; GR_write ( 0x00, false );
  _regs.GR.index= 0x02; GR_write ( 0x00, false );
  _regs.GR.index= 0x03; GR_write ( 0x00, false );
  _regs.GR.index= 0x04; GR_write ( 0x00, false );
  _regs.GR.index= 0x05; GR_write ( 0x00, false );
  _regs.GR.index= 0x06; GR_write ( 0x00, false );
  _regs.GR.index= 0x07; GR_write ( 0x00, false );
  _regs.GR.index= 0x08; GR_write ( 0x00, false );
  _regs.GR.index= 0x09; GR_write ( 0x00, false );
  _regs.GR.index= 0x0a; GR_write ( 0x00, false );
  _regs.GR.index= 0x0b; GR_write ( 0x00, false );
  _regs.GR.index= 0x31; GR_write ( 0x00, false );
  _regs.GR.index= 0;
  _regs.CR.index= 0x11; CR_write ( 0x00, false ); // IMPORTA L'ORDRE
  _regs.CR.index= 0x00; CR_write ( 0x00, false );
  _regs.CR.index= 0x01; CR_write ( 0x00, false );
  _regs.CR.index= 0x02; CR_write ( 0x00, false );
  _regs.CR.index= 0x03; CR_write ( 0x00, false );
  _regs.CR.index= 0x04; CR_write ( 0x00, false );
  _regs.CR.index= 0x05; CR_write ( 0x00, false );
  _regs.CR.index= 0x06; CR_write ( 0x00, false );
  _regs.CR.index= 0x07; CR_write ( 0x00, false );
  _regs.CR.index= 0x08; CR_write ( 0x00, false );
  _regs.CR.index= 0x09; CR_write ( 0x00, false );
  _regs.CR.index= 0x0a; CR_write ( 0x00, false );
  _regs.CR.index= 0x0b; CR_write ( 0x00, false );
  _regs.CR.index= 0x0c; CR_write ( 0x00, false );
  _regs.CR.index= 0x0d; CR_write ( 0x00, false );
  _regs.CR.index= 0x0e; CR_write ( 0x00, false );
  _regs.CR.index= 0x0f; CR_write ( 0x00, false );
  _regs.CR.index= 0x10; CR_write ( 0x00, false );
  _regs.CR.index= 0x12; CR_write ( 0x00, false );
  _regs.CR.index= 0x13; CR_write ( 0x00, false );
  _regs.CR.index= 0x14; CR_write ( 0x00, false );
  _regs.CR.index= 0x15; CR_write ( 0x00, false );
  _regs.CR.index= 0x16; CR_write ( 0x00, false );
  _regs.CR.index= 0x17; CR_write ( 0x00, false );
  _regs.CR.index= 0x18; CR_write ( 0x00, false );
  _regs.CR.index= 0x1a; CR_write ( 0x00, false );
  _regs.CR.index= 0x1b; CR_write ( 0x00, false );
  _regs.CR.index= 0x1d; CR_write ( 0x00, false );
  _regs.CR.index= 0x38; CR_write ( 0x00, false );
  _regs.CR.index= 0x3e; CR_write ( 0x04, false );
  _regs.CR.index= 0;
  _regs.AR.mode_data= true; _regs.AR.index= 0x10; AR_write ( 0x00, false );
  _regs.AR.mode_data= true; _regs.AR.index= 0x12; AR_write ( 0x00, false );
  _regs.AR.mode_data= true; _regs.AR.index= 0x13; AR_write ( 0x00, false );
  _regs.AR.mode_data= true; _regs.AR.index= 0x14; AR_write ( 0x00, false );
  _regs.AR.mode_data= true;
  _regs.AR.index= 0;
  _regs.AR.display_enabled= false;
  memset ( _regs.AR.pal, 0, sizeof(_regs.AR.pal) );
  _regs.AR.overscan_color= 0;

  // Altres.
  init_vga_mem (); // L'ordre importa (després de GR)
  
} // end init_regs


static void
update_vclk (void)
{

  // INCLOU LA MULT PER 10K QUE FAIG
  static const double REFERENCE= 14.31818*10000.0;

  long old_mul;
  uint8_t den;
  double tmp;
  
  
  // És impossible fer la conversió completa dels clocks en tmp, però
  // farem el millor possible.
  old_mul= _timing.cc_mul;
  assert ( old_mul > 0 );

  // Actualitza cc_mul
  den= _regs.SR.vclk[_regs.misc.vlck_freq_ind].den;
  if ( den <= 1 )
    {
      _timing.cc_mul= 251800;
      _warning ( _udata,
                 "[VGA] update_vclk - el denominador és 0, "
                 "CC_MUL fixat a 251800" );
    }
  else
    {
      tmp=
        REFERENCE *
        ((double) (_regs.SR.vclk[_regs.misc.vlck_freq_ind].num&0x7f));
      tmp/= (den&0x1)!=0 ? (((double) (den>>1))*2.0) : ((double) (den>>1));
      _timing.cc_mul= (long) tmp;
      if ( _timing.cc_mul < 100 )
        {
          _timing.cc_mul= 251800;
          _warning ( _udata,
                     "[VGA] update_vclk - el valor de CC_MUL és molt menut, "
                     "CC_MUL fixat a 251800" );
        }
    }
  /* // HO DEIXE COM A REFERÈNCIA (AÇÒ SERIA VGA BASIC)
  switch ( _regs.misc.vlck_freq_ind )
    {
    case 0: // 25.180 MHz
      _timing.cc_mul= 251800;
      break;
    case 1: // 28.325 MHz
      _timing.cc_mul= 283250;
      break;
    case 2: // 41.165 MHz
      _timing.cc_mul= 411650;
      break;
    case 3: // 36.082 MHz
      _timing.cc_mul= 360820;
      break;
    default: // WTF
      fprintf ( stderr, "update_vclk - WTF !!! \n" );
      exit ( EXIT_FAILURE );
      break;
    }
  */
  if ( _regs.SR.clocking_mode.dot_clock_div2 )
    _timing.cc_mul/= 2;
  
  // Acaba de fer la conversió de vcc_tmp
  // NOTA!! No és perfecta, però més no es pot fer.
  _timing.vcc_tmp= (_timing.vcc_tmp*_timing.cc_mul)/old_mul;
  
} // end update_vclk


static int
calc_cc_to_update_screen (void)
{
  
  int dotsperchar,end_scanline,dots,tmp,cline,end_vdisplay,last_scanline,ret;
  long tmpl;
  

  // Preparació.
  dotsperchar= _regs.SR.clocking_mode.dot_clock_8_9 ? 8 : 9;
  end_scanline= ((int) _regs.CR.horizontal_total) + 5;
  end_vdisplay= ((int) (_regs.CR.overflow.vertical_display_end |
                        _regs.CR.vertical_display_end)) + 1;
  last_scanline= ((int) (_regs.CR.overflow.vertical_total |
                         _regs.CR.vertical_total)) + 2;
  if ( _regs.CR.mode.vregs_by_two )
    {
      end_vdisplay*= 2;
      last_scanline*= 2;
    }
  
  // Dots per a acabar scanline actual.
  tmp= end_scanline <= _render.H ? 1 : (end_scanline-_render.H);
  dots= tmp*dotsperchar;
  
  // Línies que falten per a aplegar a l'update.
  cline= _render.scanline+1; // La línia anterior ha acabat.
  if ( cline > last_scanline ) // Cas extrem !!!
    {
      last_scanline= 2048;
      if ( cline > last_scanline ) cline= last_scanline;
    }
  if ( cline > end_vdisplay )
    {
      tmp= (last_scanline-cline) + end_vdisplay;
    }
  else tmp= end_vdisplay-cline;
  dots+= tmp*end_scanline*dotsperchar;

  // Resta dots ja processats.
  dots-= _render.char_dots;

  // Transforma a cicles cpu.
  tmpl= ((long) dots)*_timing.cc_div;
  ret= (int) (tmpl / _timing.cc_mul) + ((tmpl%_timing.cc_mul)!=0);
  
  return ret;
  
} // end calc_cc_to_update_screen


static void
update_cc_to_event (void)
{

  int cc,tmp;

  
  // Per defecte 1s
  _timing.cctoEvent= PC_ClockFreq;
  // --> Update screen
  tmp= calc_cc_to_update_screen ();
  assert ( tmp > 0 );
  if ( tmp < _timing.cctoEvent ) _timing.cctoEvent= tmp;
  
  // Actualitza PC_NextEventCC
  cc= next_event_cc ();
  cc+= PC_Clock; // Medim sempre des de que PC_Clock és 0
  if ( cc < PC_NextEventCC )
    PC_NextEventCC= cc;
  
} // end update_cc_to_event


static void
render_chars_black (
                    const int chars,
                    const int dotsperchar
                    )
{

  int begin,end,i;

  
  begin= _render.scanline*FB_WIDTH + _render.H*dotsperchar;
  end= begin + chars*dotsperchar;
  for ( i= begin; i < end; ++i )
    {
      _render.fb[i].r= 0;
      _render.fb[i].g= 0;
      _render.fb[i].b= 0;
    }
  
} // end render_chars_black


static void
get_color_dac (
               const uint8_t  index,
               uint8_t       *r,
               uint8_t       *g,
               uint8_t       *b
               )
{

  // No és ùna conversió de color perfecta, però és molt més eficient
  // que multiplicar o consultar una taula. Si vull que siga perfecte taula
  
  uint8_t tmp;
  

  // NOTA!! No entenc si el pixel_mask es gasta ací.
  tmp= _dac.v[index][0]&_regs.pixel_mask;
  *r= (tmp<<2);
  tmp= _dac.v[index][1]&_regs.pixel_mask;
  *g= (tmp<<2);
  tmp= _dac.v[index][2]&_regs.pixel_mask;
  *b= (tmp<<2);
  
  _render.pixel_bus= (uint8_t) index;
  
} // end get_color_dac


static void
get_color_palette (
                   const uint8_t  index,
                   uint8_t       *r,
                   uint8_t       *g,
                   uint8_t       *b
                   )
{
  get_color_dac ( _regs.AR.pal[index&0xF], r, g, b );
} // end get_color_palette


static void
render_chars_overscan (
                       const int chars,
                       const int dotsperchar
                       )
{
  
  int begin,end,i;
  uint8_t r,g,b;


  // NOTA!! No entenc el significat de Secondary Red, Secondary Green,
  // etc. Simplement ho vaig a emprar com un índex en la entrada de
  // colors del DAC.
  get_color_dac ( _regs.AR.overscan_color, &r, &g, &b );
  begin= _render.scanline*FB_WIDTH + _render.H*dotsperchar;
  end= begin + chars*dotsperchar;
  for ( i= begin; i < end; ++i )
    {
      _render.fb[i].r= r;
      _render.fb[i].g= g;
      _render.fb[i].b= b;
    }
  
} // end render_chars_overscan


static int
render_addr2pos (
                 const int addr,
                 const int scanline
                 )
{

  int ret;

  
  if ( _regs.CR.underline_scanline.double_word_mode )
    {
      /* // CAL REVISITAR AÇÒ, HO DEIXE COMENTAT!
      if ( _regs.CR.ext_disp_ctrl.ext_addr_wrap_enabled )
        ret= ((addr<<2)|((addr>>19)&0x3))&0x3FFFFF;
        else*/ if ( _regs.CR.mode.addr_wrap )
        ret= (addr<<2)|((addr>>14)&0x3);
      else
        ret= (addr<<2)|((addr>>12)&0x3);
    }
  else if ( !_regs.CR.mode.byte_word_mode ) // word mode
    {
      /*
      if ( _regs.CR.ext_disp_ctrl.ext_addr_wrap_enabled )
        {
          printf("render_addr2pos - TODO A !!!!\n");
          exit(EXIT_FAILURE);
        }
        else*/ if ( _regs.CR.mode.addr_wrap )
        ret= (addr<<1)|((addr>>15)&0x1);
      else
        ret= (addr<<1)|((addr>>13)&0x1);
    }
  else ret= addr;

  // CGA
  if ( _regs.CR.mode.compatibility_cga_mode )
    ret= (ret&0xDFFF)|((scanline&0x1)<<13);

  return /*_regs.CR.ext_disp_ctrl.ext_addr_wrap_enabled ?
           ret :*/
    ret&PLANE_MASK;
  
} // end render_addr2pos


static void
render_apply_panning (void)
{

  static const int PANNING_MODE13[16]=
    { 0, 0, 1, 1, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3 };
  static const int PANNING_9BIT[16]=
    { 1, 2, 3, 4, 5, 6, 7, 8, 0, 0, 0, 0, 0, 0, 0, 0 };
  static const int PANNING_8BIT[16]=
    { 0, 1, 2, 3, 4, 5, 6, 7, 1, 1, 1, 1, 1, 1, 1, 1 };
  
  int panning,i;
  PC_RGB *line;
  

  // Obté panning
  if ( _regs.GR.misc.apa_mode && _regs.GR.mode.color256 )
    panning= PANNING_MODE13[_regs.AR.pixel_panning];
  else if ( _regs.SR.clocking_mode.dot_clock_8_9 )
    panning= PANNING_8BIT[_regs.AR.pixel_panning];
  else
    panning= PANNING_9BIT[_regs.AR.pixel_panning];

  // Aplica
  if ( panning > 0 )
    {
      line= &_render.fb[_render.scanline*FB_WIDTH];
      for ( i= 0; i < FB_WIDTH-panning; ++i )
        line[i]= line[i+panning];
    }
  
} // end render_apply_panning


static void
render_chars_text (
                   const int chars,
                   const int dotsperchar
                   )
{

  static const int PLANE2_OFFSETS[]= {
    0, 16*1024, 32*1024, 48*1024,
    8*1024, 24*1024, 40*1024, 56*1024
  };
  
  int height,charsperline,off,i,pos,char_y,j,p2_offset,cursor_pos,
    scanline_src;
  uint8_t index,attr,r_bg,g_bg,b_bg,r_fg,g_fg,b_fg,pattern;
  PC_RGB *p;
  

  // NOTA!!! Pixel double clock pot estar per un temps actiu abans
  // d'activat el mode color 256. Pixel double clock té sentit per ell
  // mateix però la implementació és un poc estranya i farragosa, per
  // tant he decidit ignorar aquest valor quan estem en mode text.
  //assert ( !_regs.AR.attr_ctrl_mode.pixel_double_clock );
  
  height= ((int) _regs.CR.char_cell_height.char_cell_height) + 1;
  scanline_src=
    _regs.CR.char_cell_height.scan_double ?
    _render.scanline>>1 : _render.scanline;
  // IMPORTANT!!!!! El pitch sense aplicar la traducció d'adreça és
  // sempre OFFSET*2. El tema de multiplicar per l'amplària de la
  // paraula fa referència a adreces ja transformades.
  charsperline= ((int) (_regs.CR.ext_disp_ctrl.offset_overflow |
                        (uint16_t) _regs.CR.offset))<<1;
  off= _render.start_addr + (scanline_src/height)*charsperline + _render.H;
  char_y= scanline_src%height;
  p= &_render.fb[_render.scanline*FB_WIDTH + _render.H*dotsperchar];
  for ( i= 0; i < chars; ++i )
    {

      // NOTA!! Aparentment en text mode s'accedeixen a posicions
      // parelles.  O millor dit, l'adreça és multiplica per 2 (no sé
      // per què), açò també afecta a l'adreça del cursor.
      // Obté índex/atribut
      pos= render_addr2pos ( off+i, scanline_src );
      index= _vga_mem.p[0][pos];
      attr= _vga_mem.p[1][pos];

      // Obté colors
      if ( _regs.AR.attr_ctrl_mode.blink_enabled )
        {
          get_color_palette ( (attr>>4)&0x7, &r_bg, &g_bg, &b_bg );
          if ( (attr&0x80)!=0 && _render.blink )
            { r_fg= r_bg; g_fg= g_bg; b_fg= b_bg; }
          else
            get_color_palette ( attr&0xF, &r_fg, &g_fg, &b_fg );
        }
      else
        {
          get_color_palette ( attr>>4, &r_bg, &g_bg, &b_bg );
          get_color_palette ( attr&0xF, &r_fg, &g_fg, &b_fg );
        }

      // Prepara informació cursor
      if ( !_regs.CR.text_cursor_start.text_cursor_disabled )
        {
          cursor_pos=
            ((int)
             ((((uint32_t) _regs.CR.text_cursor_locH)) |
              ((uint32_t) _regs.CR.text_cursor_locL))) +
            ((int) (_regs.CR.text_cursor_end.text_cursor_skew))*2; // odd/even
          cursor_pos= render_addr2pos ( cursor_pos, scanline_src );
        }
      else cursor_pos= 0;
      
      // Pinta cursor si cal
      if ( !_regs.CR.text_cursor_start.text_cursor_disabled &&
           !_render.blink &&
           cursor_pos == pos &&
           _regs.CR.text_cursor_start.text_cursor_start <= char_y &&
           char_y <= _regs.CR.text_cursor_end.text_cursor_end )
        {
          for ( j= 0; j < dotsperchar; ++j, ++p )
            { p->r= r_fg; p->g= g_fg; p->b= b_fg; }
        }
      // Pinta subrallat si és el cas
      else if ( (attr&0x77) == 0x01 &&
           ((int) _regs.CR.underline_scanline.underline_scanline) == char_y )
        {
          for ( j= 0; j < dotsperchar; ++j, ++p )
            { p->r= r_fg; p->g= g_fg; p->b= b_fg; }
        }
      // Pinta contingut
      else
        {

          // Obté patró
          p2_offset= PLANE2_OFFSETS[((attr&0x08)!=0) ?
                                    _regs.SR.char_map.secondary_map :
                                    _regs.SR.char_map.primary_map];
          p2_offset+= ((int) index)*32 + char_y;
          pattern= _vga_mem.p[2][p2_offset];
            
          // Dibuixa
          // --> Patró (8 bits)
          for ( j= 0; j < 8; ++j, ++p )
            {
              if ( (pattern&0x80) != 0 ) { p->r= r_fg; p->g= g_fg; p->b= b_fg; }
              else                       { p->r= r_bg; p->g= g_bg; p->b= b_bg; }
              pattern<<= 1;
            }
          // --> Bit 9 si cal
          if ( dotsperchar == 9 )
            {
              if ( _regs.AR.attr_ctrl_mode.line_graphics_enabled &&
                   index >= 0xc0 && index <= 0xdf )
                *p= *(p-1);
              else { p->r= r_bg; p->g= g_bg; p->b= b_bg; }
              ++p;
            }
          
        }
      
    }
  
} // end render_chars_text


static void
render_chars_planar (
                     const int chars,
                     const int dotsperchar
                     )
{
  
  int bytesperline,off,i,pos,j,tmp,bytes_H,off_H,height,
    scanline_src;
  uint8_t b0,b1,b2,b3,color;
  PC_RGB *p;
  
  
  // Comprovacions
  /* // DONA PER SAC REVISAR????
  if ( !_regs.AR.attr_ctrl_mode.pixel_double_clock )
    _warning ( _udata, "render_chars_planar - renderitzant VGA"
               " sense Pixel Double Clock" );
  */
  
  // Obté offsets i valors
  scanline_src=
    _regs.CR.char_cell_height.scan_double ?
    _render.scanline>>1 : _render.scanline;
  height= ((int) _regs.CR.char_cell_height.char_cell_height) + 1;
  // IMPORTANT!!!!! El pitch sense aplicar la traducció d'adreça és
  // sempre OFFSET*2. El tema de multiplicar per l'amplària de la
  // paraula fa referència a adreces ja transformades.
  bytesperline= ((int) (_regs.CR.ext_disp_ctrl.offset_overflow |
                        (uint16_t) _regs.CR.offset))<<1;
  tmp= _render.H*dotsperchar;
  bytes_H= tmp>>3; off_H= tmp&0x7;
  off= _render.start_addr + (scanline_src/height)*bytesperline + bytes_H;
  p= &_render.fb[_render.scanline*FB_WIDTH + _render.H*dotsperchar];
  
  // Dibuixa
  // --> Llig primer byte.
  pos= render_addr2pos ( off, scanline_src );
  b0= _vga_mem.p[0][pos]<<off_H;
  b1= _vga_mem.p[1][pos]<<off_H;
  b2= _vga_mem.p[2][pos]<<off_H;
  b3= _vga_mem.p[3][pos]<<off_H;
  // --> Itera
  for ( i= 0; i < chars; ++i )
    for ( j= 0; j < dotsperchar; ++j, ++p )
      {

        // Pinta píxel
        color= (b0>>7)|((b1&0x80)>>6)|((b2&0x80)>>5)|((b3&0x80)>>4);
        color&= _regs.AR.color_plane.enable;
        get_color_palette ( color, &(p->r), &(p->g), &(p->b) );
        
        // Següent pixel
        if ( ++off_H == 8 )
          {
            pos= render_addr2pos ( ++off, scanline_src );
            b0= _vga_mem.p[0][pos];
            b1= _vga_mem.p[1][pos];
            b2= _vga_mem.p[2][pos];
            b3= _vga_mem.p[3][pos];
            off_H= 0;
          }
        else { b0<<= 1; b1<<= 1; b2<<= 1; b3<<= 1; }
        
      }
  
} // end render_chars_planar


static void
render_chars_packed (
                     const int chars,
                     const int dotsperchar
                     )
{
  
  int bytesperline,off,i,pos,j,tmp,bytes_H,off_H,height,
    scanline_src;
  uint8_t b0,b2,color,plane_enable_02,plane_enable_13,plane_enable;
  bool pair_02;
  PC_RGB *p;
  
  
  // Comprovacions
  assert ( !_regs.AR.attr_ctrl_mode.pixel_double_clock );
  
  // Obté offsets i valors
  scanline_src=
    _regs.CR.char_cell_height.scan_double ?
    _render.scanline>>1 : _render.scanline;
  height= ((int) _regs.CR.char_cell_height.char_cell_height) + 1;
  // IMPORTANT!!!!! El pitch sense aplicar la traducció d'adreça és
  // sempre OFFSET*2. El tema de multiplicar per l'amplària de la
  // paraula fa referència a adreces ja transformades.
  bytesperline= ((int) (_regs.CR.ext_disp_ctrl.offset_overflow |
                        (uint16_t) _regs.CR.offset))<<1;
  tmp= _render.H*dotsperchar;
  bytes_H= tmp>>3; off_H= tmp&0x7;
  if ( off_H >= 4 ) { off_H-= 4; pair_02= false; }
  else              pair_02= true;
  off= _render.start_addr + (scanline_src/height)*bytesperline + bytes_H;
  p= &_render.fb[_render.scanline*FB_WIDTH + _render.H*dotsperchar];
  // --> Plane enable
  plane_enable_02=
    ((_regs.AR.color_plane.enable&0x1)!=0 ? 0x3 : 0x0) |
    ((_regs.AR.color_plane.enable&0x4)!=0 ? 0xC : 0x0);
  plane_enable_13=
    ((_regs.AR.color_plane.enable&0x2)!=0 ? 0x3 : 0x0) |
    ((_regs.AR.color_plane.enable&0x8)!=0 ? 0xC : 0x0);
  plane_enable= pair_02 ? plane_enable_02 : plane_enable_13;
  
  // Dibuixa
  // --> Llig primer byte.
  pos= render_addr2pos ( off, scanline_src );
  b0= _vga_mem.p[pair_02?0:1][pos]<<(2*off_H);
  b2= _vga_mem.p[pair_02?2:3][pos]<<(2*off_H);
  
  // --> Itera
  for ( i= 0; i < chars; ++i )
    for ( j= 0; j < dotsperchar; ++j, ++p )
      {

        // Pinta píxel
        color= (b0>>6)|((b2&0xC0)>>4);
        color&= plane_enable;
        get_color_palette ( color, &(p->r), &(p->g), &(p->b) );
        
        // Següent pixel
        if ( ++off_H == 4 )
          {
            if ( pair_02 )
              {
                pair_02= false;
                plane_enable= plane_enable_13;
                b0= _vga_mem.p[1][pos];
                b2= _vga_mem.p[3][pos];
              }
            else
              {
                pair_02= true;
                plane_enable= plane_enable_02;
                pos= render_addr2pos ( ++off, scanline_src );
                b0= _vga_mem.p[0][pos];
                b2= _vga_mem.p[2][pos];
              }
            off_H= 0;
          }
        else { b0<<= 2; b2<<= 2; }
        
      }
  
} // end render_chars_packed


static void
render_chars_256color (
                       const int chars,
                       const int dotsperchar
                       )
{
  
  int bytesperline,off,i,pos,j,tmp,bytes_H,off_H,height,
    scanline_src;
  uint8_t plane,color;
  bool even;
  PC_RGB *p;
  
  
  // Comprovacions
  if ( !_regs.AR.attr_ctrl_mode.pixel_double_clock )
    _warning ( _udata, "render_chars_256color - renderitzant VGA"
               " sense Pixel Double Clock" );
  
  // Obté offsets i valors
  scanline_src=
    _regs.CR.char_cell_height.scan_double ?
    _render.scanline>>1 : _render.scanline;
  height= ((int) _regs.CR.char_cell_height.char_cell_height) + 1;
  // IMPORTANT!!!!! El pitch sense aplicar la traducció d'adreça és
  // sempre OFFSET*2. El tema de multiplicar per l'amplària de la
  // paraula fa referència a adreces ja transformades.
  bytesperline= ((int) (_regs.CR.ext_disp_ctrl.offset_overflow |
                        (uint16_t) _regs.CR.offset))<<1;
  tmp= _render.H*dotsperchar;
  // 4 pixels per byte, però, com va a double clock. 8 pixels falsos
  // per byte.
  bytes_H= tmp>>3; off_H= tmp&0x7;
  even= (off_H&0x1)==0;
  plane= off_H>>1;
  off= _render.start_addr + (scanline_src/height)*bytesperline + bytes_H;
  p= &_render.fb[_render.scanline*FB_WIDTH + ((_render.H*dotsperchar)>>1)];
  // NOTA!!! Pixel Double Clock Select --> Palette registers, AR0–ARF,
  // are bypassed. Per tant, deduisc que el plane enable ací no té efecte.
  
  // Dibuixa
  // --> Inicialitza pos
  pos= render_addr2pos ( off, scanline_src );
  // --> Itera
  for ( i= 0; i < chars; ++i )
    for ( j= 0; j < dotsperchar; ++j )
      if ( !even ) // odd
        {
          
          // Pinta píxel
          color= _vga_mem.p[plane][pos];
          get_color_dac ( color, &(p->r), &(p->g), &(p->b) );
          ++p;
          
          // Següent pixel
          if ( ++plane == 4 )
            {
              pos= render_addr2pos ( ++off, scanline_src );
              plane= 0;
            }
          even= true;
          
        }
      else even= false;
  
} // end render_chars_256color


static void
render_chars_vga_lut (
                      const int chars,
                      const int dotsperchar
                      )
{
  
  int bytesperline,off,i,j,tmp,height,scanline_src;
  uint8_t color;
  PC_RGB *p;
  
  
  // Obté offsets i valors
  scanline_src=
    _regs.CR.char_cell_height.scan_double ?
    _render.scanline>>1 : _render.scanline;
  height= ((int) _regs.CR.char_cell_height.char_cell_height) + 1;
  bytesperline= ((int) (_regs.CR.ext_disp_ctrl.offset_overflow |
                        (uint16_t) _regs.CR.offset))<<3;
  tmp= _render.H*dotsperchar;
  
  // 1 pixel per byte
  off= _render.start_addr + (scanline_src/height)*bytesperline + tmp;
  p= &_render.fb[_render.scanline*FB_WIDTH + tmp];
  
  // Dibuixa
  for ( i= 0; i < chars; ++i )
    for ( j= 0; j < dotsperchar; ++j )
      {
        color= _vram[off++];
        get_color_dac ( color, &(p->r), &(p->g), &(p->b) );
        ++p;
      }
  
} // end render_chars_vga_lut


static void
render_chars_rgb565 (
                     const int chars,
                     const int dotsperchar
                     )
{
  
  int bytesperline,off,i,j,tmp,height,scanline_src;
  uint16_t color;
  PC_RGB *p;
  
  
  // Obté offsets i valors
  scanline_src=
    _regs.CR.char_cell_height.scan_double ?
    _render.scanline>>1 : _render.scanline;
  height= ((int) _regs.CR.char_cell_height.char_cell_height) + 1;
  bytesperline= ((int) (_regs.CR.ext_disp_ctrl.offset_overflow |
                        (uint16_t) _regs.CR.offset))<<3;
  tmp= _render.H*dotsperchar;
  
  // 1 pixel cada 2 bytes (1 uint16_t)
  off= _render.start_addr + (scanline_src/height)*(bytesperline/2) + tmp;
  p= &_render.fb[_render.scanline*FB_WIDTH + tmp];
  
  // Dibuixa
  for ( i= 0; i < chars; ++i )
    for ( j= 0; j < dotsperchar; ++j )
      {
        color= ((const uint16_t *) _vram)[off++];
        p->r= ((color>>11)&0x1F)<<3;
        p->g= ((color>>5)&0x3F)<<2;
        p->b= (color&0x1F)<<3;
        ++p;
      }
  
} // end render_chars_rgb565


static void
render_chars_rgb555 (
                     const int chars,
                     const int dotsperchar
                     )
{
  
  int bytesperline,off,i,j,tmp,height,scanline_src;
  uint16_t color;
  PC_RGB *p;
  
  
  // Obté offsets i valors
  scanline_src=
    _regs.CR.char_cell_height.scan_double ?
    _render.scanline>>1 : _render.scanline;
  height= ((int) _regs.CR.char_cell_height.char_cell_height) + 1;
  bytesperline= ((int) (_regs.CR.ext_disp_ctrl.offset_overflow |
                        (uint16_t) _regs.CR.offset))<<3;
  tmp= _render.H*dotsperchar;
  
  // 1 pixel cada 2 bytes (1 uint16_t)
  off= _render.start_addr + (scanline_src/height)*(bytesperline/2) + tmp;
  p= &_render.fb[_render.scanline*FB_WIDTH + tmp];
  
  // Dibuixa
  for ( i= 0; i < chars; ++i )
    for ( j= 0; j < dotsperchar; ++j )
      {
        color= ((const uint16_t *) _vram)[off++];
        if ( _regs.hdr.control_32k_color_enabled && (color&0x8000) != 0 )
          get_color_dac ( (uint8_t) (color&0xFF) , &(p->r), &(p->g), &(p->b) );
        else
          {
            p->r= ((color>>10)&0x1F)<<3;
            p->g= ((color>>5)&0x1F)<<3;
            p->b= (color&0x1F)<<3;
          }
        ++p;
      }
  
} // end render_chars_rgb555


static void
render_chars_extended_modes (
                             const int chars,
                             const int dotsperchar
                             )
{

  if ( _regs.hdr.mode_555_enabled )
    {
      if ( _regs.hdr.all_ext_modes_enabled )
        {
          if ( _regs.SR.r7.srt == 0x03 && _regs.hdr.clocking_mode_is_1 )
            {
              switch ( _regs.hdr.ext_mode )
                {
                case 0: render_chars_rgb555 ( chars, dotsperchar ); break;
                case 1: render_chars_rgb565 ( chars, dotsperchar ); break;
                default: goto todo;
                }
            }
          else goto todo;
        }
      else
        goto todo;
    }
  else if ( _regs.hdr.all_ext_modes_enabled )
    goto todo;
  else
    {
      if ( _regs.SR.r7.srt == 0x00 && !_regs.hdr.clocking_mode_is_1 )
        render_chars_vga_lut ( chars, dotsperchar );
      else goto todo;
    }

  // Pot sobreescriure part de la línia.
  // IMPORTANT!!! En realitat si s'activa, té efecte en el VSYNC. Quan
  // s'active ja ho canviaré ficant un camp en _render.
  if ( _regs.CR.vid_win_master_ctrl.video_window_master_enabled )
    {
      // --> El format per al window
      // _regs.CR.vid_win_master_ctrl.video_display_format
      
      PC_MSG("SVGA - Video window");
      exit ( EXIT_FAILURE );
    }
    
  return;
  
 todo:
  PC_MSGF("SVGA - render_chars_extended_modes "
          "(Sequencer and CRTC Clocking Control:%X,HDR:%X,Video"
          " Display Format:%X)",
          _regs.SR.r7.srt,_regs.hdr.val,
          _regs.CR.vid_win_master_ctrl.video_display_format);
  
} // end render_chars_extended_modes


static void
render_chars (
              const int chars,
              const int dotsperchar
              )
{

  int end_vdisplay,end_hdisplay;

  
  if ( chars == 0 ) return;

  // Cada vegada que s'intenta renderitzar algo es fica el pixel_bus a
  // 0 i si finalment es renderitza alguna cosa es modifica. Açò en
  // realitat no és molt important.
  _render.pixel_bus= 0x00;
  
  // NOTA!! No vaig a pintar el overscan però ja que hi ha color ho
  // pinte.
  end_vdisplay= ((int) (_regs.CR.overflow.vertical_display_end |
                        _regs.CR.vertical_display_end)) + 1;
  end_hdisplay= ((int) _regs.CR.horizontal_display_end) + 1;
  if ( !_regs.AR.display_enabled )
    render_chars_overscan ( chars, dotsperchar );
  else if ( _render.in_vblank || _render.in_vretrace )
    {
      // Renderitze de negre si per algun motiu està en la regió de la
      // pantalla visible.
      if ( _render.V < end_vdisplay ) render_chars_black ( chars, dotsperchar );
    }
  else if ( _render.V >= end_vdisplay )
    render_chars_overscan ( chars, dotsperchar );
  else if ( _render.in_hblank || _render.in_hretrace )
    {
      // Renderitze de negre si per algun motiu està en la regió de la
      // pantalla visible.
      if ( _render.H < end_hdisplay ) render_chars_black ( chars, dotsperchar );
    }
  else if ( _render.H >= end_hdisplay )
    render_chars_overscan ( chars, dotsperchar );
  else if ( !_regs.GR.misc.apa_mode )
    render_chars_text ( chars, dotsperchar );
  else if ( !_regs.GR.mode.color256 )
    {
      if ( !_regs.GR.mode.shift_reg_mode_is_1 )
        render_chars_planar ( chars, dotsperchar );
      else
        render_chars_packed ( chars, dotsperchar );
    }
  else
    {
      if ( !_regs.SR.r7.extended_display_modes_enabled )
        render_chars_256color ( chars, dotsperchar );
      else
        render_chars_extended_modes ( chars, dotsperchar );
    }
    
} // end render_chars


// Torna cicles (dot clocks) pendents
static int
run_render__ (
              int       dots,
              const int dotsperchar
              )
{
  
  int end_scanline,next_event_H,end_display,next_event_hblank,
    next_event_hretrace,available_dots,required_dots,ret,new_H,tmp,
    width,height;
  uint8_t tmp8;
  
  
  // Calcula següent event horizontal (next_event_H)
  // NOTA!! end_h vol dir que apleguem a que _render.H= next_event_H però el
  // comptador de dots està a 0, és a dir, estem al principi del
  // caràcter next_event_H.
  // --> scanline end
  end_scanline= ((int) _regs.CR.horizontal_total) + 5;
  // Açò és per si modifiquen paràmetres a meitat renderitzat.
  if ( end_scanline <= _render.H ) end_scanline= _render.H+1;
  next_event_H= end_scanline;
  // --> display end
  end_display= ((int) _regs.CR.horizontal_display_end) + 1;
  if ( end_display < next_event_H && end_display > _render.H )
    next_event_H= end_display;
  // --> horizontal blanking
  if ( _render.in_hblank )
    {
      tmp8= // 6 bits
        _regs.CR.horizontal_blanking_end.horizontal_blanking_end |
        _regs.CR.horizontal_sync_end.horizontal_blanking_end;
      next_event_hblank= (int) ((_render.H&(~0x3F))|tmp8);
      if ( (_render.H&0x3F) >= tmp8 ) next_event_hblank+= 0x40;
      // NOTA!! No queda clar si pot passar de línia o no. El manual
      // de la targeta diu que això no es deuria fer (però no diu què
      // ocorre), per una altra banda
      // https://web.stanford.edu/class/cs140/projects/pintos/specs/freevga/vga/crtcreg.htm#03
      // diu que sí pot passar. Mirant el funcionament de GODS diria
      // que volien ficar que el HBLANK acabara en 0x37 (el total és
      // 0x3C) però interpretaren mal com es programava i ficaren
      // 0x17, forçant que acavara en 0x57. El resultat és que passa
      // de línia i sols mostra mitja pantalla. Així que...
      if ( next_event_hblank > end_scanline ) next_event_hblank= end_scanline;
    }
  else next_event_hblank= (int) _regs.CR.horizontal_blanking_start;
  if ( next_event_hblank < next_event_H && next_event_hblank > _render.H )
    next_event_H= next_event_hblank;
  // --> horizontal retracing
  if ( _render.in_hretrace )
    {
      tmp8= // 5 bits
        _regs.CR.horizontal_sync_end.horizontal_sync_end;
      next_event_hretrace= (int) ((_render.H&(~0x1F))|tmp8);
      if ( (_render.H&0x1F) >= tmp8 ) next_event_hretrace+= 0x20;
      // Repetisc jugada
      if ( next_event_hretrace > end_scanline )
        next_event_hretrace= end_scanline;
    }
  else
    next_event_hretrace=
      (int) (_regs.CR.horizontal_sync_start +
             _regs.CR.horizontal_sync_end.horizontal_sync_delay);
  if ( next_event_hretrace < next_event_H && next_event_hretrace > _render.H )
    next_event_H= next_event_hretrace;
  
  // Processa els ciles
  available_dots= _render.char_dots + dots;
  assert ( _render.H < next_event_H );
  required_dots= (next_event_H-_render.H)*dotsperchar;
  // --> No canvia l'estat
  if ( available_dots < required_dots )
    {
      _render.char_dots= available_dots%dotsperchar;
      new_H= _render.H + available_dots/dotsperchar;
      render_chars ( new_H-_render.H, dotsperchar );
      ret= 0;
    }
  else
    {

      // Mou al següent a la posició del següent event i prepara ret.
      _render.char_dots= 0;
      new_H= next_event_H;
      render_chars ( new_H-_render.H, dotsperchar );
      ret= available_dots-required_dots;

      // Actualitza estat.
      // --> hblanking
      if ( new_H == next_event_hblank )
        _render.in_hblank= !_render.in_hblank;
      // --> hretrace
      if ( new_H == next_event_hretrace )
        _render.in_hretrace= !_render.in_hretrace;
      // --> end scanline
      if ( new_H == end_scanline )
        {

          // Aplica panning
          if ( !_render.in_vblank && !_render.in_vretrace )
            render_apply_panning ();
          
          // Reseteja H.
          new_H= 0;
          
          // Mou Y
          if ( !_regs.CR.mode.vregs_by_two || _render.scanline%2==1 )
            ++_render.V;
          ++_render.scanline;

          // Actualitza estat timing vertical
          // --> vblank
          if ( _render.in_vblank )
            {
              /* // IMPLEMENTACIÓ ANTIGA!!!! COMPTE!!!
              tmp= (int) _regs.CR.vertical_blank_end;
              if ( _regs.CR.ext_disp_ctrl.blanking_control_is_1 ||
                   _regs.CR.ext_disp_ctrl.blank_end_extensions_enabled )
                {
                  tmp|= (int) _regs.CR.misc_ctrl.vblank_end;
                  if ( (_render.V&0x3FF) == tmp ) _render.in_vblank= false;
                }
              else if ( (_render.V&0xFF) == tmp ) _render.in_vblank= false;
              */
              // IMPLEMENTACIÓ ASSUMINT QUE EL END ÉS WIDTH!!!
              if ( _render.V == _render.vblank_end ) _render.in_vblank= false;
            }
          else
            {
              tmp=
                (int ) (_regs.CR.char_cell_height.vertical_blank_start |
                        _regs.CR.overflow.vertical_blanking_start |
                        _regs.CR.vertical_blank_start);
              /* // IMPLEMENTACIÓ ANTIGA!!! COMPTE!!!
              if ( _render.V == tmp ) _render.in_vblank= true;
              */
              // IMPLEMENTACIÓ ASSUMINT QUE EL END ÉS WIDTH!!!
              if ( _render.V == tmp )
                {
                  _render.in_vblank= true;
                  _render.vblank_end= tmp;
                  tmp= (int) _regs.CR.vertical_blank_end;
                  if ( _regs.CR.ext_disp_ctrl.blanking_control_is_1 ||
                       _regs.CR.ext_disp_ctrl.blank_end_extensions_enabled )
                    tmp|= (int) _regs.CR.misc_ctrl.vblank_end;
                  _render.vblank_end+= tmp;
                }
            }
          // --> vretrace
          if ( _render.in_vretrace )
            {
              if ( (_render.V&0xF) ==
                   _regs.CR.vertical_sync_end.vertical_sync_end )
                _render.in_vretrace= false;
            }
          else
            {
              tmp=
                (int) (_regs.CR.overflow.vertical_retrace_start |
                       _regs.CR.vertical_sync_start);
              if ( _render.V == tmp )
                {
                  _render.start_addr= (int)
                    (_regs.CR.ov_ext_ctrl.screen_start_a_addr |
                     _regs.CR.ext_disp_ctrl.screen_start_a_addr |
                     _regs.CR.ext_disp_ctrl.ext_disp_start_addr |
                     ((uint32_t) _regs.CR.screen_start_a_addrH) |
                     ((uint32_t) _regs.CR.screen_start_a_addrL));
                  _render.in_vretrace= true;
                }
            }
          // --> display end
          tmp=
            ((int) (_regs.CR.overflow.vertical_display_end |
                    _regs.CR.vertical_display_end)) + 1;
          if ( _render.V == tmp )
            {
              width= end_display*dotsperchar;
              // NOTA!!! Açò és una nyapa. No tinc clar què fer quan
              // els modes extended estan activats. Però en el FIFA96
              // no es veu bé si s'aplica açò amb el mode activat.
              if ( _regs.AR.attr_ctrl_mode.pixel_double_clock &&
                   !_regs.SR.r7.extended_display_modes_enabled )
                width/= 2;
              height= tmp;
              _update_screen ( _udata, _render.fb, width, height, FB_WIDTH );
            }
          // --> Última scanline
          tmp=
            ((int) (_regs.CR.overflow.vertical_total |
                    _regs.CR.vertical_total)) + 2;
          if ( _render.V == tmp || _render.scanline >= 2048 )
            {
              _render.V= 0;
              _render.scanline= 0;
              if ( ++_render.blink_counter == 16 )
                {
                  _render.blink_counter= 0;
                  _render.blink= !_render.blink;
                }
              // IMPLEMENTACIÓ NOVA!!!
              _render.in_vblank= false;
              _render.in_vretrace= false; // ???? 
            }
          
        }
      
    }
  
  // Actualitza H.
  _render.H= new_H;
  
  return ret;
  
} // end run_render__


static int
run_render (
            int dots
            )
{

  int dotsperchar,tmp,ret;


  dotsperchar= _regs.SR.clocking_mode.dot_clock_8_9 ? 8 : 9;
  tmp= _render.char_dots + dots;
  if ( tmp < dotsperchar ) // Cas ràpid
    {
      _render.char_dots= tmp;
      ret= 0;
    }
  else ret= run_render__ ( dots, dotsperchar );

  return ret;
  
} // end run_render


static void
clock (
         const bool update_cc2event
         )
{

  int cc,vcc;
  long tmp;
  
  
  // Processa cicles
  cc= PC_Clock-_timing.cc_used;
  if ( cc > 0 ) { _timing.cc+= cc; _timing.cc_used+= cc; }
  
  // Processa cicles
  tmp= ((long) _timing.cc)*_timing.cc_mul + _timing.vcc_tmp;
  vcc= (int) (tmp / _timing.cc_div);
  _timing.vcc_tmp= tmp % _timing.cc_div;
  _timing.cc= 0;
  while ( vcc > 0 )
    vcc= run_render ( vcc );
  
  // Actualitza cctoEvent
  if ( update_cc2event )
    update_cc_to_event ();

} // end clock




/**********************/
/* FUNCIONS PÚBLIQUES */
/**********************/

PC_Error
PC_svga_cirrus_clgd5446_init (
                              PC_Warning            *warning,
                              PC_UpdateScreen       *update_screen,
                              PC_VGAMemAccess       *vga_mem_access,
                              PC_VGAMemLinearAccess *vga_mem_linear_access,
                              uint8_t               *optrom,
                              const size_t           optrom_size,
                              void                  *udata
                              )
{
  
  _warning= warning;
  _update_screen= update_screen;
  _vga_mem_access= vga_mem_access;
  _vga_mem_linear_access= vga_mem_linear_access;
  _udata= udata;
  _trace_enabled= false;

  // BIOS. Segons el manual la bios es de 32K pero la de SeaBIOS és
  // més gran (38K).
  if ( optrom == NULL || optrom_size < 32*1024 || optrom_size >= 64*1024 )
    return PC_BADOPTROM;
  _bios.mask= optrom_size==32*1024 ? 0xFFFF8000 : 0xFFFF0000;
  _bios.v8= (const uint8_t *) optrom;
  _bios.size= optrom_size;
  _bios.size_1= optrom_size-1;
  _bios.size_3= optrom_size-3;
  _bios.size_7= optrom_size-7;

  // Timing.
  _timing.cc_used= 0;
  _timing.cc= 0;
  _timing.cctoEvent= 0;
  assert ( PC_ClockFreq%100 == 0 ); // gaste MHz*10000
  _timing.cc_div= (long) (PC_ClockFreq/100);
  _timing.cc_mul= 1; // La primera vegada no importa molt, però és necessari
  _timing.vcc_tmp= 0;
  
  // Rendering
  memset ( _render.fb, 0, sizeof(_render.fb) );
  _render.H= 0;
  _render.V= 0;
  _render.char_dots= 0;
  _render.scanline= 0;
  _render.in_hblank= false;
  _render.in_hretrace= false;
  _render.in_vblank= false;
  _render.in_vretrace= false;
  _render.blink= false;
  _render.blink_counter= 0;
  _render.pixel_bus= 0x00;
  _render.start_addr= 0;
  
  // Altres
  memset ( _vram, 0, sizeof(_vram) );
  init_pci_regs ();
  init_regs ();

  update_vclk ();
  update_cc_to_event ();
  
  return PC_NOERROR;
  
} // end PC_svga_cirrus_clgd5446_init


const uint8_t *
PC_svga_cirrus_clgd5446_get_vram (void)
{
  return &(_vram[0]);
} // end PC_svga_cirrus_clgd5446_get_vram
