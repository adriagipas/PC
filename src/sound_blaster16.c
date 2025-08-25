/*
 * Copyright 2023-2025 Adrià Giménez Pastor.
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
 *  sound_blaster16.c - Implementació d'una targeta SoundBlaster16
 *                      ISA.
 *
 */

/*
 * NOTES FM!!!!
 *
 * - He assumit que TREMOLO fa referència a la modulació AM de LFO, i
 *   VIBRATO fa referència a la modulació PM de LFO.
 *
 * - He decidit implementar-ho basant-me en el codi del xip YM2612
 *   (fm.c) de la MD. Una cosa que no entenc és com es poden emprar a
 *   la vegada dos freqüències ~6Hz (Vib) i 3.7Hz (Tre) d'un únic
 *   LFO. Per simplificar he decidit implementar-ho com dos LFO
 *   separats. Un altre dubte és que no pareix que hi hasca cap
 *   control del LFO des de fora, i que s'aplica operació a
 *   operació. Quan s'activa a una operació es reinicia?? Vaig a
 *   implementar-ho com comptadors individuals per a cada operador,
 *   cosa que es poc realista.
 *
 * NOTES DSP!!!
 *
 * - En el DSP de la SoundBlaster 16 és posible configurar IRQ i DMA
 *   (exceptuant si és un model PnP). Per simplicitat he decidit no
 *   permetre-ho. Qualsevol intent de modificar el DMA o l'IRQ
 *   l'ignoraré.
 *
 * - Crec que per a eixida no és cert, però per simplificar no vaig a
 *   permetre freqüències de mostreig fora del rang [4000,44100].
 *
 * - IMPORTANTÍSIM!!!! Igual he de simular amb un gra més fi
 *   l'activació del DREQ. Ara va a blocs, de colp llig moltes dades i
 *   es para.
 *
 */

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#include "PC.h"




/**********/
/* MACROS */
/**********/

#define EG_MAX_ATTENUATION 0x3FF

// 6.069607204861111 Hz (volta completa comptador 7 bits) que
// s'aproxima als 6, 6.1 i 6.06689453125 Hz que he trobat per la
// documentació. Són cicles de FM (14.32Mhz/288)
#define FM_VIB_CC 64

// 3.6995701058201056 Hz (volta completa comptador 7 bits) que
// s'aproxima als 3.7 Hz que he trobat per la
// documentació. Són cicles de FM (14.32Mhz/288)
#define FM_AM_CC 105

#define OUT2PHASEMOD(VAL) ((VAL>>1)&0x3FF)

#define FM_BUF_SIZE (PC_AUDIO_BUFFER_SIZE*3)

#define DSP_OUT_BUF_SIZE 4



/*************/
/* CONSTANTS */
/*************/

// Cicles del Master Clock FM per increment
static const long FM_TIMERS_CC[2]=
  {
    1146, // ~80us
    4582  // ~320us
  };


// La segona dimensió representa l'amplitut d'un quart d'ona sinuidal
// (0..1) representat en 5 bits (0..0x1F). Aquesta amplitut està
// multiplicada pel percentatge indexat en la primera dimensió (DVB).
static const int16_t FM_LFO_PM_INC[2][8]= {
  {0, 0, 0, 1,  1,  1,  2,  2}, // 7%
  {0, 0, 1, 2,  2,  2,  3,  4}  // 14%
};


// IMPORTANT!! La modulació amplitut aplica atenuacions molt
// estranyes que no es poden aproximar fàcilment amb operacions de
// bits a partir d'un comptador. Per aquest motiu he decidit fer una
// taula amb valors aproximats per a cada pas del comptador. Açò
// representa una línia desde 0 fins a atenuació màxima per a cada
// entrada (64 pasos). 0 -> 1dB, 1 -> 4.8db
static const int16_t FM_AM_TABLE[2][64]= {
  // 0dB -> ~ 1dB
  {0x000,0x000,0x000,0x001, 0x001,0x001,0x001,0x001,
   0x001,0x002,0x002,0x002, 0x002,0x002,0x002,0x003,
   0x003,0x003,0x003,0x003, 0x003,0x004,0x004,0x004,
   0x004,0x004,0x004,0x005, 0x005,0x005,0x005,0x005,

   0x005,0x006,0x006,0x006, 0x006,0x006,0x006,0x007,
   0x007,0x007,0x007,0x007, 0x007,0x008,0x008,0x008,
   0x008,0x008,0x008,0x009, 0x009,0x009,0x009,0x009,
   0x009,0x00a,0x00a,0x00a, 0x00a,0x00a,0x00a,0x00b},

  // 0dB -> ~4.8dB
  {0x000,0x001,0x002,0x002, 0x003,0x004,0x005,0x006,
   0x007,0x007,0x008,0x009, 0x00a,0x00b,0x00b,0x00c,
   0x00d,0x00e,0x00f,0x00f, 0x010,0x011,0x012,0x013,
   0x014,0x014,0x015,0x016, 0x017,0x018,0x018,0x019,

   0x01a,0x01b,0x01c,0x01c, 0x01d,0x01e,0x01f,0x020,
   0x021,0x021,0x022,0x023, 0x024,0x025,0x025,0x026,
   0x027,0x028,0x029,0x029, 0x02a,0x02b,0x02c,0x02d,
   0x02e,0x02e,0x02f,0x030, 0x031,0x032,0x032,0x033}
};


// El primer índex són els 4 bits superiors del FNUM (l'actual??) i el
// segon ínex és el bloc. Els valors representen valors d'atenuació de
// 10 bits com de costum:
//
// |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0
// |---------------------------------------------------------------------------
// |   48  |   24  |   12  |   6   |   3   |  1.5  |  0.75 | 0.375 | 0.1875|0.09375|
// 
static const int16_t FM_KSL_ATT[16][8]= {
  {0x000,0x000,0x000,0x000, 0x000,0x000,0x000,0x000},
  {0x000,0x000,0x000,0x000, 0x000,0x020,0x040,0x060},
  {0x000,0x000,0x000,0x000, 0x020,0x040,0x060,0x080},
  {0x000,0x000,0x000,0x014, 0x034,0x054,0x074,0x094},

  {0x000,0x000,0x000,0x020, 0x040,0x060,0x080,0x0a0},
  {0x000,0x000,0x00c,0x02c, 0x04c,0x06c,0x08c,0x0ac},
  {0x000,0x000,0x014,0x034, 0x054,0x074,0x094,0x0b4},
  {0x000,0x000,0x01c,0x03c, 0x05c,0x07c,0x09c,0x0bc},

  {0x000,0x000,0x020,0x040, 0x060,0x080,0x0a0,0x0c0},
  {0x000,0x008,0x028,0x048, 0x068,0x088,0x0a8,0x0c8},
  {0x000,0x00c,0x02c,0x04c, 0x06c,0x08c,0x0ac,0x0cc},
  {0x000,0x010,0x030,0x050, 0x070,0x090,0x0b0,0x0d0},

  {0x000,0x014,0x034,0x054, 0x074,0x094,0x0b4,0x0d4},
  {0x000,0x018,0x038,0x058, 0x078,0x098,0x0b8,0x0d8},
  {0x000,0x01c,0x03c,0x05c, 0x07c,0x09c,0x0bc,0x0dc},
  {0x000,0x020,0x040,0x060, 0x080,0x0a0,0x0c0,0x0e0}
};


// Shifts per a cada rate.
static const int FM_EG_COUNTER_SHIFT[64]= {
  11, 11, 11, 11,     // 0-3
  10, 10, 10, 10,     // 4-7
  9,  9,  9,  9,      // 8-11
  8,  8,  8,  8,      // 12-15
  7,  7,  7,  7,      // 16-19
  6,  6,  6,  6,      // 20-23
  5,  5,  5,  5,      // 24-27
  4,  4,  4,  4,      // 28-31
  3,  3,  3,  3,      // 32-35
  2,  2,  2,  2,      // 36-39
  1,  1,  1,  1,      // 40-43
  0,  0,  0,  0,      // 44-47
  0,  0,  0,  0,      // 48-51
  0,  0,  0,  0,      // 52-55
  0,  0,  0,  0,      // 56-59
  0,  0,  0,  0       // 60-63
};


static const int FM_EG_ATTENUATION_INCREMENT[64][8]= {
  
  // 0-3
  {0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0},
  {0,1,0,1,0,1,0,1},
  {0,1,0,1,0,1,0,1},

  // 4-7
  {0,1,0,1,0,1,0,1},
  {0,1,0,1,0,1,0,1},
  {0,1,1,1,0,1,1,1},
  {0,1,1,1,0,1,1,1},

  // 8-11
  {0,1,0,1,0,1,0,1},
  {0,1,0,1,1,1,0,1},
  {0,1,1,1,0,1,1,1},
  {0,1,1,1,1,1,1,1},

  // 12-15
  {0,1,0,1,0,1,0,1},
  {0,1,0,1,1,1,0,1},
  {0,1,1,1,0,1,1,1},
  {0,1,1,1,1,1,1,1},

  // 16-19
  {0,1,0,1,0,1,0,1},
  {0,1,0,1,1,1,0,1},
  {0,1,1,1,0,1,1,1},
  {0,1,1,1,1,1,1,1},

  // 20-23
  {0,1,0,1,0,1,0,1},
  {0,1,0,1,1,1,0,1},
  {0,1,1,1,0,1,1,1},
  {0,1,1,1,1,1,1,1},

  // 24-27
  {0,1,0,1,0,1,0,1},
  {0,1,0,1,1,1,0,1},
  {0,1,1,1,0,1,1,1},
  {0,1,1,1,1,1,1,1},

  // 28-31
  {0,1,0,1,0,1,0,1},
  {0,1,0,1,1,1,0,1},
  {0,1,1,1,0,1,1,1},
  {0,1,1,1,1,1,1,1},

  // 32-35
  {0,1,0,1,0,1,0,1},
  {0,1,0,1,1,1,0,1},
  {0,1,1,1,0,1,1,1},
  {0,1,1,1,1,1,1,1},

  // 36-39
  {0,1,0,1,0,1,0,1},
  {0,1,0,1,1,1,0,1},
  {0,1,1,1,0,1,1,1},
  {0,1,1,1,1,1,1,1},

  // 40-43
  {0,1,0,1,0,1,0,1},
  {0,1,0,1,1,1,0,1},
  {0,1,1,1,0,1,1,1},
  {0,1,1,1,1,1,1,1},

  // 44-47
  {0,1,0,1,0,1,0,1},
  {0,1,0,1,1,1,0,1},
  {0,1,1,1,0,1,1,1},
  {0,1,1,1,1,1,1,1},

  // 48-51
  {1,1,1,1,1,1,1,1},
  {1,1,1,2,1,1,1,2},
  {1,2,1,2,1,2,1,2},
  {1,2,2,2,1,2,2,2},

  // 52-55
  {2,2,2,2,2,2,2,2},
  {2,2,2,4,2,2,2,4},
  {2,4,2,4,2,4,2,4},
  {2,4,4,4,2,4,4,4},

  // 56-59
  {4,4,4,4,4,4,4,4},
  {4,4,4,8,4,4,4,8},
  {4,8,4,8,4,8,4,8},
  {4,8,8,8,4,8,8,8},

  // 60-63
  {8,8,8,8,8,8,8,8},
  {8,8,8,8,8,8,8,8},
  {8,8,8,8,8,8,8,8},
  {8,8,8,8,8,8,8,8}
  
};


// APUNTS:
//
// - Representa un quart del cercle
//
// - Com la fase és 10bits un quart és 8bits per tant 256 valors
//   diferents.
//
// - Els valors ja estàn desats en format atenuació per poder sumar-ho
//   a l'atenuació. Bàsicament (-log(VAL)/log(2))/256.
//
// - El valor final utilitza 12 bits en format 4.8.
//
static const int16_t FM_SIN_TABLE[256]= {
  0x859, 0x6C3, 0x607, 0x58B, 0x52E, 0x4E4, 0x4A6, 0x471, 
  0x443, 0x41A, 0x3F5, 0x3D3, 0x3B5, 0x398, 0x37E, 0x365, 
  0x34E, 0x339, 0x324, 0x311, 0x2FF, 0x2ED, 0x2DC, 0x2CD, 
  0x2BD, 0x2AF, 0x2A0, 0x293, 0x286, 0x279, 0x26D, 0x261, 
  0x256, 0x24B, 0x240, 0x236, 0x22C, 0x222, 0x218, 0x20F, 
  0x206, 0x1FD, 0x1F5, 0x1EC, 0x1E4, 0x1DC, 0x1D4, 0x1CD, 
  0x1C5, 0x1BE, 0x1B7, 0x1B0, 0x1A9, 0x1A2, 0x19B, 0x195, 
  0x18F, 0x188, 0x182, 0x17C, 0x177, 0x171, 0x16B, 0x166, 
  0x160, 0x15B, 0x155, 0x150, 0x14B, 0x146, 0x141, 0x13C, 
  0x137, 0x133, 0x12E, 0x129, 0x125, 0x121, 0x11C, 0x118, 
  0x114, 0x10F, 0x10B, 0x107, 0x103, 0x0FF, 0x0FB, 0x0F8, 
  0x0F4, 0x0F0, 0x0EC, 0x0E9, 0x0E5, 0x0E2, 0x0DE, 0x0DB, 
  0x0D7, 0x0D4, 0x0D1, 0x0CD, 0x0CA, 0x0C7, 0x0C4, 0x0C1, 
  0x0BE, 0x0BB, 0x0B8, 0x0B5, 0x0B2, 0x0AF, 0x0AC, 0x0A9, 
  0x0A7, 0x0A4, 0x0A1, 0x09F, 0x09C, 0x099, 0x097, 0x094, 
  0x092, 0x08F, 0x08D, 0x08A, 0x088, 0x086, 0x083, 0x081, 
  0x07F, 0x07D, 0x07A, 0x078, 0x076, 0x074, 0x072, 0x070, 
  0x06E, 0x06C, 0x06A, 0x068, 0x066, 0x064, 0x062, 0x060, 
  0x05E, 0x05C, 0x05B, 0x059, 0x057, 0x055, 0x053, 0x052, 
  0x050, 0x04E, 0x04D, 0x04B, 0x04A, 0x048, 0x046, 0x045, 
  0x043, 0x042, 0x040, 0x03F, 0x03E, 0x03C, 0x03B, 0x039, 
  0x038, 0x037, 0x035, 0x034, 0x033, 0x031, 0x030, 0x02F, 
  0x02E, 0x02D, 0x02B, 0x02A, 0x029, 0x028, 0x027, 0x026, 
  0x025, 0x024, 0x023, 0x022, 0x021, 0x020, 0x01F, 0x01E, 
  0x01D, 0x01C, 0x01B, 0x01A, 0x019, 0x018, 0x017, 0x017, 
  0x016, 0x015, 0x014, 0x014, 0x013, 0x012, 0x011, 0x011, 
  0x010, 0x00F, 0x00F, 0x00E, 0x00D, 0x00D, 0x00C, 0x00C, 
  0x00B, 0x00A, 0x00A, 0x009, 0x009, 0x008, 0x008, 0x007, 
  0x007, 0x007, 0x006, 0x006, 0x005, 0x005, 0x005, 0x004, 
  0x004, 0x004, 0x003, 0x003, 0x003, 0x002, 0x002, 0x002, 
  0x002, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 
  0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000
};


// APUNTS
//
// - Cada entrada representa un valor entre 0 i 1 (approx). La idea és
//   que fa reerència a la part fraccional que volem convertir.
//
// - Cada entrada en realitat és 2^-entrada
//
// - Cada entrada es representa amb 11bits
//
static const int16_t FM_POW_TABLE[256]= {
  0x7FA, 0x7F5, 0x7EF, 0x7EA, 0x7E4, 0x7DF, 0x7DA, 0x7D4, 
  0x7CF, 0x7C9, 0x7C4, 0x7BF, 0x7B9, 0x7B4, 0x7AE, 0x7A9, 
  0x7A4, 0x79F, 0x799, 0x794, 0x78F, 0x78A, 0x784, 0x77F, 
  0x77A, 0x775, 0x770, 0x76A, 0x765, 0x760, 0x75B, 0x756, 
  0x751, 0x74C, 0x747, 0x742, 0x73D, 0x738, 0x733, 0x72E, 
  0x729, 0x724, 0x71F, 0x71A, 0x715, 0x710, 0x70B, 0x706, 
  0x702, 0x6FD, 0x6F8, 0x6F3, 0x6EE, 0x6E9, 0x6E5, 0x6E0, 
  0x6DB, 0x6D6, 0x6D2, 0x6CD, 0x6C8, 0x6C4, 0x6BF, 0x6BA, 
  0x6B5, 0x6B1, 0x6AC, 0x6A8, 0x6A3, 0x69E, 0x69A, 0x695, 
  0x691, 0x68C, 0x688, 0x683, 0x67F, 0x67A, 0x676, 0x671, 
  0x66D, 0x668, 0x664, 0x65F, 0x65B, 0x657, 0x652, 0x64E, 
  0x649, 0x645, 0x641, 0x63C, 0x638, 0x634, 0x630, 0x62B, 
  0x627, 0x623, 0x61E, 0x61A, 0x616, 0x612, 0x60E, 0x609, 
  0x605, 0x601, 0x5FD, 0x5F9, 0x5F5, 0x5F0, 0x5EC, 0x5E8, 
  0x5E4, 0x5E0, 0x5DC, 0x5D8, 0x5D4, 0x5D0, 0x5CC, 0x5C8, 
  0x5C4, 0x5C0, 0x5BC, 0x5B8, 0x5B4, 0x5B0, 0x5AC, 0x5A8, 
  0x5A4, 0x5A0, 0x59C, 0x599, 0x595, 0x591, 0x58D, 0x589, 
  0x585, 0x581, 0x57E, 0x57A, 0x576, 0x572, 0x56F, 0x56B, 
  0x567, 0x563, 0x560, 0x55C, 0x558, 0x554, 0x551, 0x54D, 
  0x549, 0x546, 0x542, 0x53E, 0x53B, 0x537, 0x534, 0x530, 
  0x52C, 0x529, 0x525, 0x522, 0x51E, 0x51B, 0x517, 0x514, 
  0x510, 0x50C, 0x509, 0x506, 0x502, 0x4FF, 0x4FB, 0x4F8, 
  0x4F4, 0x4F1, 0x4ED, 0x4EA, 0x4E7, 0x4E3, 0x4E0, 0x4DC, 
  0x4D9, 0x4D6, 0x4D2, 0x4CF, 0x4CC, 0x4C8, 0x4C5, 0x4C2, 
  0x4BE, 0x4BB, 0x4B8, 0x4B5, 0x4B1, 0x4AE, 0x4AB, 0x4A8, 
  0x4A4, 0x4A1, 0x49E, 0x49B, 0x498, 0x494, 0x491, 0x48E, 
  0x48B, 0x488, 0x485, 0x482, 0x47E, 0x47B, 0x478, 0x475, 
  0x472, 0x46F, 0x46C, 0x469, 0x466, 0x463, 0x460, 0x45D, 
  0x45A, 0x457, 0x454, 0x451, 0x44E, 0x44B, 0x448, 0x445, 
  0x442, 0x43F, 0x43C, 0x439, 0x436, 0x433, 0x430, 0x42D, 
  0x42A, 0x428, 0x425, 0x422, 0x41F, 0x41C, 0x419, 0x416, 
  0x414, 0x411, 0x40E, 0x40B, 0x408, 0x406, 0x403, 0x400
};




/*********/
/* TIPUS */
/*********/

typedef struct
{

  bool    enabled;
  uint8_t counter;
  uint8_t init_val;
  bool    irq_done;
  bool    irq_enabled;
  long    cc;
  
} fm_timer_t;

typedef struct
{
  
  bool     keyon;   // Inidica que ja ha sigut activat
  int32_t  out;     // Eixida. El valor estarà entre [-8192,8191]
  int32_t  phase;   // Fase actual (19 bits)
  int32_t  pg;      // Increment que genera el Phase Generator
  int      keycode; // Key code (Block|Note)
  int16_t  tlevel;  // Atenuació per defecte.
  int16_t  ksl_att; // Atenuació KSL (depèn de FNUM/BLOCK)
  // Valors del canal
  uint16_t fnum;
  int      block;
  struct
  {

    // Eixida del EG. És un valor de 10 bits que representa
    // l'atenuació mesurada en decibels, des de 0db fins a ~96db. La
    // interpretació de cada bit (9 -- 0) segons Nemesis seria:
    //
    //  48   |   24  |   12   |   6     |   3   |  1.5  |
    //  0.75 | 0.375 | 0.1875 | 0.09375 |
    int16_t out;
    
    // Els dos tenen la mateixa interpretació que out.
    int16_t sustain;
    
    // Més estat.
    enum {
      EG_ATTACK,
      EG_DECAY,
      EG_SUSTAIN,
      EG_RELEASE
    }    state;

    // Timing. El EG s'executa cada 3 mostres.
    int      cc;
    uint32_t counter;

    // Rates
    int ar_rate;
    int dr_rate;
    int rr_rate;
    
  }        eg;  // Envelope generator
  struct
  {
    bool    enabled;
    int     cc;      // Cicles FM acumulats.
    uint8_t counter; // Comptador de 128 bits.
  }        vib;
  struct
  {
    bool    enabled;
    int     cc;      // Cicles FM acumulats.
    uint8_t counter; // Comptador de 128 bits.
  }        am;
  struct
  {
    uint8_t am_vib_egt_ksr_mult;
    uint8_t ksl_tl;
    uint8_t ar_dr;
    uint8_t sl_rr;
    uint8_t ws;
  }        regs; // Registres
  
} fm_op_t;

// Canal
typedef struct fm_channel_t fm_channel_t;

struct fm_channel_t
{
  
  fm_op_t *slots2[2]; // Punters a operadors mode 2
  fm_op_t *slots4[4]; // Punters a operadors mode 4
  enum {
    CHN_DISABLED,
    CHN_OP2,
    CHN_OP4
  }        mode;
  uint8_t  feedback;  // Tipus de feedback.
  int32_t  fb_buf[2]; // Buffer per a feedback
  int32_t  out;       // Eixida en 16bits amb signe
  bool     l,r;       // Indica si el canal s'emet o no pel altaveu
                      // corresponent.
  
  struct
  {
    uint8_t fnumL;
    uint8_t kon_block_fnumH;
    uint8_t chd_chc_chb_cha_fb_cnt;
  }       regs;     // Registres

  // Punter a canal colaborador.
  fm_channel_t *chn_col;
  
};




/*********/
/* ESTAT */
/*********/

// Callback
static PC_Warning *_warning;
static void *_udata;

// Xip FM YMF262 (OPL3)
static struct
{

  // Adreces
  uint8_t addr[2];
  
  // Timers
  fm_timer_t timers[2];

  // Operadors.
  fm_op_t ops[2][18];

  // Canals.
  fm_channel_t channels[2][9];
  
  // Timing.
  long cc_accum; // Ja estan multiplicats.
  long cc_mul;
  long cc_div;
  long cc_fm_accum; // Cicles FM pendents de gastar

  // Altres.
  bool    opl3_mode;
  uint8_t connection_sel_reg;
  int     dvb; // 0 - 7 cent, 1 - 14 cent
  int     dam; // 0 - 1dB, 1 - 4.8dB
  int     nts; // 0 o 1
  int     cc_delay_status; // Emulem que llegir de l'status costa un
                           // poc més de de 23/35 micros segons.
  
  // Buffer eixida
  struct
  {
    int     N;
    int     p;
    int32_t l[FM_BUF_SIZE];
    int32_t r[FM_BUF_SIZE];
    double  fss; // Factor susbsampling per a passar a 44.1KHz
    double  pss; // Posició subsampling per a passar a 44.1KHz
  } out;
  
} _fm;

// Xip DSP (Digital Sound Processor)
static struct
{

  enum {
    DSP_WAIT_CMD= 0,
    DSP_WAIT_ARG1,
    DSP_WAIT_ARG1_2,
    DSP_WAIT_ARG2_2,
    DSP_WAIT_ARG1_3,
    DSP_WAIT_ARG2_3,
    DSP_WAIT_ARG3_3,
    DSP_READY
  }       state;        // Estat intern respecte a les dades
                        // d'entrada.
  bool    reset_flag;   // S'utilitza per a fer el reset.
  bool    pcspeaker_on; // En realitat en 4.xx no s'utilitza, és un
                        // flag
                        // que es manté per retrocompatibilitat.
  int     block_transfer_size; // Block transfer size (S'empra en el
                               // High-Speed mode i auto-init DMA)
  struct
  {
    // NOTA!! Hi ha dos maneres d'introduir la frequencia 40h i 41h, i
    // no em queda clar la relació entre els dos. Així que he decidit
    // fer els càlculs en l'operació.
    double  freq;
    double  ifreq;
    double  ratio; // freq/44100
    bool    mono;
    enum {
      DSP_FORMAT_U8, // Ho deixe, però igual és redundant
      DSP_FORMAT_U8_STEREO,
      DSP_FORMAT_U8_MONO,
      DSP_FORMAT_S8_STEREO,
      DSP_FORMAT_S8_MONO,
      DSP_FORMAT_U16_STEREO,
      DSP_FORMAT_U16_MONO,
      DSP_FORMAT_S16_STEREO,
      DSP_FORMAT_S16_MONO,
      DSP_FORMAT_ADPCM8_4
    }       type;
  }       format;
  struct
  {
    bool    started;
    int32_t step;
    int32_t current;
  }       adpcm;
  struct
  {
    uint8_t v[DSP_OUT_BUF_SIZE];
    int     p;
    int     N;
  }       out; // Buffer d'eixida
  struct
  {
    uint8_t val;
    bool    empty;
    uint8_t cmd;
    uint8_t args[3];
  }       in; // Buffer d'entrada
  struct
  {
    int16_t l[PC_AUDIO_BUFFER_SIZE*2];
    int16_t r[PC_AUDIO_BUFFER_SIZE*2];
    int     p;
    int     N;
    double  pss; // Posició subsampling
    bool    stop_dma; // Força una parada de DMA
  }       render;
  struct
  {
    enum {
      DSP_DMA_NONE,
      DSP_DMA_SINGLE,
      DSP_DMA_AUTO_INIT,
      DSP_DMA_AUTO_INIT_FINISH
    }    state;
    int  counter;
    int  init_counter;
    bool in_clock; // Per si es llança un dreq mentre es crida a
                   // 'clock' que crida a signal.
    bool paused;
    bool irq_on;
    bool dreq; // Desa el valor de dreq per no estar enviant missatges
               // sense parar al mòdul de DMA.
    int16_t l_sample;
    bool    waiting_l_sample;
  }       dma;
  struct
  {
    enum {
      DSP_DMA16_NONE,
      DSP_DMA16_SINGLE,
      DSP_DMA16_AUTO_INIT,
      DSP_DMA16_AUTO_INIT_FINISH
    }    state;
    int  counter;
    int  init_counter;
    bool in_clock; // Per si es llança un dreq mentre es crida a
                   // 'clock' que crida a signal.
    bool paused;
    bool irq_on;
    bool dreq; // Desa el valor de dreq per no estar enviant missatges
               // sense parar al mòdul de DMA.
    int16_t l_sample;
    bool    waiting_l_sample;
  }       dma16;
  uint8_t test_reg;
} _dsp;

static struct
{
  
  uint8_t addr;
  uint8_t mic_vol; // 0 to 31 -> – 62 dB to 0 dB, in 2 dB steps.
  uint8_t midi_vol_l; // 0 to 31 -> – 62 dB to 0 dB, in 2 dB steps.
  uint8_t midi_vol_r;
  uint8_t cd_vol_l; // 0 to 31 -> – 62 dB to 0 dB, in 2 dB steps.
  uint8_t cd_vol_r;
  uint8_t master_vol_l; // 0 to 31 -> – 62 dB to 0 dB, in 2 dB steps.
  uint8_t master_vol_r;
  uint8_t voice_vol_l; // 0 to 31 -> – 62 dB to 0 dB, in 2 dB steps.
  uint8_t voice_vol_r;
  uint8_t line_vol_l; // 0 to 31 -> – 62 dB to 0 dB, in 2 dB steps.
  uint8_t line_vol_r;
  uint8_t input_gain_l; // 0 to 3 -> 0 dB to 18 dB, in 6 dB steps.
  uint8_t input_gain_r;
  uint8_t output_gain_l; // 0 to 3 -> 0 dB to 18 dB, in 6 dB steps.
  uint8_t output_gain_r;
  uint8_t treble_l; // 0 to 7 ⇒ – 14 dB to 0 dB, in 2 dB steps. 8 to
                    // 15 ⇒ 0 dB to 14 dB, in 2 dB steps.
  uint8_t treble_r; // 0 to 7 ⇒ – 14 dB to 0 dB, in 2 dB steps. 8 to
                    // 15 ⇒ 0 dB to 14 dB, in 2 dB steps.
  uint8_t bass_l; // 0 to 7 ⇒ – 14 dB to 0 dB, in 2 dB steps. 8 to
                  // 15 ⇒ 0 dB to 14 dB, in 2 dB steps.
  uint8_t bass_r; // 0 to 7 ⇒ – 14 dB to 0 dB, in 2 dB steps. 8 to
                  // 15 ⇒ 0 dB to 14 dB, in 2 dB steps.
  uint8_t out_switches;
  uint8_t in_switches_l;
  uint8_t in_switches_r;
  uint8_t pc_speaker_vol; // 0 to 3 ⇒ – 18 dB to 0 dB, in 6 dB steps.
  bool    agc_on;
  
} _mixer;

// Gestionar cicles
static struct
{

  int cc_used;
  int cc;
  int cctoEvent;

  // Per a passar a mostres a 44100Hz (cc*(cc_mul))/cc_div
  long cc_mul;
  long cc_div;
  long cc_remain; // Ja multiplicades
  
} _timing;

// Buffer d'eixida.
static struct
{

  int16_t buf[PC_AUDIO_BUFFER_SIZE*2];
  int     N; // 0...PC_AUDIO_BUFFER_SIZE-1
  
} _out;




/*********************/
/* FUNCIONS PRIVADES */
/*********************/

// FM //////////////////////////////////////////////////////////////////////////

static void
fm_timer_reset (
                fm_timer_t *t
                )
{

  t->enabled= false;
  t->counter= 0x00;
  t->init_val= 0x00;
  t->irq_done= false;
  t->irq_enabled= false;
  t->cc= 0;
  
} // end fm_timer_reset


static void
fm_timers_control (
                   const uint8_t data
                   )
{

  if ( data&0x80 ) // Reset the flags for timers
    {
      _fm.timers[0].irq_done= false;
      _fm.timers[1].irq_done= false;
    }
  else
    {
      
      // Timer 1.
      if ( data&0x40 ) _fm.timers[0].irq_enabled= false;
      else
        {
          _fm.timers[0].irq_enabled= true;
          if ( data&0x01 )
            {
              _fm.timers[0].counter= _fm.timers[0].init_val;
              _fm.timers[0].enabled= true;
            }
          else
            {
              _fm.timers[0].enabled= false;
              _fm.timers[0].cc= 0;
            }
        }

      // Timer 2.
      if ( data&0x20 ) _fm.timers[1].irq_enabled= false;
      else
        {
          _fm.timers[1].irq_enabled= true;
          if ( data&0x02 )
            {
              _fm.timers[1].counter= _fm.timers[1].init_val;
              _fm.timers[1].enabled= true;
            }
          else
            {
              _fm.timers[1].enabled= false;
              _fm.timers[1].cc= 0;
            }
        }
        
    }
  
} // end fm_timers_control


static void
fm_timers_clock (
                 const long cc // Cicles Master Clock FM
                 )
{

  int i;
  fm_timer_t *t;
  long CC,total_cc;
  uint16_t clocks,remain;
  bool irq;
  
  
  for ( i= 0; i < 2; ++i )
    {
      t= &_fm.timers[i];
      if ( t->enabled )
        {
          CC= FM_TIMERS_CC[i];
          total_cc= t->cc+cc;
          clocks= (uint16_t) (total_cc/CC);
          t->cc= total_cc%CC;
          if ( clocks > 0 )
            {
              irq= false;
              remain= 0x100 - (uint16_t) t->counter;
              while ( clocks >= remain )
                {
                  irq= true;
                  clocks-= remain;
                  t->counter= t->init_val;
                  remain= 0x100 - (uint16_t) t->counter;
                }
              if ( irq && t->irq_enabled )
                {
                  t->irq_done= true;
                  // Pareix que no es genera cap IRQ. Pista:
                  // https://www.vogons.org/viewtopic.php?t=60590
                  /*
                  printf("[CAL IMPLEMENTAR] SB16 FM Timer %d -"
                         " CAL GENERAR IRQ?!!!\n",i+1);
                  */
                }
              t->counter+= (uint8_t) clocks;
            }
        }
    }
  
} // end fm_timers_clock


// Pot tornar -1
static int
fm_timers_cc_to_event (void)
{

  int ret,i,tmp;
  fm_timer_t *t;
  long cc;
  

  ret= -1;
  for ( i= 0; i < 2; ++i )
    {
      t= &_fm.timers[i];
      if ( t->enabled && t->irq_enabled )
        {
          cc= 0x100 - (long) t->counter;
          cc*= FM_TIMERS_CC[i];
          cc-= t->cc;
          cc*= _fm.cc_div;
          tmp= (int) (cc/_fm.cc_mul);
          if ( (cc%_fm.cc_mul) != 0 ) ++tmp;
          assert ( tmp > 0 );
          if ( ret == -1 || tmp < ret ) ret= tmp;
        }
    }

  return ret;
  
} // end fm_timers_cc_to_event


// A partir del valor de rate inicial (R) i el keycode, torna el rate final.
static int
fm_op_calc_eg_rate (
                    const fm_op_t *op,
                    const int      R
                    )
{
  
  int Rof,ksr,ret;


  if ( R == 0 ) ret= 0;
  else
    {
      ksr= (int) ((op->regs.am_vib_egt_ksr_mult>>4)&0x1);
      Rof= op->keycode>>(2*(1-ksr));
      ret= 4*R + Rof;
      if ( ret > 63 ) ret= 63;
    }
  
  return ret;
  
} // end fm_op_calc_eg_rate


static void
fm_op_update_eg_ar_rate (
                         fm_op_t *op
                         )
{
  op->eg.ar_rate= fm_op_calc_eg_rate ( op, (int) (op->regs.ar_dr>>4) );
} // end fm_op_update_eg_ar_rate


static void
fm_op_update_eg_dr_rate (
                         fm_op_t *op
                         )
{
  op->eg.dr_rate= fm_op_calc_eg_rate ( op, (int) (op->regs.ar_dr&0xF) );
} // end fm_op_update_eg_dr_rate


static void
fm_op_update_eg_rr_rate (
                         fm_op_t *op
                         )
{
  op->eg.rr_rate= fm_op_calc_eg_rate ( op, (int) (op->regs.sl_rr&0xF) );
} // end fm_op_update_eg_rr_rate


static void
fm_op_update_eg_all_rates (
                           fm_op_t *op
                           )
{

  fm_op_update_eg_ar_rate ( op );
  fm_op_update_eg_dr_rate ( op );
  fm_op_update_eg_rr_rate ( op );
  
} // end fm_op_update_eg_all_rates


// Es crida cada vegada que canvia algun paràmetre que pot afectar a
// la phase o a altres paràmetres precalculats relacionats amb EG.
static void
fm_op_update_pg_and_eg (
                        fm_op_t *op
                        )
{

  uint16_t fnum;
  int block,mult,f10,f9,keycode,ksl;
  int32_t pg;
  int16_t pm_inc;
  uint8_t pm_counter,pm_quarter;
  
  
  // Extrau fnum i block
  fnum= op->fnum;
  block= op->block;

  // Aplica LFO phase modulation a fnum. (VIB)
  if ( op->vib.enabled )
    {
      // El comptador LFO té 7 bits (128 pasos), però per a modular
      // sols tenen pes els 5 superiors. Recordem que els 128 pasos
      // simulen una volta completa.
      pm_counter= (op->vib.counter&0x7F)>>2;
      if ( pm_counter != 0 )
        {

          // Obté quart
          pm_quarter= pm_counter&0x7;
          if ( pm_counter&0x8 )
            pm_quarter= (~pm_quarter)&0x7;

          // Calcula increment.
          //
          // Aparentment és (fnum*inc)/512. No tinc clar el 512, però
          // bàsicament és com que hi han 9 bits del resultat que són
          // decimals i es descarten.
          //
          // Els 9 bits crec que són 5 per l'amplitut de l'ona que
          // multiplica (és a dir normalitzat entre 0 i 1), i la resta
          // pareix que es divideix per 16 (Pot ser està relacionat
          // amb el block???)
          pm_inc= ((int16_t) fnum)*FM_LFO_PM_INC[_fm.dvb][pm_quarter];
          if ( pm_counter&0x10 )
            pm_inc= -pm_inc;
          pm_inc>>= 9;
          
          // Aplica
          fnum= (fnum + (uint16_t) pm_inc)&0x3FF;
          
        }
    }

  // Calcula keycode
  f10= (fnum>>9)&0x1;
  f9= (fnum>>8)&0x1;
  keycode= (block<<1) | (_fm.nts==0 ? f9 : f10);
  op->keycode= keycode;

  // El Keycode afecta als rates del EG.
  fm_op_update_eg_all_rates ( op );

  // Calcula atenuació KSL (Depenent FNUM/BLOCK). NOTA!!! He decidit
  // utilitzar el fnum dinàmic.
  ksl= op->regs.ksl_tl>>6;
  switch ( ksl )
    {
      // 0db/oct
    case 0: op->ksl_att= 0; break;
      // 3db/oct
    case 1: op->ksl_att= FM_KSL_ATT[(fnum>>6)&0xF][block]; break;
      // 1.5db/oct
    case 2: op->ksl_att= FM_KSL_ATT[(fnum>>6)&0xF][block]>>1; break;
      // 6db/oct
    case 3: op->ksl_att= FM_KSL_ATT[(fnum>>6)&0xF][block]<<1; break;
    }
  
  // Calcula pg base
  pg= (int32_t) ((uint32_t) fnum);
  if ( block == 0 )     pg>>= 1;
  else if ( block > 1 ) pg<<= block-1;

  // Aplica multiple
  mult= op->regs.am_vib_egt_ksr_mult&0xF;
  if ( mult == 0 )     pg/= 2;
  else if ( mult > 1 )
    {
      if ( mult >= 0xE )       pg*= 15;
      else if ( mult >= 0xC )  pg*= 12;
      else if ( mult >= 0xA )  pg*= 10;
      else                     pg*= mult;
      pg&= 0x7FFFF; // Pot desbordar (19 bits???????)
    }
  
  // Asigna nou valor phase generator
  op->pg= pg;
  
} // end fm_op_update_pg_and_eg


static void
fm_op_set_ar_dr (
                 fm_op_t       *op,
                 const uint8_t  data
                 )
{

  op->regs.ar_dr= data;
  fm_op_update_eg_ar_rate ( op );
  fm_op_update_eg_dr_rate ( op );
  
} // end fm_op_set_ar_dr


static void
fm_op_set_sl_rr (
                 fm_op_t       *op,
                 const uint8_t  data
                 )
{

  op->regs.sl_rr= data;
  fm_op_update_eg_rr_rate ( op );
  op->eg.sustain= (int16_t) (data>>4);
  if ( op->eg.sustain == 0xF )
    op->eg.sustain|= 0x10;
  op->eg.sustain<<= 5;
  
} // end fm_op_set_sl_rr


static void
fm_op_set_ws (
              fm_op_t       *op,
              const uint8_t  data
              )
{
  op->regs.ws= data;
} // end fm_op_set_ws


static void
fm_op_set_am_vib_egt_ksr_mult (
                               fm_op_t       *op,
                               const uint8_t  data
                               )
{

  //NOTA!! He decidit resetejar els comptadors de VIB i AM en el
  //keyon. És a dir, fins i quan estan desactivats compten.
  
  // VIB
  op->vib.enabled= (data&0x40)!=0;

  // AM
  op->am.enabled= (data&0x80)!=0;
  
  // Registre i actualitza.
  op->regs.am_vib_egt_ksr_mult= data;
  fm_op_update_pg_and_eg ( op );
  
} // end fm_op_set_am_vib_egt_ksr_mult


static void
fm_op_set_ksl_tl (
                  fm_op_t       *op,
                  const uint8_t  data
                  )
{

  op->regs.ksl_tl= data;
  op->tlevel= ((int16_t) (data&0x3F))<<3;
  fm_op_update_pg_and_eg ( op );
  
} // end fm_op_set_ksl_tl


static void
fm_op_keyon (
             fm_op_t *op
             )
{
  
  if ( !op->keyon )
    {
      op->eg.out= op->eg.ar_rate >= 62 ? 0 : EG_MAX_ATTENUATION;
      op->eg.state= EG_ATTACK;
      op->phase= 0;
      op->keyon= true;
      op->vib.cc= FM_VIB_CC;
      op->vib.counter= 0;
      op->am.cc= FM_AM_CC;
      op->am.counter= 0;
    }
  
} // end fm_op_keyon


static void
fm_op_keyoff (
              fm_op_t *op
              )
{

  if ( op->keyon )
    {
      op->eg.state= EG_RELEASE;
      op->keyon= false;
    }
  
} // end fm_op_keyoff


static void
fm_op_eg_clock (
                fm_op_t *op
                )
{
  
  int rate,counter_shift_value,update_cycle,inc,egt;
  
  
  // Clock i comprova que no estiga parat.
  ++op->eg.counter;
  if ( op->eg.state == EG_RELEASE && op->eg.out == EG_MAX_ATTENUATION )
    return;

  // Obté EGT
  egt= (op->regs.am_vib_egt_ksr_mult>>5)&0x1;
    
  // Selecciona rate (Vaig a fer-ho com una cadena)
  rate= -1;
  if ( op->eg.state == EG_RELEASE ) rate= op->eg.rr_rate;
  else
    {
      if ( op->eg.state == EG_ATTACK )
        {
          if ( op->eg.out == 0 )
            op->eg.state= EG_DECAY;
          else rate= op->eg.ar_rate;
        }
      if ( op->eg.state == EG_DECAY )
        {
          if ( op->eg.out >= op->eg.sustain )
            op->eg.state= EG_SUSTAIN;
          else rate= op->eg.dr_rate;
        }
      if ( op->eg.state == EG_SUSTAIN )
        {
          // NOTA!!!! El EG_RELEASE sols s'activa amb keyoff. Per tant,
          // estaguent en mode EG_SUSTAIN podem estar fent release. El
          // motiu es que açò permet modificar EGT en qualsevol moment per
          // tornar a estat Sustain.
          // Quan estat en SL, EGT=1 (sustain) i KEYON es manté.
          // NOTA!! Comprovar el keyon en realitat és redundant però
          // millor ho deixe.
          if ( egt == 1 && op->keyon ) return;
          else rate= op->eg.rr_rate;
        }
    }
  assert ( rate != -1 );
  
  // Actualitza
  counter_shift_value= FM_EG_COUNTER_SHIFT[rate];
  if ( op->eg.counter%(1<<counter_shift_value) == 0 )
    {

      // Calcula increment
      update_cycle= (op->eg.counter>>counter_shift_value)&0x7;
      inc= FM_EG_ATTENUATION_INCREMENT[rate][update_cycle];

      // Actualitza
      if ( op->eg.state == EG_ATTACK )
        {
          if ( rate < 62 )
            {
              op->eg.out+= ((int16_t) ((~op->eg.out)*inc))>>4;
              if ( op->eg.out < 0 ) op->eg.out= 0;
            }
        }
      else
        {
          op->eg.out+= inc;
          if ( op->eg.out > EG_MAX_ATTENUATION )
            op->eg.out= EG_MAX_ATTENUATION;
        }
      
    }
  
} // end fm_op_eg_clock


// A partir de la fase (un comptador de 10 bits) obté l'amplitut de
// l'ona base (0..1) en valor absolut, i si el signe és positiu o
// negatiu. En realitat el valor no està en un format preparat per
// poder-se sumar als valors de l'atenuació (veura taules
// FM_SIN_TABLE).
static void
fm_op_get_wave_out (
                    fm_op_t   *op,
                    const int  phase,
                    int16_t   *out,
                    bool      *sign_neg
                    )
{

  const int16_t OUT_0= 0x859; // No aplega a ser 0.
  const int16_t OUT_1= 0x000;
  
  int ws,ind_exp;
  uint8_t sin_ind;
  int16_t tmp;
  
  
  ws= _fm.opl3_mode ? (op->regs.ws&0x7) : (op->regs.ws&0x3);
  switch ( ws )
    {
      
      // Sinus
    case 0:
      sin_ind= (uint8_t) (phase&0xFF);
      if ( phase&0x100 ) sin_ind= ~sin_ind;
      *out= FM_SIN_TABLE[sin_ind];
      *sign_neg= (phase&0x200)!=0;
      break;
      
      // Mitja sinus
    case 1:
      *sign_neg= false;
      if ( (phase&0x200) == 0 )
        {
          sin_ind= (uint8_t) (phase&0xFF);
          if ( phase&0x100 ) sin_ind= ~sin_ind;
          *out= FM_SIN_TABLE[sin_ind];
        }
      else *out= OUT_0;
      break;

      // Doble sinus positiu
    case 2:
      *sign_neg= false;
      sin_ind= (uint8_t) (phase&0xFF);
      if ( phase&0x100 ) sin_ind= ~sin_ind;
      *out= FM_SIN_TABLE[sin_ind];
      break;

      // Primer i tercer quart de doble sinus positiu
    case 3:
      *sign_neg= false;
      sin_ind= (uint8_t) (phase&0xFF);
      *out= phase&0x100 ? OUT_0 : FM_SIN_TABLE[sin_ind];
      break;
      
      // Primera meitat amb sinus a doble freqüència.
    case 4:
      if ( (phase&0x200) == 0 )
        {
          // NOTA !! Utilitze taula de sinus subsamplejant
          sin_ind= ((uint8_t) (phase&0x7F))<<1;
          if ( phase&0x80 ) sin_ind= ~sin_ind;
          *out= FM_SIN_TABLE[sin_ind];
          *sign_neg= (phase&0x100)!=0;
        }
      else { *sign_neg= false; *out= OUT_0; }
      break;

      // Primera meitat amb sinus doble freqüència positiu
    case 5:
      *sign_neg= false;
      if ( (phase&0x200) == 0 )
        {
          // NOTA !! Utilitze taula de sinus subsamplejant
          sin_ind= ((uint8_t) (phase&0x7F))<<1;
          if ( phase&0x80 ) sin_ind= ~sin_ind;
          *out= FM_SIN_TABLE[sin_ind];
        }
      else *out= OUT_0;
      break;

      // Square wave
    case 6:
      *out= OUT_1;
      *sign_neg= (phase&0x200)!=0;
      break;

      // Exponencial.
    case 7:
      // NOTA!!! Fent calculs he descobert que la corba, que seria de
      // 512 steps, segueix aquest patró:
      // 0x000,0x010,0x020,0x030,.... Per tant no cal fer taula. De
      // totes maneres ficare com a valor més menut OUT_0.
      if ( (phase&0x200) == 0 )
        {
          ind_exp= phase&0x1FF;
          *sign_neg= false;
        }
      else
        {
          ind_exp= (~phase)&0x1FF;
          *sign_neg= true;
        }
      tmp= ind_exp*0x10;
      if ( tmp > OUT_0 ) tmp= OUT_0;
      *out= tmp;
      break;
    }
  
} // end fm_op_get_wave_out


static void
fm_op_clock (
             fm_op_t       *op,
             const int16_t  ph_mod // 10bit
             )
{

  int16_t phase,att,am_att,out_att,out;
  bool sign_neg;
  uint8_t w,f;
  
  
  // Incrementa phase i modula
  op->phase= (op->phase + op->pg)&0x7FFFF; // Comptador intern (19bits??)
  phase= (op->phase>>9); // Descarta precisió inferior (volem 10 bits)
  phase= (phase + ph_mod)&0x3FF; // Suma 10bits

  // Calcula atenuació EG.
  if ( ++op->eg.cc == 3 )
    {
      fm_op_eg_clock ( op );
      op->eg.cc= 0;
    }
  att= op->eg.out;

  // Aplica total level
  att+= op->tlevel;

  // Aplica atenuació KSL
  att+= op->ksl_att;

  // Aplica Amplitude Modulation (Tremolo)
  if ( op->am.enabled )
    {
      // NOTA!!! El LFO és un contador de 7bits (128 pasos) el bit
      // superior és el signe. Però en l'atenuació interpretem el
      // signe al revés.
      am_att= (int16_t) (op->am.counter&0x3F);
      if ( (op->am.counter&0x40) == 0 )
        am_att= (~am_att)&0x3F;
      am_att= FM_AM_TABLE[_fm.dam][am_att];
      att+= am_att;
    }

  // Comprova que l'atenuació no supera el màxim
  if ( att > EG_MAX_ATTENUATION )
    att= EG_MAX_ATTENUATION;

  // Obté atenuació eixida (wav + att)
  // NOTA!! out_att és 13 bits en format 5.8
  fm_op_get_wave_out ( op, phase, &out_att, &sign_neg );
  out_att= out_att + (att<<2);

  // Db a lineal
  // out és un valor de 13bits
  w= (uint8_t) (out_att>>8);
  f= (uint8_t) (out_att&0xFF);
  out= (FM_POW_TABLE[f]<<2)>>w;

  // Aplica signe (Complement a 2)
  // out és un valor de 14bits amb signe
  op->out= (int32_t) out;
  if ( sign_neg && out != 0 )
    op->out= -op->out;
  
  // Actualitza LFOs
  // --> VIB
  if ( --op->vib.cc == 0 )
    {
      ++op->vib.counter;
      op->vib.cc= FM_VIB_CC;
      // Canvis en el vib afecten a la phase.
      if ( op->vib.enabled )
        fm_op_update_pg_and_eg ( op );
    }
  // --> AM
  if ( --op->am.cc == 0 )
    {
      ++op->am.counter;
      op->am.cc= FM_AM_CC;
    }
  
} // end fm_op_clock


// Es crida per un canal cada vegada que es modifica algun paràmetre
// del canal que pot afectar a l'estat de l'operador.
static void
fm_op_update (
              fm_op_t      *op,
              fm_channel_t *chn
              )
{


  // Extrau fnum i block
  op->fnum=
    ((uint16_t) chn->regs.fnumL) |
    ((((uint16_t) chn->regs.kon_block_fnumH)&0x3)<<8);
  op->block= (chn->regs.kon_block_fnumH>>2)&0x7;

  // Actualitza.
  fm_op_update_pg_and_eg ( op );
  
} // end fm_op_update


static void
fm_op_reset (
             fm_op_t *op
             )
{

  op->keyon= false;
  op->out= 0;
  op->phase= 0;
  op->pg= 0;
  op->keycode= 0;
  op->tlevel= 0;
  op->ksl_att= 0;
  op->fnum= 0;
  op->block= 0;
  op->eg.out= EG_MAX_ATTENUATION;
  op->eg.sustain= EG_MAX_ATTENUATION;
  op->eg.cc= 0;
  op->eg.state= EG_RELEASE;
  op->eg.counter= 0;
  op->eg.ar_rate= 0;
  op->eg.dr_rate= 0;
  op->eg.rr_rate= 0;
  op->vib.enabled= false;
  op->vib.counter= 0x00;
  op->vib.cc= FM_VIB_CC;
  op->am.enabled= false;
  op->am.counter= 0x00;
  op->am.cc= FM_AM_CC;
  fm_op_set_am_vib_egt_ksr_mult ( op, 0x00 );
  fm_op_set_ar_dr ( op, 0x00 );
  fm_op_set_sl_rr ( op, 0xf0 );
  fm_op_set_ksl_tl ( op, 0x00 );
  fm_op_set_ws ( op, 0x00 );
  
} // end fm_op_reset


static void
fm_connect_channels (void)
{

  int array,i,j;


  for ( array= 0; array < 2; ++array )
    {

      // Tres primers canals (Canals que poden ser de 2 i de 4 slots).
      for ( i= 0; i < 3; ++i )
        for ( j= 0; j < 2; ++j )
          {
            _fm.channels[array][i].slots2[j]= &_fm.ops[array][i+3*j];
            _fm.channels[array][i].slots4[j]= &_fm.ops[array][i+3*j];
            _fm.channels[array][i].slots4[j+2]= &_fm.ops[array][i+3*j+3];
          }
      
      // Tres seqgüents (Canals que sols poden ser 2).
      for ( i= 3; i < 6; ++i )
        for ( j= 0; j < 2; ++j )
          {
            _fm.channels[array][i].slots2[j]= &_fm.ops[array][i+3*j+3];
            _fm.channels[array][i].slots4[j]= NULL;
            _fm.channels[array][i].slots4[j+2]= NULL;
          }

      // Tres últims (Canals que sols poden ser 2). Rhytm???????
      for ( i= 6; i < 9; ++i )
        for ( j= 0; j < 2; ++j )
          {
            _fm.channels[array][i].slots2[j]= &_fm.ops[array][i+3*j+6];
            _fm.channels[array][i].slots4[j]= NULL;
            _fm.channels[array][i].slots4[j+2]= NULL;
          }
      
    }
  
} // end fm_connect_channels


static void
fm_channel_update_ops (
                       fm_channel_t *chn
                       )
{

  int i;

  
  switch ( chn->mode )
    {
    case CHN_OP2:
      for ( i= 0; i < 2; ++i )
        fm_op_update ( chn->slots2[i], chn );
      break;
    case CHN_OP4:
      for ( i= 0; i < 4; ++i )
        fm_op_update ( chn->slots4[i], chn );
      break;
    default:
    }
  
} // end fm_channel_update_ops


static void
fm_channel_set_fnumL (
                      fm_channel_t  *chn,
                      const uint8_t  data
                      )
{

  chn->regs.fnumL= data;
  fm_channel_update_ops ( chn );
  
} // end fm_channel_set_fnumL


static void
fm_channel_set_kon_block_fnumH (
                                fm_channel_t  *chn,
                                const uint8_t  data
                                )
{
  
  int i;
  
  
  // Registre i operacions.
  chn->regs.kon_block_fnumH= data;
  fm_channel_update_ops ( chn );
  
  // KON
  switch ( chn->mode )
    {
    case CHN_OP2:
      for ( i= 0; i < 2; ++i )
        {
          if ( data&0x20 ) fm_op_keyon ( chn->slots2[i] );
          else             fm_op_keyoff ( chn->slots2[i] );
        }
      break;
    case CHN_OP4:
      for ( i= 0; i < 4; ++i )
        {
          if ( data&0x20 ) fm_op_keyon ( chn->slots4[i] );
          else             fm_op_keyoff ( chn->slots4[i] );
        }
      break;
    default:
    }
  
} // end fm_channel_set_kon_block_fnumH


static void
fm_channel_set_chd_chc_chb_cha_fb_cnt (
                                       fm_channel_t  *chn,
                                       const uint8_t  data
                                       )
{
  
  // Registre.
  chn->regs.chd_chc_chb_cha_fb_cnt= data;

  // FB
  chn->feedback= (data>>1)&0x7;

  // CHA - CHB
  chn->l= (data&0x10)!=0;
  chn->r= (data&0x20)!=0;
  if ( data&0xC0 )
    _warning ( _udata, "SB16 FM: s'han activat canals D-C" );
  
} // end fm_channel_set_chd_chc_chb_cha_fb_cnt


static int16_t
fm_channel_calc_feedback (
                          fm_channel_t *chn
                          )
{

  int16_t ret;
  int32_t tmp;
  
  
  if ( chn->feedback != 0 )
    {
      tmp= chn->fb_buf[0] + chn->fb_buf[1];
      ret= (tmp>>(10-chn->feedback))&0x3FF;
    }
  else ret= 0;
  
  return ret;
  
} // end fm_channel_calc_feedback


static void
fm_channel_clock_op2_alg0 (
                           fm_channel_t *chn
                           )
{

  // OP1
  fm_op_clock ( chn->slots2[0], fm_channel_calc_feedback ( chn ) );
  chn->fb_buf[0]= chn->fb_buf[1];
  chn->fb_buf[1]= chn->slots2[0]->out;

  // OP2
  fm_op_clock ( chn->slots2[1], OUT2PHASEMOD(chn->slots2[0]->out) );

  // OUT
  chn->out= chn->slots2[1]->out;
  
} // end fm_channel_clock_op2_alg0


static void
fm_channel_clock_op2_alg1 (
                           fm_channel_t *chn
                           )
{

  // OP1
  fm_op_clock ( chn->slots2[0], fm_channel_calc_feedback ( chn ) );
  chn->fb_buf[0]= chn->fb_buf[1];
  chn->fb_buf[1]= chn->slots2[0]->out;

  // OP2
  fm_op_clock ( chn->slots2[1], 0 );

  // OUT
  chn->out= chn->slots2[0]->out + chn->slots2[1]->out;
  
} // end fm_channel_clock_op2_alg1


static void
fm_channel_clock_op4_alg0 (
                           fm_channel_t *chn
                           )
{

  // OP1
  fm_op_clock ( chn->slots4[0], fm_channel_calc_feedback ( chn ) );
  chn->fb_buf[0]= chn->fb_buf[1];
  chn->fb_buf[1]= chn->slots4[0]->out;

  // OP2
  fm_op_clock ( chn->slots4[1], OUT2PHASEMOD(chn->slots4[0]->out) );

  // OP3
  fm_op_clock ( chn->slots4[2], OUT2PHASEMOD(chn->slots4[1]->out) );

  // OP4
  fm_op_clock ( chn->slots4[3], OUT2PHASEMOD(chn->slots4[2]->out) );

  // OUT
  chn->out= chn->slots4[3]->out;
  
} // end fm_channel_clock_op4_alg0


static void
fm_channel_clock_op4_alg1 (
                           fm_channel_t *chn
                           )
{
  
  // OP1
  fm_op_clock ( chn->slots4[0], fm_channel_calc_feedback ( chn ) );
  chn->fb_buf[0]= chn->fb_buf[1];
  chn->fb_buf[1]= chn->slots4[0]->out;

  // OP2
  fm_op_clock ( chn->slots4[1], OUT2PHASEMOD(chn->slots4[0]->out) );

  // OP3
  fm_op_clock ( chn->slots4[2], 0 );

  // OP4
  fm_op_clock ( chn->slots4[3], OUT2PHASEMOD(chn->slots4[2]->out) );

  // OUT
  chn->out= chn->slots4[1]->out + chn->slots4[3]->out;
  
} // end fm_channel_clock_op4_alg1


static void
fm_channel_clock_op4_alg2 (
                           fm_channel_t *chn
                           )
{

  // OP1
  fm_op_clock ( chn->slots4[0], fm_channel_calc_feedback ( chn ) );
  chn->fb_buf[0]= chn->fb_buf[1];
  chn->fb_buf[1]= chn->slots4[0]->out;

  // OP2
  fm_op_clock ( chn->slots4[1], 0 );

  // OP3
  fm_op_clock ( chn->slots4[2], OUT2PHASEMOD(chn->slots4[1]->out) );

  // OP4
  fm_op_clock ( chn->slots4[3], OUT2PHASEMOD(chn->slots4[2]->out) );

  // OUT
  chn->out= chn->slots4[0]->out + chn->slots4[3]->out;
  
} // end fm_channel_clock_op4_alg2


static void
fm_channel_clock_op4_alg3 (
                           fm_channel_t *chn
                           )
{

  // OP1
  fm_op_clock ( chn->slots4[0], fm_channel_calc_feedback ( chn ) );
  chn->fb_buf[0]= chn->fb_buf[1];
  chn->fb_buf[1]= chn->slots4[0]->out;

  // OP2
  fm_op_clock ( chn->slots4[1], 0 );

  // OP3
  fm_op_clock ( chn->slots4[2], OUT2PHASEMOD(chn->slots4[1]->out) );

  // OP4
  fm_op_clock ( chn->slots4[3], 0 );

  // OUT
  chn->out= chn->slots4[0]->out + chn->slots4[2]->out + chn->slots4[3]->out;
  
} // end fm_channel_clock_op4_alg3


static void
fm_channel_clock (
                  fm_channel_t *chn
                  )
{

  int alg;

  
  switch ( chn->mode )
    {
      
    case CHN_OP2:
      alg= chn->regs.chd_chc_chb_cha_fb_cnt&0x1;
      switch ( alg )
        {
        case 0: fm_channel_clock_op2_alg0 ( chn ); break;
        case 1: fm_channel_clock_op2_alg1 ( chn ); break;
        }
      break;

    case CHN_OP4:
      alg=
        ((chn->regs.chd_chc_chb_cha_fb_cnt&0x1)<<1) |
        (chn->chn_col->regs.chd_chc_chb_cha_fb_cnt&0x1)
        ;
      switch ( alg )
        {
        case 0: fm_channel_clock_op4_alg0 ( chn ); break;
        case 1: fm_channel_clock_op4_alg1 ( chn ); break;
        case 2: fm_channel_clock_op4_alg2 ( chn ); break;
        case 3: fm_channel_clock_op4_alg3 ( chn ); break;
        }
      break;

    case CHN_DISABLED:
      chn->out= 0;
      break;
      
    default:
      
    }
  
} // end fm_channel_clock


static void
fm_channel_reset (
                  fm_channel_t *chn,
                  const bool    enabled
                  )
{

  chn->out= 0;
  chn->mode= enabled ? CHN_OP2 : CHN_DISABLED;
  chn->fb_buf[0]= 0;
  chn->fb_buf[1]= 0;
  chn->feedback= 0;
  chn->l= false;
  chn->r= false;
  fm_channel_set_fnumL ( chn, 0x00 );
  fm_channel_set_kon_block_fnumH ( chn, 0x00 );
  fm_channel_set_chd_chc_chb_cha_fb_cnt ( chn, 0x00 );
  
} // end fm_channel_reset


// Actualitza estat dels canals que depèn de registres globals.
static void
fm_update_channels (void)
{

  int array,i;
  uint8_t byte;
  
  
  // Modes. NOTA!!! Açò obliga a updatejar tots els slots actius.
  if ( _fm.opl3_mode )
    {
      byte= _fm.connection_sel_reg;
      for ( array= 0; array < 2; ++array )
        {

          // Canals 0...5
          for ( i= 0; i < 3; ++i )
            {
              if ( byte&0x1 )
                {
                  _fm.channels[array][i].mode= CHN_OP4;
                  _fm.channels[array][i+3].mode= CHN_DISABLED;
                  _fm.channels[array][i].chn_col=
                    &_fm.channels[array][i+3];
                  fm_channel_update_ops ( &_fm.channels[array][i] );
                }
              else
                {
                  _fm.channels[array][i].mode= CHN_OP2;
                  fm_channel_update_ops ( &_fm.channels[array][i] );
                  _fm.channels[array][i+3].mode= CHN_OP2;
                  fm_channel_update_ops ( &_fm.channels[array][i+3] );
                }
              byte>>= 1;
            }

          // Canals 6..8
          for ( i= 6; i < 9; ++i )
            {
              _fm.channels[array][i].mode= CHN_OP2;
              fm_channel_update_ops ( &_fm.channels[array][i] );
            }
          
        }
    }
  else
    {
      // Array 0
      for ( i= 0; i < 9; ++i )
        {
          _fm.channels[0][i].mode= CHN_OP2;
          fm_channel_update_ops ( &_fm.channels[0][i] );
        }
      // Array 1
      for ( i= 0; i < 9; ++i ) _fm.channels[1][i].mode= CHN_DISABLED;
    }

} // end fm_update_channels


static void
fm_set_connection_sel (
                       const uint8_t data
                       )
{

  _fm.connection_sel_reg= data;
  fm_update_channels ();
  
} // end fm_set_connection_sel


static void
fm_set_dam_dvb_ryt_bd_sd_tom_tc_hh (
                                    const uint8_t data
                                    )
{

  _fm.dvb= (data>>6)&0x1;
  _fm.dam= (data>>7)&0x1;
  fm_update_channels ();

  if ( data&0x20 ) // RHY
    {
      PC_MSG("SB16 FM - RYT BD SD TOM TC HH");
    }
  
} // end fm_set_dam_dvb_ryt_bd_sd_tom_tc_hh


// Pot tornar -1 per a indicar que no s'esperent events.
static int
fm_cc_to_event (void)
{

  int ret,tmp;


  ret= -1;
  tmp= fm_timers_cc_to_event ();
  if ( ret == -1 || (tmp != -1 && tmp < ret) ) ret= tmp;
  
  return ret;
  
} // end fm_cc_to_event


static void
fm_reset (void)
{

  int i,j;


  // Globals.
  _fm.addr[0]= 0x00;
  _fm.addr[1]= 0x00;
  _fm.opl3_mode= false;
  _fm.dvb= 0;
  _fm.dam= 0;
  _fm.nts= 0;
  
  // Timers.
  fm_timer_reset ( &_fm.timers[0] );
  fm_timer_reset ( &_fm.timers[1] );

  // Operadors.
  for ( i= 0; i < 2; ++i )
    for ( j= 0; j < 18; ++j )
      fm_op_reset ( &_fm.ops[i][j] );
  
  // Canals. (Important fer-ho després de operadors)
  for ( i= 0; i < 2; ++i )
    for ( j= 0; j < 9; ++j )
      fm_channel_reset ( &_fm.channels[i][j], i==0 );
  
} // end fm_reset


static void
fm_init (void)
{

  static const int DIVS[4]= {2,2,2,179};

  int i;
  
  
  // Timing (Master clock 14.32Mhz).
  _fm.cc_fm_accum= 0;
  _fm.cc_accum= 0;
  assert ( PC_ClockFreq%10000 == 0 );
  _fm.cc_div= (long) (PC_ClockFreq/10000);
  _fm.cc_mul= 1432; // 14.32Mhz/10000
  for ( i= 0; i < 4; ++i )
    if ( _fm.cc_div%DIVS[i] == 0 )
      {
        _fm.cc_div/= DIVS[i];
        _fm.cc_mul/= DIVS[i];
      }
  
  // Inicialitza canals.
  fm_connect_channels ();

  // Buffer eixida. (No es reseteja=
  _fm.out.N= 0;
  _fm.out.p= 0;
  _fm.out.pss= 0.0;
  _fm.out.fss= (14320000.0/288) / 44100.0;
  
  // Reseteja.
  fm_reset ();

  // Altres.
  _fm.cc_delay_status= (int) (PC_ClockFreq * (23.0/1000000/35) + 0.5);
  
} // end fm_init


static void
fm_run_cycle (void)
{

  int i,j,narray,npos;
  int32_t l,r,val;
  fm_channel_t *chn;
  
  
  // Calcula mostres L i R
  narray= _fm.opl3_mode ? 2 : 1;
  l= r= 0;
  for ( i= 0; i < narray; ++i )
    for ( j= 0; j < 9; ++j )
      {
        chn= &_fm.channels[i][j];
        fm_channel_clock ( chn );
        /*
        val= 4*chn->out;
        if      ( val > 32767 )  val= 32767;
        else if ( val < -32768 ) val= -32768;
        */
        val= 2*chn->out;
        if      ( val > 16256 )  val= 16256;
        else if ( val < -16384 ) val= -16384;
        if ( !_fm.opl3_mode || chn->l ) l+= val;
        if ( !_fm.opl3_mode || chn->r ) r+= val;
      }
  l/= 9*narray;
  r/= 9*narray;

  // Inserta en buffer
  if ( _fm.out.N < FM_BUF_SIZE )
    {
      npos= (_fm.out.p+_fm.out.N)%FM_BUF_SIZE;
      _fm.out.l[npos]= l;
      _fm.out.r[npos]= r;
      ++_fm.out.N;
    }
  else _warning ( _udata, "SB16 FM: out buffer overflow" );
  
} // end fm_run_cycle


static void
fm_clock (
          const int cc
          )
{
  
  long tmp_cc,sample_cc;

  
  // Calcula cicles FM.
  tmp_cc= _fm.cc_accum + ((long) cc)*_fm.cc_mul;
  sample_cc= tmp_cc/_fm.cc_div;
  _fm.cc_accum= tmp_cc%_fm.cc_div;

  // Clock.
  fm_timers_clock ( sample_cc );

  // Clock síntesi FM.
  tmp_cc= sample_cc + _fm.cc_fm_accum;
  while ( tmp_cc >= 288 )
    {
      fm_run_cycle ();
      tmp_cc-= 288;
    }
  _fm.cc_fm_accum= tmp_cc;
  
} // end fm_clock


static void
fm_get_next_44_1KHz_sample (
                            int16_t *l,
                            int16_t *r
                            )
{

  // NOTA!! Faig un subsampling senzill.


  // Obté mostra
  assert ( _fm.out.pss < 1.0 );
  if ( _fm.out.N < 1 )
    {
      _warning ( _udata, "SB16 FM: FM buffer underflow (A)" );
      *l= *r= 0;
    }
  else
    {
      *l= _fm.out.l[_fm.out.p];
      *r= _fm.out.r[_fm.out.p];
    }
  
  // Incrementa
  _fm.out.pss+= _fm.out.fss;
  while ( _fm.out.pss >= 1.0 )
    {
      _fm.out.pss-= 1.0;
      if ( _fm.out.N > 0 )
        {
          _fm.out.p= (_fm.out.p+1)%FM_BUF_SIZE;
          --_fm.out.N;
        }
      else _warning ( _udata, "SB16 FM: FM buffer underflow (B)" );
        
    }
  
} // end fm_get_next_44_1KHz_sample


// DSP /////////////////////////////////////////////////////////////////////////

static void
dsp_update_format (void)
{
  
  double freq;
  
  /*
  // Calcula freqüència.
  freq= 65536.0 - ((((uint32_t) _dsp.format.timec_reg)<<8)|0xff);
  freq= 256000000.0 / freq;
  if ( !_dsp.format.mono ) freq/= 2;
  */
  freq= _dsp.format.freq;
  if ( freq < 4000 )
    {
      _warning ( _udata,
                 "SB16 DSP: freqüència molt menuda (%g), es fixa a 4000",
                 freq );
      freq= 4000;
    }
  else if ( freq > 44100 )
    {
      _warning ( _udata,
                 "SB16 DSP: freqüència molt gran (%g), es fixa a 44100",
                 freq );
      freq= 44100;
    }
  
  // Ratio.
  _dsp.format.ratio= freq/44100.0;
  
} // end dsp_update_format


static void
dsp_reset (void)
{

  _dsp.test_reg= 0x00;
  _dsp.state= DSP_WAIT_CMD;
  _dsp.reset_flag= false;
  _dsp.pcspeaker_on= false;
  _dsp.block_transfer_size= 0;
  //_dsp.format.timec_reg= 0x00;
  _dsp.format.freq= 44100; // ????????????
  _dsp.format.ifreq= 44100; // ????????????
  _dsp.format.mono= true;
  _dsp.format.type= DSP_FORMAT_U8;
  _dsp.adpcm.started= false;
  _dsp.adpcm.step= 0;
  _dsp.adpcm.current= 0;
  _dsp.out.p= 0;
  _dsp.out.N= 0;
  _dsp.in.empty= true;
  _dsp.render.p= 0;
  _dsp.render.N= 0;
  _dsp.render.pss= 0.0;
  _dsp.render.stop_dma= false;
  if ( _dsp.dma.state != DSP_DMA_NONE && !_dsp.dma.paused )
    PC_dma_dreq ( 1, false );
  _dsp.dma.dreq= false;
  _dsp.dma.state= DSP_DMA_NONE;
  _dsp.dma.irq_on= false;
  _dsp.dma.counter= 0;
  _dsp.dma.init_counter= 0;
  _dsp.dma.paused= false;
  _dsp.dma.waiting_l_sample= true;
  _dsp.dma.l_sample= 0;
  if ( _dsp.dma16.state != DSP_DMA16_NONE && !_dsp.dma16.paused )
    PC_dma_dreq ( 5, false );
  _dsp.dma16.dreq= false;
  _dsp.dma16.state= DSP_DMA16_NONE;
  _dsp.dma16.irq_on= false;
  _dsp.dma16.counter= 0;
  _dsp.dma16.init_counter= 0;
  _dsp.dma16.paused= false;
  _dsp.dma16.waiting_l_sample= true;
  _dsp.dma16.l_sample= 0;
  //if ( _dsp.dma.irq_on ) PC_ic_irq ( 5, false );
  PC_ic_irq ( 5, false );
  dsp_update_format ();
  
} // end dsp_reset


static void
dsp_init (void)
{

  _dsp.dma.state= DSP_DMA_NONE;
  _dsp.dma.in_clock= false;
  _dsp.dma.irq_on= false;
  _dsp.dma16.state= DSP_DMA16_NONE;
  _dsp.dma16.in_clock= false;
  _dsp.dma16.irq_on= false;
  dsp_reset ();
  
} // end dsp_init


static void
dsp_dma_update_dreq (void)
{

  bool new_dreq;


  new_dreq=
    _dsp.dma.state!=DSP_DMA_NONE &&
    !_dsp.dma.paused &&
    !_dsp.render.stop_dma;
  if ( new_dreq != _dsp.dma.dreq )
    {
      PC_dma_dreq ( 1, new_dreq );
      _dsp.dma.dreq= new_dreq;
    }
  
} // end dsp_dma_update_dreq


static void
dsp_dma16_update_dreq (void)
{
  
  bool new_dreq;
  

  new_dreq=
    _dsp.dma16.state!=DSP_DMA16_NONE &&
    !_dsp.dma16.paused &&
    !_dsp.render.stop_dma;
  if ( new_dreq != _dsp.dma16.dreq )
    {
      PC_dma_dreq ( 5, new_dreq );
      _dsp.dma16.dreq= new_dreq;
    }
  
} // end dsp_dma16_update_dreq


static void
dsp_dma_set_irq (
                 const bool val
                 )
{
  
  if ( val != _dsp.dma.irq_on )
    PC_ic_irq ( 5, val );
  /* // NOTA!! No tinc clar si el IRQ és escaló o no.
  if ( val && !_dsp.dma.irq_on )
    {
      PC_ic_irq ( 5, true );
      PC_ic_irq ( 5, false );
    }
  */
  _dsp.dma.irq_on= val;
  
} // end dsp_dma_set_irq


static void
dsp_dma16_set_irq (
                   const bool val
                   )
{
  
  if ( val != _dsp.dma16.irq_on )
    PC_ic_irq ( 5, val );
  /* // NOTA!! No tinc clar si el IRQ és escaló o no.
  if ( val && !_dsp.dma.irq_on )
    {
      PC_ic_irq ( 5, true );
      PC_ic_irq ( 5, false );
    }
  */
  _dsp.dma16.irq_on= val;
  
} // end dsp_dma16_set_irq


static void
dsp_dma_init (
              const int mode,
              const int counter
              )
{

  /* MOLT COMÚ
  if ( _dsp.dma.state != DSP_DMA_NONE )
    _warning ( _udata,
               "SB16 DSP: s'ha iniciat el DMA mentre està actiu" );
  */
  _dsp.dma.state= mode;
  _dsp.dma.counter= counter;
  _dsp.dma.init_counter= counter;
  _dsp.dma.paused= false;
  _dsp.dma.waiting_l_sample= true;
  dsp_dma_update_dreq ();
  
} // end dsp_dma_init


static void
dsp_dma16_init (
                const int mode,
                const int counter
                )
{

  /* MOLT COMÚ
  if ( _dsp.dma.state != DSP_DMA_NONE )
    _warning ( _udata,
               "SB16 DSP: s'ha iniciat el DMA mentre està actiu" );
  */
  _dsp.dma16.state= mode;
  _dsp.dma16.counter= counter;
  _dsp.dma16.init_counter= counter;
  _dsp.dma16.paused= false;
  _dsp.dma16.waiting_l_sample= true;
  dsp_dma16_update_dreq ();
  
} // end dsp_dma16_init


static void
dsp_dma_finish (void)
{
  
  assert ( _dsp.dma.state != DSP_DMA_NONE );
  _dsp.dma.state= DSP_DMA_NONE;
  _dsp.dma.paused= false;
  dsp_dma_update_dreq ();
  dsp_dma_set_irq ( true );
  
} // end dsp_dma_finish


static void
dsp_dma16_finish (void)
{

  assert ( _dsp.dma16.state != DSP_DMA16_NONE );
  _dsp.dma16.state= DSP_DMA16_NONE;
  _dsp.dma16.paused= false;
  dsp_dma16_update_dreq ();
  dsp_dma16_set_irq ( true );
  
} // end dsp_dma16_finish


static void
dsp_dma_pause (void)
{

  if ( _dsp.dma.state == DSP_DMA_NONE )
    {
      /* COMÚ
      _warning ( _udata,
                 "SB16 DSP: s'ha intentat pausar el DMA però no hi"
                 " ha cap operació de DMA en marxa" );
      */
      return;
    }
  if ( _dsp.dma.paused )
    {
      /* COMÚ
      _warning ( _udata,
                 "SB16 DSP: s'ha intentat pausar el DMA però ja"
                 " està pausat" );
      */
      return;
    }

  // Pausa DMA
  _dsp.dma.paused= true;
  dsp_dma_update_dreq ();

  // Força que es buide el buffer. NO ESTÀ GENS CLAR
  /* 
  _dsp.render.N= 0;
  _dsp.render.p= 0;
  _dsp.render.pss= 0.0;
  _dsp.render.stop_dma= false;
  */
  
} // end dsp_dma_pause


static void
dsp_dma16_pause (void)
{

  if ( _dsp.dma16.state == DSP_DMA16_NONE )
    {
      /* COMÚ
      _warning ( _udata,
                 "SB16 DSP: s'ha intentat pausar el DMA però no hi"
                 " ha cap operació de DMA en marxa" );
      */
      return;
    }
  if ( _dsp.dma16.paused )
    {
      /* COMÚ
      _warning ( _udata,
                 "SB16 DSP: s'ha intentat pausar el DMA però ja"
                 " està pausat" );
      */
      return;
    }

  // Pausa DMA
  _dsp.dma16.paused= true;
  dsp_dma16_update_dreq ();

  // Força que es buide el buffer. NO ESTÀ GENS CLAR
  /* 
  _dsp.render.N= 0;
  _dsp.render.p= 0;
  _dsp.render.pss= 0.0;
  _dsp.render.stop_dma= false;
  */
  
} // end dsp_dma16_pause


static void
dsp_dma_continue (void)
{
  
  if ( _dsp.dma.state == DSP_DMA_NONE )
    {
      /*
      _warning ( _udata,
                 "SB16 DSP: s'ha intentat reiniciar el DMA però no hi"
                 " ha cap operació de DMA en marxa" );
      */
      return;
    }
  if ( !_dsp.dma.paused )
    {
      /*
      _warning ( _udata,
                 "SB16 DSP: s'ha intentat reiniciar el DMA però ja"
                 " està en marxa" );
      */
      return;
    }
  _dsp.dma.paused= false;
  dsp_dma_update_dreq ();
  
} // end dsp_dma_continue


static void
dsp_dma16_continue (void)
{
  
  if ( _dsp.dma16.state == DSP_DMA16_NONE )
    {
      /*
      _warning ( _udata,
                 "SB16 DSP: s'ha intentat reiniciar el DMA16 però no hi"
                 " ha cap operació de DMA en marxa" );
      */
      return;
    }
  if ( !_dsp.dma16.paused )
    {
      /*
      _warning ( _udata,
                 "SB16 DSP: s'ha intentat reiniciar el DMA16 però ja"
                 " està en marxa" );
      */
      return;
    }
  _dsp.dma16.paused= false;
  dsp_dma16_update_dreq ();
  
} // end dsp_dma16_continue


static void
dsp_out_add (
             const uint8_t data
             )
{

  if ( _dsp.out.N >= DSP_OUT_BUF_SIZE )
    _warning ( _udata,
               "SB16 DSP: output buffer overflow, s'ignora: %02X",
               data );
  else
    {
      _dsp.out.v[(_dsp.out.p+_dsp.out.N)%DSP_OUT_BUF_SIZE]= data;
      ++_dsp.out.N;
    }
  
} // end dsp_out_add


static void
dsp_run_command (void)
{

  int counter;
  uint16_t tmp16;
  double tmpd;
  
  
  switch ( _dsp.in.cmd )
    {
      
    case 0x14: // 8-bit single-cycle DMA mode digitized sound output
      _dsp.format.type= DSP_FORMAT_U8;
      counter= (int)
        ((((uint32_t) _dsp.in.args[0]) | ((uint32_t) _dsp.in.args[1])<<8) + 1);
      dsp_dma_init ( DSP_DMA_SINGLE, counter );
      break;

    case 0x1c: // 8-bit auto-init DMA mode digitized sound output
      _dsp.format.type= DSP_FORMAT_U8;
      dsp_dma_init ( DSP_DMA_AUTO_INIT, _dsp.block_transfer_size );
      break;
      
    case 0x40: // Set digitized sound transfer Time Constant
      //_dsp.format.timec_reg= _dsp.in.args[0];
      tmpd= 65536.0 - ((((uint32_t) _dsp.in.args[0])<<8)|0xff);
      tmpd= 256000000.0 / tmpd;
      //tmpd= 256.0 - _dsp.in.args[0]; // <-- Alternativa lleugerament diferent.
      //tmpd= 1000000.0/tmpd;
      if ( !_dsp.format.mono ) tmpd/= 2;
      _dsp.format.freq= tmpd;
      dsp_update_format ();
      break;
    case 0x41: // Set digitized sound output sampling rate
      tmp16= (((uint16_t) _dsp.in.args[0])<<8) | ((uint16_t) _dsp.in.args[1]);
      _dsp.format.freq= (double) tmp16;
      dsp_update_format ();
      break;
    case 0x42: // Set digitized sound input sampling rate
      tmp16= (((uint16_t) _dsp.in.args[0])<<8) | ((uint16_t) _dsp.in.args[1]);
      _dsp.format.ifreq= (double) tmp16;
      dsp_update_format ();
      break;
      
    case 0x45: // ¿¿??
      _warning ( _udata, "SB16 DSP: unsupported command 0x45" );
      break;
      
    case 0x48: // Set DSP block transfer size
      _dsp.block_transfer_size= (int)
        ((((uint32_t) _dsp.in.args[0]) | ((uint32_t) _dsp.in.args[1])<<8) + 1);
      break;
      
    case 0x74: // 8-bit to 4-bit ADPCM single-cycle DMA mode digitized
               // sound output without reference byte
      _dsp.format.type= DSP_FORMAT_ADPCM8_4;
      counter= (int)
        ((((uint32_t) _dsp.in.args[0]) | ((uint32_t) _dsp.in.args[1])<<8) + 1);
      _dsp.adpcm.started= true;
      dsp_dma_init ( DSP_DMA_SINGLE, counter );
      break;
    case 0x75: // 8-bit to 4-bit ADPCM single-cycle DMA mode digitized
               // sound output with reference byte
      _dsp.format.type= DSP_FORMAT_ADPCM8_4;
      counter= (int)
        ((((uint32_t) _dsp.in.args[0]) | ((uint32_t) _dsp.in.args[1])<<8) + 1);
      _dsp.adpcm.started= false;
      _dsp.adpcm.step= 0;
      _dsp.adpcm.current= 0;
      dsp_dma_init ( DSP_DMA_SINGLE, counter );
      break;

    case 0xb0 ... 0xbf:
      if ( _dsp.in.cmd&0x1 )
        _warning ( _udata, "SB16 DSP: command not supported %02X",
                   _dsp.in.cmd );
      else
        {
          if ( (_dsp.in.cmd&0x08) != 0 )
            {
              // Micro ??????????
              PC_MSGF("sound_blaster16.c - "
                      "dsp_run_command %02X (A/D)\n",_dsp.in.cmd);
              exit(EXIT_FAILURE);
            }
          if ( (_dsp.in.cmd&0x02) != 0 )
            {
              // NOTA!! La FIFO és com un buffer intermig perquè no
              // calga que les mostres es proporcionen en el temps que
              // toca, però no especifica els detalls d'eixe
              // buffer. He decidit no fer res perquè l'emulador ja
              // utilitza un buffer tipus FIFO de normal per a
              // renderitzar.
              /*
              // No sé com va el FIFO....
              printf("[CAL IMPLEMENTAR] sound_blaster16.c - "
                     "dsp_run_command %02X (FIFO)\n",_dsp.in.cmd);
              exit(EXIT_FAILURE);
              */
            }
          counter= (int)
            ((((uint32_t) _dsp.in.args[1]) |
              ((uint32_t) _dsp.in.args[2])<<8) + 1);
          switch ( (_dsp.in.args[0]>>4)&0x3 )
            {
            case 0: _dsp.format.type= DSP_FORMAT_U16_MONO; break;
            case 1: _dsp.format.type= DSP_FORMAT_S16_MONO; break;
            case 2: _dsp.format.type= DSP_FORMAT_U16_STEREO; break;
            case 3: _dsp.format.type= DSP_FORMAT_S16_STEREO; break;
            }
          dsp_dma16_init ( (_dsp.in.cmd&0x04)==0 ?
                           DSP_DMA16_SINGLE : DSP_DMA16_AUTO_INIT,
                           counter );
        }
      break;
    case 0xc0 ... 0xcf:
      if ( _dsp.in.cmd&0x1 )
        _warning ( _udata, "SB16 DSP: command not supported %02X",
                   _dsp.in.cmd );
      else
        {
          if ( (_dsp.in.cmd&0x08) != 0 )
            {
              // Micro ??????????
              PC_MSGF("sound_blaster16.c - "
                      "dsp_run_command %02X (A/D)\n",_dsp.in.cmd);
              exit(EXIT_FAILURE);
            }
          if ( (_dsp.in.cmd&0x02) != 0 )
            {
              // NOTA!! La FIFO és com un buffer intermig perquè no
              // calga que les mostres es proporcionen en el temps que
              // toca, però no especifica els detalls d'eixe
              // buffer. He decidit no fer res perquè l'emulador ja
              // utilitza un buffer tipus FIFO de normal per a
              // renderitzar.
              /*
              // No sé com va el FIFO....
              printf("[CAL IMPLEMENTAR] sound_blaster16.c - "
                     "dsp_run_command %02X (FIFO)\n",_dsp.in.cmd);
              exit(EXIT_FAILURE);
              */
            }
          counter= (int)
            ((((uint32_t) _dsp.in.args[1]) |
              ((uint32_t) _dsp.in.args[2])<<8) + 1);
          switch ( (_dsp.in.args[0]>>4)&0x3 )
            {
            case 0: _dsp.format.type= DSP_FORMAT_U8_MONO; break;
            case 1: _dsp.format.type= DSP_FORMAT_S8_MONO; break;
            case 2: _dsp.format.type= DSP_FORMAT_U8_STEREO; break;
            case 3: _dsp.format.type= DSP_FORMAT_S8_STEREO; break;
              // EN STEREO *2 ???
            }
          dsp_dma_init ( (_dsp.in.cmd&0x04)==0 ?
                         DSP_DMA_SINGLE : DSP_DMA_AUTO_INIT,
                         counter );
        }
      break;
      
    case 0xd0: // Pause 8-bit DMA mode digitized sound I/O
      dsp_dma_pause ();
      break;
    case 0xd1: // Turn on speaker
      _dsp.pcspeaker_on= true;
      break;

    case 0xd3: // Turn off speaker
      _dsp.pcspeaker_on= false;
      break;
    case 0xd4: // Continue 8-bit DMA mode digitized sound I/O
      dsp_dma_continue ();
      break;
    case 0xd5: // Pause 16-bit DMA mode digitized sound I/O
      dsp_dma16_pause ();
      break;
    case 0xd6: // Continue 16-bit DMA mode digitized sound I/O
      dsp_dma16_continue ();
      break;

    case 0xd9: // Exit 16-bit auto-init DMA mode digitized sound I/O
      _dsp.dma16.state= DSP_DMA16_AUTO_INIT_FINISH;
      break;
    case 0xda: // Exit 8-bit auto-init DMA mode digitized sound I/O
      _dsp.dma.state= DSP_DMA_AUTO_INIT_FINISH;
      break;
      
    case 0xe0: // DSP Identification. Comandament sols suportat per
               // SB2.0 (L'utilitzen per a identificar si és una
               // SB2.0. He decidit implementar-lo encara que siga una
               // SB16.
      dsp_out_add ( _dsp.in.args[0]^0xff ); 
      //dsp_out_add ( _dsp.in.args[0] ); // <-- Podria considerar fer
                                         //     que fallara?
      break;

    case 0xe1: // Get DSP version number (V4.4)
      dsp_out_add ( 0x04 );
      dsp_out_add ( 0x04 ); // <-- Abans 0x0d (V4.13) (No ens flipem
                            //     que no tinc ASP ni altres coses)
      break;
    case 0xe2: // DMA Identification.
      // Aparentment, és un comandament estrany que inicia una
      // transferència DMA amb les dades que reb, modificant-los amb
      // una XOR. L'única documentació que he trobat és el codi de
      // dosbox.
      _warning ( _udata, "SB16 DSP: DMA identification (E2)"
                 " no implementat (DATA:%02X)", _dsp.in.args[0]);
      break;

    case 0xe4: // Write Test Register
      _dsp.test_reg= _dsp.in.args[0];
      break;
      
    case 0xe7: // S'empra en les versions ESS per a detectar-les.
      break;
    case 0xe8: // Read Test Register
      dsp_out_add ( _dsp.test_reg );
      break;

    case 0xf2: // IRQ Request, 8-bit
      dsp_dma_set_irq ( true );
      break;
      
    default:
      PC_MSGF ( "SB16 DSP - unknown command %02X (B)",
                _dsp.in.cmd );
      exit ( EXIT_FAILURE );
    }
  
} // end dsp_run_command


static void
dsp_write (
           const uint8_t val
           )
{
  
  // _dsp.in.empty= false; <-- NO CAL, HO FAIG IMMEDIATAMENT.
  
  // Llig commandament i arguments.
  switch ( _dsp.state )
    {
      
    case DSP_WAIT_CMD:
      _dsp.in.cmd= val;
      switch ( _dsp.in.cmd )
        {
          // 0 args.
        case 0x1c:
        case 0x45:
        case 0xd0:
        case 0xd1:
        case 0xd3:
        case 0xd4:
        case 0xd5:
        case 0xd6:
        case 0xd9:
        case 0xda:
        case 0xe1:
        case 0xe7:
        case 0xe8:
        case 0xf2:
          _dsp.state= DSP_READY;
          break;
          
          // 1 arg.
        case 0x40:
        case 0xe0:
        case 0xe2:
        case 0xe4:
          _dsp.state= DSP_WAIT_ARG1;
          break;
          
          // 2 arg
        case 0x14:
        case 0x41:
        case 0x42:
        case 0x48:
        case 0x74:
        case 0x75:
          _dsp.state= DSP_WAIT_ARG1_2;
          break;

          // 3 arg
        case 0xb0 ... 0xbf:
        case 0xc0 ... 0xcf:
          _dsp.state= DSP_WAIT_ARG1_3;
          break;
          
        default:
          PC_MSGF("SB16 DSP - unknown command %02X",val);
          exit ( EXIT_FAILURE );
        }
      break;
      
    case DSP_WAIT_ARG1:
      _dsp.in.args[0]= val;
      _dsp.state= DSP_READY;
      break;

    case DSP_WAIT_ARG1_2:
      _dsp.in.args[0]= val;
      _dsp.state= DSP_WAIT_ARG2_2;
      break;

    case DSP_WAIT_ARG2_2:
      _dsp.in.args[1]= val;
      _dsp.state= DSP_READY;
      break;

    case DSP_WAIT_ARG1_3:
      _dsp.in.args[0]= val;
      _dsp.state= DSP_WAIT_ARG2_3;
      break;

    case DSP_WAIT_ARG2_3:
      _dsp.in.args[1]= val;
      _dsp.state= DSP_WAIT_ARG3_3;
      break;

    case DSP_WAIT_ARG3_3:
      _dsp.in.args[2]= val;
      _dsp.state= DSP_READY;
      break;
      
    default:
      printf ( "%s - dsp_write - WTF!\n", __FILE__ );
      exit ( EXIT_FAILURE );
    }
  
  // Executa si es pot.
  if ( _dsp.state == DSP_READY )
    {
      dsp_run_command ();
      _dsp.state= DSP_WAIT_CMD;
    }
  
} // end dsp_write


static void
dsp_render_buf_add (
                    const int16_t l,
                    const int16_t r
                    )
{

  // NOTA!!! He decidit ficar com a grandària del buffer de
  // renderitzat 2*PC_AUDIO_BUFFER_SIZE. Aquest últim és la grandària
  // que utilitza el simulador. La idea és parar el DMA cada vegada
  // que hi hasca prou mostres per a l'emulador. El segon buffer és
  // per a les restes, he calculat que una mostra a freqüencia 4000
  // necessitaria ~12 mostres en el buffer. Per tant és important que
  // PC_AUDIO_BUFFER_SIZE siga > 12.
  
  int npos;

  
  if ( _dsp.render.N == PC_AUDIO_BUFFER_SIZE*2 )
    {
      _warning ( _udata, "SB16 DSP: render buffer overflow, es "
                 "descarten (L:%d,R:%d)", l, r );
      return;
    }
  npos= (_dsp.render.p + _dsp.render.N)%(PC_AUDIO_BUFFER_SIZE*2);
  _dsp.render.l[npos]= l;
  _dsp.render.r[npos]= r;
  ++_dsp.render.N;

  // Comprova si cal parar o no el DMA
  if ( _dsp.render.N >= PC_AUDIO_BUFFER_SIZE && !_dsp.render.stop_dma )
    {
      _dsp.render.stop_dma= true;
      dsp_dma_update_dreq ();
      dsp_dma16_update_dreq ();
    }
  
} // end dsp_render_buf_add


static void
dsp_render_resample_sample (
                            const int16_t l,
                            const int16_t r
                            )
{
  
  assert ( _dsp.render.pss < 1.0 );
  while ( _dsp.render.pss < 1.0 )
    {
      dsp_render_buf_add ( l, r );
      _dsp.render.pss+= _dsp.format.ratio;
    }
  while (_dsp.render.pss >= 1.0 )
    _dsp.render.pss-= 1.0;
  
} // end dsp_render_resample_sample


static void
dsp_adpcm_8bit_4_decode (
                         const uint8_t nibble
                         )
{
  
  bool inc;
  int delta,diff;
  int16_t sample;
  int32_t tmp;
  
  
  // Prepara
  inc= (nibble&0x8)==0;
  delta= nibble&0x7;
  diff= ((int) delta)<<(7 + _dsp.adpcm.step);
  
  // Calcula mostra
  tmp= _dsp.adpcm.current + (inc ? diff : -diff);
  if      ( tmp > 16256 )  tmp= 16256;
  else if ( tmp < -16384 ) tmp= -16384;
  
  // Actualitza
  _dsp.adpcm.current= tmp;
  if ( delta >= 5 && _dsp.adpcm.step < 3 )
    ++_dsp.adpcm.step;
  else if ( delta == 0 && _dsp.adpcm.step > 0 )
    --_dsp.adpcm.step;
  
  // Renderitza
  sample= (int16_t) tmp;
  dsp_render_resample_sample ( sample, sample );
  
} // end dsp_adpcm_8bit_4_decode


static void
dsp_adpcm_8bit_4 (
                  const uint8_t data
                  )
{
  
  // Vaig a fer-ho entre -16384 i 16256
  if ( _dsp.adpcm.started )
    {
      dsp_adpcm_8bit_4_decode ( data>>4 );
      dsp_adpcm_8bit_4_decode ( data&0xF );
    }
  else
    {
      _dsp.adpcm.current= ((int32_t) ((int8_t) data))<<7;
      _dsp.adpcm.started= true;
    }
  
} // end dsp_adpcm_8bit_4


static void
dsp_dma_write (
               const uint8_t data
               )
{

  int16_t sample;
  
  
  // Filtro inicial.
  switch ( _dsp.dma.state )
    {

      // Estats a ignorar
    case DSP_DMA_NONE:
      _warning ( _udata, "SB16 DSP: s'ha rebut una transferència"
                 " en estat de DMA: %d (data: %02X)",
                 _dsp.dma.state, data );
      return;
      break;

      // Acceptats
    case DSP_DMA_SINGLE:
    case DSP_DMA_AUTO_INIT:
    case DSP_DMA_AUTO_INIT_FINISH:
      break;

      // Cal implementar.
    default:
      PC_MSGF ( "SB16 DSP - dsp_dma_write - state:%d",
                _dsp.dma.state );
      exit ( EXIT_FAILURE );
    }

  // Comprova queden valors per llegir.
  if ( _dsp.dma.counter == 0 )
    {
      _warning ( _udata, "SB16 DSP: s'ha rebut una transferència (DATA:%02X)"
                 " però no s'esperen més valors", data );
      return;
    }
  
  // Renderitza
  switch ( _dsp.format.type )
    {
      
    case DSP_FORMAT_U8:
      if ( !_dsp.format.mono )
        {
          PC_MSG ( "SB16 DSP - dsp_dma_write - estéreo" );
          exit ( EXIT_FAILURE );
        }
      sample= (int16_t) (((int32_t) (((uint32_t) data)<<8)) - 0x8000);
      dsp_render_resample_sample ( sample, sample );
      break;
    case DSP_FORMAT_U8_STEREO:
      // He de comprobar que _dsp.format.mono coincideix???? No ho tinc clar
      sample= (int16_t) (((int32_t) (((uint32_t) data)<<8)) - 0x8000);
      if ( _dsp.dma.waiting_l_sample )
        _dsp.dma.l_sample= sample;
      else
        dsp_render_resample_sample ( _dsp.dma.l_sample, sample );
      _dsp.dma.waiting_l_sample= !_dsp.dma.waiting_l_sample;
      break;
    case DSP_FORMAT_U8_MONO:
      sample= (int16_t) (((int32_t) (((uint32_t) data)<<8)) - 0x8000);
      dsp_render_resample_sample ( sample, sample );
      break;
      
    case DSP_FORMAT_ADPCM8_4:
      if ( !_dsp.format.mono )
        {
          PC_MSG ( "SB16 DSP - dsp_dma_write"
                   " - estéreo ADPCM8_4" );
          exit ( EXIT_FAILURE );
        }
      dsp_adpcm_8bit_4 ( data );
      break;
      
    default:
      PC_MSGF ( "SB16 DSP - dsp_dma_write - format:%d",
                _dsp.format.type );
      exit ( EXIT_FAILURE );
    }

  // Decremente
  --_dsp.dma.counter;
  if ( _dsp.dma.counter == 0 && _dsp.dma.state == DSP_DMA_AUTO_INIT )
    {
      _dsp.dma.counter= _dsp.dma.init_counter;
      dsp_dma_set_irq ( true );
    }
  else if ( _dsp.dma.counter == 0 ) dsp_dma_finish ();
  
} // end dsp_dma_write


static void
dsp_dma16_write (
                 const uint16_t data
                 )
{
  
  int16_t sample;
  
  
  // Filtro inicial.
  switch ( _dsp.dma16.state )
    {

      // Estats a ignorar
    case DSP_DMA_NONE:
      _warning ( _udata, "SB16 DSP: s'ha rebut una transferència"
                 " en estat de DMA16: %d (data: %04X)",
                 _dsp.dma16.state, data );
      return;
      break;

      // Acceptats
    case DSP_DMA16_SINGLE:
    case DSP_DMA16_AUTO_INIT:
    case DSP_DMA16_AUTO_INIT_FINISH:
      break;

      // Cal implementar.
    default:
      PC_MSGF ( "SB16 DSP - dsp_dma16_write - state:%d",
                _dsp.dma16.state );
      exit ( EXIT_FAILURE );
    }
  
  // Comprova queden valors per llegir.
  if ( _dsp.dma16.counter == 0 )
    {
      _warning ( _udata, "SB16 DSP: s'ha rebut una transferència (DATA:%04X)"
                 " però no s'esperen més valors", data );
      return;
    }
  
  // Renderitza
  switch ( _dsp.format.type )
    {
      
    case DSP_FORMAT_S16_STEREO:
      // He de comprobar que _dsp.format.mono coincideix???? No ho tinc clar
      sample= (int16_t) data;
      if ( _dsp.dma16.waiting_l_sample )
        _dsp.dma16.l_sample= sample;
      else
        dsp_render_resample_sample ( _dsp.dma16.l_sample, sample );
      _dsp.dma16.waiting_l_sample= !_dsp.dma16.waiting_l_sample;
      break;
    case DSP_FORMAT_S16_MONO:
      // He de comprobar que _dsp.format.mono coincideix???? No ho tinc clar
      sample= (int16_t) data;
      dsp_render_resample_sample ( sample, sample );
      break;
      
    default:
      PC_MSGF ( "SB16 DSP - dsp_dma16_write - format:%d",
               _dsp.format.type );
      exit ( EXIT_FAILURE );
    }

  // Decremente
  --_dsp.dma16.counter;
  if ( _dsp.dma16.counter == 0 && _dsp.dma16.state == DSP_DMA16_AUTO_INIT )
    {
      _dsp.dma16.counter= _dsp.dma16.init_counter;
      dsp_dma16_set_irq ( true );
    }
  else if ( _dsp.dma16.counter == 0 ) dsp_dma16_finish ();
  
} // end dsp_dma16_write


static void
dsp_get_next_44_1KHz_sample (
                             int16_t *l,
                             int16_t *r
                             )
{

  
  if ( _dsp.render.N > 0 )
    {
      
      *l= _dsp.render.l[_dsp.render.p];
      *r= _dsp.render.r[_dsp.render.p];
      _dsp.render.p= (_dsp.render.p+1)%(PC_AUDIO_BUFFER_SIZE*2);
      --_dsp.render.N;
      
      // Comprova si cal reiniciar o no el DMA
      // NOTA!! No sé si es bona idea que el stop_dma siga el mateix
      // per a 16bit i 8bit.
      if ( _dsp.render.N < PC_AUDIO_BUFFER_SIZE && _dsp.render.stop_dma )
        {
          _dsp.render.stop_dma= false;
          dsp_dma_update_dreq ();
          dsp_dma16_update_dreq ();
        }
    }
  else
    {
      *l= *r= 0;
      _dsp.render.pss= 0.0; // Reseteja
    }
  
} // end dsp_get_next_44_1KHz_sample


// MIXER ///////////////////////////////////////////////////////////////////////
static void
mixer_reset (void)
{

  _mixer.addr= 0x00;
  _mixer.mic_vol= 0x00; // -62dB
  _mixer.midi_vol_l= 0x18; // -14dB
  _mixer.midi_vol_r= 0x18; // -14dB
  _mixer.master_vol_l= 0x18; // -14dB
  _mixer.master_vol_r= 0x18; // -14dB
  _mixer.voice_vol_l= 0x18; // -14dB
  _mixer.voice_vol_r= 0x18; // -14dB
  _mixer.line_vol_l= 0x00; // -62dB
  _mixer.line_vol_r= 0x00; // -62dB
  _mixer.cd_vol_l= 0x00; // -62dB
  _mixer.cd_vol_r= 0x00; // -62dB
  _mixer.input_gain_l= 0x0; // 0dB
  _mixer.input_gain_r= 0x0; // 0dB
  _mixer.output_gain_l= 0x0; // 0dB
  _mixer.output_gain_r= 0x0; // 0dB
  _mixer.treble_l= 0x8; // 0dB
  _mixer.treble_r= 0x8; // 0dB
  _mixer.bass_l= 0x8; // 0dB
  _mixer.bass_r= 0x8; // 0dB
  _mixer.out_switches= 0x1f;
  _mixer.in_switches_l= 0x15;
  _mixer.in_switches_r= 0x0B;
  _mixer.pc_speaker_vol= 0x0; // -18dB
  _mixer.agc_on= true;
  
} // end mixer_reset


static void
mixer_init (void)
{
  
  mixer_reset ();
  
} // end mixer_init


static uint8_t
mixer_read_data (
                 const uint8_t addr
                 )
{

  uint8_t ret;

  
  switch ( addr )
    {

    case 0x00: // Reset mixer 
      ret= 0x00; // ??? He de tornar açò o l'últim valor guardat?
      break;

    case 0x02: // No s'empra en SB16 (SBPro)
      ret= 0xff;
      break;

    case 0x04: // Voice volume.L / Voice volume.R (Versió 4bit)
      ret=
        (((_mixer.voice_vol_l>>1)&0xf)<<4) |
        ((_mixer.voice_vol_r>>1)&0xf)
        ;
      break;
      
    case 0x0a: // Mic volume (versió 3bit)
      ret= (_mixer.mic_vol>>2)&0x7;
      break;

    case 0x0c: // No s'empra en SB16 (SBPro)
      ret= 0xff;
      break;
      
    case 0x0e: // No s'empra en SB16 (SBPro)
      ret= 0xff;
      break;
      
    case 0x22: // Master volume.L / Master volume.R (Versió 4bit)
      ret=
        (((_mixer.master_vol_l>>1)&0xf)<<4) |
        ((_mixer.master_vol_r>>1)&0xf)
        ;
      break;
      
    case 0x26: // MIDI volume.L / MIDI volume.R (Versió 4bit)
      ret=
        (((_mixer.midi_vol_l>>1)&0xf)<<4) |
        ((_mixer.midi_vol_r>>1)&0xf)
        ;
      break;
      
    case 0x28: // CD volume.L / CD volume.R (Versió 4bit)
      ret=
        (((_mixer.cd_vol_l>>1)&0xf)<<4) |
        ((_mixer.cd_vol_r>>1)&0xf)
        ;
      break;

    case 0x2e: // Line volume.L / Line volume.R (Versió 4bit)
      ret=
        (((_mixer.line_vol_l>>1)&0xf)<<4) |
        ((_mixer.line_vol_r>>1)&0xf)
        ;
      break;
      
    case 0x30: // Master.L
      ret= (_mixer.master_vol_l<<3);
      break;
    case 0x31: // Master.R
      ret= (_mixer.master_vol_r<<3);
      break;
    case 0x32: // Voice.L
      ret= (_mixer.voice_vol_l<<3);
      break;
    case 0x33: // Voice.R
      ret= (_mixer.voice_vol_r<<3);
      break;
    case 0x34: // MIDI.L
      ret= (_mixer.midi_vol_l<<3);
      break;
    case 0x35: // MIDI.R
      ret= (_mixer.midi_vol_r<<3);
      break;
    case 0x36: // CD.L
      ret= (_mixer.cd_vol_l<<3);
      break;
    case 0x37: // CD.R
      ret= (_mixer.cd_vol_r<<3);
      break;
    case 0x38: // Line.L
      ret= (_mixer.line_vol_l<<3);
      break;
    case 0x39: // Line.R
      ret= (_mixer.line_vol_r<<3);
      break;
    case 0x3a: // Mic vol
      ret= (_mixer.mic_vol<<3);
      break;
    case 0x3b: // PC Speaker volume
      ret= (_mixer.pc_speaker_vol<<6);
      break;
    case 0x3c: // Output mixer switches
      ret= _mixer.out_switches&0x1F;
      break;
    case 0x3d: // Input mixer.L switches
      ret= _mixer.in_switches_l&0x7F;
      break;
    case 0x3e: // Input mixer.R switches
      ret= _mixer.in_switches_r&0x7F;
      break;
    case 0x3f: // Input Gain .L
      ret= (_mixer.input_gain_l<<6);
      break;
    case 0x40: // Input Gain .R
      ret= (_mixer.input_gain_r<<6);
      break;
    case 0x41: // Output Gain .L
      ret= (_mixer.output_gain_l<<6);
      break;
    case 0x42: // Output Gain .R
      ret= (_mixer.output_gain_r<<6);
      break;
    case 0x43: // Mic AGC
      ret= _mixer.agc_on ? 0x00 : 0x01;
      break;
    case 0x44: // Treble.L
      ret= (_mixer.treble_l<<4);
      break;
    case 0x45: // Treble.R
      ret= (_mixer.treble_r<<4);
      break;
    case 0x46: // Bass.L
      ret= (_mixer.bass_l<<4);
      break;
    case 0x47: // Bass.R
      ret= (_mixer.bass_r<<4);
      break;
      
    case 0x80: // IRQ Select
      ret= 0x02; // Fixat a IRQ5
      break;
    case 0x81: // DMA Select
      ret= 0x22; // Fixat a 8bit: DMA1, 16bit: DMA5
      break;
    case 0x82: // Interrupt Status
      ret=
        (_dsp.dma.irq_on ? 0x01 : 0x00) |
        (_dsp.dma16.irq_on ? 0x02 : 0x00)
        ;
      break;

    case 0x90:
    case 0xfe:
    case 0xff: // ???
      ret= 0xff;
      break;
    default:
      ret= 0xff;
      PC_MSGF ( "SB16 MIXER - READ DATA REG:%02X",
                _mixer.addr );
      exit ( EXIT_FAILURE );
    }
  
  return ret;
  
} // end mixer_read_data


static void
mixer_write_data (
                  const uint8_t data
                  )
{

  switch ( _mixer.addr )
    {

    case 0x00: // Reset Mixer
      mixer_reset ();
      break;
      
    case 0x02: // No s'empra en SB16 (és de SBPro)
      break;
      
    case 0x04: // Voice volume.L / Voice volume.R (Versió 4bit)
      _mixer.voice_vol_l= ((data>>4)&0xf)<<1;
      _mixer.voice_vol_r= (data&0xf)<<1;
      break;
      
    case 0x0a: // Mic volume (Versió 3bit)
      // NOTA!! Important! Segons el document 0 to 7 ⇒ – 42 dB to 0
      // dB, in 6 dB steps. Però! no sé com mapejar açò de manera
      // directa a la versió de 5 bit. He decidit ignorar això i fer
      // un mapeig trivial.
      _mixer.mic_vol= (data&0x7)<<2;
      break;

    case 0x0c: // No s'empra en SB16 (és de SBPro)
      break;
      
    case 0x0e: // No s'empra en SB16 (és de SBPro)
      break;
      
    case 0x22: // Master volume.L / Master volume.R (Versió 4bit)
      _mixer.master_vol_l= ((data>>4)&0xf)<<1;
      _mixer.master_vol_r= (data&0xf)<<1;
      break;
      
    case 0x26: // MIDI volume.L / MIDI volume.R (Versió 4bit)
      _mixer.midi_vol_l= ((data>>4)&0xf)<<1;
      _mixer.midi_vol_r= (data&0xf)<<1;
      break;

    case 0x28: // CD volume.L / CD volume.R (Versió 4bit)
      _mixer.cd_vol_l= ((data>>4)&0xf)<<1;
      _mixer.cd_vol_r= (data&0xf)<<1;
      break;
      
    case 0x2e: // Line volume.L / Line volume.R (Versió 4bit)
      _mixer.line_vol_l= ((data>>4)&0xf)<<1;
      _mixer.line_vol_r= (data&0xf)<<1;
      break;
      
    case 0x30: // Master volume.L
      _mixer.master_vol_l= (data>>3);
      break;
    case 0x31: // Master volume.R
      _mixer.master_vol_r= (data>>3);
      break;
    case 0x32: // Voice volume.L
      _mixer.voice_vol_l= (data>>3);
      break;
    case 0x33: // Voice volume.R
      _mixer.voice_vol_r= (data>>3);
      break;
    case 0x34: // MIDI volume.L
      _mixer.midi_vol_l= (data>>3);
      break;
    case 0x35: // MIDI volume.R
      _mixer.midi_vol_r= (data>>3);
      break;
    case 0x36: // CD volume.L
      _mixer.cd_vol_l= (data>>3);
      break;
    case 0x37: // CD volume.R
      _mixer.cd_vol_r= (data>>3);
      break;
    case 0x38: // Line volume.L
      _mixer.line_vol_l= (data>>3);
      break;
    case 0x39: // Line volume.R
      _mixer.line_vol_r= (data>>3);
      break;
    case 0x3a: // Mic volume
      _mixer.mic_vol= (data>>3);
      break;
    case 0x3b: // PC Speaker volume
      _mixer.pc_speaker_vol= (data>>6);
      break;
    case 0x3c: // Output mixer switches
      _mixer.out_switches= data&0x1F;
      break;
    case 0x3d: // Input mixer.L switches
      _mixer.in_switches_l= data&0x7F;
      break;
    case 0x3e: // Input mixer.R switches
      _mixer.in_switches_r= data&0x7F;
      break;
    case 0x3f: // Input Gain .L
      _mixer.input_gain_l= (data>>6);
      break;
    case 0x40: // Input Gain .R
      _mixer.input_gain_r= (data>>6);
      break;
    case 0x41: // Output Gain .L
      _mixer.output_gain_l= (data>>6);
      break;
    case 0x42: // Output Gain .R
      _mixer.output_gain_r= (data>>6);
      break;
    case 0x43: // Mic AGC
      _mixer.agc_on= (data&0x01)==0x00;
      break;
    case 0x44: // Treble .L
      _mixer.treble_l= (data>>4);
      if ( _mixer.treble_l != 8 )
        PC_MSGF("SB16 MIXER - TREBLE L %X!!!",
                _mixer.treble_l);
      break;
    case 0x45: // Treble .R
      _mixer.treble_r= (data>>4);
      if ( _mixer.treble_r != 8 )
        PC_MSGF("SB16 MIXER - TREBLE R %X!!!",
                _mixer.treble_r);
      break;
    case 0x46: // Bass .L
      _mixer.bass_l= (data>>4);
      if ( _mixer.bass_l != 8 )
        PC_MSGF("SB16 MIXER - BASS L %X!!!",
                _mixer.bass_l);
      break;
    case 0x47: // Bass .R
      _mixer.bass_r= (data>>4);
      if ( _mixer.bass_r != 8 )
        PC_MSGF("SB16 MIXER - BASS R %X!!!",
                _mixer.bass_r);
      break;

    case 0x80: // IRQ Select
      _warning ( _udata, "SB16 MIXER - s'ha intentat"
                 " modificar IRQ Select: %02X", data );
      break;
    case 0x81: // DMA Select
      _warning ( _udata, "SB16 MIXER - s'ha intentat"
                 " modificar DMA Select: %02X", data );
      break;
      
    case 0x83: // Page number DMA????
      PC_MSGF("Què és açò?????:%02X",data);
      break;

    case 0x90:
    case 0xfe:
    case 0xff: // ???
      PC_MSGF("Què és açò?????:%02X",data);
      break;
    default:
      PC_MSGF ( "SB16 MIXER - WRITE DATA REG:%02X",
                _mixer.addr );
      exit ( EXIT_FAILURE );
    }
  
} // end mixer_write_data


// GENERAL /////////////////////////////////////////////////////////////////////

static void
update_cc_to_event (void)
{

  int cc,tmp;
  long tmpl;
  

  // Per defecte 1s
  _timing.cctoEvent= PC_ClockFreq;
  tmp= fm_cc_to_event ();
  if ( tmp > 0 && tmp < _timing.cctoEvent )
    _timing.cctoEvent= tmp;
  // Una mostra a 44.1KHz
  tmpl=
    (_timing.cc_div*((long) PC_AUDIO_BUFFER_SIZE)) -
    (_timing.cc_remain + ((long) _timing.cc)*_timing.cc_mul)
    ;
  assert ( tmpl > 0 );
  tmp= (int) (tmpl / _timing.cc_mul);
  if ( tmpl%_timing.cc_mul != 0 ) ++tmp;
  assert ( tmp > 0 );
  if ( tmp < _timing.cctoEvent ) _timing.cctoEvent= tmp;
  
  // Actualitza PC_NextEventCC
  cc= PC_sb16_next_event_cc ();
  cc+= PC_Clock; // Medim sempre des de que PC_Clock és 0
  if ( cc < PC_NextEventCC )
    PC_NextEventCC= cc;
  
} // end update_cc_to_event


static void
run_sample (void)
{
  
  /*
  static const double DB_32l[32]= {
    8e-4, 1e-3, 1.3e-3, 1.6e-3, 2e-3, 2.5e-3, 3.2e-3, 4e-3,
    5e-3, 6.3e-3, 8e-3, 1e-2, 1.3e-2, 1.6e-2, 2e-2, 2.5e-2,
    3.2e-2, 4e-2, 5e-2, 6.3e-2, 8e-2, 1e-1, 1.3e-1, 1.6e-1,
    2e-1, 2-5e-1, 3.2e-1, 4e-1, 5e-1, 6.3e-1, 8e-1, 1.0
  };
  */
  // NOTA!!! VAIG A FER-HO LINIAL
  static const float DB_32l[32]= {
    0.0, 0.03225806451612903, 0.06451612903225806, 0.0967741935483871,
    0.12903225806451613, 0.16129032258064516, 0.1935483870967742,
    0.22580645161290322, 0.25806451612903225, 0.2903225806451613,
    0.3225806451612903, 0.3548387096774194, 0.3870967741935484,
    0.41935483870967744, 0.45161290322580644, 0.4838709677419355,
    0.5161290322580645, 0.5483870967741935, 0.5806451612903226,
    0.6129032258064516, 0.6451612903225806, 0.6774193548387096,
    0.7096774193548387, 0.7419354838709677, 0.7741935483870968,
    0.8064516129032258, 0.8387096774193549, 0.8709677419354839,
    0.9032258064516129, 0.9354838709677419, 0.967741935483871, 1.0
  };
  static const float GAIN[4]= {1.0,1.1,1.3,1.5}; // <-- INVENT LINIAL
  
  int16_t laux,raux;
  int32_t l,r,voice_l,voice_r;
  

  // Calcula el voice.
  voice_l= voice_r= 0;
  fm_get_next_44_1KHz_sample ( &laux, &raux );
  voice_l+= (int32_t) laux; voice_r+= (int32_t) raux;
  dsp_get_next_44_1KHz_sample ( &laux, &raux );
  voice_l+= (int32_t) laux; voice_r+= (int32_t) raux;
  voice_l/= 2; voice_r/= 2;
  if      ( voice_l > 32767 )  voice_l= 32767;
  else if ( voice_l < -32768 ) voice_l= -32768;
  if      ( voice_r > 32767 )  voice_r= 32767;
  else if ( voice_r < -32768 ) voice_r= -32768;

  // Aplica mixer.
  l= 0; r= 0;
  // --> Voice
  l+= (int32_t) (voice_l*DB_32l[_mixer.voice_vol_l]);
  r+= (int32_t) (voice_r*DB_32l[_mixer.voice_vol_r]);
  // --> CD
  PC_piix4_ide_get_next_cd_audio_sample ( &laux, &raux );
  if ( (_mixer.out_switches&0x04) != 0x00 )
    l+= (int32_t) (laux*DB_32l[_mixer.cd_vol_l]);
  if ( (_mixer.out_switches&0x02) != 0x00 )
    r+= (int32_t) (raux*DB_32l[_mixer.cd_vol_r]);
  // --> FALTA PC SPEAKER !!!!
  
  // Aplica master volum i satura.
  // NOTA!! FALTA Treble i Bass
  l= (int32_t) l*DB_32l[_mixer.master_vol_l]*GAIN[_mixer.output_gain_l];
  r= (int32_t) r*DB_32l[_mixer.master_vol_r]*GAIN[_mixer.output_gain_r];
  if      ( l > 32767 )  l= 32767;
  else if ( l < -32768 ) l= -32768;
  if      ( r > 32767 )  r= 32767;
  else if ( r < -32768 ) r= -32768;
  
  // Afegeix al buffer.
  _out.buf[2*_out.N]= (int16_t) l;
  _out.buf[2*_out.N+1]= (int16_t) r;
  if ( ++_out.N == PC_AUDIO_BUFFER_SIZE )
    {
      PC_sound_set ( _out.buf, PC_SOUND_SOURCE_SB16 );
      _out.N= 0;
    }
  
} // end run_sample


static void
clock (
       const bool update_cc2event
       )
{

  int cc;
  long sample_cc;


  // Flag important.
  _dsp.dma.in_clock= true;
  _dsp.dma16.in_clock= true;
  
  // Processa cicles
  cc= PC_Clock-_timing.cc_used;
  if ( cc > 0 ) { _timing.cc+= cc; _timing.cc_used+= cc; }

  // Processa cicles
  fm_clock ( _timing.cc );
  sample_cc= _timing.cc_remain + ((long) _timing.cc)*_timing.cc_mul;
  _timing.cc= 0;
  while ( sample_cc >= _timing.cc_div )
    {
      run_sample ();
      sample_cc-= _timing.cc_div;
    }
  _timing.cc_remain= sample_cc;
  
  // Actualitza cctoEvent
  if ( update_cc2event ) update_cc_to_event ();

  _dsp.dma.in_clock= false;
  _dsp.dma16.in_clock= false;
  
} // end clock




/**********************/
/* FUNCIONS PÚBLIQUES */
/**********************/

void
PC_sb16_init (
              PC_Warning *warning,
              void       *udata
              )
{

  static const int DIVS[4]= {3,3,7,7};
  int i;

  
  // Callbacks.
  _warning= warning;
  _udata= udata;

  // Estat.
  fm_init ();
  dsp_init ();
  mixer_init ();
  _out.N= 0; // No es reseteja
  
  // Timing.
  _timing.cc_used= 0;
  _timing.cc= 0;
  _timing.cctoEvent= 0;
  _timing.cc_remain= 0;
  assert ( PC_ClockFreq%100 == 0 );
  _timing.cc_div= (long) (PC_ClockFreq/100);
  _timing.cc_mul= 441; // 44.1Khz/100
  // --> Intenta ajustar un poc
  for ( i= 0; i < 4; ++i )
    if ( _timing.cc_div%DIVS[i] == 0 )
      {
        _timing.cc_div/= DIVS[i];
        _timing.cc_mul/= DIVS[i];
      }
  
  update_cc_to_event ();
  
} // end PC_sb16_init


int
PC_sb16_next_event_cc (void)
{

  int tmp;
  
  
  tmp= _timing.cctoEvent - _timing.cc;
  assert ( tmp > 0 );

  return tmp;
  
} // end PC_sb16_next_event_cc


void
PC_sb16_end_iter (void)
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
  
} // end PC_sb16_end_iter


void
PC_sb16_reset (void)
{

  // ATENCIÓ!! Per a no fer glitches raros en la generació del so, no
  // vaig a resetejar els timings ni l'estat del buffer.
  clock ( true );

  // Reseteja.
  dsp_reset ();
  fm_reset ();
  mixer_reset ();
  
} // end PC_sb16_reset


uint8_t
PC_sb16_fm_status (void)
{

  uint8_t ret;
  
  
  clock ( true );
  
  // There are two versions of SBPRO. The difference is in the FM chip
  // used. The earlier version uses a two-operator FM chip, while the
  // later version uses a four-operator FM chip. To distinguished
  // them, you can read the value from I/O port 388h, two-operator
  // cards will return a value of 06h, and four-operator cards will
  // return a value of 00h.
  ret=
    ((_fm.timers[0].irq_done || _fm.timers[1].irq_done) ? 0x80 : 0x00) |
    (_fm.timers[0].irq_done ? 0x40 : 0x00) |
    (_fm.timers[1].irq_done ? 0x20 : 0x00)
    ;

  // Emula un delay artifical (~23/35 microsegons). El motiu és que hi
  // ha una rutina per ahí que assumeix un sobrecost d'aquest tipus.
  PC_Clock+= _fm.cc_delay_status;
  
  return ret;
  
} // end PC_sb16_fm_status


void
PC_sb16_fm_set_addr (
                     const uint8_t addr,
                     const int     array
                     )
{

  clock ( true );

    // IMPORTANT!!! Llegint documentació no em queda clar si 0x388 és
    // un alias del array 0, o si el que fa és escriure en els dos
    // arrays a la vegada. He decidit apostar per la primera opció.
  if ( addr == 0x00 || addr >= 0xF6 )
    {
      _warning ( _udata, "SB16 FM: adreça fora de rang: %02X", addr );
      //return; AVISA PERÒ NO IMPEDEIX
    }
  _fm.addr[array]= addr;
  
} // end PC_sb16_fm_set_addr


void
PC_sb16_fm_write_data (
                       const uint8_t data,
                       const int     array
                       )
{

  clock ( false );
  
  switch ( _fm.addr[array] )
    {
      /*
    case 0x01: // LSI Test ?????
      if ( data&0x20 ) // allows the FM chips to control the waveform
                       // of each operator.
        {
          printf ( "[CAL_IMPLEMENTAR] SB16 FM - CONTROL WAVEFORM\n" );
          exit ( EXIT_FAILURE );
        }
      break;
      */
    case 0x02: // Timer 1
      if ( array == 0 ) _fm.timers[0].init_val= data;
      break;
    case 0x03: // Timer 2
      if ( array == 0 ) _fm.timers[1].init_val= data;
      break;
    case 0x04: // Timer Control Byte - Connection Sel
      if ( array == 0 ) fm_timers_control ( data );
      else              fm_set_connection_sel ( data );
      break;
    case 0x05: // NEW
      if ( array == 1 )
        {
          _fm.opl3_mode= (data&0x1)!=0;
          fm_update_channels ();
        }
      break;

    case 0x08: // NTS
      if ( array == 0 )
        {
          _fm.nts= (data&0x40)>>6;
          fm_update_channels ();
        }
      break;
      
    case 0x20 ... 0x25: // AM - VIB - EGT - KSR - MULT
    case 0x28 ... 0x2D:
    case 0x30 ... 0x35:
      fm_op_set_am_vib_egt_ksr_mult
        ( &_fm.ops[array][6*((_fm.addr[array]>>3)&0x3)+(_fm.addr[array]&0x7)],
          data );
      break;

    case 0x40 ... 0x45: // KSL - TL
    case 0x48 ... 0x4D:
    case 0x50 ... 0x55:
      fm_op_set_ksl_tl
        ( &_fm.ops[array][6*((_fm.addr[array]>>3)&0x3)+(_fm.addr[array]&0x7)],
          data );
      break;
      
    case 0x60 ... 0x65: // AR - DR
    case 0x68 ... 0x6D:
    case 0x70 ... 0x75:
      fm_op_set_ar_dr
        ( &_fm.ops[array][6*((_fm.addr[array]>>3)&0x3)+(_fm.addr[array]&0x7)],
          data );
      break;

    case 0x80 ... 0x85: // SL - RR
    case 0x88 ... 0x8D:
    case 0x90 ... 0x95:
      fm_op_set_sl_rr
        ( &_fm.ops[array][6*((_fm.addr[array]>>3)&0x3)+(_fm.addr[array]&0x7)],
          data );
      break;
      
    case 0xA0 ... 0xA8: // F NUMBER (L)
      fm_channel_set_fnumL ( &_fm.channels[array][_fm.addr[array]&0xF],
                             data );
      break;
      
    case 0xB0 ... 0xB8: // KON - BLOCK - FNUM (H)
      fm_channel_set_kon_block_fnumH
        ( &_fm.channels[array][_fm.addr[array]&0xF], data );
      break;

    case 0xBD: // DAM - DVB - RYT - BD - SD - TOM - TC - HH
      if ( array == 0 ) fm_set_dam_dvb_ryt_bd_sd_tom_tc_hh ( data );
      break;

    case 0xC0 ... 0xC8: // CHD - CHC - CHB - CHA - FB - CNT
      fm_channel_set_chd_chc_chb_cha_fb_cnt
        ( &_fm.channels[array][_fm.addr[array]&0xF], data );
      break;
      
    case 0xE0 ... 0xE5: // WS
    case 0xE8 ... 0xED:
    case 0xF0 ... 0xF5:
      fm_op_set_ws
        ( &_fm.ops[array][6*((_fm.addr[array]>>3)&0x3)+(_fm.addr[array]&0x7)],
          data );
      break;
      
    default:
      if ( _fm.addr[array] > 0 && _fm.addr[array] < 0xF6 )
        {
          PC_MSGF ( "SB16 FM - "
                    "PC_sb16_fm_write_data ARRAY:%d ADDR:%02X  DATA:%02X",
                    array, _fm.addr[array], data );
          //exit ( EXIT_FAILURE );
        }
    }
  
  update_cc_to_event ();
  
} // end PC_sb16_fm_write_data


void
PC_sb16_dsp_reset (
                   const uint8_t data
                   )
{

  bool new_val;


  clock ( true ); // Necessari??
  
  new_val= (data&0x1)!=0;
  if ( data&0xFE )
    _warning ( _udata,
               "SB16 DSP: s'ha resetejat amb un valor estrany "
               "(no és ni 0 ni 1): %02X",
               data );
  if ( !new_val && _dsp.reset_flag )
    {
      dsp_reset ();
      dsp_out_add ( 0xaa );
    }
  _dsp.reset_flag= new_val;
  
} // end PC_sb16_dsp_reset


uint8_t
PC_sb16_dsp_read_data (void)
{

  uint8_t ret;


  clock ( true ); // Necessari??
  
  if ( _dsp.out.N >= 1 )
    {
      ret= _dsp.out.v[_dsp.out.p];
      _dsp.out.p= (_dsp.out.p+1)%DSP_OUT_BUF_SIZE;
      --_dsp.out.N;
    }
  else ret= 0xff;
  
  return ret;
  
} // end PC_sb16_dsp_read_data


uint8_t
PC_sb16_dsp_read_buffer_status (void)
{

  uint8_t ret;

  
  clock ( true ); // Necessari??
  
  ret= _dsp.out.N>0 ? 0x80 : 0x00;
  dsp_dma_set_irq ( false );
  
  return ret;
  
} // end PC_sb16_dsp_read_buffer_status


uint8_t
PC_sb16_dsp_write_buffer_status (void)
{

  uint8_t ret;


  clock ( true ); // Necessari?

  ret= _dsp.in.empty ? 0x00 : 0x80;
  
  return ret;
  
} // end PC_sb16_dsp_write_buffer_status


void
PC_sb16_dsp_write (
                   const uint8_t data
                   )
{
  
  clock ( false );
  dsp_write ( data );
  update_cc_to_event (); // Necessari???
  
} // end PC_sb16_dsp_write


uint8_t
PC_sb16_dsp_ack_dma16_irq (void)
{

  uint8_t ret;


  clock ( true ); // Necessari?

  ret= 0x00; // ???????????????
  dsp_dma16_set_irq ( false );
  
  return ret;
  
} // end PC_sb16_dsp_ack_dma16_irq


void
PC_sb16_dma_signal (
                    const PC_DMA_Signal signal
                    )
{

  if ( !_dsp.dma.in_clock ) clock ( false );
  
  if ( signal == PC_DMA_SIGNAL_DACK )
    {
      // Ho deixe per quan implemente nous mètodes DMA no lobidar
      // repasar-ho. Però en principi no faig res amb aquesta senyal.
      switch ( _dsp.dma.state )
        {
        case DSP_DMA_NONE:
        case DSP_DMA_SINGLE:
        case DSP_DMA_AUTO_INIT:
        case DSP_DMA_AUTO_INIT_FINISH:
          break; 
        default:
          PC_MSGF("PC_sb16_dma_signal DACK amb estat %d",
                  _dsp.dma.state);
          exit(EXIT_FAILURE);
          break;
        }
    }
  else // PC_DMA_SIGNAL_TC
    {
      switch ( _dsp.dma.state )
        {
        case DSP_DMA_NONE: // No deuria passar, però ignorem.
          break;
        case DSP_DMA_SINGLE:
          //dsp_dma_finish (); // <-- Ho faig quan acaba el comptador
          break;
        case DSP_DMA_AUTO_INIT:
          /* // EN Doom passa molt, i és normal si el buffer del DMA i
             // de DSP difereixen.
          if ( _dsp.dma.counter != 0 )
            _warning ( _udata,
                       "SB16 DSP: durant una transferència DMA s'ha rebut "
                       "una senyal TC però el comptador actual %d != 0",
                       _dsp.dma.counter );
          */
          break;
        case DSP_DMA_AUTO_INIT_FINISH:
          dsp_dma_finish ();
          break;
        default:
          PC_MSGF("PC_sb16_dma_signal TC amb estat %d",
                  _dsp.dma.state);
          exit(EXIT_FAILURE);
          break;
        }
    }

  if ( !_dsp.dma.in_clock ) update_cc_to_event ();
  
} // end PC_sb16_dma_signal


void
PC_sb16_dma16_signal (
                      const PC_DMA_Signal signal
                      )
{
  
  if ( !_dsp.dma16.in_clock ) clock ( false );
  
  if ( signal == PC_DMA_SIGNAL_DACK )
    {
      // Ho deixe per quan implemente nous mètodes DMA no oblidar
      // repasar-ho. Però en principi no faig res amb aquesta senyal.
      switch ( _dsp.dma16.state )
        {
        case DSP_DMA16_NONE:
        case DSP_DMA16_SINGLE:
        case DSP_DMA16_AUTO_INIT:
        case DSP_DMA16_AUTO_INIT_FINISH:
          break; 
        default:
          PC_MSGF("PC_sb16_dma16_signal DACK amb estat %d",
                  _dsp.dma16.state);
          exit(EXIT_FAILURE);
          break;
        }
    }
  else // PC_DMA_SIGNAL_TC
    {
      switch ( _dsp.dma16.state )
        {
        case DSP_DMA16_NONE: // No deuria passar, però ignorem.
          break;
        case DSP_DMA16_SINGLE:
          //dsp_dma16_finish (); // <-- Ho faig quan acaba el comptador
          break;
        case DSP_DMA16_AUTO_INIT:
          /* // EN Doom passa molt, i és normal si el buffer del DMA i
             // de DSP difereixen.
          if ( _dsp.dma16.counter != 0 )
            _warning ( _udata,
                       "SB16 DSP: durant una transferència DMA16 s'ha rebut "
                       "una senyal TC però el comptador actual %d != 0",
                       _dsp.dma16.counter );
          */
          break;
        case DSP_DMA16_AUTO_INIT_FINISH:
          dsp_dma16_finish (); // Cal fer interrupció??? De moment la faig.
          break; 
        default:
          PC_MSGF("PC_sb16_dma16_signal TC amb estat %d",
                  _dsp.dma16.state);
          exit(EXIT_FAILURE);
          break;
        }
    }

  if ( !_dsp.dma16.in_clock ) update_cc_to_event ();
  
} // end PC_sb16_dma16_signal


void
PC_sb16_dma_write (
                   const uint8_t data
                   )
{
  
  clock ( false );
  dsp_dma_write ( data );
  update_cc_to_event ();
  
} // end PC_sb16_dma_write


void
PC_sb16_dma16_write (
                     const uint16_t data
                     )
{
  
  clock ( false );
  dsp_dma16_write ( data );
  update_cc_to_event ();
  
} // end PC_sb16_dma16_write


void
PC_sb16_mixer_set_addr (
                        const uint8_t addr
                        )
{

  
  clock ( false );
  _mixer.addr= addr;
  update_cc_to_event ();
  
} // end PC_sb16_mixer_set_addr


uint8_t
PC_sb16_mixer_read_data (void)
{

  uint8_t ret;

  
  clock ( true ); // Necessari??
  ret= mixer_read_data ( _mixer.addr );
  update_cc_to_event ();
  
  return ret;
  
} // end PC_sb16_mixer_read_data


void
PC_sb16_mixer_write_data (
                          const uint8_t data
                          )
{

  clock ( false );
  mixer_write_data ( data );
  update_cc_to_event ();
  
} // end PC_sb16_mixer_write_data


// NOTA!!! Açò clarament està mal. La manera correcta d'accedir als
// registres del mixer no és aquesta. Però en l'instal·lado del
// Sam&Max clarament intenta accedir al registre 0x82 (estat
// interrupcions) directament. En realitat crec que funciona igual
// però per si de cas ho he implementat.
uint8_t
PC_sb16_mixer_direct (
                      const uint8_t addr
                      )
{

  uint8_t ret;

  
  clock ( true );
  ret= mixer_read_data ( addr );

  return ret;
  
} // end PC_sb16_mixer_direct
