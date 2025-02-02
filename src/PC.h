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
 *  PC.h - Simulador de PC que implementa una placa 430TX amb un
 *         Pentium
 *
 */

#ifndef __PC_H__
#define __PC_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef __LITTLE_ENDIAN__
#define PC_LE
#elif defined __BIG_ENDIAN__
#define PC_BE
#else
#error Per favor defineix __LITTLE_ENDIAN__ o __BIG_ENDIAN__
#endif

#include "CD.h"
#include "IA32.h"


/*********/
/* TIPUS */
/*********/
// Tipus generals.

// Funció per a emetre avisos.
typedef void
(PC_Warning) (
              void       *udata,
              const char *format,
              ...
              );

// Error
typedef enum
  {
   PC_NOERROR=0,
   PC_BADBIOS,
   PC_UNK_CPU_MODEL,
   PC_BADOPTROM,
   PC_HDD_WRONG_SIZE,
   PC_FD_WRONG_SIZE
  } PC_Error;

// DMA Signal
typedef enum
  {
    PC_DMA_SIGNAL_DACK,
    PC_DMA_SIGNAL_TC
  } PC_DMA_Signal;

// 'Scancodes' teclat independent de configuració.
typedef enum
  {
    PC_SCANCODE_70= 0,
    PC_SCANCODE_71,
    PC_SCANCODE_E0_5A,
    PC_SCANCODE_69,
    PC_SCANCODE_72,
    PC_SCANCODE_7A,
    PC_SCANCODE_6B,
    PC_SCANCODE_73,
    PC_SCANCODE_74,
    PC_SCANCODE_6C,
    PC_SCANCODE_75,
    PC_SCANCODE_7D,
    PC_SCANCODE_79,
    PC_SCANCODE_77,
    PC_SCANCODE_E0_4A,
    PC_SCANCODE_7C,
    PC_SCANCODE_7B,
    PC_SCANCODE_E0_75,
    PC_SCANCODE_E0_6B,
    PC_SCANCODE_E0_72,
    PC_SCANCODE_E0_74,
    PC_SCANCODE_E0_71,
    PC_SCANCODE_E0_69,
    PC_SCANCODE_E0_7A,
    PC_SCANCODE_E0_70,
    PC_SCANCODE_E0_6C,
    PC_SCANCODE_E0_7D,
    PC_SCANCODE_14,
    PC_SCANCODE_E0_1F,
    PC_SCANCODE_11,
    PC_SCANCODE_29,
    PC_SCANCODE_E0_11,
    PC_SCANCODE_E0_27,
    PC_SCANCODE_E0_2F,
    PC_SCANCODE_E0_14,
    PC_SCANCODE_12,
    PC_SCANCODE_61,
    PC_SCANCODE_1A,
    PC_SCANCODE_22,
    PC_SCANCODE_21,
    PC_SCANCODE_2A,
    PC_SCANCODE_32,
    PC_SCANCODE_31,
    PC_SCANCODE_3A,
    PC_SCANCODE_41,
    PC_SCANCODE_49,
    PC_SCANCODE_4A,
    PC_SCANCODE_59,
    PC_SCANCODE_58,
    PC_SCANCODE_1C,
    PC_SCANCODE_1B,
    PC_SCANCODE_23,
    PC_SCANCODE_2B,
    PC_SCANCODE_34,
    PC_SCANCODE_33,
    PC_SCANCODE_3B,
    PC_SCANCODE_42,
    PC_SCANCODE_4B,
    PC_SCANCODE_4C,
    PC_SCANCODE_52,
    PC_SCANCODE_5D,
    PC_SCANCODE_0D,
    PC_SCANCODE_15,
    PC_SCANCODE_1D,
    PC_SCANCODE_24,
    PC_SCANCODE_2D,
    PC_SCANCODE_2C,
    PC_SCANCODE_35,
    PC_SCANCODE_3C,
    PC_SCANCODE_43,
    PC_SCANCODE_44,
    PC_SCANCODE_4D,
    PC_SCANCODE_54,
    PC_SCANCODE_5B,
    PC_SCANCODE_5A,
    PC_SCANCODE_0E,
    PC_SCANCODE_16,
    PC_SCANCODE_1E,
    PC_SCANCODE_26,
    PC_SCANCODE_25,
    PC_SCANCODE_2E,
    PC_SCANCODE_36,
    PC_SCANCODE_3D,
    PC_SCANCODE_3E,
    PC_SCANCODE_46,
    PC_SCANCODE_45,
    PC_SCANCODE_4E,
    PC_SCANCODE_55,
    PC_SCANCODE_66,
    PC_SCANCODE_E0_12_E0_7C,
    PC_SCANCODE_E0_7C,
    PC_SCANCODE_7F,
    PC_SCANCODE_7E,
    PC_SCANCODE_E1_14_77_E1_F0_14_F0_77,
    PC_SCANCODE_E0_7E_E0_C6,
    PC_SCANCODE_01,
    PC_SCANCODE_09,
    PC_SCANCODE_78,
    PC_SCANCODE_07,
    PC_SCANCODE_03,
    PC_SCANCODE_0B,
    PC_SCANCODE_83,
    PC_SCANCODE_0A,
    PC_SCANCODE_05,
    PC_SCANCODE_06,
    PC_SCANCODE_04,
    PC_SCANCODE_0C,
    PC_SCANCODE_76,
    PC_SCANCODE_ALL // Serveix per a saber el número d'scancodes i
                    // també per a netejar l'estat.
  } PC_Scancode;

// Macros per als scancodes d'un teclat espanyol (SET2 untraslated) (KBDSP).
#define PC_KBDSP_NUM_0 PC_SCANCODE_70
#define PC_KBDSP_NUM_PUNTO PC_SCANCODE_71
#define PC_KBDSP_NUM_ENTRAR PC_SCANCODE_E0_5A

#define PC_KBDSP_NUM_1 PC_SCANCODE_69
#define PC_KBDSP_NUM_2 PC_SCANCODE_72
#define PC_KBDSP_NUM_3 PC_SCANCODE_7A

#define PC_KBDSP_NUM_4 PC_SCANCODE_6B
#define PC_KBDSP_NUM_5 PC_SCANCODE_73
#define PC_KBDSP_NUM_6 PC_SCANCODE_74

#define PC_KBDSP_NUM_7 PC_SCANCODE_6C
#define PC_KBDSP_NUM_8 PC_SCANCODE_75
#define PC_KBDSP_NUM_9 PC_SCANCODE_7D
#define PC_KBDSP_NUM_SUMA PC_SCANCODE_79

#define PC_KBDSP_BLOQ_NUM PC_SCANCODE_77
#define PC_KBDSP_NUM_DIV PC_SCANCODE_E0_4A
#define PC_KBDSP_NUM_MUL PC_SCANCODE_7C
#define PC_KBDSP_NUM_RESTA PC_SCANCODE_7B

#define PC_KBDSP_ARRIBA PC_SCANCODE_E0_75
#define PC_KBDSP_IZQUIERDA PC_SCANCODE_E0_6B
#define PC_KBDSP_ABAJO PC_SCANCODE_E0_72
#define PC_KBDSP_DERECHA PC_SCANCODE_E0_74

#define PC_KBDSP_SUPR PC_SCANCODE_E0_71
#define PC_KBDSP_FIN PC_SCANCODE_E0_69
#define PC_KBDSP_AV_PAG PC_SCANCODE_E0_7A

#define PC_KBDSP_INSERT PC_SCANCODE_E0_70
#define PC_KBDSP_INICIO PC_SCANCODE_E0_6C
#define PC_KBDSP_RE_PAG PC_SCANCODE_E0_7D

#define PC_KBDSP_CONTROL PC_SCANCODE_14
#define PC_KBDSP_WINDOWS PC_SCANCODE_E0_1F
#define PC_KBDSP_ALT PC_SCANCODE_11
#define PC_KBDSP_ESPACIO PC_SCANCODE_29
#define PC_KBDSP_ALT_GR PC_SCANCODE_E0_11
#define PC_KBDSP_WIN_DERECHA PC_SCANCODE_E0_27
#define PC_KBDSP_MENU PC_SCANCODE_E0_2F
#define PC_KBDSP_CONTROL_DERECHA PC_SCANCODE_E0_14

#define PC_KBDSP_MAYUS PC_SCANCODE_12
#define PC_KBDSP_MENOR PC_SCANCODE_61
#define PC_KBDSP_Z PC_SCANCODE_1A
#define PC_KBDSP_X PC_SCANCODE_22
#define PC_KBDSP_C PC_SCANCODE_21
#define PC_KBDSP_V PC_SCANCODE_2A
#define PC_KBDSP_B PC_SCANCODE_32
#define PC_KBDSP_N PC_SCANCODE_31
#define PC_KBDSP_M PC_SCANCODE_3A
#define PC_KBDSP_COMA PC_SCANCODE_41
#define PC_KBDSP_PUNTO PC_SCANCODE_49
#define PC_KBDSP_GUION PC_SCANCODE_4A
#define PC_KBDSP_MAYUS_DERECHA PC_SCANCODE_59

#define PC_KBDSP_BLOQ_MAYUS PC_SCANCODE_58
#define PC_KBDSP_A PC_SCANCODE_1C
#define PC_KBDSP_S PC_SCANCODE_1B
#define PC_KBDSP_D PC_SCANCODE_23
#define PC_KBDSP_F PC_SCANCODE_2B
#define PC_KBDSP_G PC_SCANCODE_34
#define PC_KBDSP_H PC_SCANCODE_33
#define PC_KBDSP_J PC_SCANCODE_3B
#define PC_KBDSP_K PC_SCANCODE_42
#define PC_KBDSP_L PC_SCANCODE_4B
#define PC_KBDSP_ENYE PC_SCANCODE_4C
#define PC_KBDSP_TILDE PC_SCANCODE_52
#define PC_KBDSP_C_TRENCADA PC_SCANCODE_5D

#define PC_KBDSP_TABULADOR PC_SCANCODE_0D
#define PC_KBDSP_Q PC_SCANCODE_15
#define PC_KBDSP_W PC_SCANCODE_1D
#define PC_KBDSP_E PC_SCANCODE_24
#define PC_KBDSP_R PC_SCANCODE_2D
#define PC_KBDSP_T PC_SCANCODE_2C
#define PC_KBDSP_Y PC_SCANCODE_35
#define PC_KBDSP_U PC_SCANCODE_3C
#define PC_KBDSP_I PC_SCANCODE_43
#define PC_KBDSP_O PC_SCANCODE_44
#define PC_KBDSP_P PC_SCANCODE_4D
#define PC_KBDSP_ACCENT_OBERT PC_SCANCODE_54
#define PC_KBDSP_SIGNO_MAS PC_SCANCODE_5B
#define PC_KBDSP_ENTRAR PC_SCANCODE_5A

#define PC_KBDSP_SUPER_O PC_SCANCODE_0E
#define PC_KBDSP_1 PC_SCANCODE_16
#define PC_KBDSP_2 PC_SCANCODE_1E
#define PC_KBDSP_3 PC_SCANCODE_26
#define PC_KBDSP_4 PC_SCANCODE_25
#define PC_KBDSP_5 PC_SCANCODE_2E
#define PC_KBDSP_6 PC_SCANCODE_36
#define PC_KBDSP_7 PC_SCANCODE_3D
#define PC_KBDSP_8 PC_SCANCODE_3E
#define PC_KBDSP_9 PC_SCANCODE_46
#define PC_KBDSP_0 PC_SCANCODE_45
#define PC_KBDSP_COMILLAS PC_SCANCODE_4E
#define PC_KBDSP_ABRE_EXCLAMACION PC_SCANCODE_55
#define PC_KBDSP_RETROCESO PC_SCANCODE_66

#define PC_KBDSP_IMP_PNT PC_SCANCODE_E0_12_E0_7C
#define PC_KBDSP_CONTROL_IMP_PNT PC_SCANCODE_E0_7C
#define PC_KBDSP_MAYUS_IMP_PNT PC_SCANCODE_7F
#define PC_KBDSP_ALT_IMP_PNT PC_SCANCODE_7F
#define PC_KBDSP_BLOQ_DESPL PC_SCANCODE_7E
#define PC_KBDSP_PAUSA PC_SCANCODE_E1_14_77_E1_F0_14_F0_77
#define PC_KBDSP_CONTROL_PAUSA PC_SCANCODE_E0_7E_E0_C6

#define PC_KBDSP_F9 PC_SCANCODE_01
#define PC_KBDSP_F10 PC_SCANCODE_09
#define PC_KBDSP_F11 PC_SCANCODE_78
#define PC_KBDSP_F12 PC_SCANCODE_07

#define PC_KBDSP_F5 PC_SCANCODE_03
#define PC_KBDSP_F6 PC_SCANCODE_0B
#define PC_KBDSP_F7 PC_SCANCODE_83
#define PC_KBDSP_F8 PC_SCANCODE_0A

#define PC_KBDSP_F1 PC_SCANCODE_05
#define PC_KBDSP_F2 PC_SCANCODE_06
#define PC_KBDSP_F3 PC_SCANCODE_04
#define PC_KBDSP_F4 PC_SCANCODE_0C

#define PC_KBDSP_ESC PC_SCANCODE_76

// Per a indicar botó del ratolí.
typedef enum
  {
    PC_MOUSE_BUTTON_LEFT= 0x01,
    PC_MOUSE_BUTTON_RIGHT= 0x02,
    PC_MOUSE_BUTTON_MIDDLE= 0x04
  } PC_MouseButton;

// Interfície proporcionada per una funció PCI.
// El rang de ADDR per cada grandària és:
//   8bit  -> 0x00..0xFF
//   16bit -> 0x00..0x7F
//   32bit -> 0x00..0x3F
typedef struct
{

  // Lectura.
  uint8_t (*read8) (const uint8_t addr);
  uint16_t (*read16) (const uint8_t addr);
  uint32_t (*read32) (const uint8_t addr);

  // Escriptura.
  void (*write8) (const uint8_t addr,const uint8_t data);
  void (*write16) (const uint8_t addr,const uint16_t data);
  void (*write32) (const uint8_t addr,const uint32_t data);

  // Identificador. Utilitzat per a depurar
  const char *id;
  
} PC_PCIFunction;

typedef struct
{
  
  // Lectura
  bool (*read8) (const uint16_t port,uint8_t *data);
  bool (*read16) (const uint16_t port,uint16_t *data);
  bool (*read32) (const uint16_t port,uint32_t *data);
  
  // Escriptura
  bool (*write8) (const uint16_t port,const uint8_t data);
  bool (*write16) (const uint16_t port,const uint16_t data);
  bool (*write32) (const uint16_t port,const uint32_t data);
  
} PC_PCIPorts;

typedef struct
{

  // Lectura
  bool (*read8) (const uint64_t addr,uint8_t *data);
  bool (*read16) (const uint64_t addr,uint16_t *data);
  bool (*read32) (const uint64_t addr,uint32_t *data);
  bool (*read64) (const uint64_t addr,uint64_t *data);

  // Escriptura
  bool (*write8) (const uint64_t addr,const uint8_t data);
  bool (*write16) (const uint64_t addr,const uint16_t data);
  bool (*write32) (const uint64_t addr,const uint32_t data);
  
} PC_PCIMem;

typedef struct
{
  int  (*next_event_cc) (void);
  void (*end_iter) (void);
} PC_PCIClock;

// Engloba a PFIFunction/PCIPorts
typedef struct
{
  const PC_PCIFunction **func;
  int                    N; // Número de funcions
  const PC_PCIPorts     *ports; // Port ser NULL.
  const PC_PCIMem       *mem; // Pot ser NULL.
  const PC_PCIClock     *clock; // Pot ser NULL.
  void                 (*set_mode_trace) (const bool);
  void                 (*reset) (void);
} PC_PCICallbacks;

typedef enum
  {
    PC_PCI_DEVICE_SVGA_CIRRUS_CLGD5446= 0,
    PC_PCI_DEVICE_NULL
  } PC_PCIDevice;

#define PC_CFG_QEMU_COMPATIBLE 0x01

typedef enum
  {
    PC_QEMU_BOOT_ORDER_NONE=0,
    PC_QEMU_BOOT_ORDER_FLOPPY,
    PC_QEMU_BOOT_ORDER_HD,
    PC_QEMU_BOOT_ORDER_CD,
    PC_QEMU_BOOT_ORDER_SENTINEL
  } PC_QemuBootOrderDev;

// Especifica les característiques del MOUSE del 'host. En realitat
// són característiques de l'ordinador on s'executa el simulador que
// no és fàcil obtindre fàcilment. De fet açò és una aproximació a com
// funciona el 'host' i es podria enrequir amb altres modes.
typedef struct
{
  
  // Píxels per milímitre del ratolí del 'host'. Ha de ser un valor
  // positiu. 
  float resolution;
  
  /* // POSIBLES FUTURES MILLORES
  // Píxels que s'ha de moure el ratolí (real) abans que comence a
  // accelerar-se. Ha de ser un valor positiu.
  int sensitivity;

  // Una vegada passat el llindar de la 'sensitivity' el moviment es
  // multiplica per aquest valor. Ha de ser un valor positiu.
  float acceleration;
  */
  
} PC_HostMouse;

// Configura les característiques del PC.
typedef struct
{

  enum {
    PC_RAM_SIZE_4MB= 0,
    PC_RAM_SIZE_8MB,
    PC_RAM_SIZE_16MB,
    PC_RAM_SIZE_24MB,
    PC_RAM_SIZE_32MB,
    PC_RAM_SIZE_48MB,
    PC_RAM_SIZE_64MB,
    PC_RAM_SIZE_96MB,
    PC_RAM_SIZE_128MB,
    PC_RAM_SIZE_192MB,
    PC_RAM_SIZE_256MB,
    PC_RAM_SIZE_SENTINEL
  }              ram_size;
  // S'utilitza quan PC_CFG_QEMU_COMPATIBLE està activat.
  //
  // Inicialitza posicions 0x38 i 0x3D de la CMOSRAM. Serveix perquè
  // la BIOS sàpiga l'ordre d'inicialització.
  struct
  {
    bool                check_floppy_sign; // Si és cert, la BIOS
                                           // comprova que un disquet
                                           // té el camp MBR correcte
                                           // abans de fer boot.
    PC_QemuBootOrderDev order[3]; // Es poden repetir. El 0 té més
                                  // prioritat que la resta.
  }              qemu_boot_order;
  uint32_t       flags;
  struct
  {
    PC_PCIDevice  dev;
    uint8_t      *optrom; // Pot ser NULL
    size_t        optrom_size;
  } pci_devs[PC_PCI_DEVICE_NULL+1];
  // Acabat amb .dev==PC_PCI_DEVICE_NULL
  IA32_CPU_MODEL cpu_model;
  // Disqueteres - 0:A 1:B
  enum
    {
      PC_DISKETTE_NONE=0,
      PC_DISKETTE_360K=1,
      PC_DISKETTE_1M2=2,
      PC_DISKETTE_720K=3,
      PC_DISKETTE_1M44=4
    } diskettes[4];
  
  // Característiques del ratolí real.
  PC_HostMouse host_mouse;
  
} PC_Config;

typedef struct PC_File_ PC_File;

#define PC_FILE(PTR) ((PC_File *) (PTR))

#define PC_FILE_CLASS                                                   \
  bool read_only;                                                       \
  long nbytes;                                                          \
  int (*seek) (PC_File *f,long offset );                                \
  long (*tell) (PC_File *f);                                            \
  int (*read) (PC_File *f,void *dst,long nbytes);                      \
  int (*write) (PC_File *f,void *src,long nbytes);                     \
  void (*free) (PC_File *f);

// Torna 0 si tot ha anat bé.
#define PC_file_seek(FILE,OFFSET)               \
  (FILE)->seek ( (FILE), (OFFSET) )

// Torna -1 en cas d'error
#define PC_file_tell(FILE)                      \
  (FILE)->tell ( (FILE) )

// Torna 0 si tot ha anat bé. Llig o no llig.
#define PC_file_read(FILE,P_DST,NBYTES)             \
  (FILE)->read ( (FILE), (P_DST), (NBYTES) )

// Torna 0 si tot ha anat bé. Escriu o no escriu.
#define PC_file_write(FILE,P_SRC,NBYTES)                \
  (FILE)->write ( (FILE), (P_SRC), (NBYTES) )

#define PC_file_free(FILE)                      \
  (FILE)->free ( (FILE) )

struct PC_File_
{
  PC_FILE_CLASS;
};

// Per a definir un color en RGB
typedef struct
{
  uint8_t r,g,b;
} PC_RGB;

// Es cridat per la targeta gràfica per a refrescar la pantalla.
typedef void (PC_UpdateScreen) (
                                void         *udata,
                                const PC_RGB *fb,
                                const int     width,
                                const int     height,
                                const int     line_stride
                                );

typedef struct
{
  CD_Info *info;
  CD_Disc *current;
} PC_CDRom;


/*********/
/* UTILS */
/*********/
// Útils.

#if PC_BE
uint16_t PC_swap16(const uint16_t val);
uint32_t PC_swap32(const uint32_t val);
uint64_t PC_swap64(const uint64_t val);
#define PC_SWAP16(VAL) PC_swap16(VAL)
#define PC_SWAP32(VAL) PC_swap32(VAL)
#define PC_SWAP64(VAL) PC_swap64(VAL)
#else
#define PC_SWAP16(VAL) (VAL)
#define PC_SWAP32(VAL) (VAL)
#define PC_SWAP64(VAL) (VAL)
#endif


/*********/
/* FILES */
/*********/
// Implementa fitxers

// Retorna NULL en cas d'error
PC_File *
PC_file_new_from_file (
                       const char *file_name,
                       const bool  read_only
                       );

/*********/
/* CDROM */
/*********/
// Simulador d'un lector CD-ROM. Torna NULL si no s'ha pogut crear.
PC_CDRom *
PC_cdrom_new (void);

// Allibera memòria.
void
PC_cdrom_free (
               PC_CDRom *cdrom
               );

// Insert un disc en el lector. Si hi havia un disc anterior el
// tanca. Torna cert si tot ha anat bé. En cas d'error no es tanca
// l'anterior. Si es proporciona err, en cas d'error es fica el
// missatge d'error.
// NOTA!! Si file_name is NULL es buida el lector.
bool
PC_cdrom_insert_disc (
                      PC_CDRom    *cdrom,
                      const char  *file_name,
                      char       **err
                      );


/*********/
/* SOUND */
/*********/
// Mòdul que gestiona l'eixida de so.

// Poc més de mijta centèssima de segon
#define PC_AUDIO_BUFFER_SIZE 256

/* Tipus de la funció que actualitza es crida per a reproduir so. Es
 * proporcionen dos canals intercalats (esquerra/dreta). Cada mostra
 * està codificada com un valor de 16 bits amb signe amb una
 * freqüència de 44100Hz.
 */
typedef void (PC_PlaySound) (
                             const int16_t  samples[PC_AUDIO_BUFFER_SIZE*2],
                             void          *udata
                             );

void
PC_sound_init (
               PC_Warning   *warning,
               PC_PlaySound *play_sound,
               void         *udata
               );

#define PC_SOUND_SOURCE_SPEAKER 0
#define PC_SOUND_SOURCE_SB16    1

void
PC_sound_set (
              const int16_t samples[PC_AUDIO_BUFFER_SIZE*2],
              const int     source_id
              );


/*******/
/* UCP */
/*******/

// La CPU emprada (és l'intèrpret)
extern IA32_Interpreter PC_CPU;

// Versió amb JIT
extern IA32_JIT *PC_CPU_JIT;

// Constant configurable.
#define PC_JIT_BITS_PAGE 12

void
PC_cpu_init (
             PC_Warning      *warning,
             void            *udata,
             const PC_Config *config
             );

void
PC_cpu_close (void);

void
PC_cpu_reset (void);

// Decodifica la següent instrucció. Torna el valor del EIP per a la
// següent instrucció (ATENCIÓ!!! no és l'adreça física)
bool
PC_cpu_dis (
            IA32_Inst *inst,
            uint32_t  *eip
            );

// Com PC_cpu_dis però per a JIT.
bool
PC_cpu_jit_dis (
                IA32_Inst *inst,
                uint32_t  *eip
                );

/********/
/* MTXC */
/********/
// Xip 82439TX. Primer encarregat de gestionar el bus PCI.


typedef enum
  {
   PC_READ8,
   PC_READ16,
   PC_READ32,
   PC_READ64,
   PC_WRITE8,
   PC_WRITE16,
   PC_WRITE32
  } PC_MemAccessType;

// Utilitzat per a tracejar accessos al mapa de memòria.
typedef void (PC_MemAccess) (
                             const PC_MemAccessType  type,
                             const uint64_t          addr,
                             const uint64_t          data,
                             void                   *udata
                             );

typedef enum
  {
   PC_PCI_READ8,
   PC_PCI_READ16,
   PC_PCI_READ32,
   PC_PCI_WRITE8,
   PC_PCI_WRITE16,
   PC_PCI_WRITE32
  } PC_PCIRegAccessType;

// Utilitzat per a tracejar accessos als registres PCI
// addr ací sempre està en el rang 0x00..0xFF
// func_name pot ser NULL, en eixe cas indica que s'ha intentat llegir
// sense conexió
typedef void (PC_PCIRegAccess) (
                                const PC_PCIRegAccessType  type,
                                const uint8_t              addr,
                                const uint32_t             data,
                                const char                *func_name,
                                void                      *udata
                                );

void
PC_mtxc_init (
              PC_Warning            *warning,
              PC_MemAccess          *mem_access,
              PC_PCIRegAccess       *pci_reg_access, // Pot ser NULL
              const PC_PCICallbacks *pci_devs[], // Acava en NULL
              void                  *udata,
              const PC_Config       *config
              );

void
PC_mtxc_reset (
               const bool use_jit
               );

void
PC_mtxc_close (void);

uint32_t
PC_mtxc_confadd_read (void);

void
PC_mtxc_confadd_write (
                       const uint32_t data,
                       const bool     use_jit
                       );

uint8_t
PC_mtxc_confdata_read8 (
                        const uint8_t low_addr // 2 bits inferiors
                        );

uint16_t
PC_mtxc_confdata_read16 (
                         const uint8_t low_addr // 1 bit inferior
                         );

uint32_t
PC_mtxc_confdata_read32 (void);

void
PC_mtxc_confdata_write8 (
                         const uint8_t low_addr,
                         const uint8_t data
                         );

void
PC_mtxc_confdata_write16 (
                          const uint8_t  low_addr,
                          const uint16_t data
                          );

void
PC_mtxc_confdata_write32 (
                          const uint32_t data
                          );

void
PC_mtxc_set_mode_trace (
                        const bool val
                        );


/*******************/
/* 82371AB (PIIX4) */
/*******************/
// PIIX4 is a multi-function PCI device that integrates many
// system-level functions
// Implementa els components principals de PIIX4. Alguns components
// estan implementats en moduls separats.

typedef enum
  {
    PC_IDE_DEVICE_TYPE_NONE= 0,
    PC_IDE_DEVICE_TYPE_HDD,
    PC_IDE_DEVICE_TYPE_CDROM
  } PC_IDEDeviceType;

// Per simplificar els HDD han de medir-se sectors de 512 bytes, on
// cada track conté 63 sectors, cada 'head' té 65535 cilindres, i com
// a màxim hi han 255 caps.
typedef union
{
  PC_IDEDeviceType type;
  struct
  {
    PC_IDEDeviceType  type;
    PC_File          *f;
  }                hdd;
  struct
  {
    PC_IDEDeviceType type;
    PC_CDRom         *cdrom;
  }                cdrom;
} PC_IDEDevice;

PC_Error
PC_piix4_init (
               const uint8_t   *bios,
               size_t           bios_size,
               PC_IDEDevice     ide_devices[2][2],
               PC_Warning      *warning,
               void            *udata,
               const PC_Config *config
               );

void
PC_piix4_reset (void);

bool
PC_piix4_mem_read8 (
                    const uint64_t  addr,
                    uint8_t        *data
                    );

bool
PC_piix4_mem_read16 (
                     const uint64_t  addr,
                     uint16_t       *data
                     );

bool
PC_piix4_mem_read32 (
                     const uint64_t  addr,
                     uint32_t       *data
                     );

bool
PC_piix4_mem_read64 (
                     const uint64_t  addr,
                     uint64_t       *data
                     );

bool
PC_piix4_mem_write8 (
                     const uint64_t addr,
                     const uint8_t  data
                     );

bool
PC_piix4_mem_write16 (
                      const uint64_t addr,
                      const uint16_t data
                      );

bool
PC_piix4_mem_write32 (
                      const uint64_t addr,
                      const uint32_t data
                      );

void
PC_piix4_pci_isa_bridge_init (
                              PC_Warning *warning,
                              void       *udata
                              );

void
PC_piix4_pci_isa_bridge_reset (void);

void
PC_piix4_pci_isa_bridge_reset_control_write (
                                             const uint8_t data,
                                             const bool    use_jit
                                             );

// Torna cert si el dispositiu ha gestionat l'escriptura.
bool
PC_piix4_pci_isa_bridge_port_write8 (
                                     const uint16_t port,
                                     const uint8_t  data
                                     );

PC_Error
PC_piix4_ide_init (
                   PC_IDEDevice  ide_devices[2][2],
                   PC_Warning   *warning,
                   void         *udata
                   );

void
PC_piix4_ide_reset (void);

int
PC_piix4_ide_next_event_cc (void);

void
PC_piix4_ide_end_iter (void);

void
PC_piix4_ide_get_next_cd_audio_sample (
                                       int16_t *l,
                                       int16_t *r
                                       );

// Torna cert si el dispositiu ha gestionat la lectura.
bool
PC_piix4_ide_port_read8 (
                         const uint16_t  port,
                         uint8_t        *data
                         );

// Torna cert si el dispositiu ha gestionat la lectura.
bool
PC_piix4_ide_port_read16 (
                          const uint16_t  port,
                          uint16_t       *data
                          );

// Torna cert si el dispositiu ha gestionat la lectura.
bool
PC_piix4_ide_port_read32 (
                          const uint16_t  port,
                          uint32_t       *data
                          );

// Torna cert si el dispositiu ha gestionat l'escriptura.
bool
PC_piix4_ide_port_write8 (
                         const uint16_t port,
                         const uint8_t  data
                         );

// Torna cert si el dispositiu ha gestionat l'escriptura.
bool
PC_piix4_ide_port_write16 (
                           const uint16_t port,
                           const uint16_t data
                           );

// Torna cert si el dispositiu ha gestionat l'escriptura.
bool
PC_piix4_ide_port_write32 (
                           const uint16_t port,
                           const uint32_t data
                           );

void
PC_piix4_usb_init (
                   PC_Warning *warning,
                   void       *udata
                   );

void
PC_piix4_usb_reset (void);

// Torna cert si el dispositiu ha gestionat la lectura.
bool
PC_piix4_usb_port_read8 (
                         const uint16_t  port,
                         uint8_t        *data
                         );

// Torna cert si el dispositiu ha gestionat la lectura.
bool
PC_piix4_usb_port_read16 (
                          const uint16_t  port,
                          uint16_t       *data
                          );

// Torna cert si el dispositiu ha gestionat la lectura.
bool
PC_piix4_usb_port_read32 (
                          const uint16_t  port,
                          uint32_t       *data
                          );

// Torna cert si el dispositiu ha gestionat l'escriptura.
bool
PC_piix4_usb_port_write8 (
                         const uint16_t port,
                         const uint8_t  data
                         );

// Torna cert si el dispositiu ha gestionat l'escriptura.
bool
PC_piix4_usb_port_write16 (
                           const uint16_t port,
                           const uint16_t data
                           );

// Torna cert si el dispositiu ha gestionat l'escriptura.
bool
PC_piix4_usb_port_write32 (
                           const uint16_t port,
                           const uint32_t data
                           );

void
PC_piix4_power_management_init (
                                PC_Warning *warning,
                                void       *udata
                                );

void
PC_piix4_power_management_reset (void);

// Torna cert si el dispositiu ha gestionat la lectura.
bool
PC_piix4_power_management_port_read8 (
                                      const uint16_t  port,
                                      uint8_t        *data
                                      );

// Torna cert si el dispositiu ha gestionat la lectura.
bool
PC_piix4_power_management_port_read16 (
                                       const uint16_t  port,
                                       uint16_t       *data
                                       );

// Torna cert si el dispositiu ha gestionat la lectura.
bool
PC_piix4_power_management_port_read32 (
                                       const uint16_t  port,
                                       uint32_t       *data
                                       );

// Torna cert si el dispositiu ha gestionat l'escriptura.
bool
PC_piix4_power_management_port_write8 (
                                       const uint16_t port,
                                       const uint8_t  data
                                       );

// Torna cert si el dispositiu ha gestionat l'escriptura.
bool
PC_piix4_power_management_port_write16 (
                                        const uint16_t port,
                                        const uint16_t data
                                        );

// Torna cert si el dispositiu ha gestionat l'escriptura.
bool
PC_piix4_power_management_port_write32 (
                                        const uint16_t port,
                                        const uint32_t data
                                        );

extern const PC_PCIFunction PC_PIIX4_PCIFunction_pci_isa_bridge;
extern const PC_PCIFunction PC_PIIX4_PCIFunction_ide;
extern const PC_PCIFunction PC_PIIX4_PCIFunction_usb;
extern const PC_PCIFunction PC_PIIX4_PCIFunction_power_management;


/***************************/
/* 82371AB (PIIX4) - Timer */
/***************************/
// Timer/Counters

typedef void (PC_TimerOutChanged) (
                                   const int  timer,
                                   const bool out
                                   );

void
PC_timers_init (
                PC_Warning         *warning,
                PC_TimerOutChanged *timer_out_changed,
                void               *udata,
                const PC_Config    *config
                );

void
PC_timers_reset (void);

void
PC_timers_set_mode_trace (
                          const bool val
                          );

int
PC_timers_next_event_cc (void);

void
PC_timers_end_iter (void);

void
PC_timers_control_write (
                         const uint8_t data
                         );

void
PC_timers_data_write (
                      const int     id,
                      const uint8_t data
                      );

uint8_t
PC_timers_data_read (
                     const int id
                     );

void
PC_timers_gate2_set (
                     const bool val
                     );

bool
PC_timers_gate2_get (void);

bool
PC_timers_out2_get (void);

bool
PC_timers_get_refresh_request_toggle (void);


/********************************************/
/* 82371AB (PIIX4) - Power Management Timer */
/********************************************/
// PMTimer

void
PC_pmtimer_init (
                 PC_Warning *warning,
                 void       *udata
                 );

int
PC_pmtimer_next_event_cc (void);

void
PC_pmtimer_end_iter (void);

// Torna el valor del timer (24-bit)
uint32_t
PC_pmtimer_get (void);




/*************************/
/* 82371AB (PIIX4) - RTC */
/*************************/
// Real Time Clock

#define PC_CMOSRAM_SIZE 256

// Tipus de la funció utilitzada per a demanar la memòria RAM CMOS. Es
// supossa que esta memòria es estàtica. Ha de ser de 256 bytes.
typedef uint8_t * (PC_GetCMOSRAM) (
                                   void *udata
                                   );

// Utilitzat per a tracejar els accessoss a la CMOSRAM.
typedef void (PC_CMOSRAMAccess) (
                                 const bool    read,
                                 const uint8_t addr,  // 0x00..0x7F
                                 const uint8_t data
                                 );

// S'utilitza per a inicialitzar el rellotge cada vegada que
// s'inicialitza el simulador.
// day_week 1 (Dilluns) ... 7
// month 1 ... 12
// hh 0 ... 23
typedef void (PC_GetCurrentTime) (
                                  void    *udata,
                                  uint8_t *ss,
                                  uint8_t *mm,
                                  uint8_t *hh,
                                  uint8_t *day_week,
                                  uint8_t *day_month,
                                  uint8_t *month,
                                  int     *year
                                  );

void
PC_rtc_init (
             PC_Warning        *warning,
             PC_GetCurrentTime *get_current_time,
             PC_GetCMOSRAM     *get_cmos_ram,
             PC_CMOSRAMAccess  *cmos_ram_access, // Pot ser NULL
             void              *udata,
             const PC_Config   *config
             );

int
PC_rtc_next_event_cc (void);

void
PC_rtc_end_iter (void);

void
PC_rtc_write_rtci (
                   const uint8_t data
                   );

// OJO!!! Açò teoricament no es llig però aparentment es llig
uint8_t
PC_rtc_rtci_read (void);

uint8_t
PC_rtc_rtcd_read (void);

void
PC_rtc_rtcd_write (
                   const uint8_t data
                   );

void
PC_rtc_set_mode_trace (
                       const bool val
                       );


/*****************************/
/* 82371AB (PIIX4) - DMA ISA */
/****************************/
// Controlador DMA ISA (no confundir amb el PCI DMA)

// Utilitzat per a tracejar accessos al mapa de memòria.
typedef void (PC_DMATransfer8) (
                                const int      channel,
                                const uint32_t addr,
                                const uint8_t  data,
                                const bool     is_read,
                                void          *udata
                                );
typedef void (PC_DMATransfer16) (
                                 const int       channel,
                                 const uint32_t  addr,
                                 const uint16_t  data,
                                 const bool      is_read,
                                 void           *udata
                                 );

void
PC_dma_init (
             PC_Warning       *warning,
             PC_DMATransfer8  *dma_transfer8,
             PC_DMATransfer16 *dma_transfer16,
             void             *udata,
             const PC_Config  *config
             );

void
PC_dma_reset (void);

void
PC_dma_set_mode_jit (
                     const bool val
                     );

void
PC_dma_set_mode_trace (
                       const bool val
                       );

int
PC_dma_next_event_cc (void);

void
PC_dma_end_iter (void);

void
PC_dma_dcom_write (
                   const int     dmaid,
                   const uint8_t data
                   );

void
PC_dma_dr_write (
                 const int     dmaid,
                 const uint8_t data
                 );

// No importa el valor que s'escriga.
void
PC_dma_dmc_write (
                  const int dmaid
                  );

// No importa el valor que s'escriga.
void
PC_dma_dclm_write (
                   const int dmaid
                   );

void
PC_dma_dcm_write (
                  const int     dmaid,
                  const uint8_t data
                  );

void
PC_dma_wsmb_write (
                   const int     dmaid,
                   const uint8_t data
                   );

void
PC_dma_dcbp_write (
                   const int dmaid
                   );

uint8_t
PC_dma_status (
               const int dmaid
               );

void
PC_dma_dbaddr_write (
                     const int     chn_id, // 0 .. 7
                     const uint8_t data
                     );

uint8_t
PC_dma_dbaddr_read (
                    const int chn_id // 0 .. 7
                    );

void
PC_dma_dbcnt_write (
                    const int     chn_id, // 0 .. 7
                    const uint8_t data
                    );

uint8_t
PC_dma_dbcnt_read (
                   const int chn_id // 0 .. 7
                   );

void
PC_dma_dlpage_write (
                     const int     chn_id, // 0 .. 7
                     const uint8_t data
                     );

uint8_t
PC_dma_dlpage_read (
                    const int chn_id // 0 .. 7
                    );

// Senyal per a demanar al controlador l'inici d'una transferència.
void
PC_dma_dreq (
             const int  chn_id,
             const bool val
             );


/******************************************/
/* 82371AB (PIIX4) - Interrupt Controller */
/******************************************/
// Controlador d'interrupcions.

// Es crida cada vegada que el xip d'interrupcions envia una
// interrupció a la UCP.
typedef void (PC_InterruptionServiced) (
                                        const int     irq, // 0 .. 15
                                        const uint8_t vec
                                        );

void
PC_ic_init (
            PC_Warning              *warning,
            PC_InterruptionServiced *int_serviced,
            void                    *udata,
            const PC_Config         *config
            );

void
PC_ic_reset (void);

void
PC_ic_set_mode_trace (
                      const bool val
                      );

void
PC_ic_irq (
           const int  irq,
           const bool in
           );

void
PC_ic_cmd_write (
                 const int     icid,
                 const uint8_t data
                 );

void
PC_ic_data_write (
                  const int     icid,
                  const uint8_t data
                  );

uint8_t
PC_ic_cmd_read (
                const int icid
                );

uint8_t
PC_ic_data_read (
                 const int icid
                 );

// line--> 0:A, 1:B, 2:C, 3:D
uint8_t
PC_ic_pirqrc_read (
                   const int line
                   );

// line--> 0:A, 1:B, 2:C, 3:D
void
PC_ic_pirqrc_write (
                    const int     line,
                    const uint8_t data
                    );

// reg 0 o 1
uint8_t
PC_ic_elcr_read (
                 const int reg
                 );

// reg 0 o 1
void
PC_ic_elcr_write (
                  const int     reg,
                  const uint8_t data
                  );


/****************/
/* Floppy Disks */
/****************/
// Controlador de disqueteres

// Utilitzat per a tracejar accessos al mapa de memòria.
typedef void (PC_FloppyFIFOAccess) (
                                    const int      drive,
                                    const uint8_t  data,
                                    const bool     is_read,
                                    const bool     in_exec_phase,
                                    const bool     in_dma,
                                    void          *udata
                                    );

void
PC_fd_set_mode_trace (
                      const bool val
                      );

void
PC_fd_init (
            PC_Warning          *warning,
            PC_FloppyFIFOAccess *fifo_access,
            void                *udata,
            const PC_Config     *config
            );

void
PC_fd_reset (void);

int
PC_fd_next_event_cc (void);

void
PC_fd_end_iter (void);

void
PC_fd_dor_write (
                 const uint8_t data
                 );

uint8_t
PC_fd_msr_read (void);

void
PC_fd_fifo_write (
                  const uint8_t data
                  );

uint8_t
PC_fd_fifo_read (void);

void
PC_fd_ccr_write (
                 const uint8_t data
                 );

// Si file és NULL lleva el disquet (pero no esborra res). Si no és
// NULL reemplaça l'anterior. Com algunes imatges estan truncades vaig
// a permitir que les dimensions siguen inferiors, però internament
// s'assumeix que el que falta és 0.
// Aquesta funció sempre elimina l'anterior si és necessari.
PC_Error
PC_fd_insert_floppy (
                     PC_File   *file,
                     const int  drv
                     );

void
PC_fd_dma_signal (
                  const PC_DMA_Signal signal
                  );

// Llig un byte del fifo durant una operació de lectura
uint8_t
PC_fd_dma_read (void);


/********/
/* PS/2 */
/********/
// 8042 PS/2 controller

void
PC_ps2_init (
             PC_Warning      *warning,
             void            *udata,
             const PC_Config *config
             );

void
PC_ps2_reset (void);

int
PC_ps2_next_event_cc (void);

void
PC_ps2_end_iter (void);

void
PC_ps2_data_write (
                   const uint8_t data
                   );

uint8_t
PC_ps2_data_read (void);

uint8_t
PC_ps2_status (void);

void
PC_ps2_command (
                const uint8_t data
                );


/**************/
/* PC Speaker */
/**************/
// PC Speaker

void
PC_speaker_init (
                 PC_Warning *warning,
                 void       *udata
                 );

int
PC_speaker_next_event_cc (void);

void
PC_speaker_end_iter (void);

void
PC_speaker_reset (void);

void
PC_speaker_set_out (
                    const bool val
                    );

void
PC_speaker_enable_timer (
                         const bool val
                         );

void
PC_speaker_data_enable (
                        const bool enabled
                        );

bool
PC_speaker_get_enabled (void);


/*******************/
/* SoundBlaster 16 */
/*******************/
// SoundBlaster 16 (ISA). Està sempre instal·lada amb configuració
// estàndar:
//
//  SET BLASTER=A220 I5 D1 [H5 M220 P330]
//

void
PC_sb16_init (
              PC_Warning *warning,
              void       *udata
              );

int
PC_sb16_next_event_cc (void);

void
PC_sb16_end_iter (void);

void
PC_sb16_reset (void);

uint8_t
PC_sb16_fm_status (void);

void
PC_sb16_fm_set_addr (
                     const uint8_t addr,
                     const int     array
                     );

void
PC_sb16_fm_write_data (
                       const uint8_t data,
                       const int     array
                       );

void
PC_sb16_dsp_reset (
                   const uint8_t data
                   );

uint8_t
PC_sb16_dsp_read_data (void);

uint8_t
PC_sb16_dsp_read_buffer_status (void);

uint8_t
PC_sb16_dsp_write_buffer_status (void);

void
PC_sb16_dsp_write (
                   const uint8_t data
                   );

uint8_t
PC_sb16_dsp_ack_dma16_irq (void);

void
PC_sb16_dma_signal (
                    const PC_DMA_Signal signal
                    );

void
PC_sb16_dma16_signal (
                      const PC_DMA_Signal signal
                      );

void
PC_sb16_dma_write (
                   const uint8_t data
                   );

void
PC_sb16_dma16_write (
                     const uint16_t data
                     );

void
PC_sb16_mixer_set_addr (
                        const uint8_t addr
                        );

uint8_t
PC_sb16_mixer_read_data (void);

void
PC_sb16_mixer_write_data (
                          const uint8_t data
                          );


/*******/
/* I/O */
/*******/
// Mapa de ports E/E. Basat en 430TX
// Utilitzat per a tracejar accessos als ports I/O.
// El tipus READ64 no s'utilitza mai en aquest callback!!!
typedef void (PC_PortAccess) (
                              const PC_MemAccessType  type,
                              const uint16_t          port,
                              const uint32_t          data,
                              void                   *udata
                              );

// Utilitzat per SeaBios per a imprimir informació de depuració.
typedef void (PC_WriteSeaBiosDebugPort) (
                                         const char  c,
                                         void       *udata
                                         );

// El 'game port' s'implementa com una funció.
// L'argument que rep és el byte que s'espera abans de llegir. Però
// típicament eixe byte és basura.
// Torna l'estat del game port. Vore flags.
typedef uint8_t (PC_GamePort) (const uint8_t  byte,
                               void          *udata);

#define PC_GAME_PORT_AXIS_X   0x01
#define PC_GAME_PORT_AXIS_Y   0x02
#define PC_GAME_PORT_DELTA_X  0x04
#define PC_GAME_PORT_DELTA_Y  0x08
#define PC_GAME_PORT_BUTTON_A 0x10
#define PC_GAME_PORT_BUTTON_B 0x20
#define PC_GAME_PORT_BUTTON_C 0x40
#define PC_GAME_PORT_BUTTON_D 0x80

void
PC_io_init (
            PC_Warning               *warning,
            PC_WriteSeaBiosDebugPort *write_sb_dbg_port,
            PC_PortAccess            *port_access,
            const PC_PCICallbacks    *pci_devs[], // Acava en NULL
            void                     *udata,
            const PC_Config          *config
            );

void
PC_io_reset (void);

void
PC_io_set_mode_trace (
                      const bool val
                      );


/*******/
/* PCI */
/*******/
// Dispositius PCI

// NOTA! plane es fica a -1 si s'esta accedint en mode linear o extés.
typedef void (PC_VGAMemAccess) (
                                const bool      is_read,
                                const int       plane,
                                const uint32_t  offset,
                                const uint8_t   data,
                                void           *udata
                                );
                              
// Utilitzat per a tracejar accessos al mapa de memòria.
typedef void (PC_VGAMemLinearAccess) (
                                      const PC_MemAccessType  type,
                                      const int               aperture,
                                      const uint32_t          addr,
                                      const uint64_t          data,
                                      void                   *udata
                                      );
PC_Error
PC_svga_cirrus_clgd5446_init (
                              PC_Warning            *warning,
                              PC_UpdateScreen       *update_screen,
                              PC_VGAMemAccess       *vga_mem_access,
                              PC_VGAMemLinearAccess *vga_mem_linear_acces,
                              uint8_t               *optrom,
                              const size_t           optrom_size,
                              void                  *udata
                              );

// Torna un punter a la memòria interna de 1MB
const uint8_t *
PC_svga_cirrus_clgd5446_get_vram (void);

extern const PC_PCICallbacks PC_svga_cirrus_clgd5446;


/********/
/* MAIN */
/********/
// Funcions principals del simulador

#ifdef PC_DEBUG
#define PC_MSGF(FORMAT,...) PC_msg((FORMAT), __VA_ARGS__)
#define PC_MSG(MSG) PC_msg((MSG))
#else
#define PC_MSGF(FORMAT,...) {}
#define PC_MSG(MSG) {}
#endif
                              
// Clocks que es porten executats en l'actual iteració. Pot anar
// canviant durant la iteració.
extern int PC_Clock;

// Cicles per segon que executa el processador
extern long PC_ClockFreq;

// Cicles fins al següent event
extern int PC_NextEventCC;

// Tipus de funció per a saber quin a sigut l'últim pas d'execució de
// la UCP.
typedef void (PC_CPUInst) (
                           const IA32_Inst *inst, // Punter a inst
                           const uint32_t   eip, // Comptador programa
                           void            *udata
                           );

typedef void (PC_TraceSoftInt) (
                                void           *udata,
                                const uint8_t   vector,
                                const IA32_CPU *cpu
                                );

// Els camps poden ser NULL.
typedef struct
{

  PC_CPUInst    *cpu_inst; // Es crida cada vegada que s'executa una
                           // instrucció.
  PC_MemAccess  *mem_access; // Es crida cada vegada que es produeix un
                             // accés al mapa de memòria física.
  PC_PortAccess *port_access; // Es crida cada vegada que es produeix un
                              // accés als ports I/O.
  PC_PCIRegAccess *pci_reg_access; // Es crida cada vegada que
                                   // s'accedeix a un registre PCI
  PC_CMOSRAMAccess *cmos_ram_access;
  PC_TimerOutChanged *timer_out_changed;
  PC_InterruptionServiced *int_serviced;
  PC_VGAMemAccess *vga_mem_access;
  PC_VGAMemLinearAccess *vga_mem_linear_access;
  PC_FloppyFIFOAccess *floppy_fifo_access;
  PC_DMATransfer8 *dma_transfer8;
  PC_DMATransfer16 *dma_transfer16;
  PC_TraceSoftInt *trace_soft_int;
  
} PC_TraceCallbacks;

// Conté informació necessària per a comunicar-se amb el 'frontend'.
typedef struct
{

  PC_Warning               *warning;    // Funció per a mostrar avisos.
  PC_WriteSeaBiosDebugPort *write_sb_dbg_port; // Pot ser NULL.
  PC_GetCMOSRAM            *get_cmos_ram;
  PC_GetCurrentTime        *get_current_time;
  PC_UpdateScreen          *update_screen;
  PC_PlaySound             *play_sound;
  const PC_TraceCallbacks  *trace; // Pot ser NULL.
  
} PC_Frontend;

// Inicialitza la llibreria.
// ide_devices --> Primera dimensió: bloc 0 o 1
//             --> Segona dimensió: master - slave
PC_Error
PC_init (
         uint8_t           *bios,
         size_t             bios_size,
         PC_IDEDevice       ide_devices[2][2],
         const PC_Frontend *frontend,  // Frontend
         void              *udata,     // Dades frontend
         const PC_Config   *config
         );

// Executa cicles del PC. Aquesta funció executa almenys els cicles
// indicats (probablement n'executarà uns pocs més).
int
PC_iter (
         const int cc
         );

// Executa cicles del PC en mode JIT. Aquesta funció executa almenys
// els cicles indicats (probablement n'executarà uns pocs més). El
// mode JIT no està pensat per a ser intercalat amb el mode
// normal. Fer-ho pot provocar comportaments inesperats.
int
PC_jit_iter (
             const int cc
             );

// Executa el següent pas d'UCP en mode traça. Tot aquelles funcions
// de 'callback' que no són nul·les es cridaran si és el cas. Torna el
// número de cicles.
int
PC_trace (void);

// Igual que PC_trace però gastant JIT per a l'execució
int
PC_jit_trace (void);

void
PC_close (void);

// Per a indicar que s'ha presionat una tecla
void
PC_kbd_press (
              const PC_Scancode key
              );

// Per a indicar que s'ha soltat una tecla
void
PC_kbd_release (
                const PC_Scancode key
                );

// Per a forçar de colp que totes les tecles es solten.
void
PC_kbd_clear (void);

// Per a indicar al simulador que s'ha mogut el ratolí. S'especifica
// el desplaçament en x (deltax) i y (deltay) des de l'última cridada
// en píxels.
void
PC_mouse_motion (
                 int32_t deltax,
                 int32_t deltay
                 );

// Indica que s'ha apretat el botó.
void
PC_mouse_button_press (
                       const PC_MouseButton but
                       );

// Indica que s'ha soltat el botó.
void
PC_mouse_button_release (
                         const PC_MouseButton but
                         );

// Reseteja l'estat intern que controla el moviment del
// ratolí. Interessant cridar-la cada vegada que el simulador perd el
// focus del ratolí.
void
PC_mouse_motion_clear (void);

// Modifica en calent les característiques del ratolí.
void
PC_set_host_mouse (
                   PC_HostMouse host_mouse
                   );

// Connecta un mando al 'game port'. El mando s'implementa com una
// funció. Per a desconnectar-lo es pot passar NULL.
// Inicialment sempre està NULL.
// Resetejar no desconnecta.
void
PC_connect_game_port (
                      PC_GamePort *game_port
                      );

// Emprat per a mostrar missatges de depuració.
void
PC_msg (
        const char *fmt,
        ...
        );


#endif // __PC_H__
