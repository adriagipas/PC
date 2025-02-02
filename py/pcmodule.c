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
 *  pcmodule.c - Mòdul que implementa un simulador de PC antic.
 *
 */

#include <Python.h>
#include <SDL/SDL.h>
#include <SDL/SDL_mouse.h>
#include <GL/gl.h>
#include <GL/glext.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <glib.h>
#include <time.h>

#include "PC.h"




/**********/
/* MACROS */
/**********/

#define CHECK_INITIALIZED                                               \
  do {                                                                  \
    if ( !_initialized )                                                \
      {                                                                 \
        PyErr_SetString ( PCError, "Module must be initialized" );      \
        return NULL;                                                    \
      }                                                                 \
  } while(0)

#define SHOW_EIP_CC        					\
  if ( _tracer.dbg_flags&DBG_SHOW_EIP_CC )        		\
    printf ( "STP: %016lX CC: %016lX EIP: %08X  ",              \
             _tracer.steps, _tracer.cc, _tracer.eip )

#define DBG_SHOW_EIP_CC        0x0001
#define DBG_CPU_INST           0x0002
#define DBG_MEM_ACCESS         0x0004
#define DBG_PORT_ACCESS        0x0008
#define DBG_PCI_REG_ACCESS     0x0010
#define DBG_CMOS_RAM_ACCESS    0x0020
#define DBG_TIMER_OUT_CHANGED  0x0040
#define DBG_INT_SERVICED       0x0080
#define DBG_VGA_MEM_ACCESS     0x0100
#define DBG_FLOPPY_FIFO_ACCESS 0x0200
#define DBG_DMA_TRANSFER8      0x0400
#define DBG_TRACE_SOFT_INT     0x0800
#define DBG_DMA_TRANSFER16     0x1000

#define NBUFF 4




/*********/
/* TIPUS */
/*********/

typedef struct
{
  
  int16_t      *v;
  volatile int  full;
  
} buffer_t;




/*********/
/* ESTAT */
/*********/

// Error.
static PyObject *PCError;

// Inicialitzat.
static bool _initialized;

// Es fica a cert quan es veu una instrucció desconeguda.
static bool _unk_inst= false;

// Pantalla.
static struct
{
  
  int          width;
  int          height;
  SDL_Surface *surface;
  
} _screen;

// Estat so.
static struct
{
  
  buffer_t buffers[NBUFF];
  int      buff_in;
  int      buff_out;
  char     silence;
  int      pos;
  int      size;
  int      nsamples;
  double   ratio;
  double   pos2;
  
} _audio;


// BIOS.
static uint8_t *_bios;
static uint8_t *_vgabios;

// Tracer.
static struct
{
  int       dbg_flags;
  uint64_t  cc;
  uint32_t  eip;
  uint64_t  steps;
} _tracer;

// HDD
static PC_File *_hdd;

// FD
static PC_File *_fd[4];

// CDROM
static PC_CDRom *_cdrom;

// Unix epoch
static int _use_unix_epoch;

// Ratolí.
static struct
{
  bool active;
} _mouse;





/*********/
/* DEBUG */
/*********/

static const char *
get_inst_mnemonic (
        	   const IA32_Mnemonic name
        	   )
{

  switch ( name )
    {
    case IA32_AAD         : return "AAD        ";
    case IA32_AAM         : return "AAM        ";
    case IA32_AAS         : return "AAS        ";
    case IA32_ADC8        : return "ADCb       ";
    case IA32_ADC16       : return "ADCw       ";
    case IA32_ADC32       : return "ADCd       ";
    case IA32_ADD8        : return "ADDb       ";
    case IA32_ADD16       : return "ADDw       ";
    case IA32_ADD32       : return "ADDd       ";
    case IA32_AND16       : return "ANDw       ";
    case IA32_AND8        : return "ANDb       ";
    case IA32_AND32       : return "ANDd       ";
    case IA32_BOUND16     : return "BOUNDw     ";
    case IA32_BOUND32     : return "BOUNDd     ";
    case IA32_BSF16       : return "BSFw       ";
    case IA32_BSF32       : return "BSFd       ";
    case IA32_BSR16       : return "BSRw       ";
    case IA32_BSR32       : return "BSRd       ";
    case IA32_BSWAP       : return "BSWAP      ";
    case IA32_BT16        : return "BTw        ";
    case IA32_BT32        : return "BTd        ";
    case IA32_BTC16       : return "BTCw       ";
    case IA32_BTC32       : return "BTCd       ";
    case IA32_BTR16       : return "BTRw       ";
    case IA32_BTR32       : return "BTRd       ";
    case IA32_BTS16       : return "BTSw       ";
    case IA32_BTS32       : return "BTSd       ";
    case IA32_CALL32_FAR  : return "lCALLd     ";
    case IA32_CALL16_FAR  : return "lCALLw     ";
    case IA32_CALL32_NEAR : return "CALLd      ";
    case IA32_CALL16_NEAR : return "CALLw      ";
    case IA32_CBW         : return "CBW        ";
    case IA32_CDQ         : return "CDQ        ";
    case IA32_CLC         : return "CLC        ";
    case IA32_CLD         : return "CLD        ";
    case IA32_CLI         : return "CLI        ";
    case IA32_CLTS        : return "CLTS       ";
    case IA32_CMC         : return "CMC        ";
    case IA32_CMP8        : return "CMPb       ";
    case IA32_CMP16       : return "CMPw       ";
    case IA32_CMP32       : return "CMPd       ";
    case IA32_CMPS8       : return "CMPSb      ";
    case IA32_CMPS16      : return "CMPSw      ";
    case IA32_CMPS32      : return "CMPSd      ";
    case IA32_CPUID       : return "CPUID      ";
    case IA32_CWD         : return "CWD        ";
    case IA32_CWDE        : return "CWDE       ";
    case IA32_DAA         : return "DAA        ";
    case IA32_DAS         : return "DAS        ";
    case IA32_DEC8        : return "DECb       ";
    case IA32_DEC16       : return "DECw       ";
    case IA32_DEC32       : return "DECd       ";
    case IA32_DIV8        : return "DIVb       ";
    case IA32_DIV16       : return "DIVw       ";
    case IA32_DIV32       : return "DIVd       ";
    case IA32_ENTER16     : return "ENTERw     ";
    case IA32_ENTER32     : return "ENTERd     ";
    case IA32_F2XM1       : return "F2XM1      ";
    case IA32_FABS        : return "FABS       ";
    case IA32_FADD32      : return "FADDf      ";
    case IA32_FADD64      : return "FADDd      ";
    case IA32_FADD80      : return "FADD       ";
    case IA32_FADDP80     : return "FADDP      ";
    case IA32_FBSTP       : return "FBSTP      ";
    case IA32_FCHS        : return "FCHS       ";
    case IA32_FCLEX       : return "FCLEX      ";
    case IA32_FCOM32      : return "FCOMf      ";
    case IA32_FCOM64      : return "FCOMd      ";
    case IA32_FCOM80      : return "FCOM       ";
    case IA32_FCOMP32     : return "FCOMPf     ";
    case IA32_FCOMP64     : return "FCOMPd     ";
    case IA32_FCOMP80     : return "FCOMP      ";
    case IA32_FCOMPP      : return "FCOMPP     ";
    case IA32_FCOS        : return "FCOS       ";
    case IA32_FDIV32      : return "FDIVf      ";
    case IA32_FDIV64      : return "FDIVd      ";
    case IA32_FDIV80      : return "FDIV       ";
    case IA32_FDIVP80     : return "FDIVP      ";
    case IA32_FDIVR32     : return "FDIVRf     ";
    case IA32_FDIVR64     : return "FDIVRd     ";
    case IA32_FDIVR80     : return "FDIVR      ";
    case IA32_FDIVRP80    : return "FDIVRP     ";
    case IA32_FFREE       : return "FFREE      ";
    case IA32_FILD16      : return "FILDw      ";
    case IA32_FILD32      : return "FILDd      ";
    case IA32_FILD64      : return "FILDld     ";
    case IA32_FIMUL32     : return "FIMULw     ";
    case IA32_FINIT       : return "FINIT      ";
    case IA32_FIST32      : return "FISTd      ";
    case IA32_FISTP16     : return "FISTPw     ";
    case IA32_FISTP32     : return "FISTPd     ";
    case IA32_FISTP64     : return "FISTPld    ";
    case IA32_FLD1        : return "FLD1       ";
    case IA32_FLD32       : return "FLDf       ";
    case IA32_FLD64       : return "FLDd       ";
    case IA32_FLD80       : return "FLD        ";
    case IA32_FLDCW       : return "FLDCW      ";
    case IA32_FLDL2E      : return "FLDL2E     ";
    case IA32_FLDLN2      : return "FLDLN2     ";
    case IA32_FLDZ        : return "FLDZ       ";
    case IA32_FMUL32      : return "FMULf      ";
    case IA32_FMUL64      : return "FMULd      ";
    case IA32_FMUL80      : return "FMUL       ";
    case IA32_FMULP80     : return "FMULP      ";
    case IA32_FNSTSW      : return "FNSTSW     ";
    case IA32_FPATAN      : return "FPATAN     ";
    case IA32_FPREM       : return "FPREM      ";
    case IA32_FPTAN       : return "FPTAN      ";
    case IA32_FRNDINT     : return "FRNDINT    ";
    case IA32_FRSTOR16    : return "FRSTORw     ";
    case IA32_FRSTOR32    : return "FRSTORd     ";
    case IA32_FSAVE16     : return "FSAVEw     ";
    case IA32_FSAVE32     : return "FSAVEd     ";
    case IA32_FSCALE      : return "FSCALE     ";
    case IA32_FSETPM      : return "FSETPM     ";
    case IA32_FSIN        : return "FSIN       ";
    case IA32_FSQRT       : return "FSQRT      ";
    case IA32_FST32       : return "FSTf       ";
    case IA32_FST64       : return "FSTd       ";
    case IA32_FST80       : return "FST        ";
    case IA32_FSTP32      : return "FSTPf      ";
    case IA32_FSTP64      : return "FSTPd      ";
    case IA32_FSTP80      : return "FSTP       ";
    case IA32_FSTCW       : return "FSTCW      ";
    case IA32_FSTSW       : return "FSTSW      ";
    case IA32_FSUB80      : return "FSUB       ";
    case IA32_FSUB64      : return "FSUBd      ";
    case IA32_FSUB32      : return "FSUBf      ";
    case IA32_FSUBP80     : return "FSUBP      ";
    case IA32_FSUBR32     : return "FSUBRf     ";
    case IA32_FSUBR64     : return "FSUBRd     ";
    case IA32_FSUBR80     : return "FSUBR      ";
    case IA32_FSUBRP80    : return "FSUBRP     ";
    case IA32_FTST        : return "FTST       ";
    case IA32_FXAM        : return "FXAM       ";
    case IA32_FXCH        : return "FXCH       ";
    case IA32_FYL2X       : return "FYL2X      ";
    case IA32_FWAIT       : return "FWAIT      ";
    case IA32_HLT         : return "HLT        ";
    case IA32_IDIV8       : return "IDIVb      ";
    case IA32_IDIV16      : return "IDIVw      ";
    case IA32_IDIV32      : return "IDIVd      ";
    case IA32_IMUL8       : return "IMULb      ";
    case IA32_IMUL16      : return "IMULw      ";
    case IA32_IMUL32      : return "IMULd      ";
    case IA32_IN          : return "IN         ";
    case IA32_INC8        : return "INCb       ";
    case IA32_INC16       : return "INCw       ";
    case IA32_INC32       : return "INCd       ";
    case IA32_INS8        : return "INSb       ";
    case IA32_INS16       : return "INSw       ";
    case IA32_INS32       : return "INSd       ";
    case IA32_INT16       : return "INTw       ";
    case IA32_INT32       : return "INTd       ";
    case IA32_INTO16      : return "INTOw      ";
    case IA32_INTO32      : return "INTOd      ";
    case IA32_INVLPG16    : return "INVLPGw    ";
    case IA32_INVLPG32    : return "INVLPGd    ";
    case IA32_IRET16      : return "IRETw      ";
    case IA32_IRET32      : return "IRETd      ";
    case IA32_JA32        : return "JAd        ";
    case IA32_JA16        : return "JAw        ";
    case IA32_JAE32       : return "JAEd       ";
    case IA32_JAE16       : return "JAEw       ";
    case IA32_JB32        : return "JBd        ";
    case IA32_JB16        : return "JBw        ";
    case IA32_JCXZ32      : return "JCXZd      ";
    case IA32_JCXZ16      : return "JCXZw      ";
    case IA32_JE32        : return "JEd        ";
    case IA32_JE16        : return "JEw        ";
    case IA32_JECXZ32     : return "JECXZd     ";
    case IA32_JECXZ16     : return "JECXZw     ";
    case IA32_JG32        : return "JGd        ";
    case IA32_JG16        : return "JGw        ";
    case IA32_JGE32       : return "JGEd       ";
    case IA32_JGE16       : return "JGEw       ";
    case IA32_JL32        : return "JLd        ";
    case IA32_JL16        : return "JLw        ";
    case IA32_JMP32_FAR   : return "lJMPd      ";
    case IA32_JMP16_FAR   : return "lJMPw      ";
    case IA32_JMP32_NEAR  : return "JMPd       ";
    case IA32_JMP16_NEAR  : return "JMPw       ";
    case IA32_JNA32       : return "JNAd       ";
    case IA32_JNA16       : return "JNAw       ";
    case IA32_JNE32       : return "JNEd       ";
    case IA32_JNE16       : return "JNEw       ";
    case IA32_JNG32       : return "JNGd       ";
    case IA32_JNG16       : return "JNGw       ";
    case IA32_JNO32       : return "JNOd       ";
    case IA32_JNO16       : return "JNOw       ";
    case IA32_JNS32       : return "JNSd       ";
    case IA32_JNS16       : return "JNSw       ";
    case IA32_JO32        : return "JOd        ";
    case IA32_JO16        : return "JOw        ";
    case IA32_JP32        : return "JPd        ";
    case IA32_JP16        : return "JPw        ";
    case IA32_JPO32       : return "JPOd       ";
    case IA32_JPO16       : return "JPOw       ";
    case IA32_JS32        : return "JSd        ";
    case IA32_JS16        : return "JSw        ";
    case IA32_LAHF        : return "LAHF       ";
    case IA32_LAR16       : return "LARw       ";
    case IA32_LAR32       : return "LARd       ";
    case IA32_LDS16       : return "LDSw       ";
    case IA32_LDS32       : return "LDSd       ";
    case IA32_LEA16       : return "LEAw       ";
    case IA32_LEA32       : return "LEAd       ";
    case IA32_LEAVE16     : return "LEAVEw     ";
    case IA32_LEAVE32     : return "LEAVEd     ";
    case IA32_LES16       : return "LESw       ";
    case IA32_LES32       : return "LESd       ";
    case IA32_LFS16       : return "LFSw       ";
    case IA32_LFS32       : return "LFSd       ";
    case IA32_LGDT16      : return "LGDTw      ";
    case IA32_LGDT32      : return "LGDTd      ";
    case IA32_LGS16       : return "LGSw       ";
    case IA32_LGS32       : return "LGSd       ";
    case IA32_LIDT16      : return "LIDTw      ";
    case IA32_LIDT32      : return "LIDTd      ";
    case IA32_LLDT        : return "LLDT       ";
    case IA32_LMSW        : return "LMSW       ";
    case IA32_LODS8       : return "LODSb      ";
    case IA32_LODS16      : return "LODSw      ";
    case IA32_LODS32      : return "LODSd      ";
    case IA32_LOOP16      : return "LOOPw      ";
    case IA32_LOOP32      : return "LOOPd      ";
    case IA32_LOOPE16     : return "LOOPEw     ";
    case IA32_LOOPE32     : return "LOOPEd     ";
    case IA32_LOOPNE16    : return "LOOPNEw    ";
    case IA32_LOOPNE32    : return "LOOPNEd    ";
    case IA32_LSL16       : return "LSLw       ";
    case IA32_LSL32       : return "LSLd       ";
    case IA32_LSS16       : return "LSSw       ";
    case IA32_LSS32       : return "LSSd       ";
    case IA32_LTR         : return "LTR        ";
    case IA32_MOV8        : return "MOVb       ";
    case IA32_MOV16       : return "MOVw       ";
    case IA32_MOV32       : return "MOVd       ";
    case IA32_MOVS8       : return "MOVSb      ";
    case IA32_MOVS16      : return "MOVSw      ";
    case IA32_MOVS32      : return "MOVSd      ";
    case IA32_MOVSX16     : return "MOVSXw     ";
    case IA32_MOVSX32W    : return "MOVSXdw    ";
    case IA32_MOVSX32B    : return "MOVSXdb    ";
    case IA32_MOVZX16     : return "MOVZXw     ";
    case IA32_MOVZX32W    : return "MOVZXdw    ";
    case IA32_MOVZX32B    : return "MOVZXdb    ";
    case IA32_MUL8        : return "MULb       ";
    case IA32_MUL16       : return "MULw       ";
    case IA32_MUL32       : return "MULd       ";
    case IA32_NEG8        : return "NEGb       ";
    case IA32_NEG16       : return "NEGw       ";
    case IA32_NEG32       : return "NEGd       ";
    case IA32_NOP         : return "NOP        ";
    case IA32_NOT8        : return "NOTb       ";
    case IA32_NOT16       : return "NOTw       ";
    case IA32_NOT32       : return "NOTd       ";
    case IA32_OUT         : return "OUT        ";
    case IA32_OUTS8       : return "OUTSb      ";
    case IA32_OUTS16      : return "OUTSw      ";
    case IA32_OUTS32      : return "OUTSd      ";
    case IA32_OR8         : return "ORb        ";
    case IA32_OR16        : return "ORw        ";
    case IA32_OR32        : return "ORd        ";
    case IA32_POP16       : return "POPw       ";
    case IA32_POP32       : return "POPd       ";
    case IA32_POPA16      : return "POPAw      ";
    case IA32_POPA32      : return "POPAd      ";
    case IA32_POPF16      : return "POPFw      ";
    case IA32_POPF32      : return "POPFd      ";
    case IA32_PUSH16      : return "PUSHw      ";
    case IA32_PUSH32      : return "PUSHd      ";
    case IA32_PUSHA16     : return "PUSHAw     ";
    case IA32_PUSHA32     : return "PUSHAd     ";
    case IA32_PUSHF16     : return "PUSHFw     ";
    case IA32_PUSHF32     : return "PUSHFd     ";
    case IA32_RCL8        : return "RCLb       ";
    case IA32_RCL16       : return "RCLw       ";
    case IA32_RCL32       : return "RCLd       ";
    case IA32_RCR8        : return "RCRb       ";
    case IA32_RCR16       : return "RCRw       ";
    case IA32_RCR32       : return "RCRd       ";
    case IA32_RET32_FAR   : return "lRETd      ";
    case IA32_RET16_FAR   : return "lRETw      ";
    case IA32_RET32_NEAR  : return "RETd       ";
    case IA32_RET16_NEAR  : return "RETw       ";
    case IA32_ROL8        : return "ROLb       ";
    case IA32_ROL16       : return "ROLw       ";
    case IA32_ROL32       : return "ROLd       ";
    case IA32_ROR8        : return "RORb       ";
    case IA32_ROR16       : return "RORw       ";
    case IA32_ROR32       : return "RORd       ";
    case IA32_SAHF        : return "SAHF       ";
    case IA32_SAR8        : return "SARb       ";
    case IA32_SAR16       : return "SARw       ";
    case IA32_SAR32       : return "SARd       ";
    case IA32_SCAS8       : return "SCASb      ";
    case IA32_SCAS16      : return "SCASw      ";
    case IA32_SCAS32      : return "SCASd      ";
    case IA32_SETA        : return "SETA       ";
    case IA32_SETAE       : return "SETAE      ";
    case IA32_SETB        : return "SETB       ";
    case IA32_SETE        : return "SETE       ";
    case IA32_SETG        : return "SETG       ";
    case IA32_SETGE       : return "SETGE      ";
    case IA32_SETL        : return "SETL       ";
    case IA32_SETNA       : return "SETNA      ";
    case IA32_SETNE       : return "SETNE      ";
    case IA32_SETNG       : return "SETNG      ";
    case IA32_SETS        : return "SETS       ";
    case IA32_SBB8        : return "SBBb       ";
    case IA32_SBB16       : return "SBBw       ";
    case IA32_SBB32       : return "SBBd       ";
    case IA32_SGDT16      : return "SGDTw      ";
    case IA32_SGDT32      : return "SGDTd      ";
    case IA32_SHL8        : return "SHLb       ";
    case IA32_SHL16       : return "SHLw       ";
    case IA32_SHL32       : return "SHLd       ";
    case IA32_SHLD16      : return "SHLDw      ";
    case IA32_SHLD32      : return "SHLDd      ";
    case IA32_SHR8        : return "SHRb       ";
    case IA32_SHR16       : return "SHRw       ";
    case IA32_SHR32       : return "SHRd       ";
    case IA32_SHRD16      : return "SHRDw      ";
    case IA32_SHRD32      : return "SHRDd      ";
    case IA32_SIDT16      : return "SIDTw      ";
    case IA32_SIDT32      : return "SIDTd      ";
    case IA32_SLDT        : return "SLDT       ";
    case IA32_SMSW16      : return "SMSWw      ";
    case IA32_SMSW32      : return "SMSWd      ";
    case IA32_STC         : return "STC        ";
    case IA32_STD         : return "STD        ";
    case IA32_STI         : return "STI        ";
    case IA32_STOS8       : return "STOSb      ";
    case IA32_STOS16      : return "STOSw      ";
    case IA32_STOS32      : return "STOSd      ";
    case IA32_STR         : return "STR        ";
    case IA32_SUB8        : return "SUBb       ";
    case IA32_SUB16       : return "SUBw       ";
    case IA32_SUB32       : return "SUBd       ";
    case IA32_TEST8       : return "TESTb      ";
    case IA32_TEST16      : return "TESTw      ";
    case IA32_TEST32      : return "TESTd      ";
    case IA32_VERR        : return "VERR       ";
    case IA32_VERW        : return "VERW       ";
    case IA32_WBINVD      : return "WBINVD     ";
    case IA32_XOR8        : return "XORb       ";
    case IA32_XOR16       : return "XORw       ";
    case IA32_XOR32       : return "XORd       ";
    case IA32_XCHG8       : return "XCHGb      ";
    case IA32_XCHG16      : return "XCHGw      ";
    case IA32_XCHG32      : return "XCHGd      ";
    case IA32_XLATB16     : return "XLATBw     ";
    case IA32_XLATB32     : return "XLATBd     ";
    case IA32_UNK         :
    default               :
      _unk_inst= true;
      return "UNK        ";
    }
  
} // end get_inst_mnemonic


static const char *
get_inst_prefix (
                 const IA32_Prefix prefix
                 )
{

  switch ( prefix )
    {

    case IA32_PREFIX_REP :  return "rep   ";
    case IA32_PREFIX_REPE:  return "repe  ";
    case IA32_PREFIX_REPNE: return "repne ";
    default:
    case IA32_PREFIX_NONE:  return "      ";
    }
  
} // end get_inst_prefix

/*
static const char *
get_inst_reg_name (
        	   const int reg
        	   )
{

  static char buffer[10];


  switch ( reg )
    {
    case 0: return "zero";
    case 1: return "$at";
    case 2 ... 3: sprintf ( buffer, "$v%d", reg-2 ); break;
    case 4 ... 7: sprintf ( buffer, "$a%d", reg-4 ); break;
    case 8 ... 15: sprintf ( buffer, "$t%d", reg-8 ); break;
    case 16 ... 23: sprintf ( buffer, "$s%d", reg-16 ); break;
    case 24 ... 25: sprintf ( buffer, "$t%d", reg-24+8 ); break;
    case 26 ... 27: sprintf ( buffer, "$k%d", reg-26 ); break;
    case 28: return "$gp";
    case 29: return "$sp";
    case 30: return "$fp";
    case 31:
    default: return "$ra";
    }
  
  return &(buffer[0]);
  
} // end get_inst_reg_name
*/

static void
print_data_seg (
                const IA32_InstOp *op
                )
{

  switch ( op->data_seg )
    {
    case IA32_SS: printf ( "SS:" ); break;
    case IA32_ES: printf ( "ES:" ); break;
    case IA32_CS: printf ( "CS:" ); break;
    case IA32_FS: printf ( "FS:" ); break;
    case IA32_GS: printf ( "GS:" ); break;
    case IA32_UNK_SEG: printf ( "UNK:" ); break;
    case IA32_DS:
    default: break;
    }
  
} // end print_data_seg


static void
print_sib (
           const IA32_InstOp *op
           )
{

  switch ( op->sib_scale )
    {
    case IA32_SIB_SCALE_NONE: break;
    case IA32_SIB_SCALE_EAX: printf ( "EAX + " ); break;
    case IA32_SIB_SCALE_ECX: printf ( "ECX + " ); break;
    case IA32_SIB_SCALE_EDX: printf ( "EDX + " ); break;
    case IA32_SIB_SCALE_EBX: printf ( "EBX + " ); break;
    case IA32_SIB_SCALE_EBP: printf ( "EBP + " ); break;
    case IA32_SIB_SCALE_ESI: printf ( "ESI + " ); break;
    case IA32_SIB_SCALE_EDI: printf ( "EDI + " ); break;
    case IA32_SIB_SCALE_EAX_2: printf ( "EAX*2 + " ); break;
    case IA32_SIB_SCALE_ECX_2: printf ( "ECX*2 + " ); break;
    case IA32_SIB_SCALE_EDX_2: printf ( "EDX*2 + " ); break;
    case IA32_SIB_SCALE_EBX_2: printf ( "EBX*2 + " ); break;

    case IA32_SIB_SCALE_ESI_2: printf ( "ESI*2 + " ); break;
    case IA32_SIB_SCALE_EDI_2: printf ( "EDI*2 + " ); break;
    case IA32_SIB_SCALE_EAX_4: printf ( "EAX*4 + " ); break;
    case IA32_SIB_SCALE_ECX_4: printf ( "ECX*4 + " ); break;
    case IA32_SIB_SCALE_EDX_4: printf ( "EDX*4 + " ); break;
    case IA32_SIB_SCALE_EBX_4: printf ( "EBX*4 + " ); break;
    case IA32_SIB_SCALE_EBP_4: printf ( "EBP*4 + " ); break;
    case IA32_SIB_SCALE_ESI_4: printf ( "ESI*4 + " ); break;
    case IA32_SIB_SCALE_EDI_4: printf ( "EDI*4 + " ); break;
    case IA32_SIB_SCALE_EAX_8: printf ( "EAX*8 + " ); break;

    case IA32_SIB_SCALE_EDX_8: printf ( "EDX*8 + " ); break;
    case IA32_SIB_SCALE_EBX_8: printf ( "EBX*8 + " ); break;
    case IA32_SIB_SCALE_EBP_8: printf ( "EBP*8 + " ); break;
    case IA32_SIB_SCALE_ESI_8: printf ( "ESI*8 + " ); break;
    case IA32_SIB_SCALE_EDI_8: printf ( "EDI*8 + " ); break;
    default: printf ( "??? + (SIB_SCALE_NONE:%d op:%d)",
                      IA32_SIB_SCALE_NONE,op->sib_scale );
    }
  
  switch ( op->sib_val )
    {
    case IA32_SIB_VAL_EAX: printf ( "EAX" ); break;
    case IA32_SIB_VAL_ECX: printf ( "ECX" ); break;
    case IA32_SIB_VAL_EDX: printf ( "EDX" ); break;
    case IA32_SIB_VAL_EBX: printf ( "EBX" ); break;
    case IA32_SIB_VAL_ESP: printf ( "ESP" ); break;
    case IA32_SIB_VAL_DISP32:
      printf ( "%d (%08X)", (int32_t) op->sib_u32, op->sib_u32 );
      break;
    case IA32_SIB_VAL_EBP: printf ( "EBP" ); break;
    case IA32_SIB_VAL_ESI: printf ( "ESI" ); break;
    case IA32_SIB_VAL_EDI: printf ( "EDI" ); break;
    default: printf ( "??? (SIB_VAL_EAX:%d op:%d)",
                      IA32_SIB_VAL_EAX, op->sib_val );
    }
  
} // end print_sib


static void
print_inst_op (
               const IA32_InstOp *op,
               const uint32_t     next_addr
               )
{

  uint32_t tmp;

  
  putchar ( ' ' );
  switch ( op->type )
    {
    case IA32_DR0: printf ( "DR0" ); break;
    case IA32_DR1: printf ( "DR1" ); break;
    case IA32_DR2: printf ( "DR2" ); break;
    case IA32_DR3: printf ( "DR3" ); break;
    case IA32_DR4: printf ( "DR4" ); break;
    case IA32_DR5: printf ( "DR5" ); break;
    case IA32_DR6: printf ( "DR6" ); break;
    case IA32_DR7: printf ( "DR7" ); break;
    case IA32_CR0: printf ( "CR0" ); break;
    case IA32_CR1: printf ( "CR1" ); break;
    case IA32_CR2: printf ( "CR2" ); break;
    case IA32_CR3: printf ( "CR3" ); break;
    case IA32_CR4: printf ( "CR4" ); break;
    case IA32_CR8: printf ( "CR8" ); break;
    case IA32_AL: printf ( "AL" ); break;
    case IA32_CL: printf ( "CL" ); break;
    case IA32_DL: printf ( "DL" ); break;
    case IA32_BL: printf ( "BL" ); break;
    case IA32_AH: printf ( "AH" ); break;
    case IA32_CH: printf ( "CH" ); break;
    case IA32_DH: printf ( "DH" ); break;
    case IA32_BH: printf ( "BH" ); break;
    case IA32_AX: printf ( "AX" ); break;
    case IA32_CX: printf ( "CX" ); break;
    case IA32_DX: printf ( "DX" ); break;
    case IA32_BX: printf ( "BX" ); break;
    case IA32_SP: printf ( "SP" ); break;
    case IA32_BP: printf ( "BP" ); break;
    case IA32_SI: printf ( "SI" ); break;
    case IA32_DI: printf ( "DI" ); break;
    case IA32_EAX: printf ( "EAX" ); break;
    case IA32_ECX: printf ( "ECX" ); break;
    case IA32_EDX: printf ( "EDX" ); break;
    case IA32_EBX: printf ( "EBX" ); break;
    case IA32_ESP: printf ( "ESP" ); break;
    case IA32_EBP: printf ( "EBP" ); break;
    case IA32_ESI: printf ( "ESI" ); break;
    case IA32_EDI: printf ( "EDI" ); break;
    case IA32_SEG_ES: printf ( "ES" ); break;
    case IA32_SEG_CS: printf ( "CS" ); break;
    case IA32_SEG_SS: printf ( "SS" ); break;
    case IA32_SEG_DS: printf ( "DS" ); break;
    case IA32_SEG_FS: printf ( "FS" ); break;
    case IA32_SEG_GS: printf ( "GS" ); break;
    case IA32_XMM0: printf ( "XMM0" ); break;
    case IA32_XMM1: printf ( "XMM1" ); break;
    case IA32_XMM2: printf ( "XMM2" ); break;
    case IA32_XMM3: printf ( "XMM3" ); break;
    case IA32_XMM4: printf ( "XMM4" ); break;
    case IA32_XMM5: printf ( "XMM5" ); break;
    case IA32_XMM6: printf ( "XMM6" ); break;
    case IA32_XMM7: printf ( "XMM7" ); break;
    case IA32_IMM8: printf ( "%02X (%d)", op->u8, (int8_t) op->u8 ); break;
    case IA32_IMM16: printf ( "%04X (%d)", op->u16, (int16_t) op->u16 ); break;
    case IA32_IMM32: printf ( "%08X (%d)", op->u32, (int32_t) op->u32 ); break;
    case IA32_ADDR16_BX_SI:
      print_data_seg ( op );
      printf ( "[BX+SI]" );
      break;
    case IA32_ADDR16_BX_DI:
      print_data_seg ( op );
      printf ( "[BX+DI]" );
      break;

    case IA32_ADDR16_BP_DI:
      print_data_seg ( op );
      printf ( "[BP+DI]" );
      break;
      
    case IA32_ADDR16_BP_DISP8:
      print_data_seg ( op );
      printf ( "[BP" );
      printf ( " + %d (%02X)]", (int8_t) op->u8, op->u8 );
      break;
    case IA32_ADDR16_BX_DISP8:
      print_data_seg ( op );
      printf ( "[BX" );
      printf ( " + %d (%02X)]", (int8_t) op->u8, op->u8 );
      break;
    case IA32_ADDR16_BX_SI_DISP8:
      print_data_seg ( op );
      printf ( "[BX + SI" );
      printf ( " + %d (%02X)]", (int8_t) op->u8, op->u8 );
      break;
    case IA32_ADDR16_BX_DI_DISP8:
      print_data_seg ( op );
      printf ( "[BX + DI" );
      printf ( " + %d (%02X)]", (int8_t) op->u8, op->u8 );
      break;
    case IA32_ADDR16_BP_SI_DISP8:
      print_data_seg ( op );
      printf ( "[BP + SI" );
      printf ( " + %d (%02X)]", (int8_t) op->u8, op->u8 );
      break;
    case IA32_ADDR16_BP_DI_DISP8:
      print_data_seg ( op );
      printf ( "[BP + DI" );
      printf ( " + %d (%02X)]", (int8_t) op->u8, op->u8 );
      break;
      
    case IA32_ADDR16_SI:
      print_data_seg ( op );
      printf ( "[SI]" );
      break;
    case IA32_ADDR16_DI:
      print_data_seg ( op );
      printf ( "[DI]" );
      break;
    case IA32_ADDR16_DISP16:
      printf ( "(" );
      print_data_seg ( op );
      printf ( "%04X)", op->u16 );
      break;
    case IA32_ADDR16_BX:
      print_data_seg ( op );
      printf ( "[BX]" );
      break;

    case IA32_ADDR16_SI_DISP8:
      print_data_seg ( op );
      printf ( "[SI" );
      printf ( " + %d (%02X)]", (int8_t) op->u8, op->u8 );
      break;
    case IA32_ADDR16_DI_DISP8:
      print_data_seg ( op );
      printf ( "[DI" );
      printf ( " + %d (%02X)]", (int8_t) op->u8, op->u8 );
      break;

    case IA32_ADDR16_BX_DI_DISP16:
      print_data_seg ( op );
      printf ( "[BX+DI" );
      printf ( " + %d (%04X)]", (int16_t) op->u16, op->u16 );
      break;
      
    case IA32_ADDR16_SI_DISP16:
      print_data_seg ( op );
      printf ( "[SI" );
      printf ( " + %d (%04X)]", (int16_t) op->u16, op->u16 );
      break;
    case IA32_ADDR16_DI_DISP16:
      print_data_seg ( op );
      printf ( "[DI" );
      printf ( " + %d (%04X)]", (int16_t) op->u16, op->u16 );
      break;
    case IA32_ADDR16_BP_DISP16:
      print_data_seg ( op );
      printf ( "[BP" );
      printf ( " + %d (%04X)]", (int16_t) op->u16, op->u16 );
      break;
    case IA32_ADDR16_BX_DISP16:
      print_data_seg ( op );
      printf ( "[BX" );
      printf ( " + %d (%04X)]", (int16_t) op->u16, op->u16 );
      break;
    case IA32_ADDR32_EAX:
      print_data_seg ( op );
      printf ( "[EAX]" );
      break;
    case IA32_ADDR32_ECX:
      print_data_seg ( op );
      printf ( "[ECX]" );
      break;
    case IA32_ADDR32_EDX:
      print_data_seg ( op );
      printf ( "[EDX]" );
      break;
    case IA32_ADDR32_EBX:
      print_data_seg ( op );
      printf ( "[EBX]" );
      break;
    case IA32_ADDR32_SIB:
      print_data_seg ( op );
      printf ( "[" );
      print_sib ( op );
      printf ( "]" );
      break;
    case IA32_ADDR32_DISP32:
      print_data_seg ( op );
      printf ( "[%08X]", op->u32 );
      break;
    case IA32_ADDR32_ESI:
      print_data_seg ( op );
      printf ( "[ESI]" );
      break;
    case IA32_ADDR32_EDI:
      print_data_seg ( op );
      printf ( "[EDI]" );
      break;
    case IA32_ADDR32_EAX_DISP8:
      print_data_seg ( op );
      printf ( "[EAX" );
      printf ( " + %d (%02X)]", (int8_t) op->u8, op->u8 );
      break;
    case IA32_ADDR32_ECX_DISP8:
      print_data_seg ( op );
      printf ( "[ECX" );
      printf ( " + %d (%02X)]", (int8_t) op->u8, op->u8 );
      break;
    case IA32_ADDR32_EDX_DISP8:
      print_data_seg ( op );
      printf ( "[EDX" );
      printf ( " + %d (%02X)]", (int8_t) op->u8, op->u8 );
      break;
    case IA32_ADDR32_EBX_DISP8:
      print_data_seg ( op );
      printf ( "[EBX" );
      printf ( " + %d (%02X)]", (int8_t) op->u8, op->u8 );
      break;
    case IA32_ADDR32_SIB_DISP8:
      print_data_seg ( op );
      printf ( "[" );
      print_sib ( op );
      printf ( " + %d (%02X)]", (int8_t) op->u8, op->u8 );
      break;
    case IA32_ADDR32_EBP_DISP8:
      print_data_seg ( op );
      printf ( "[EBP" );
      printf ( " + %d (%02X)]", (int8_t) op->u8, op->u8 );
      break;
    case IA32_ADDR32_ESI_DISP8:
      print_data_seg ( op );
      printf ( "[ESI" );
      printf ( " + %d (%02X)]", (int8_t) op->u8, op->u8 );
      break;
    case IA32_ADDR32_EDI_DISP8:
      print_data_seg ( op );
      printf ( "[EDI" );
      printf ( " + %d (%02X)]", (int8_t) op->u8, op->u8 );
      break;
    case IA32_ADDR32_EAX_DISP32:
      print_data_seg ( op );
      printf ( "[EAX" );
      printf ( " + %d (%08X)]", (int32_t) op->u32, op->u32 );
      break;
    case IA32_ADDR32_ECX_DISP32:
      print_data_seg ( op );
      printf ( "[ECX" );
      printf ( " + %d (%08X)]", (int32_t) op->u32, op->u32 );
      break;
    case IA32_ADDR32_EDX_DISP32:
      print_data_seg ( op );
      printf ( "[EDX" );
      printf ( " + %d (%08X)]", (int32_t) op->u32, op->u32 );
      break;
    case IA32_ADDR32_EBX_DISP32:
      print_data_seg ( op );
      printf ( "[EBX" );
      printf ( " + %d (%08X)]", (int32_t) op->u32, op->u32 );
      break;
    case IA32_ADDR32_SIB_DISP32:
      print_data_seg ( op );
      printf ( "[" );
      print_sib ( op );
      printf ( " + %d (%08X)]", (int32_t) op->u32, op->u32 );
      break;
    case IA32_ADDR32_EBP_DISP32:
      print_data_seg ( op );
      printf ( "[EBP" );
      printf ( " + %d (%08X)]", (int32_t) op->u32, op->u32 );
      break;
    case IA32_ADDR32_ESI_DISP32:
      print_data_seg ( op );
      printf ( "[ESI" );
      printf ( " + %d (%08X)]", (int32_t) op->u32, op->u32 );
      break;
    case IA32_ADDR32_EDI_DISP32:
      print_data_seg ( op );
      printf ( "[EDI" );
      printf ( " + %d (%08X)]", (int32_t) op->u32, op->u32 );
      break;
      
    case IA32_REL8:
      tmp= next_addr + (uint32_t) ((int32_t) ((int8_t) op->u8));
      printf ( "%d (%08X)", (int8_t) op->u8, tmp );
      break;
    case IA32_REL16:
      tmp= (next_addr + (uint32_t) ((int32_t) ((int16_t) op->u16)))&0xFFFF;
      printf ( "%d (%08X)", (int16_t) op->u16, tmp );
      break;
    case IA32_REL32:
      tmp= (next_addr + (uint32_t) ((int32_t) op->u32));
      printf ( "%d (%08X)", (int32_t) op->u32, tmp );
      break;
    case IA32_PTR16_16: printf ( "%04X:%04X", op->ptr16, op->u16 ); break;
    case IA32_PTR16_32: printf ( "%04X:%08X", op->ptr16, op->u32 ); break;
    case IA32_MOFFS_OFF32:
      printf ( "[" );
      print_data_seg ( op );
      printf ( "%04X]", op->u32 );
      break;
    case IA32_MOFFS_OFF16:
      printf ( "[" );
      print_data_seg ( op );
      printf ( "%02X]", op->u16 );
      break;
    case IA32_CONSTANT_1:
      printf ( "1" );
      break;
    case IA32_CONSTANT_3:
      printf ( "3" );
      break;
    case IA32_USE_ADDR32:
      printf ( "{mode addr32}" );
      break;
    case IA32_USE_ADDR16:
      printf ( "{mode addr16}" );
      break;
    case IA32_FPU_STACK_POS:
      printf ( "ST(%d)", op->fpu_stack_pos );
      break;
    default:
      printf ( "??? %d %d", op->type,IA32_ADDR32_EAX_DISP32);
    }
  
} // end print_inst_op


static void
cpu_inst (
          const IA32_Inst *inst,
          const uint32_t   eip,
          void            *udata
          )
{

  int n;
  uint32_t next_addr;

  if ( !(_tracer.dbg_flags&DBG_CPU_INST) ) return;

  SHOW_EIP_CC;
  next_addr= eip + inst->nbytes;
  printf ( "[CPU]" );
  for ( n= 0; n < inst->nbytes; ++n ) printf ( " %02X", inst->bytes[n] );
  for ( ; n < 15; ++n ) printf ( "   " );
  printf ( "%s", get_inst_prefix ( inst->prefix ) );
  printf ( "%s", get_inst_mnemonic ( inst->name ) );
  if ( inst->ops[0].type != IA32_NONE )
    print_inst_op ( &inst->ops[0], next_addr );
  if ( inst->ops[1].type != IA32_NONE )
    {
      putchar ( ',' );
      print_inst_op ( &inst->ops[1], next_addr );
    }
  if ( inst->ops[2].type != IA32_NONE )
    {
      putchar ( ',' );
      print_inst_op ( &inst->ops[2], next_addr );
    }
  putchar ( '\n' );
  
} // end dbg_cpu_inst


static void
trace_soft_int (
                void           *udata,
                const uint8_t   vector,
                const IA32_CPU *cpu
                )
{

  if ( !(_tracer.dbg_flags&DBG_TRACE_SOFT_INT) ) return;

  SHOW_EIP_CC;
  printf ( "[SOFTINT] vec:%02X EAX:%08X EBX:%08X ECX:%08X\n",
           vector, cpu->eax.v, cpu->ebx.v, cpu->ecx.v );
  
} // end trace_soft_int


static void
mem_access (
            const PC_MemAccessType  type,
            const uint64_t          addr,
            const uint64_t          data,
            void                   *udata
            )
{

  if ( !(_tracer.dbg_flags&DBG_MEM_ACCESS) ) return;
  
  SHOW_EIP_CC;
  switch ( type )
    {
    case PC_READ8:
      printf ( "[MEM] %016lX --> %02X\n", addr, (uint8_t) data );
      break;
    case PC_READ16:
      printf ( "[MEM] %016lX --> %04X\n", addr, (uint16_t) data );
      break;
    case PC_READ32:
      printf ( "[MEM] %016lX --> %08X\n", addr, (uint32_t) data );
      break;
    case PC_READ64:
      printf ( "[MEM] %016lX --> %016lX\n", addr, data );
      break;
    case PC_WRITE8:
      printf ( "[MEM] %016lX <-- %02X\n", addr, (uint8_t) data );
      break;
    case PC_WRITE16:
      printf ( "[MEM] %016lX <-- %04X\n", addr, (uint16_t) data );
      break;
    case PC_WRITE32:
      printf ( "[MEM] %016lX <-- %08X\n", addr, (uint32_t) data );
      break;
    default: break;
    }
  
} // end mem_access


static void
port_access (
             const PC_MemAccessType  type,
             const uint16_t          port,
             const uint32_t          data,
             void                   *udata
             )
{

  if ( !(_tracer.dbg_flags&DBG_PORT_ACCESS) ) return;
  
  SHOW_EIP_CC;
  switch ( type )
    {
    case PC_READ8:
      printf ( "[IO] %04X --> %02X\n", port, (uint8_t) data );
      break;
    case PC_READ16:
      printf ( "[IO] %04X --> %04X\n", port, (uint16_t) data );
      break;
    case PC_READ32:
      printf ( "[IO] %04X --> %08X\n", port, data );
      break;
    case PC_WRITE8:
      printf ( "[IO] %04X <-- %02X\n", port, (uint8_t) data );
      break;
    case PC_WRITE16:
      printf ( "[IO] %04X <-- %04X\n", port, (uint16_t) data );
      break;
    case PC_WRITE32:
      printf ( "[IO] %04X <-- %08X\n", port, data );
      break;
    default: break;
    }
  
} // end port_access


static void
pci_reg_access (
                const PC_PCIRegAccessType  type,
                const uint8_t              addr,
                const uint32_t             data,
                const char                *func_name,
                void                      *udata
                )
{

  if ( !(_tracer.dbg_flags&DBG_PCI_REG_ACCESS) ) return;
  
  SHOW_EIP_CC;
  printf ( "[PCI] [%s]::%02X ", func_name!=NULL ? func_name : "", addr );
  switch ( type )
    {
    case PC_PCI_READ8:
      printf ( "--> %02X\n", (uint8_t) data );
      break;
    case PC_PCI_READ16:
      printf ( "--> %04X\n", (uint16_t) data );
      break;
    case PC_PCI_READ32:
      printf ( "--> %08X\n", data );
      break;
    case PC_PCI_WRITE8:
      printf ( "<-- %02X\n", (uint8_t) data );
      break;
    case PC_PCI_WRITE16:
      printf ( "<-- %04X\n", (uint16_t) data );
      break;
    case PC_PCI_WRITE32:
      printf ( "<-- %08X\n", data );
      break;
    default: break;
    }
  
} // end pci_reg_access


static void
cmos_ram_access (
                 const bool    read,
                 const uint8_t addr,
                 const uint8_t data
                 )
{

  if ( !(_tracer.dbg_flags&DBG_CMOS_RAM_ACCESS) ) return;
  
  SHOW_EIP_CC;
  printf ( "[CMOSRAM] %02X ", addr );
  if ( read ) printf ( "--> %02X\n", data );
  else        printf ( "<-- %02X\n", data );
  
} // end cmos_ram_access


static void
timer_out_changed (
                   const int  timer,
                   const bool out
                   )
{

  if ( !(_tracer.dbg_flags&DBG_TIMER_OUT_CHANGED) ) return;
  
  SHOW_EIP_CC;
  printf ( "[TIMER%d] %d\n", timer, out?1:0 );
  
} // end timer_out_changed


static void
int_serviced (
              const int     irq,
              const uint8_t vec
              )
{

  if ( !(_tracer.dbg_flags&DBG_INT_SERVICED) ) return;
  
  SHOW_EIP_CC;
  printf ( "[IC] irq:%d vec:%02X\n", irq, vec );
  
} // end int_serviced


static void
vga_mem_access (
                const bool      is_read,
                const int       plane,
                const uint32_t  offset,
                const uint8_t   data,
                void           *udata
                )
{

  if ( !(_tracer.dbg_flags&DBG_VGA_MEM_ACCESS) ) return;
  
  SHOW_EIP_CC;
  if ( is_read )
    {
      if ( plane != -1 )
        printf ( "[VGAMEM] P%d:%08X --> %02X\n", plane, offset, data );
      else
        printf ( "[VGAMEM] %08X --> %02X\n", offset, data );
    }
  else
    {
      if ( plane != -1 )
        printf ( "[VGAMEM] P%d:%08X <-- %02X\n", plane, offset, data );
      else
        printf ( "[VGAMEM] %08X <-- %02X\n", offset, data );
    }
  
} // end vga_mem_access


static void
vga_mem_linear_access (
                       const PC_MemAccessType  type,
                       const int               aperture,
                       const uint32_t          addr,
                       const uint64_t          data,
                       void                   *udata
                       )
{

  if ( !(_tracer.dbg_flags&DBG_VGA_MEM_ACCESS) ) return;
  
  SHOW_EIP_CC;
  switch ( type )
    {
    case PC_READ8:
      printf ( "[VGAMEM] %d:%08X --> %02X\n", aperture, addr, (uint8_t) data );
      break;
    case PC_READ16:
      printf ( "[VGAMEM] %d:%08X --> %04X\n", aperture, addr, (uint16_t) data );
      break;
    case PC_READ32:
      printf ( "[VGAMEM] %d:%08X --> %08X\n", aperture, addr, (uint32_t) data );
      break;
    case PC_READ64:
      printf ( "[VGAMEM] %d:%08X --> %016lX\n", aperture, addr, data );
      break;
    case PC_WRITE8:
      printf ( "[VGAMEM] %d:%08X <-- %02X\n", aperture, addr, (uint8_t) data );
      break;
    case PC_WRITE16:
      printf ( "[VGAMEM] %d:%08X <-- %04X\n", aperture, addr, (uint16_t) data );
      break;
    case PC_WRITE32:
      printf ( "[VGAMEM] %d:%08X <-- %08X\n", aperture, addr, (uint32_t) data );
      break;
    default: break;
    }
  
} // end vga_mem_linear_access


static void
floppy_fifo_access (
                    const int      drv,
                    const uint8_t  data,
                    const bool     is_read,
                    const bool     in_exec_phase,
                    const bool     in_dma,
                    void          *udata
                    )
{
  
  if ( !(_tracer.dbg_flags&DBG_FLOPPY_FIFO_ACCESS) ) return;
  
  SHOW_EIP_CC;
  if ( is_read )
    printf ( "[FDFIFO] FD%d --> %02X", drv, data );
  else
    printf ( "[FDFIFO] FD%d <-- %02X", drv, data );
  if ( in_exec_phase ) printf ( " (EXEC_PHASE)" );
  if ( in_dma ) printf ( " (DMA)" );
  printf ( "\n" );
  
} // end floppy_fifo_access


static void
dma_transfer8 (
               const int      channel,
               const uint32_t addr,
               const uint8_t  data,
               const bool     is_read,
               void          *udata
               )
{
  
  if ( !(_tracer.dbg_flags&DBG_DMA_TRANSFER8) ) return;
  
  SHOW_EIP_CC;
  if ( is_read )
    printf ( "[DMAISA] CHN%d %06X --> %02X\n", channel, addr, data );
  else
    printf ( "[DMAISA] CHN%d %06X <-- %02X\n", channel, addr, data );
  
} // end dma_transfer8


static void
dma_transfer16 (
                const int      channel,
                const uint32_t addr,
                const uint16_t data,
                const bool     is_read,
                void          *udata
                )
{
  
  if ( !(_tracer.dbg_flags&DBG_DMA_TRANSFER16) ) return;
  
  SHOW_EIP_CC;
  if ( is_read )
    printf ( "[DMAISA] CHN%d %06X --> %04X\n", channel, addr, data );
  else
    printf ( "[DMAISA] CHN%d %06X <-- %04X\n", channel, addr, data );
  
} // end dma_transfer16




/*********************/
/* FUNCIONS PRIVADES */
/*********************/

static PC_Scancode
get_scancode (
              const SDL_Event *event
              )
{

  PC_Scancode ret;


  // NOTA!! No vaig a implementar les combinacions rares!!!! Tampoc
  // PC_KBDSP_BLOQ_DESPL, PC_KBDSP_PAUSA, PC_KBDSP_WIN_DERECHA
  //
  // FALTEN ELS DOS BOTONS DE TILDES !!! ES MAPEJEN AL MATEIX BOTO!!!
  switch ( event->key.keysym.sym )
    {
    case SDLK_ESCAPE: ret= PC_KBDSP_ESC; break;
    case SDLK_F1: ret= PC_KBDSP_F1; break;
    case SDLK_F2: ret= PC_KBDSP_F2; break;
    case SDLK_F3: ret= PC_KBDSP_F3; break;
    case SDLK_F4: ret= PC_KBDSP_F4; break;
    case SDLK_F5: ret= PC_KBDSP_F5; break;
    case SDLK_F6: ret= PC_KBDSP_F6; break;
    case SDLK_F7: ret= PC_KBDSP_F7; break;
    case SDLK_F8: ret= PC_KBDSP_F8; break;
    case SDLK_F9: ret= PC_KBDSP_F9; break;
    case SDLK_F10: ret= PC_KBDSP_F10; break;
    case SDLK_F11: ret= PC_KBDSP_F11; break;
    case SDLK_F12: ret= PC_KBDSP_F12; break;
    case SDLK_PRINT: ret= PC_KBDSP_IMP_PNT; break;
    case SDLK_BACKSPACE: ret= PC_KBDSP_RETROCESO; break;
    case SDLK_WORLD_1: ret= PC_KBDSP_ABRE_EXCLAMACION; break;
    case SDLK_QUOTE: ret= PC_KBDSP_COMILLAS; break;
    case SDLK_0: ret= PC_KBDSP_0; break;
    case SDLK_1: ret= PC_KBDSP_1; break;
    case SDLK_2: ret= PC_KBDSP_2; break;
    case SDLK_3: ret= PC_KBDSP_3; break;
    case SDLK_4: ret= PC_KBDSP_4; break;
    case SDLK_5: ret= PC_KBDSP_5; break;
    case SDLK_6: ret= PC_KBDSP_6; break;
    case SDLK_7: ret= PC_KBDSP_7; break;
    case SDLK_8: ret= PC_KBDSP_8; break;
    case SDLK_9: ret= PC_KBDSP_9; break;
    case SDLK_TAB: ret= PC_KBDSP_TABULADOR; break;
      //case SDLK_COMPOSE: ret= PC_KBDSP_ACCENT_OBERT; break;
    case SDLK_PLUS: ret= PC_KBDSP_SIGNO_MAS; break;
    case SDLK_RETURN: ret= PC_KBDSP_ENTRAR; break;
    case SDLK_CAPSLOCK: ret= PC_KBDSP_BLOQ_MAYUS; break;
    case SDLK_WORLD_81: ret= PC_KBDSP_ENYE; break;
      //case SDLK_COMPOSE: ret= PC_KBDSP_TILDE; break;
    case SDLK_WORLD_71: ret= PC_KBDSP_C_TRENCADA; break;
    case SDLK_LSHIFT: ret= PC_KBDSP_MAYUS; break;
    case SDLK_LESS: ret= PC_KBDSP_MENOR; break;
    case SDLK_COMMA: ret= PC_KBDSP_COMA; break;
    case SDLK_MINUS: ret= PC_KBDSP_GUION; break;
    case SDLK_PERIOD: ret= PC_KBDSP_PUNTO; break;
    case SDLK_RSHIFT: ret= PC_KBDSP_MAYUS_DERECHA; break;
    case SDLK_LCTRL: ret= PC_KBDSP_CONTROL; break;
    case SDLK_LSUPER: ret= PC_KBDSP_WINDOWS; break;
    case SDLK_LALT: ret= PC_KBDSP_ALT; break;
    case SDLK_SPACE: ret= PC_KBDSP_ESPACIO; break;
    case SDLK_MODE: ret= PC_KBDSP_ALT_GR; break;
    case SDLK_MENU: ret= PC_KBDSP_MENU; break;
    case SDLK_RCTRL: ret= PC_KBDSP_CONTROL_DERECHA; break;
    case SDLK_RALT: ret= PC_KBDSP_ALT_GR; break;
    case SDLK_a: ret= PC_KBDSP_A; break;
    case SDLK_b: ret= PC_KBDSP_B; break;
    case SDLK_c: ret= PC_KBDSP_C; break;
    case SDLK_d: ret= PC_KBDSP_D; break;
    case SDLK_e: ret= PC_KBDSP_E; break;
    case SDLK_f: ret= PC_KBDSP_F; break;
    case SDLK_g: ret= PC_KBDSP_G; break;
    case SDLK_h: ret= PC_KBDSP_H; break;
    case SDLK_i: ret= PC_KBDSP_I; break;
    case SDLK_j: ret= PC_KBDSP_J; break;
    case SDLK_k: ret= PC_KBDSP_K; break;
    case SDLK_l: ret= PC_KBDSP_L; break;
    case SDLK_m: ret= PC_KBDSP_M; break;
    case SDLK_n: ret= PC_KBDSP_N; break;
    case SDLK_o: ret= PC_KBDSP_O; break;
    case SDLK_p: ret= PC_KBDSP_P; break;
    case SDLK_q: ret= PC_KBDSP_Q; break;
    case SDLK_r: ret= PC_KBDSP_R; break;
    case SDLK_s: ret= PC_KBDSP_S; break;
    case SDLK_t: ret= PC_KBDSP_T; break;
    case SDLK_u: ret= PC_KBDSP_U; break;
    case SDLK_v: ret= PC_KBDSP_V; break;
    case SDLK_w: ret= PC_KBDSP_W; break;
    case SDLK_x: ret= PC_KBDSP_X; break;
    case SDLK_y: ret= PC_KBDSP_Y; break;
    case SDLK_z: ret= PC_KBDSP_Z; break;
    case SDLK_DELETE: ret= PC_KBDSP_SUPR; break;
    case SDLK_END: ret= PC_KBDSP_FIN; break;
    case SDLK_PAGEDOWN: ret= PC_KBDSP_AV_PAG; break;
    case SDLK_INSERT: ret= PC_KBDSP_INSERT; break;
    case SDLK_HOME: ret= PC_KBDSP_INICIO; break;
    case SDLK_PAGEUP: ret= PC_KBDSP_RE_PAG; break;
    case SDLK_UP: ret= PC_KBDSP_ARRIBA; break;
    case SDLK_DOWN: ret= PC_KBDSP_ABAJO; break;
    case SDLK_RIGHT: ret= PC_KBDSP_DERECHA; break;
    case SDLK_LEFT: ret= PC_KBDSP_IZQUIERDA; break;
    case SDLK_KP0: ret= PC_KBDSP_NUM_0; break;
    case SDLK_KP1: ret= PC_KBDSP_NUM_1; break;
    case SDLK_KP2: ret= PC_KBDSP_NUM_2; break;
    case SDLK_KP3: ret= PC_KBDSP_NUM_3; break;
    case SDLK_KP4: ret= PC_KBDSP_NUM_4; break;
    case SDLK_KP5: ret= PC_KBDSP_NUM_5; break;
    case SDLK_KP6: ret= PC_KBDSP_NUM_6; break;
    case SDLK_KP7: ret= PC_KBDSP_NUM_7; break;
    case SDLK_KP8: ret= PC_KBDSP_NUM_8; break;
    case SDLK_KP9: ret= PC_KBDSP_NUM_9; break;
    case SDLK_KP_PERIOD: ret= PC_KBDSP_NUM_PUNTO; break;
    case SDLK_KP_ENTER: ret= PC_KBDSP_NUM_ENTRAR; break;
    case SDLK_KP_PLUS: ret= PC_KBDSP_NUM_SUMA; break;
    case SDLK_NUMLOCK: ret= PC_KBDSP_BLOQ_NUM; break;
    case SDLK_KP_DIVIDE: ret= PC_KBDSP_NUM_DIV; break;
    case SDLK_KP_MULTIPLY: ret= PC_KBDSP_NUM_MUL; break;
    case SDLK_KP_MINUS: ret= PC_KBDSP_NUM_RESTA; break;
    case SDLK_WORLD_26: ret= PC_KBDSP_SUPER_O; break;
    default: ret= PC_SCANCODE_ALL;
    }

  return ret;
  
} // end get_scancode


static void
check_signals (
               bool *stop,
               bool *reset
               )
{
  
  SDL_Event event;
  PC_Scancode key;
  
  
  *stop= *reset= false;
  while ( SDL_PollEvent ( &event ) )
    switch ( event.type )
      {
      case SDL_ACTIVEEVENT:
        if ( event.active.state&SDL_APPINPUTFOCUS &&
             !event.active.gain )
          PC_kbd_clear ();
        break;
      case SDL_QUIT:
        *stop= true;
        break;
      case SDL_KEYDOWN:
        if ( event.key.keysym.sym == SDLK_q &&
             (event.key.keysym.mod&KMOD_CTRL)!=0 )
          {
            if ( _mouse.active )
              {
                _mouse.active= false;
                //SDL_CaptureMouse ( SDL_FALSE );
                SDL_WM_GrabInput ( SDL_GRAB_OFF );
                SDL_ShowCursor ( SDL_ENABLE );
              }
            *stop= true;
          }
        else
          {
            key= get_scancode ( &event );
            if ( key != PC_SCANCODE_ALL )
              {
                PC_kbd_press ( key );
                if ( key == PC_KBDSP_BLOQ_MAYUS )
                  PC_kbd_release ( key );
              }
            else fprintf(stderr,"KEY_DOWN key:%d mod:%d\n",
                         event.key.keysym.sym,
                         event.key.keysym.mod);
          }
        break;
      case SDL_KEYUP:
        key= get_scancode ( &event );
        if ( key != PC_SCANCODE_ALL )
          {
            if ( key == PC_KBDSP_BLOQ_MAYUS )
              PC_kbd_press ( key );
            PC_kbd_release ( key );
          }
        else fprintf(stderr,"KEY_UP key:%d mod:%d\n",
                     event.key.keysym.sym,
                     event.key.keysym.mod);
        break;
      case SDL_MOUSEMOTION:
        if ( _mouse.active )
          PC_mouse_motion ( (int32_t) event.motion.xrel,
                            (int32_t) event.motion.yrel );
        break;
      case SDL_MOUSEBUTTONUP:
        if ( _mouse.active )
          {
            switch ( event.button.button )
              {
              case SDL_BUTTON_LEFT:
                PC_mouse_button_release ( PC_MOUSE_BUTTON_LEFT );
                break;
              case SDL_BUTTON_MIDDLE:
                PC_mouse_button_release ( PC_MOUSE_BUTTON_MIDDLE );
                break;
              case SDL_BUTTON_RIGHT:
                PC_mouse_button_release ( PC_MOUSE_BUTTON_RIGHT );
                break;
              }
          }
        break;
      case SDL_MOUSEBUTTONDOWN:
        if ( !_mouse.active )
          {
            _mouse.active= true;
            //SDL_CaptureMouse ( SDL_TRUE );
            SDL_WM_GrabInput ( SDL_GRAB_ON );
            SDL_ShowCursor ( SDL_DISABLE ); 
          }
        else
          {
            switch ( event.button.button )
              {
              case SDL_BUTTON_LEFT:
                PC_mouse_button_press ( PC_MOUSE_BUTTON_LEFT );
                break;
              case SDL_BUTTON_MIDDLE:
                PC_mouse_button_press ( PC_MOUSE_BUTTON_MIDDLE );
                break;
              case SDL_BUTTON_RIGHT:
                PC_mouse_button_press ( PC_MOUSE_BUTTON_RIGHT );
                break;
              }
          }
        break;
      default: break;
      }
  
} // end check_signals


static void
loop (void)
{

  const gint64 SLEEP= 1000;
  
  int cc,maxcc,tmp;
  gint64 t0,last,sleep;
  bool stop,reset;
  
  
  stop= reset= false;
  cc= 0;
  maxcc= (int) ((PC_ClockFreq/1000000.0)*SLEEP*2 + 0.5);
  last= g_get_monotonic_time ();
  g_usleep ( SLEEP );
  for (;;)
    {
      
      // Temps i casos especials.
      t0= g_get_monotonic_time ();
      assert ( t0 > last );
      
      // Executa.
      tmp= (int) ((PC_ClockFreq/1000000.0)*(t0-last) + 0.5);
      cc+= tmp>maxcc ? maxcc : tmp;
      while ( cc > 0 )
        cc-= PC_iter ( cc );
      check_signals ( &stop, &reset );
      if ( stop ) return;
      
      // Actualitza.
      last= t0;
      sleep= SLEEP - (g_get_monotonic_time () - t0);
      
      // Quant més xicotet siga l'interval millor, però més
      // lent. Menys d'1ms va molt lent.
      if ( sleep > 0 ) g_usleep ( sleep );
      /*else fprintf ( stdout,
                     "IEEE2 [WW] Està tardant massa el simulador (%ld) !!!\n",
                     sleep );
      */
    }
  
} // end loop


static void
jit_loop (void)
{
  
  const gint64 SLEEP= 1000;
  const gint64 MIN_DELAY= -10000;

  int cc,cc_iter;
  gint64 t0,tf,delay;
  bool stop,reset;

  
  stop= reset= false;
  t0= g_get_monotonic_time ();
  cc_iter= (int) ((PC_ClockFreq/1000000.0)*SLEEP + 0.5);
  cc= 0;
  delay= 0;
  for (;;)
    {

      // Executa.
      cc+= cc_iter;
      while ( cc > 0 )
        cc-= PC_jit_iter ( cc );
      check_signals ( &stop, &reset );
      if ( stop ) return;
      
      // Delay
      tf= g_get_monotonic_time ();
      delay+= SLEEP-(tf-t0); t0= tf;
      if ( delay >= SLEEP ) g_usleep ( SLEEP );
      else if ( delay < MIN_DELAY ) delay= MIN_DELAY;
      
    }
  
} // end jit_loop


static void 
sres_changed (
              const int width,
              const int height
              )
{
  
  // Nou surface.
  _screen.width= width;
  _screen.height= height;
  //_screen.surface= SDL_SetVideoMode ( width, height, 32, 0 );
  _screen.surface= SDL_SetVideoMode ( width, height, 32,
                                      SDL_HWSURFACE | SDL_GL_DOUBLEBUFFER );
  if ( _screen.surface == NULL )
    {
      fprintf ( stderr, "FATAL ERROR!!!: %s", SDL_GetError () );
      SDL_Quit ();
      return;
    }
  SDL_WM_SetCaption ( "PC", "PC" );
  
} // end sres_changed


static void
audio_callback (
                void  *userdata,
                Uint8 *stream,
                int    len
                )
{
  
  int i;
  const int16_t *buffer;
  
  
  assert ( _audio.size == len );
  if ( _audio.buffers[_audio.buff_out].full )
    {
      buffer= _audio.buffers[_audio.buff_out].v;
      for ( i= 0; i < len; ++i )
        stream[i]= ((Uint8 *) buffer)[i];
      _audio.buffers[_audio.buff_out].full= 0;
      _audio.buff_out= (_audio.buff_out+1)%NBUFF;
    }
  else {for ( i= 0; i < len; ++i ) stream[i]= _audio.silence;}
  
} // end audio_callback


// Torna 0 si tot ha anat bé.
static const char *
init_audio (void)
{
  
  SDL_AudioSpec desired, obtained;
  int n;
  Uint8 *mem;
  
  
  // Únic camp de l'estat que s'inicialitza abans.
  _audio.buff_out= _audio.buff_in= 0;
  for ( n= 0; n < NBUFF; ++n ) _audio.buffers[n].full= 0;
  
  // Inicialitza.
  desired.freq= 44100;
  desired.format= AUDIO_S16;
  desired.channels= 2;
  desired.samples= 2048;
  desired.size= 8192;
  desired.callback= audio_callback;
  desired.userdata= NULL;
  if ( SDL_OpenAudio ( &desired, &obtained ) == -1 )
    return SDL_GetError ();
  if ( obtained.format != desired.format )
    {
      fprintf ( stderr, "Força format audio\n" );
      SDL_CloseAudio ();
      if ( SDL_OpenAudio ( &desired, NULL ) == -1 )
        return SDL_GetError ();
      obtained= desired;
    }
  
  // Inicialitza estat.
  mem= malloc ( obtained.size*NBUFF );
  for ( n= 0; n < NBUFF; ++n, mem+= obtained.size )
    _audio.buffers[n].v= (int16_t *) mem;
  _audio.silence= (char) obtained.silence;
  _audio.pos= 0;
  _audio.size= obtained.size;
  _audio.nsamples= _audio.size/2;
  if ( obtained.freq > 44100 )
    {
      SDL_CloseAudio ();
      return "Freqüència massa gran";
    }
  _audio.ratio= 44100 / (double) obtained.freq;
  _audio.pos2= 0.0;
  
  return NULL;
  
} // end init_audio 


static void
close_audio (void)
{
  
  SDL_CloseAudio ();
  free ( _audio.buffers[0].v );
  
} // end close_audio




/************/
/* FRONTEND */
/************/

static void
warning (
         void       *udata,
         const char *format,
         ...
         )
{
  
  va_list ap;
  
  
  va_start ( ap, format );
  fprintf ( stderr, "[WW] " );
  vfprintf ( stderr, format, ap );
  putc ( '\n', stderr );
  va_end ( ap );
  
} // end warning


static void
write_sea_bios_debug_port (
                           const char  c,
                           void       *udata
                           )
{

  static int pos= 0;


  if ( pos == 0 ) fprintf ( stderr, "[II] " );
  fputc ( c, stderr );
  if ( c == '\n' ) pos= 0;
  else             ++pos;
  fflush ( stderr );
  
} // end write_sea_bios_debug_port


static uint8_t *
get_cmos_ram (
              void *udata
              )
{

  static uint8_t ram[256];

  return &(ram[0]);
  
} // end get_cmos_ram


static void
get_current_time (
                  void    *udata,
                  uint8_t *ss,
                  uint8_t *mm,
                  uint8_t *hh,
                  uint8_t *day_week,
                  uint8_t *day_month,
                  uint8_t *month,
                  int     *year
                  )
{

  GTimeZone *tz;
  GDateTime *date;


  if ( _use_unix_epoch )
    {
      *ss= 0;
      *mm= 0;
      *hh= 0;
      *day_week= 4;
      *day_month= 1;
      *month= 1;
      *year= 1970;
    }
  else
    {
      tz= NULL;
      date= NULL;
      tz= g_time_zone_new_local ();
      if ( tz == NULL ) goto error;
      date= g_date_time_new_now ( tz );
      if ( date == NULL ) goto error;
      *ss= g_date_time_get_second ( date );
      *mm= g_date_time_get_minute ( date );
      *hh= g_date_time_get_hour ( date );
      *day_week= g_date_time_get_day_of_week ( date );
      *day_month= g_date_time_get_day_of_month ( date );
      *month= g_date_time_get_month ( date );
      *year= g_date_time_get_year ( date );
      g_time_zone_unref ( tz );
      g_date_time_unref ( date );
    }
  
  return;
  
 error:
  *ss= *mm= *hh= *day_week= *day_month= *month= 0;
  *year= 0;
  if ( tz == NULL ) g_time_zone_unref ( tz );
  if ( date == NULL ) g_date_time_unref ( date );
  
} // end get_current_time


static void
update_screen (
               void         *udata,
               const PC_RGB *fb,
               const int     width,
               const int     height,
               const int     line_stride
               )
{

  Uint8 *p;
  int r,c;
  
  
  if ( width != _screen.width || height != _screen.height )
    sres_changed ( width, height );
  if ( SDL_MUSTLOCK ( _screen.surface ) )
    SDL_LockSurface ( _screen.surface );
  p= (Uint8 *) _screen.surface->pixels;
  for ( r= 0; r < height; ++r )
    {
      for ( c= 0; c < width; ++c )
        {
          *(p++)= ((Uint8) fb[c].b);
          *(p++)= ((Uint8) fb[c].g);
          *(p++)= ((Uint8) fb[c].r);
          *(p++)= 0x00;
        }
      fb+= line_stride;
    }
  if ( SDL_MUSTLOCK ( _screen.surface ) )
    SDL_UnlockSurface ( _screen.surface );

  if ( SDL_Flip ( _screen.surface ) == -1 )
    {
      fprintf ( stderr, "ERROR FATAL !!!: %s\n", SDL_GetError () );
      SDL_Quit ();
    }
  
} // end update_screen 

//static FILE *F;
static void
play_sound (
            const int16_t  samples[PC_AUDIO_BUFFER_SIZE*2],
            void          *udata
            )
{

  int nofull, j;
  int16_t *buffer;
  
  //fwrite(samples,PC_AUDIO_BUFFER_SIZE*2,1,F);
  for (;;)
    {
      
      while ( _audio.buffers[_audio.buff_in].full ) SDL_Delay ( 1 );
      buffer= _audio.buffers[_audio.buff_in].v;
      
      j= (int) (_audio.pos2 + 0.5);
      while ( (nofull= (_audio.pos != _audio.nsamples)) &&
              j < PC_AUDIO_BUFFER_SIZE )
        {
          buffer[_audio.pos++]= samples[2*j];
          buffer[_audio.pos++]= samples[2*j+1];
          _audio.pos2+= _audio.ratio;
          j= (int) (_audio.pos2 + 0.5);
        }
      if ( !nofull )
        {
          _audio.pos= 0;
          _audio.buffers[_audio.buff_in].full= 1;
          _audio.buff_in= (_audio.buff_in+1)%NBUFF;
        }
      if ( j >= PC_AUDIO_BUFFER_SIZE )
        {
          _audio.pos2-= PC_AUDIO_BUFFER_SIZE;
          break;
        }
      
    }
  
} // end play_sound




/******************/
/* FUNCIONS MÒDUL */
/******************/

static PyObject *
PC_close_ (
           PyObject *self,
           PyObject *args
           )
{

  int i;

  
  if ( !_initialized ) Py_RETURN_NONE;

  PC_close ();
  PC_cdrom_free ( _cdrom ); _cdrom= NULL;
  PC_file_free ( _hdd ); _hdd= NULL;
  for ( i= 0; i < 4; ++i )
    if ( _fd[i] != NULL )
      {
        PC_file_free ( _fd[i] );
        _fd[i]= NULL;
      }
  free ( _bios ); _bios= NULL;
  free ( _vgabios ); _vgabios= NULL;
  close_audio ();
  SDL_Quit ();
  _initialized= false;
  //fclose(F);
  Py_RETURN_NONE;
  
} // end PC_close_


static PyObject *
PC_init_module (
                PyObject *self,
                PyObject *args,
                PyObject *kwargs
                )
{

  static const PC_TraceCallbacks trace_callbacks=
    {
     cpu_inst,
     mem_access,
     port_access,
     pci_reg_access,
     cmos_ram_access,
     timer_out_changed,
     int_serviced,
     vga_mem_access,
     vga_mem_linear_access,
     floppy_fifo_access,
     dma_transfer8,
     dma_transfer16,
     trace_soft_int
    };
  static const PC_Frontend frontend=
    {
      warning,
      write_sea_bios_debug_port,
      get_cmos_ram,
      get_current_time,
      update_screen,
      play_sound,
      &trace_callbacks
    };
  static PC_Config config=
    {
      .flags= PC_CFG_QEMU_COMPATIBLE,
      .ram_size= PC_RAM_SIZE_32MB,
      .qemu_boot_order= {
        .check_floppy_sign= true,
        .order= {
          PC_QEMU_BOOT_ORDER_FLOPPY,
          PC_QEMU_BOOT_ORDER_HD,
          PC_QEMU_BOOT_ORDER_NONE }
      },
      .pci_devs= {
        {
          .dev= PC_PCI_DEVICE_SVGA_CIRRUS_CLGD5446,
          .optrom= NULL,
          .optrom_size= 0
        },
        {
          .dev= PC_PCI_DEVICE_NULL
        }
      },
      .cpu_model= IA32_CPU_P5_60MHZ,
      .diskettes= {
        PC_DISKETTE_1M44,
        PC_DISKETTE_1M2,
        PC_DISKETTE_NONE,
        PC_DISKETTE_NONE
      },
      .host_mouse= {
        .resolution= 25.0
        /*
        .sensitivity= 0,
        .acceleration= 5.0
        */
      }
    };

  static char *kwlist[]= {"bios","vgabios","hdd","use_unix_epoch",NULL};
  
  const char *err2;
  PyObject *bytes,*vga_bytes;
  Py_ssize_t bios_size,vga_bios_size;
  PC_Error err;
  PC_IDEDevice ide_devices[2][2];
  const char *hdd;
  int i;
  
  //F= fopen("out.s16","wb");
  _use_unix_epoch= 0;
  if ( _initialized ) Py_RETURN_NONE;
  if ( !PyArg_ParseTupleAndKeywords ( args, kwargs, "O!O!z|p",
                                      kwlist,
                                      &PyBytes_Type, &bytes,
                                      &PyBytes_Type, &vga_bytes,
                                      &hdd, &_use_unix_epoch ) )
    return NULL;
  
  // Prepara.
  _bios= NULL;
  _vgabios= NULL;
  _hdd= NULL;
  _cdrom= NULL;
  for ( i= 0; i < 4; ++i )
    _fd[i]= NULL;
  
  // Comprova BIOS.
  bios_size= PyBytes_Size ( bytes );
  _bios= (uint8_t *) malloc ( bios_size );
  if ( _bios == NULL )
    {
      PyErr_SetString ( PCError, "Cannot allocate memory for bios" );
      goto error;
    }
  memcpy ( _bios, PyBytes_AS_STRING ( bytes ), bios_size );

  // Comprova VGABIOS
  vga_bios_size= PyBytes_Size ( vga_bytes );
  _vgabios= (uint8_t *) malloc ( vga_bios_size );
  if ( _vgabios == NULL )
    {
      PyErr_SetString ( PCError, "Cannot allocate memory for vgabios" );
      goto error;
    }
  memcpy ( _vgabios, PyBytes_AS_STRING ( vga_bytes ), vga_bios_size );

  // HDD
  if ( hdd != NULL )
    {
      _hdd= PC_file_new_from_file ( hdd, false );
      if ( _hdd == NULL )
        {
          PyErr_Format ( PCError, "Cannot open '%s'", hdd );
          goto error;
        }
    }
  else _hdd= NULL;

  // CDROM
  _cdrom= PC_cdrom_new ();
  if ( _cdrom == NULL )
    {
      PyErr_SetString ( PCError, "Cannot allocate memory for cdrom" );
      goto error;
    }
    
  // SDL
  if ( SDL_Init ( SDL_INIT_VIDEO |
                  SDL_INIT_NOPARACHUTE |
                  SDL_INIT_AUDIO ) == -1 )
    {
      PyErr_SetString ( PCError, SDL_GetError () );
      goto error;
    }
  _screen.surface= NULL;
  _screen.width= -1;
  _screen.height= -1;
  if ( (err2= init_audio ()) != NULL )
    {
      PyErr_SetString ( PCError, err2 );
      SDL_Quit ();
      return NULL; 
    }
  
  // Tracer.
  _tracer.dbg_flags= 0;
  _tracer.eip= 0;
  _tracer.cc= 0;
  _tracer.steps= 0;

  // Inicialitza el simulador.
  config.pci_devs[0].optrom= _vgabios;
  config.pci_devs[0].optrom_size= vga_bios_size;
  ide_devices[0][0].hdd.type= PC_IDE_DEVICE_TYPE_HDD;
  ide_devices[0][0].hdd.f= _hdd;
  ide_devices[0][1].type= PC_IDE_DEVICE_TYPE_NONE;
  ide_devices[1][0].cdrom.type= PC_IDE_DEVICE_TYPE_CDROM;
  ide_devices[1][0].cdrom.cdrom= _cdrom;
  ide_devices[1][1].type= PC_IDE_DEVICE_TYPE_NONE;
  err= PC_init ( _bios, (size_t) bios_size,
                 ide_devices,
                 &frontend, NULL,
                 &config );
  switch ( err )
    {
    case PC_BADBIOS:
      PyErr_SetString ( PCError, "Invalid BIOS" );
      goto error;
    case PC_BADOPTROM:
      PyErr_SetString ( PCError, "Invalid OptionROM (VGABIOS)" );
      goto error;
    case PC_UNK_CPU_MODEL:
      PyErr_SetString ( PCError, "Unknown CPU model" );
      goto error;
    case PC_HDD_WRONG_SIZE:
      PyErr_SetString ( PCError, "HDD wrong size" );
      goto error;
    case PC_NOERROR:
    default: break;
    }
  _tracer.eip= PC_CPU.cpu->eip;

  // Mouse.
  _mouse.active= false;
  
  _initialized= true;
  
  Py_RETURN_NONE;

 error:
  if ( _hdd != NULL ) PC_file_free ( _hdd );
  _hdd= NULL;
  if ( _cdrom != NULL ) PC_cdrom_free ( _cdrom );
  _cdrom= NULL;
  if ( _bios != NULL ) free ( _bios );
  _bios= NULL;
  if ( _vgabios != NULL ) free ( _vgabios );
  _vgabios= NULL;
  return NULL;
  
} // end PC_init_module


static PyObject *
PC_loop_module (
                PyObject *self,
                PyObject *args
                )
{

  int n;

  
  CHECK_INITIALIZED;
  
  for ( n= 0; n < NBUFF; ++n ) _audio.buffers[n].full= 0;
  SDL_PauseAudio ( 0 );
  loop ();
  SDL_PauseAudio ( 1 );
  
  Py_RETURN_NONE;
  
} // end PC_loop_module


static PyObject *
PC_steps_module (
                 PyObject *self,
                 PyObject *args
                 )
{

  int nsteps,cc;
  int n;

  
  CHECK_INITIALIZED;

  if ( !PyArg_ParseTuple ( args, "i", &nsteps ) )
    return NULL;
  
  for ( n= 0; n < NBUFF; ++n ) _audio.buffers[n].full= 0;
  SDL_PauseAudio ( 0 );
  cc= nsteps;
  while ( cc > 0 )
    cc-= PC_iter ( cc );
  SDL_PauseAudio ( 1 );
  
  Py_RETURN_NONE;
  
} // end PC_steps_module


static PyObject *
PC_jit_loop_module (
                    PyObject *self,
                    PyObject *args
                    )
{

  int n;

  
  CHECK_INITIALIZED;
  
  for ( n= 0; n < NBUFF; ++n ) _audio.buffers[n].full= 0;
  SDL_PauseAudio ( 0 );
  jit_loop ();
  SDL_PauseAudio ( 1 );
  
  Py_RETURN_NONE;
  
} // end PC_jit_loop_module


static PyObject *
PC_jit_steps_module (
                     PyObject *self,
                     PyObject *args
                     )
{

  int nsteps,cc;
  int n;

  
  CHECK_INITIALIZED;

  if ( !PyArg_ParseTuple ( args, "i", &nsteps ) )
    return NULL;
  
  for ( n= 0; n < NBUFF; ++n ) _audio.buffers[n].full= 0;
  SDL_PauseAudio ( 0 );
  cc= nsteps;
  while ( cc > 0 )
    cc-= PC_jit_iter ( cc );
  SDL_PauseAudio ( 1 );
  
  Py_RETURN_NONE;
  
} // end PC_jit_steps_module


static PyObject *
PC_trace_module (
                 PyObject *self,
                 PyObject *args
                 )
{
  
  int cc,nsteps,n,inst_cc;
  uint32_t bk_addr;
  bool use_bk_addr;
  
  
  CHECK_INITIALIZED;

  _unk_inst= false;
  bk_addr= _tracer.eip;
  nsteps= 1;
  if ( !PyArg_ParseTuple ( args, "|iI", &nsteps, &bk_addr ) )
    return NULL;
  use_bk_addr= (bk_addr!=_tracer.eip);
  SDL_PauseAudio ( 0 );
  cc= 0;
  for ( n= 0; n < nsteps && !_unk_inst; ++n)
    {
      inst_cc= PC_trace ();
      cc+= inst_cc;
      ++_tracer.steps;
      _tracer.cc+= (uint64_t) inst_cc;
      if ( use_bk_addr && bk_addr == _tracer.eip )
        {
          _tracer.eip= PC_CPU.cpu->eip;
          break;
        }
      _tracer.eip= PC_CPU.cpu->eip;
    }
  SDL_PauseAudio ( 1 );
  if ( PyErr_Occurred () != NULL ) return NULL;
  
  return PyLong_FromLong ( cc );
  
} // end PC_trace_module


static PyObject *
PC_jit_trace_module (
                     PyObject *self,
                     PyObject *args
                     )
{
  
  int cc,nsteps,n,inst_cc;
  uint32_t bk_addr;
  bool use_bk_addr;
  
  
  CHECK_INITIALIZED;

  _unk_inst= false;
  bk_addr= _tracer.eip;
  nsteps= 1;
  if ( !PyArg_ParseTuple ( args, "|iI", &nsteps, &bk_addr ) )
    return NULL;
  use_bk_addr= (bk_addr!=_tracer.eip);
  SDL_PauseAudio ( 0 );
  cc= 0;
  for ( n= 0; n < nsteps && !_unk_inst; ++n)
    {
      inst_cc= PC_jit_trace ();
      cc+= inst_cc;
      ++_tracer.steps;
      _tracer.cc+= (uint64_t) inst_cc;
      if ( use_bk_addr && bk_addr == _tracer.eip )
        {
          _tracer.eip= PC_CPU.cpu->eip;
          break;
        }
      _tracer.eip= PC_CPU.cpu->eip;
    }
  SDL_PauseAudio ( 1 );
  if ( PyErr_Occurred () != NULL ) return NULL;
  
  return PyLong_FromLong ( cc );
  
} // end PC_jit_trace_module


static PyObject *
PC_config_debug (
                 PyObject *self,
                 PyObject *args
                 )
{
  
  int flags;
  
  
  CHECK_INITIALIZED;
  if ( !PyArg_ParseTuple ( args, "i", &flags ) )
    return NULL;
  _tracer.dbg_flags= flags;
  
  Py_RETURN_NONE;
  
} // end PC_config_debug


static void
print_seg_reg (
               const IA32_SegmentRegister *seg,
               const char                 *name
               )
{

   SHOW_EIP_CC;
   printf ( "[CPU] %s => selector:%04X\n", name, seg->v );
   SHOW_EIP_CC;
   printf ( "[CPU]         addr:%08X firstb:%08X lastb:%08X\n",
            seg->h.lim.addr, seg->h.lim.firstb, seg->h.lim.lastb );
   SHOW_EIP_CC;
   printf ( "[CPU]         is32:%d r:%d w:%d x:%d null:%d "
            "tss_32:%d d/nc:%d pl:%d dpl:%d\n",
            seg->h.is32, seg->h.readable, seg->h.writable,
            seg->h.executable, seg->h.isnull, seg->h.tss_is32,
            seg->h.data_or_nonconforming, seg->h.pl, seg->h.dpl );
   
} // end print_seg_reg


static const char *
get_fpu_tag_name (
                  const int tag
                  )
{

  switch ( tag )
    {
    case 0: return "[Valid]  ";
    case 1: return "[Zero]   ";
    case 2: return "[Special]";
    case 3: return "[Empty]  ";
    default: return "[???]   ";
    }
  
} // end get_fpu_tag_name


static PyObject *
PC_print_regs (
               PyObject *self,
               PyObject *args
               )
{

  int i;

  
  CHECK_INITIALIZED;

  // Registres especials.
  SHOW_EIP_CC;
  printf ( "[CPU] EIP:%08X EFLAGS:%08X\n",
           PC_CPU.cpu->eip, PC_CPU.cpu->eflags );
  
  // Registres generals
  SHOW_EIP_CC;
  printf ( "[CPU] EAX:%08X EBX:%08X ECX:%08X EDX:%08X\n",
           PC_CPU.cpu->eax.v, PC_CPU.cpu->ebx.v,
           PC_CPU.cpu->ecx.v, PC_CPU.cpu->edx.v );
  SHOW_EIP_CC;
  printf ( "[CPU] ESI:%08X EDI:%08X EBP:%08X ESP:%08X\n",
           PC_CPU.cpu->esi.v, PC_CPU.cpu->edi.v,
           PC_CPU.cpu->ebp.v, PC_CPU.cpu->esp.v );

  // Segments
  print_seg_reg ( &(PC_CPU.cpu->cs), "CS  " );
  print_seg_reg ( &(PC_CPU.cpu->ss), "SS  " );
  print_seg_reg ( &(PC_CPU.cpu->ds), "DS  " );
  print_seg_reg ( &(PC_CPU.cpu->es), "ES  " );
  print_seg_reg ( &(PC_CPU.cpu->fs), "FS  " );
  print_seg_reg ( &(PC_CPU.cpu->gs), "GS  " );

  // LDTR
  print_seg_reg ( &(PC_CPU.cpu->ldtr), "LDTR" );

  // TR
  print_seg_reg ( &(PC_CPU.cpu->tr), "TR  " );
  
  // IDT/GDT
  SHOW_EIP_CC;
  printf ( "[CPU] IDT.base:%04X IDT.limit:%02X GDT.base:%04X GDT.limit:%02X\n",
           PC_CPU.cpu->idtr.addr, PC_CPU.cpu->idtr.lastb,
           PC_CPU.cpu->gdtr.addr, PC_CPU.cpu->gdtr.lastb );

  // Control generals
  SHOW_EIP_CC;
  printf ( "[CPU] CR0:%08X CR2:%08X CR3:%08X CR4:%08X\n",
           PC_CPU.cpu->cr0, PC_CPU.cpu->cr2, PC_CPU.cpu->cr3, PC_CPU.cpu->cr4 );

  // Debug
  SHOW_EIP_CC;
  printf ( "[CPU] DR0:%08X DR1:%08X DR2:%08X DR3:%08X\n",
           PC_CPU.cpu->dr0, PC_CPU.cpu->dr1, PC_CPU.cpu->dr2, PC_CPU.cpu->dr3 );
  SHOW_EIP_CC;
  printf ( "[CPU] DR4:%08X DR5:%08X DR6:%08X DR7:%08X\n",
           PC_CPU.cpu->dr4, PC_CPU.cpu->dr5, PC_CPU.cpu->dr6, PC_CPU.cpu->dr7 );
  
  // FPU registers
  SHOW_EIP_CC;
  printf ( "[CPU] FPU Registers:\n" );
  for ( i= 0; i < 8; ++i )
    {
      SHOW_EIP_CC;
      printf ( "[CPU]   %s R%d: %s %Lg\n",
               i==PC_CPU.cpu->fpu.top?"*":" ",
               i,
               get_fpu_tag_name ( PC_CPU.cpu->fpu.regs[i].tag ),
               PC_CPU.cpu->fpu.regs[i].v );
    }
  SHOW_EIP_CC;
  printf ( "[CPU]   Status:%04X Control:%04X Opcode:%04X\n",
           PC_CPU.cpu->fpu.status, PC_CPU.cpu->fpu.control,
           PC_CPU.cpu->fpu.opcode );
  SHOW_EIP_CC;
  printf ( "[CPU]   Last Instruction Pointer: %04X:%08X\n",
           PC_CPU.cpu->fpu.iptr.selector, PC_CPU.cpu->fpu.iptr.offset );
  SHOW_EIP_CC;
  printf ( "[CPU]   Last Data Pointer:        %04X:%08X\n",
           PC_CPU.cpu->fpu.dptr.selector, PC_CPU.cpu->fpu.dptr.offset );
  
  Py_RETURN_NONE;
  
} // end PC_print_regs


static PyObject *
PC_key_press (
              PyObject *self,
              PyObject *args
              )
{

  int key;

  
  CHECK_INITIALIZED;

  if ( !PyArg_ParseTuple ( args, "i", &key ) )
    return NULL;
  PC_kbd_press ( key );

  Py_RETURN_NONE;
  
} // end PC_key_press


static PyObject *
PC_key_release (
                PyObject *self,
                PyObject *args
                )
{
  
  int key;

  
  CHECK_INITIALIZED;

  if ( !PyArg_ParseTuple ( args, "i", &key ) )
    return NULL;
  PC_kbd_release ( key );
  
  Py_RETURN_NONE;
  
} // end PC_key_release


static PyObject *
PC_set_floppy (
               PyObject *self,
               PyObject *args
               )
{

  const char *fn;
  int drv;
  PC_File *f;
  PC_Error err;
  
  
  CHECK_INITIALIZED;

  if ( !PyArg_ParseTuple ( args, "zi", &fn, &drv ) )
    return NULL;

  if ( drv < 0 || drv > 4 )
    {
      PyErr_SetString ( PCError, "drv < 0 ||  drv > 4" );
      return NULL;
    }

  f= PC_file_new_from_file ( fn, true );
  if ( f == NULL )
    {
      PyErr_Format ( PCError, "unable to open %s", fn );
      return NULL;
    }
  if ( _fd[drv] != NULL )
    {
      PC_file_free ( _fd[drv] );
      _fd[drv]= NULL;
    }
  _fd[drv]= f;
  err= PC_fd_insert_floppy ( f, drv );
  if ( err != PC_NOERROR )
    {
      PyErr_Format ( PCError, "wrong size format for %s", fn );
      PC_file_free ( f );
      _fd[drv]= NULL;
      return NULL;
    }

  Py_RETURN_NONE;
  
} // end PC_set_floppy


static PyObject *
PC_set_cdrom (
              PyObject *self,
              PyObject *args
              )
{

  const char *fn;
  char *err;
  
  
  CHECK_INITIALIZED;

  if ( !PyArg_ParseTuple ( args, "z", &fn ) )
    return NULL;

  err= NULL;
  if ( !PC_cdrom_insert_disc ( _cdrom, fn, &err ) )
    {
      PyErr_Format ( PCError, "unable to insert disc '%s': %s", fn, err );
      free ( err );
      return NULL;
    }
  
  Py_RETURN_NONE;
  
} // end PC_set_cdrom


static PyObject *
PC_cirrus_clgd5446_get_vram (
                             PyObject *self,
                             PyObject *args
                             )
{

  CHECK_INITIALIZED;
  
  return PyBytes_FromStringAndSize
    (
     (const char *) PC_svga_cirrus_clgd5446_get_vram (),
     1*1024*1024 );
  
} // end PC_cirrus_clgd5446_get_vram




/************************/
/* INICIALITZACIÓ MÒDUL */
/************************/

static PyMethodDef PCMethods[]=
  {
   { "close", PC_close_, METH_VARARGS,
      "Free module resources and close the module" },
   { "init", (PyCFunction) PC_init_module, METH_VARARGS|METH_KEYWORDS,
     "Initialize the module (requires bios as argument)" },
   { "loop", PC_loop_module, METH_VARARGS,
      "Run the simulator into a loop and block" },
   { "steps", PC_steps_module, METH_VARARGS,
      "Run the simulator the number of specified steps as"
      " fast as possible" },
   { "jit_loop", PC_jit_loop_module, METH_VARARGS,
      "Run the simulator into a loop and block using JIT" },
   { "jit_steps", PC_jit_steps_module, METH_VARARGS,
      "Run the simulator the number of specified steps as"
      " fast as possible using JIT" },
   { "trace", PC_trace_module, METH_VARARGS,
     "Executes nstep instruction\n"
     "  trace(1,0x00007C46)"},
   { "jit_trace", PC_jit_trace_module, METH_VARARGS,
     "Executes nstep instruction using JIT\n"
     "  jit_trace(1,0x00007C46)"},
   { "config_debug", PC_config_debug, METH_VARARGS,
      "Enable C debugger" },
   { "print_regs", PC_print_regs, METH_NOARGS,
     "Print CPU registers" },
   { "key_press", PC_key_press, METH_VARARGS,
     "Press key" },
   { "key_release", PC_key_release, METH_VARARGS,
     "Press release" },
   { "set_floppy", PC_set_floppy, METH_VARARGS,
     "Set floppy" },
   { "set_cdrom", PC_set_cdrom, METH_VARARGS,
     "Set cdrom" },
   { "cirrus_clgd5446_get_vram", PC_cirrus_clgd5446_get_vram, METH_NOARGS,
      "Returns vgram from CLGD5446" },
   { NULL, NULL, 0, NULL }
  };


static struct PyModuleDef PCmodule=
  {
    PyModuleDef_HEAD_INIT,
    "PC",
    NULL,
    -1,
    PCMethods
  };


PyMODINIT_FUNC
PyInit_PC (void)
{
  
  PyObject *m;
  
  
  m= PyModule_Create ( &PCmodule );
  if ( m == NULL ) return NULL;
  
  _initialized= false;
  PCError= PyErr_NewException ( "PC.error", NULL, NULL );
  Py_INCREF ( PCError );
  PyModule_AddObject ( m, "error", PCError );

  // Debug flags.
  PyModule_AddIntConstant ( m, "DBG_MEM_ACCESS", DBG_MEM_ACCESS );
  PyModule_AddIntConstant ( m, "DBG_PORT_ACCESS", DBG_PORT_ACCESS );
  PyModule_AddIntConstant ( m, "DBG_PCI_REG_ACCESS", DBG_PCI_REG_ACCESS );
  PyModule_AddIntConstant ( m, "DBG_CMOS_RAM_ACCESS", DBG_CMOS_RAM_ACCESS );
  PyModule_AddIntConstant ( m, "DBG_TIMER_OUT_CHANGED", DBG_TIMER_OUT_CHANGED );
  PyModule_AddIntConstant ( m, "DBG_INT_SERVICED", DBG_INT_SERVICED );
  PyModule_AddIntConstant ( m, "DBG_CPU_INST", DBG_CPU_INST );
  PyModule_AddIntConstant ( m, "DBG_SHOW_EIP_CC", DBG_SHOW_EIP_CC );
  PyModule_AddIntConstant ( m, "DBG_VGA_MEM_ACCESS", DBG_VGA_MEM_ACCESS );
  PyModule_AddIntConstant ( m, "DBG_FLOPPY_FIFO_ACCESS",
                            DBG_FLOPPY_FIFO_ACCESS );
  PyModule_AddIntConstant ( m, "DBG_DMA_TRANSFER8", DBG_DMA_TRANSFER8 );
  PyModule_AddIntConstant ( m, "DBG_DMA_TRANSFER16", DBG_DMA_TRANSFER16 );
  PyModule_AddIntConstant ( m, "DBG_TRACE_SOFT_INT", DBG_TRACE_SOFT_INT );

  // KBDSP
  PyModule_AddIntConstant ( m, "KBDSP_NUM_0", PC_KBDSP_NUM_0 );
  PyModule_AddIntConstant ( m, "KBDSP_NUM_PUNTO", PC_KBDSP_NUM_PUNTO );
  PyModule_AddIntConstant ( m, "KBDSP_NUM_ENTRAR", PC_KBDSP_NUM_ENTRAR );
  PyModule_AddIntConstant ( m, "KBDSP_NUM_1", PC_KBDSP_NUM_1 );
  PyModule_AddIntConstant ( m, "KBDSP_NUM_2", PC_KBDSP_NUM_2 );
  PyModule_AddIntConstant ( m, "KBDSP_NUM_3", PC_KBDSP_NUM_3 );
  PyModule_AddIntConstant ( m, "KBDSP_NUM_4", PC_KBDSP_NUM_4 );
  PyModule_AddIntConstant ( m, "KBDSP_NUM_5", PC_KBDSP_NUM_5 );
  PyModule_AddIntConstant ( m, "KBDSP_NUM_6", PC_KBDSP_NUM_6 );
  PyModule_AddIntConstant ( m, "KBDSP_NUM_7", PC_KBDSP_NUM_7 );
  PyModule_AddIntConstant ( m, "KBDSP_NUM_8", PC_KBDSP_NUM_8 );
  PyModule_AddIntConstant ( m, "KBDSP_NUM_9", PC_KBDSP_NUM_9 );
  PyModule_AddIntConstant ( m, "KBDSP_NUM_SUMA", PC_KBDSP_NUM_SUMA );
  PyModule_AddIntConstant ( m, "KBDSP_BLOQ_NUM", PC_KBDSP_BLOQ_NUM );
  PyModule_AddIntConstant ( m, "KBDSP_NUM_DIV", PC_KBDSP_NUM_DIV );
  PyModule_AddIntConstant ( m, "KBDSP_NUM_MUL", PC_KBDSP_NUM_MUL );
  PyModule_AddIntConstant ( m, "KBDSP_NUM_RESTA", PC_KBDSP_NUM_RESTA );
  PyModule_AddIntConstant ( m, "KBDSP_ARRIBA", PC_KBDSP_ARRIBA );
  PyModule_AddIntConstant ( m, "KBDSP_IZQUIERDA", PC_KBDSP_IZQUIERDA );
  PyModule_AddIntConstant ( m, "KBDSP_ABAJO", PC_KBDSP_ABAJO );
  PyModule_AddIntConstant ( m, "KBDSP_DERECHA", PC_KBDSP_DERECHA );
  PyModule_AddIntConstant ( m, "KBDSP_SUPR", PC_KBDSP_SUPR );
  PyModule_AddIntConstant ( m, "KBDSP_FIN", PC_KBDSP_FIN );
  PyModule_AddIntConstant ( m, "KBDSP_AV_PAG", PC_KBDSP_AV_PAG );
  PyModule_AddIntConstant ( m, "KBDSP_INSERT", PC_KBDSP_INSERT );
  PyModule_AddIntConstant ( m, "KBDSP_INICIO", PC_KBDSP_INICIO );
  PyModule_AddIntConstant ( m, "KBDSP_RE_PAG", PC_KBDSP_RE_PAG );
  PyModule_AddIntConstant ( m, "KBDSP_CONTROL", PC_KBDSP_CONTROL );
  PyModule_AddIntConstant ( m, "KBDSP_WINDOWS", PC_KBDSP_WINDOWS );
  PyModule_AddIntConstant ( m, "KBDSP_ALT", PC_KBDSP_ALT );
  PyModule_AddIntConstant ( m, "KBDSP_ESPACIO", PC_KBDSP_ESPACIO );
  PyModule_AddIntConstant ( m, "KBDSP_ALT_GR", PC_KBDSP_ALT_GR );
  PyModule_AddIntConstant ( m, "KBDSP_WIN_DERECHA", PC_KBDSP_WIN_DERECHA );
  PyModule_AddIntConstant ( m, "KBDSP_MENU", PC_KBDSP_MENU );
  PyModule_AddIntConstant ( m, "KBDSP_CONTROL_DERECHA",
                            PC_KBDSP_CONTROL_DERECHA );
  PyModule_AddIntConstant ( m, "KBDSP_MAYUS", PC_KBDSP_MAYUS );
  PyModule_AddIntConstant ( m, "KBDSP_MENOR", PC_KBDSP_MENOR );
  PyModule_AddIntConstant ( m, "KBDSP_Z", PC_KBDSP_Z );
  PyModule_AddIntConstant ( m, "KBDSP_X", PC_KBDSP_X );
  PyModule_AddIntConstant ( m, "KBDSP_C", PC_KBDSP_C );
  PyModule_AddIntConstant ( m, "KBDSP_V", PC_KBDSP_V );
  PyModule_AddIntConstant ( m, "KBDSP_B", PC_KBDSP_B );
  PyModule_AddIntConstant ( m, "KBDSP_N", PC_KBDSP_N );
  PyModule_AddIntConstant ( m, "KBDSP_M", PC_KBDSP_M );
  PyModule_AddIntConstant ( m, "KBDSP_COMA", PC_KBDSP_COMA );
  PyModule_AddIntConstant ( m, "KBDSP_PUNTO", PC_KBDSP_PUNTO );
  PyModule_AddIntConstant ( m, "KBDSP_GUION", PC_KBDSP_GUION );
  PyModule_AddIntConstant ( m, "KBDSP_MAYUS_DERECHA", PC_KBDSP_MAYUS_DERECHA );
  PyModule_AddIntConstant ( m, "KBDSP_BLOQ_MAYUS", PC_KBDSP_BLOQ_MAYUS );
  PyModule_AddIntConstant ( m, "KBDSP_A", PC_KBDSP_A );
  PyModule_AddIntConstant ( m, "KBDSP_S", PC_KBDSP_S );
  PyModule_AddIntConstant ( m, "KBDSP_D", PC_KBDSP_D );
  PyModule_AddIntConstant ( m, "KBDSP_F", PC_KBDSP_F );
  PyModule_AddIntConstant ( m, "KBDSP_G", PC_KBDSP_G );
  PyModule_AddIntConstant ( m, "KBDSP_H", PC_KBDSP_H );
  PyModule_AddIntConstant ( m, "KBDSP_J", PC_KBDSP_J );
  PyModule_AddIntConstant ( m, "KBDSP_K", PC_KBDSP_K );
  PyModule_AddIntConstant ( m, "KBDSP_L", PC_KBDSP_L );
  PyModule_AddIntConstant ( m, "KBDSP_ENYE", PC_KBDSP_ENYE );
  PyModule_AddIntConstant ( m, "KBDSP_TILDE", PC_KBDSP_TILDE );
  PyModule_AddIntConstant ( m, "KBDSP_C_TRENCADA", PC_KBDSP_C_TRENCADA );
  PyModule_AddIntConstant ( m, "KBDSP_TABULADOR", PC_KBDSP_TABULADOR );
  PyModule_AddIntConstant ( m, "KBDSP_Q", PC_KBDSP_Q );
  PyModule_AddIntConstant ( m, "KBDSP_W", PC_KBDSP_W );
  PyModule_AddIntConstant ( m, "KBDSP_E", PC_KBDSP_E );
  PyModule_AddIntConstant ( m, "KBDSP_R", PC_KBDSP_R );
  PyModule_AddIntConstant ( m, "KBDSP_T", PC_KBDSP_T );
  PyModule_AddIntConstant ( m, "KBDSP_Y", PC_KBDSP_Y );
  PyModule_AddIntConstant ( m, "KBDSP_U", PC_KBDSP_U );
  PyModule_AddIntConstant ( m, "KBDSP_I", PC_KBDSP_I );
  PyModule_AddIntConstant ( m, "KBDSP_O", PC_KBDSP_O );
  PyModule_AddIntConstant ( m, "KBDSP_P", PC_KBDSP_P );
  PyModule_AddIntConstant ( m, "KBDSP_ACCENT_OBERT", PC_KBDSP_ACCENT_OBERT );
  PyModule_AddIntConstant ( m, "KBDSP_SIGNO_MAS", PC_KBDSP_SIGNO_MAS );
  PyModule_AddIntConstant ( m, "KBDSP_ENTRAR", PC_KBDSP_ENTRAR );
  PyModule_AddIntConstant ( m, "KBDSP_SUPER_O", PC_KBDSP_SUPER_O );
  PyModule_AddIntConstant ( m, "KBDSP_1", PC_KBDSP_1 );
  PyModule_AddIntConstant ( m, "KBDSP_2", PC_KBDSP_2 );
  PyModule_AddIntConstant ( m, "KBDSP_3", PC_KBDSP_3 );
  PyModule_AddIntConstant ( m, "KBDSP_4", PC_KBDSP_4 );
  PyModule_AddIntConstant ( m, "KBDSP_5", PC_KBDSP_5 );
  PyModule_AddIntConstant ( m, "KBDSP_6", PC_KBDSP_6 );
  PyModule_AddIntConstant ( m, "KBDSP_7", PC_KBDSP_7 );
  PyModule_AddIntConstant ( m, "KBDSP_8", PC_KBDSP_8 );
  PyModule_AddIntConstant ( m, "KBDSP_9", PC_KBDSP_9 );
  PyModule_AddIntConstant ( m, "KBDSP_0", PC_KBDSP_0 );
  PyModule_AddIntConstant ( m, "KBDSP_COMILLAS", PC_KBDSP_COMILLAS );
  PyModule_AddIntConstant ( m, "KBDSP_ABRE_EXCLAMACION",
                            PC_KBDSP_ABRE_EXCLAMACION );
  PyModule_AddIntConstant ( m, "KBDSP_RETROCESO", PC_KBDSP_RETROCESO );
  PyModule_AddIntConstant ( m, "KBDSP_IMP_PNT", PC_KBDSP_IMP_PNT );
  PyModule_AddIntConstant ( m, "KBDSP_CONTROL_IMP_PNT",
                            PC_KBDSP_CONTROL_IMP_PNT );
  PyModule_AddIntConstant ( m, "KBDSP_MAYUS_IMP_PNT", PC_KBDSP_MAYUS_IMP_PNT );
  PyModule_AddIntConstant ( m, "KBDSP_ALT_IMP_PNT", PC_KBDSP_ALT_IMP_PNT );
  PyModule_AddIntConstant ( m, "KBDSP_BLOQ_DESPL", PC_KBDSP_BLOQ_DESPL );
  PyModule_AddIntConstant ( m, "KBDSP_PAUSA", PC_KBDSP_PAUSA );
  PyModule_AddIntConstant ( m, "KBDSP_CONTROL_PAUSA", PC_KBDSP_CONTROL_PAUSA );
  PyModule_AddIntConstant ( m, "KBDSP_F9", PC_KBDSP_F9 );
  PyModule_AddIntConstant ( m, "KBDSP_F10", PC_KBDSP_F10 );
  PyModule_AddIntConstant ( m, "KBDSP_F11", PC_KBDSP_F11 );
  PyModule_AddIntConstant ( m, "KBDSP_F12", PC_KBDSP_F12 );
  PyModule_AddIntConstant ( m, "KBDSP_F5", PC_KBDSP_F5 );
  PyModule_AddIntConstant ( m, "KBDSP_F6", PC_KBDSP_F6 );
  PyModule_AddIntConstant ( m, "KBDSP_F7", PC_KBDSP_F7 );
  PyModule_AddIntConstant ( m, "KBDSP_F8", PC_KBDSP_F8 );
  PyModule_AddIntConstant ( m, "KBDSP_F1", PC_KBDSP_F1 );
  PyModule_AddIntConstant ( m, "KBDSP_F2", PC_KBDSP_F2 );
  PyModule_AddIntConstant ( m, "KBDSP_F3", PC_KBDSP_F3 );
  PyModule_AddIntConstant ( m, "KBDSP_F4", PC_KBDSP_F4 );
  PyModule_AddIntConstant ( m, "KBDSP_ESC", PC_KBDSP_ESC );
  
  return m;

} // end PyInit_PC
