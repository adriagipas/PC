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
 *  cpu.c - Implementació del mòdul de la cpu.
 *
 */


#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#include "PC.h"




/*********/
/* ESTAT */
/*********/

// Callbacks.
static PC_Warning *_warning;
static void *_udata;

// Registres.
static IA32_CPU _regs;

// Decodificador
static IA32_Disassembler _dis;
static IA32_Disassembler _dis_jit;




/***********/
/* MÈTODES */
/***********/

static void
lock (
      void *udata
      )
{
  fprintf ( stderr, "cpu::lock no s'ha implementat\n" );
} // end lock


static void
unlock (
        void *udata
        )
{
  fprintf ( stderr, "cpu::unlock no s'ha implementat\n" );
} // end unlock




/***********************/
/* VARIABLES PÚBLIQUES */
/***********************/

IA32_Interpreter PC_CPU;

IA32_JIT *PC_CPU_JIT;




/**********************/
/* FUNCIONS PÚBLIQUES */
/**********************/

void
PC_cpu_init (
             PC_Warning      *warning,
             void            *udata,
             const PC_Config *config
             )
{

  static const size_t RAM_SIZE_MB[PC_RAM_SIZE_SENTINEL]=
    { 4, 8, 16, 24, 32, 48, 64, 96, 128, 192, 256 };

  IA32_JIT_MemArea mem_areas[2];

  
  // Callback.
  _warning= warning;
  _udata= udata;
  
  // Registres CPU
  IA32_cpu_init ( &_regs, IA32_CPU_POWER_UP, config->cpu_model );

  // Intèrpret
  PC_CPU.cpu= &_regs;
  PC_CPU.udata= udata;
  PC_CPU.warning= warning;
  PC_CPU.lock= lock;
  PC_CPU.unlock= unlock;
  PC_CPU.trace_soft_int= NULL;
  // --> Dels callback de memòria s'encarregua el mòdul de memòria!!
  // --> No passar res fer l'init sense inicialitzar-ho.
  IA32_interpreter_init ( &PC_CPU );
  
  // JIT
  // Aposte per pàgines de 4K (12 bits)
  mem_areas[0].addr= 0x00000000;
  mem_areas[0].size= RAM_SIZE_MB[config->ram_size]*1024*1024;
  mem_areas[1].addr= 0xFFF80000;
  mem_areas[1].size= (0xFFFFFFFF-mem_areas[1].addr)+1;
  PC_CPU_JIT= IA32_jit_new ( &_regs, PC_JIT_BITS_PAGE, true, mem_areas, 2 );
  PC_CPU_JIT->udata= udata;
  PC_CPU_JIT->warning= warning;

  // Decodificador.
  IA32_interpreter_init_dis ( &PC_CPU, &_dis );
  IA32_jit_init_dis ( PC_CPU_JIT, &_dis_jit );

} // end PC_cpu_init


void
PC_cpu_close (void)
{
  IA32_jit_free ( PC_CPU_JIT );
} // end PC_cpu_close


void
PC_cpu_reset (void)
{

  // Registres CPU
  IA32_cpu_init ( &_regs, IA32_CPU_RESET, _regs.model );

  // Intèrpret
  IA32_interpreter_init ( &PC_CPU );

  // JIT
  IA32_jit_reset ( PC_CPU_JIT );
  
} // end PC_cpu_reset


bool
PC_cpu_dis (
            IA32_Inst *inst,
            uint32_t  *eip
            )
{

  // L'offset s'ignora en este decodificador!
  if ( !IA32_dis ( &_dis, 0, inst ) )
    return false;
  *eip= PC_CPU.cpu->eip;

  return true;
  
} // end PC_cpu_dis


bool
PC_cpu_jit_dis (
                IA32_Inst *inst,
                uint32_t  *eip
                )
{

  // L'offset s'ignora en este decodificador!
  if ( !IA32_dis ( &_dis_jit, 0, inst ) )
    return false;
  *eip= PC_CPU_JIT->_cpu->eip;

  return true;
  
} // end PC_cpu_dis
