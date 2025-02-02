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
 *  main.c - Implementació de les funcions principals.
 *
 */


#include <limits.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdlib.h>

#include "PC.h"




/**********/
/* MACROS */
/**********/

// Cicles per instrucció. Per simplificar moltíssim vaig a ficar 2 o 2.5.
// El valor es calcula com CC_PER_INST/SCALE_FREQ
// NOTA!!! 4/2 reflexa millor la realitat.
#define CC_PER_INST 4
#define SCALE_FREQ 2




/*********/
/* ESTAT */
/*********/

// Callbacks
static PC_CPUInst *_cpu_inst;
static void *_udata;
static PC_TraceSoftInt *_trace_soft_int;

// Configuració.
static PC_Config _config;

// Dispositius PCI connectats.
const PC_PCICallbacks * _pci_callbacks[PC_PCI_DEVICE_NULL+1];

// Indica que estem en mode jit.
static bool _jit_mode;




/***********************/
/* VARIABLES PÚBLIQUES */
/***********************/

int PC_Clock;

long PC_ClockFreq;

int PC_NextEventCC;




/*********************/
/* FUNCIONS PRIVADES */
/*********************/

#if PC_BE
uint16_t
PC_swap16 (
           const uint16_t val
           )
{

  uint16_t ret;
  
  
  ((uint8_t *) &ret)[0]= ((const uint8_t *) &val)[1];
  ((uint8_t *) &ret)[1]= ((const uint8_t *) &val)[0];

  return ret;
  
} // end PC_swap16


uint32_t
PC_swap32 (
           const uint32_t val
           )
{

  uint32_t ret;
  
  
  ((uint8_t *) &ret)[0]= ((const uint8_t *) &val)[3];
  ((uint8_t *) &ret)[1]= ((const uint8_t *) &val)[2];
  ((uint8_t *) &ret)[2]= ((const uint8_t *) &val)[1];
  ((uint8_t *) &ret)[3]= ((const uint8_t *) &val)[0];

  return ret;
  
} // end PC_swap32


uint64_t
PC_swap64 (
           const uint64_t val
           )
{

  uint64_t ret;
  
  
  ((uint8_t *) &ret)[0]= ((const uint8_t *) &val)[7];
  ((uint8_t *) &ret)[1]= ((const uint8_t *) &val)[6];
  ((uint8_t *) &ret)[2]= ((const uint8_t *) &val)[5];
  ((uint8_t *) &ret)[3]= ((const uint8_t *) &val)[4];
  ((uint8_t *) &ret)[4]= ((const uint8_t *) &val)[3];
  ((uint8_t *) &ret)[5]= ((const uint8_t *) &val)[2];
  ((uint8_t *) &ret)[6]= ((const uint8_t *) &val)[1];
  ((uint8_t *) &ret)[7]= ((const uint8_t *) &val)[0];
  
  return ret;
  
} // end PC_swap64
#endif




/**********************/
/* FUNCIONS PÚBLIQUES */
/**********************/

PC_Error
PC_init (
         uint8_t           *bios,
         size_t             bios_size,
         PC_IDEDevice       ide_devices[2][2],
         const PC_Frontend *frontend,
         void              *udata,
         const PC_Config   *config
         )
{
  
  PC_Error err;
  bool pci_devs[PC_PCI_DEVICE_NULL];
  int i;
  
  
  _config= *config;
  _jit_mode= false;
  
  // Tracer.
  _cpu_inst= frontend->trace!=NULL ?
    frontend->trace->cpu_inst : NULL;
  _trace_soft_int= frontend->trace!=NULL ?
    frontend->trace->trace_soft_int : NULL;
  
  // Clock.
  PC_Clock= 0;
  PC_NextEventCC= INT_MAX;

  // ClockFreq.
  switch ( config->cpu_model )
    {
    case IA32_CPU_P5_60MHZ:    PC_ClockFreq= 60000000; break;
    case IA32_CPU_P5_66MHZ:    PC_ClockFreq= 66000000; break;
    case IA32_CPU_P54C_75MHZ:  PC_ClockFreq= 75000000; break;
    case IA32_CPU_P54C_90MHZ:  PC_ClockFreq= 90000000; break;
    case IA32_CPU_P54C_100MHZ: PC_ClockFreq= 100000000; break;
    default: return PC_UNK_CPU_MODEL;
    }
  PC_ClockFreq*= SCALE_FREQ;
  
  // Prepara PCIdevs
  err= PC_NOERROR;
  for ( i= 0; i != PC_PCI_DEVICE_NULL; ++i )
    pci_devs[i]= false;
  for ( i= 0;
        config->pci_devs[i].dev != PC_PCI_DEVICE_NULL && err == PC_NOERROR;
        ++i )
    if ( !pci_devs[config->pci_devs[i].dev] )
      {
        pci_devs[config->pci_devs[i].dev]= true;
        switch ( config->pci_devs[i].dev )
          {
          case PC_PCI_DEVICE_SVGA_CIRRUS_CLGD5446:
            err= PC_svga_cirrus_clgd5446_init
              ( frontend->warning,
                frontend->update_screen,
                frontend->trace!=NULL ?
                frontend->trace->vga_mem_access:
                NULL,
                frontend->trace!=NULL ?
                frontend->trace->vga_mem_linear_access:
                NULL,
                config->pci_devs[i].optrom,
                config->pci_devs[i].optrom_size,
                udata );
            _pci_callbacks[i]= &PC_svga_cirrus_clgd5446;
            break;
          default: break; // CALLA
          }
      }
  _pci_callbacks[i]= NULL;
  if ( err != PC_NOERROR ) return err;
  
  // Mòduls.
  PC_cpu_init ( frontend->warning, udata, config );
  PC_io_init ( frontend->warning,
               frontend->write_sb_dbg_port,
               frontend->trace!=NULL?frontend->trace->port_access:NULL,
               _pci_callbacks,
               udata, &_config );
  PC_mtxc_init ( frontend->warning,
                 frontend->trace!=NULL?frontend->trace->mem_access:NULL,
                 frontend->trace!=NULL?frontend->trace->pci_reg_access:NULL,
                 _pci_callbacks,
                 udata, &_config );
  err= PC_piix4_init ( bios, bios_size, ide_devices,
                       frontend->warning, udata, &_config );
  if ( err != PC_NOERROR ) return err;
  PC_rtc_init ( frontend->warning,
                frontend->get_current_time,
                frontend->get_cmos_ram,
                frontend->trace!=NULL?frontend->trace->cmos_ram_access:NULL,
                udata, &_config );
  PC_dma_init ( frontend->warning,
                frontend->trace!=NULL?frontend->trace->dma_transfer8:NULL,
                frontend->trace!=NULL?frontend->trace->dma_transfer16:NULL,
                udata, &_config );
  PC_ic_init ( frontend->warning,
               frontend->trace!=NULL?frontend->trace->int_serviced:NULL,
               udata, &_config );
  PC_timers_init ( frontend->warning,
                   frontend->trace!=NULL?
                   frontend->trace->timer_out_changed:NULL,
                   udata, &_config );
  PC_pmtimer_init ( frontend->warning,
                    udata );
  PC_ps2_init ( frontend->warning, udata, &_config );
  PC_fd_init ( frontend->warning,
               frontend->trace!=NULL?
               frontend->trace->floppy_fifo_access:NULL,
               udata, &_config );
  PC_speaker_init ( frontend->warning, udata );
  PC_sb16_init ( frontend->warning, udata );
  PC_sound_init ( frontend->warning, frontend->play_sound, udata );
  
  return PC_NOERROR;
  
} // end PC_init


//#include <glib.h>
//gint64 __CC;
int
PC_iter (
         const int cc
         )
{
  
  /* NOTA!!! L'esquema de clock(end_iter) està relacionat en executar
   * un bloc d'instruccions seguit i que es puguen produir events en
   * eixe bloc. Les interrupcions són molt sensibles a això. Per
   * aquest motiu 'INT' clockeja tots els components abans
   * d'actualitzar les mascares i altres valors. No obstant açò amb la
   * nova aproximació de parar abans de cada event en realitat ja no
   * és necessaria que 'INT' faça res. Sí que és cert, que si en el
   * futur la únitat mínima deixara de ser la instrucció i fora un
   * bloc d'instruccions tornaria a ser necessari. Per aquest motiu he
   * decidit comentar l'esquema clock(end_iter), per si de cas en el
   * futur ho recupere.
   */

  int cc_remain,cc_total,tmp,i;

  
  if ( _jit_mode ) { _jit_mode= false; PC_dma_set_mode_jit ( false ); }
  
  //gint64 t0,tf,A,B,C,D;A=B=C=D=__CC=0;
  cc_remain= cc;
  cc_total= 0;
  while ( cc_remain > 0 )
    {
      //t0=g_get_monotonic_time();
      // Inicialitza iteració.
      PC_NextEventCC= cc_remain;
      tmp= PC_timers_next_event_cc ();
      if ( tmp < PC_NextEventCC ) PC_NextEventCC= tmp;
      tmp= PC_pmtimer_next_event_cc ();
      if ( tmp < PC_NextEventCC ) PC_NextEventCC= tmp;
      tmp= PC_rtc_next_event_cc ();
      if ( tmp < PC_NextEventCC ) PC_NextEventCC= tmp;
      tmp= PC_dma_next_event_cc ();
      if ( tmp < PC_NextEventCC ) PC_NextEventCC= tmp;
      tmp= PC_ps2_next_event_cc ();
      if ( tmp < PC_NextEventCC ) PC_NextEventCC= tmp;
      tmp= PC_fd_next_event_cc ();
      if ( tmp < PC_NextEventCC ) PC_NextEventCC= tmp;
      tmp= PC_piix4_ide_next_event_cc ();
      if ( tmp < PC_NextEventCC ) PC_NextEventCC= tmp;
      tmp= PC_speaker_next_event_cc ();
      if ( tmp < PC_NextEventCC ) PC_NextEventCC= tmp;
      tmp= PC_sb16_next_event_cc ();
      if ( tmp < PC_NextEventCC ) PC_NextEventCC= tmp;
      for ( i= 0; _pci_callbacks[i] != NULL; ++i )
        if ( _pci_callbacks[i]->clock != NULL )
          {
            tmp= _pci_callbacks[i]->clock->next_event_cc ();
            if ( tmp < PC_NextEventCC ) PC_NextEventCC= tmp;
          }
      PC_Clock= 0;
      // Itera tot els que es puga.
      // NOTA!! Qualsevol actualització de PC_NextEventCC s'ha de fer
      // tenint en compte que en aquesta iteració en portem PC_Clock
      // cicles executats.
      //tf= g_get_monotonic_time();A+= tf-t0;t0=tf;
      do {
        IA32_exec_next_inst ( &PC_CPU );
        PC_Clock+= CC_PER_INST;
      } while ( PC_Clock < PC_NextEventCC );
      //tf= g_get_monotonic_time();B+= tf-t0;t0=tf;
      // Consumeix cicles pendents (executa possible event)
      PC_timers_end_iter ();
      PC_pmtimer_end_iter ();
      PC_rtc_end_iter ();
      PC_dma_end_iter ();
      PC_ps2_end_iter ();
      PC_fd_end_iter ();
      PC_piix4_ide_end_iter ();
      PC_speaker_end_iter (); // <-- Després de timers
      PC_sb16_end_iter ();
      //tf= g_get_monotonic_time();D+= tf-t0;tf=t0;
      for ( i= 0; _pci_callbacks[i] != NULL; ++i )
        if ( _pci_callbacks[i]->clock != NULL )
          _pci_callbacks[i]->clock->end_iter ();
      //tf= g_get_monotonic_time();C+= tf-t0;t0=tf;
      // Prepara següent iteració.
      cc_total+= PC_Clock;
      cc_remain-= PC_Clock;
      PC_Clock= 0;
      
    }
  
  return cc_total;
  
} // end PC_iter

uint64_t __CCSIM;
int
PC_jit_iter (
             const int cc
             )
{
  
  /* NOTA!!! L'esquema de clock(end_iter) està relacionat en executar
   * un bloc d'instruccions seguit i que es puguen produir events en
   * eixe bloc. Les interrupcions són molt sensibles a això. Per
   * aquest motiu 'INT' clockeja tots els components abans
   * d'actualitzar les mascares i altres valors. No obstant açò amb la
   * nova aproximació de parar abans de cada event en realitat ja no
   * és necessaria que 'INT' faça res. Sí que és cert, que si en el
   * futur la únitat mínima deixara de ser la instrucció i fora un
   * bloc d'instruccions tornaria a ser necessari. Per aquest motiu he
   * decidit comentar l'esquema clock(end_iter), per si de cas en el
   * futur ho recupere.
   */

  int cc_remain,cc_total,tmp,i;


  if ( !_jit_mode ) { _jit_mode= true; PC_dma_set_mode_jit ( true ); }

  //gint64 t0,tf,A,B,C,D;A=B=C=D=__CC=0;__CCSIM=0;
  cc_remain= cc;
  cc_total= 0;
  while ( cc_remain > 0 )
    {
      //t0=g_get_monotonic_time();
      // Inicialitza iteració.
      PC_NextEventCC= cc_remain;
      tmp= PC_timers_next_event_cc ();
      if ( tmp < PC_NextEventCC ) PC_NextEventCC= tmp;
      tmp= PC_pmtimer_next_event_cc ();
      if ( tmp < PC_NextEventCC ) PC_NextEventCC= tmp;
      tmp= PC_rtc_next_event_cc ();
      if ( tmp < PC_NextEventCC ) PC_NextEventCC= tmp;
      tmp= PC_dma_next_event_cc ();
      if ( tmp < PC_NextEventCC ) PC_NextEventCC= tmp;
      tmp= PC_ps2_next_event_cc ();
      if ( tmp < PC_NextEventCC ) PC_NextEventCC= tmp;
      tmp= PC_fd_next_event_cc ();
      if ( tmp < PC_NextEventCC ) PC_NextEventCC= tmp;
      tmp= PC_piix4_ide_next_event_cc ();
      if ( tmp < PC_NextEventCC ) PC_NextEventCC= tmp;
      tmp= PC_speaker_next_event_cc ();
      if ( tmp < PC_NextEventCC ) PC_NextEventCC= tmp;
      tmp= PC_sb16_next_event_cc ();
      if ( tmp < PC_NextEventCC ) PC_NextEventCC= tmp;
      for ( i= 0; _pci_callbacks[i] != NULL; ++i )
        if ( _pci_callbacks[i]->clock != NULL )
          {
            tmp= _pci_callbacks[i]->clock->next_event_cc ();
            if ( tmp < PC_NextEventCC ) PC_NextEventCC= tmp;
          }
      PC_Clock= 0;
      // Itera tot els que es puga.
      // NOTA!! Qualsevol actualització de PC_NextEventCC s'ha de fer
      // tenint en compte que en aquesta iteració en portem PC_Clock
      // cicles executats.
      //tf= g_get_monotonic_time();A+= tf-t0;t0=tf;
      do {
        IA32_jit_exec_next_inst ( PC_CPU_JIT );
        PC_Clock+= CC_PER_INST;
      } while ( PC_Clock < PC_NextEventCC );
      //tf= g_get_monotonic_time();B+= tf-t0;t0=tf;
      
      // Consumeix cicles pendents (executa possible event)
      PC_timers_end_iter ();
      PC_pmtimer_end_iter ();
      PC_rtc_end_iter ();
      PC_dma_end_iter ();
      PC_ps2_end_iter ();
      PC_fd_end_iter ();
      PC_piix4_ide_end_iter ();
      PC_speaker_end_iter (); // <-- Després de timers
      PC_sb16_end_iter ();
      //tf= g_get_monotonic_time();D+= tf-t0;tf=t0;
      for ( i= 0; _pci_callbacks[i] != NULL; ++i )
        if ( _pci_callbacks[i]->clock != NULL )
          _pci_callbacks[i]->clock->end_iter ();
      //tf= g_get_monotonic_time();C+= tf-t0;t0=tf;
      // Prepara següent iteració.
      cc_total+= PC_Clock;
      cc_remain-= PC_Clock;
      PC_Clock= 0;
      
    }
  
  return cc_total;
  
} // end PC_jit_iter


int
PC_trace (void)
{

  IA32_Inst inst;
  int ret,i;
  uint32_t eip;
  

  if ( _jit_mode ) { _jit_mode= false; PC_dma_set_mode_jit ( false ); }
  
  if ( _cpu_inst != NULL )
    {
      if ( PC_cpu_dis ( &inst, &eip ) )
        _cpu_inst ( &inst, eip, _udata );
    }

  // Inicialitza traça
  PC_io_set_mode_trace ( true );
  PC_mtxc_set_mode_trace ( true );
  PC_rtc_set_mode_trace ( true );
  PC_ic_set_mode_trace ( true );
  PC_timers_set_mode_trace ( true );
  PC_fd_set_mode_trace ( true );
  PC_dma_set_mode_trace ( true );
  for ( i= 0; _pci_callbacks[i] != NULL; ++i )
    if ( _pci_callbacks[i]->set_mode_trace != NULL )
      _pci_callbacks[i]->set_mode_trace ( true );
  PC_CPU.trace_soft_int= _trace_soft_int;
    
  // Inicialitza iteració
  PC_NextEventCC= 1;
  IA32_exec_next_inst ( &PC_CPU );
  PC_Clock+= CC_PER_INST;

  // End iter
  PC_timers_end_iter ();
  PC_pmtimer_end_iter ();
  PC_rtc_end_iter ();
  PC_dma_end_iter ();
  PC_ps2_end_iter ();
  PC_fd_end_iter ();
  PC_piix4_ide_end_iter ();
  PC_speaker_end_iter ();  // <-- Després de timers
  PC_sb16_end_iter ();
  for ( i= 0; _pci_callbacks[i] != NULL; ++i )
    if ( _pci_callbacks[i]->clock != NULL )
      _pci_callbacks[i]->clock->end_iter ();

  // Finalitza traça
  PC_CPU.trace_soft_int= NULL;
  for ( i= 0; _pci_callbacks[i] != NULL; ++i )
    if ( _pci_callbacks[i]->set_mode_trace != NULL )
      _pci_callbacks[i]->set_mode_trace ( false );
  PC_dma_set_mode_trace ( false );
  PC_fd_set_mode_trace ( false );
  PC_timers_set_mode_trace ( false );
  PC_ic_set_mode_trace ( false );
  PC_rtc_set_mode_trace ( false );
  PC_mtxc_set_mode_trace ( false );
  PC_io_set_mode_trace ( false );
  ret= PC_Clock;
  PC_Clock= 0;

  return ret;
  
} // end PC_trace


int
PC_jit_trace (void)
{

  IA32_Inst inst;
  int ret,i;
  uint32_t eip;
  

  if ( !_jit_mode ) { _jit_mode= true; PC_dma_set_mode_jit ( true ); }
  
  if ( _cpu_inst != NULL )
    {
      if ( PC_cpu_jit_dis ( &inst, &eip ) )
        _cpu_inst ( &inst, eip, _udata );
    }
  
  // Inicialitza traça
  PC_io_set_mode_trace ( true );
  PC_mtxc_set_mode_trace ( true );
  PC_rtc_set_mode_trace ( true );
  PC_ic_set_mode_trace ( true );
  PC_timers_set_mode_trace ( true );
  PC_fd_set_mode_trace ( true );
  PC_dma_set_mode_trace ( true );
  for ( i= 0; _pci_callbacks[i] != NULL; ++i )
    if ( _pci_callbacks[i]->set_mode_trace != NULL )
      _pci_callbacks[i]->set_mode_trace ( true );
  
  // Inicialitza iteració
  PC_NextEventCC= 1;
  IA32_jit_exec_next_inst ( PC_CPU_JIT );
  PC_Clock+= CC_PER_INST;
  
  // End iter
  PC_timers_end_iter ();
  PC_pmtimer_end_iter ();
  PC_rtc_end_iter ();
  PC_dma_end_iter ();
  PC_ps2_end_iter ();
  PC_fd_end_iter ();
  PC_piix4_ide_end_iter ();
  PC_speaker_end_iter (); // <-- Després de timers
  PC_sb16_end_iter ();
  for ( i= 0; _pci_callbacks[i] != NULL; ++i )
    if ( _pci_callbacks[i]->clock != NULL )
      _pci_callbacks[i]->clock->end_iter ();

  // Finalitza traça
  for ( i= 0; _pci_callbacks[i] != NULL; ++i )
    if ( _pci_callbacks[i]->set_mode_trace != NULL )
      _pci_callbacks[i]->set_mode_trace ( false );
  PC_dma_set_mode_trace ( false );
  PC_fd_set_mode_trace ( false );
  PC_timers_set_mode_trace ( false );
  PC_ic_set_mode_trace ( false );
  PC_rtc_set_mode_trace ( false );
  PC_mtxc_set_mode_trace ( false );
  PC_io_set_mode_trace ( false );
  ret= PC_Clock;
  PC_Clock= 0;

  return ret;
  
} // end PC_jit_trace


void
PC_close (void)
{
  
  PC_mtxc_close ();
  PC_cpu_close ();
  
} // end PC_close


void
PC_msg (
        const char *fmt,
        ...
        )
{

  va_list ap;


  va_start ( ap, fmt );
  printf ( "[CAL_IMPLEMENTAR] " );
  vprintf ( fmt, ap );
  printf ( "\n" );
  va_end ( ap );
  
} // end PC_msg
