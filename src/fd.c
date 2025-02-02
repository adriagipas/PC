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
 *  fd.c - Implementació del controlador 82077AA de disqueteres.
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

#define FIFO_SIZE 16
#define OP_NUM_ARGS 9

#define SECTOR_SIZE 512




/*********/
/* ESTAT */
/*********/

// Callbacks.
static PC_Warning *_warning;
static PC_FloppyFIFOAccess *_fifo_access;
static void *_udata;

static const PC_Config *_config;

// Funcions lectura tracejables
static void (*_fifo_write) (const uint8_t data);
static uint8_t (*_fifo_read) (void);
static uint8_t (*_dma_read) (void);

// Estat
static struct
{

  // DIGITAL OUTPUT REGISTER (DOR)
  struct
  {
    bool    motor_enabled[4];
    bool    irq_dma_enabled;
    bool    reset; // Quan passa de cert a fals fa un reset.
    uint8_t drive_sel; // 0..3
  } dor;

  // MAIN STATUS REGISTER (MSR)
  struct
  {
    bool rqm; // Indica que es pot transerir dades
    bool dio; // Indica la direcció. Cert -> llegir
    bool non_dma;
    bool command_busy;
    bool drv_busy[4];
  } msr;

  // DATARATE SELECT REGISTER (DSR)
  struct
  {
    // FALTEN COSES
    uint8_t drate;
  } dsr;
  
} _regs;

// Estat que no està en els registres
static struct
{
  
  bool drive_polling;
  bool implied_seek;
  int  fifo_thr;
  enum {
    RESET_STATE,
    WAIT_CMD,
    WAIT_RECALIBRATE_ARG,
    WAIT_READ_ID_ARG,
    WAIT_SPECIFY_ARG1,
    WAIT_SPECIFY_ARG2,
    WAIT_READ_DATA_ARGS,
    WAIT_READ_DATA_ARG8,
    WAIT_SEEK_ARG1,
    WAIT_SEEK_ARG2,
    EXEC_PHASE,
    READ_RESULTS
  }    cmd_state;
  struct
  {
    PC_File *f;
    int      num_C;
    int      num_H;
    int      num_S;
    int      current_C;
    bool     head_ready;
  }    files[4];
  struct
  {
    enum {
      OP_NONE,
      OP_RECALIBRATE,
      OP_READ_ID,
      OP_READ_DATA_DMA,
      OP_SEEK
    }       op;
    uint8_t args[OP_NUM_ARGS];
    int     N;
  }    op_state[4];
  struct
  {
    enum {
      INT_NONE,
      INT_POLLING4,
      INT_POLLING3,
      INT_POLLING2,
      INT_POLLING1,
      INT_RECALIBRATE,
      INT_SEEK
    }       state;
    uint8_t st0;
    uint8_t pcn;
    int     drv;
  }    int_state;
  uint8_t srt; // Step Rate Time
  uint8_t hut; // Head Unload Time
  uint8_t hlt; // Head Load Time
  bool    use_dma;
  struct
  {
    uint8_t v[OP_NUM_ARGS];
    int     N;
  }       cmd_args;
  struct
  {
    uint8_t drv;
    uint8_t buf[SECTOR_SIZE];
    int     N; // bytes en el buffer
    int     p; // bytes ja processats
    int     current_sec; // Comença per 0
    int     end_sec; // Comença per 0
    long    track_offset;
    int     C,H,S; // CHS del següent sector a llegir
    enum {
      DMA_OP_NONE,
      DMA_OP_READ_DATA
    }       op;
  }       dma_state; // Informació on desar estat operacions DMA
  
} _state;

// FIFO
static struct
{
  int     p;
  int     N;
  uint8_t v[FIFO_SIZE];
} _fifo;

// Timing
static struct
{

  int cc_used;
  int cc;
  int cctoReset; // 0 vol dir que no estem esperant cap reset
  int cctoProcByte; // 0 vol dir que no estem esperant a processar
  int cctoReadResult; // 0 vol dir que no estem esperant a llegir
  int cctoOp[4];
  int cctoHUT[4];
  int cctoEvent;
  
  int cc__proc_byte; // Valor d'espera per a proc_byte
  int cc__read_result; // Valor d'espera per a proc_byte
  int cc__srt; // Cicles SRT
  int cc__hut; // Cicles HUT
  int cc__hlt; // Cicles HLT
  int cc__byte; // Cicles per byte (Ho gaste per a DMA)
  
} _timing;

// Per a indicar que estem en meitat d'un clock
static bool _in_clock;




/*********************/
/* FUNCIONS PRIVADES */
/*********************/

static void
update_cc_to_event (void)
{

  int cc,i;

  
  // Per defecte 1s
  _timing.cctoEvent= PC_ClockFreq;
  // Esperant a un reset
  if ( _timing.cctoReset > 0 && _timing.cctoReset < _timing.cctoEvent )
    _timing.cctoEvent= _timing.cctoReset;
  // Esperant a què es processe un byte
  if ( _timing.cctoProcByte > 0 && _timing.cctoProcByte < _timing.cctoEvent )
    _timing.cctoEvent= _timing.cctoProcByte;
  // Esperant a què es puga llegir el byte següent
  if ( _timing.cctoReadResult > 0 &&
       _timing.cctoReadResult < _timing.cctoEvent )
    _timing.cctoEvent= _timing.cctoReadResult;
  // Esperant operació
  for ( i= 0; i < 4; ++i )
    if ( _timing.cctoOp[i] > 0 && _timing.cctoOp[i] < _timing.cctoEvent )
      _timing.cctoEvent= _timing.cctoOp[i];
  // Esperant HUT
  for ( i= 0; i < 4; ++i )
    if ( _timing.cctoHUT[i] > 0 && _timing.cctoHUT[i] < _timing.cctoEvent )
      _timing.cctoEvent= _timing.cctoHUT[i];
  
  // Actualitza PC_NextEventCC
  cc= PC_fd_next_event_cc ();
  cc+= PC_Clock; // Medim sempre des de que PC_Clock és 0
  if ( cc < PC_NextEventCC )
    PC_NextEventCC= cc;
  
} // end update_cc_to_event


static void
update_cc_variable (void)
{

  int srt_us,hut_us,hlt_us;
  double f;
  

  // Calcula microsegons
  switch ( _regs.dsr.drate )
    {
      // 500 Kbps
    case 0:
      srt_us= 16000 - 1000*_state.srt;
      hut_us= _state.hut==0x00 ? 256000 : (16000*_state.hut);
      hlt_us= _state.hlt==0x00 ? 256000 : (2000*_state.hlt);
      break;
      // 300 Kbps (Approx)
    case 1:
      f= (26.7-1.67)/15;
      srt_us= (int) (1000*(26.7-_state.srt*f) + 0.5);
      f= (426-26.7)/15;
      hut_us=
        _state.hut==0x00 ?
        426000 :
        ((int) (1000*(26.7+f*(_state.hut-1)) + 0.5));
      f= (426-3.3)/127;
      hlt_us=
        _state.hlt==0x00 ?
        426000 :
        ((int) (1000*(3.3+f*(_state.hlt-1)) + 0.5));
      break;
      // 250 Kbps
    case 2:
      srt_us= 32000 - 2000*_state.srt;
      hut_us= _state.hut==0x00 ? 512000 : (32000*_state.hut);
      hlt_us= _state.hlt==0x00 ? 512000 : (4000*_state.hlt);
      break;
      // 1 Mbps
    case 3:
      srt_us= 8000 - 500*_state.srt;
      hut_us= _state.hut==0x00 ? 128000 : (8000*_state.hut);
      hlt_us= _state.hlt==0x00 ? 128000 : (1000*_state.hlt);
      break;
    default:
      srt_us= hlt_us= hut_us= 0; // CALLA !!!
      printf ( "[EE] fd.c - update_cc_variable - WTF !!! \n");
      exit ( EXIT_FAILURE );
    }

  // Passa a cicles
  _timing.cc__srt= (PC_ClockFreq*srt_us)/1000000;
  if ( _timing.cc__srt == 0 ) _timing.cc__srt= 1;
  _timing.cc__hut= (PC_ClockFreq*hut_us)/1000000;
  if ( _timing.cc__hut == 0 ) _timing.cc__hut= 1;
  _timing.cc__hlt= (PC_ClockFreq*hlt_us)/1000000;
  if ( _timing.cc__hlt == 0 ) _timing.cc__hlt= 1;

  // Cicles d'un byte
  switch ( _regs.dsr.drate )
    {
    case 0: _timing.cc__byte= (PC_ClockFreq*8)/500000; break; // 500K
    case 1: _timing.cc__byte= (PC_ClockFreq*8)/300000; break; // 300K
    case 2: _timing.cc__byte= (PC_ClockFreq*8)/250000; break; // 250K
    case 3: _timing.cc__byte= (PC_ClockFreq*8)/1000000; break; // 1M
    }
  
} // end upadte_cc_variable


// Torna els cicles que cal per transferir els bytes indicats
static int
bytes2cc (
          const int bytes
          )
{

  int us,ret;
  
  
  // Calcula microsegons
  switch ( _regs.dsr.drate )
    {
      // 500 Kbps
    case 0: us= 16*bytes; break;
      // 300 Kbps (Approx)
    case 1:
      us= (int) ((((long) 1000000)*((long) (8*bytes))) / ((long) 300000));
      break;
      // 250 Kbps
    case 2: us= 32*bytes; break;
      // 1 Mbps
    case 3: us= 8*bytes; break;
    default:
      us= 0; // CALLA !!!
      printf ( "[EE] fd.c - bytes2cc - WTF !!! \n");
      exit ( EXIT_FAILURE );
    }

  // Passa a cicles
  ret= (int) (((int64_t) PC_ClockFreq*(int64_t) us)/((int64_t) 1000000));

  // Li reste els cicles corresponent a 1.5us per algun motiu.
  ret-= (PC_ClockFreq*1500)/1000000000;
  if ( ret <= 0 ) ret= 1;

  return ret;
  
} // end bytes2cc


static void
init_regs (void)
{

  // DOR
  _regs.dor.motor_enabled[0]= false;
  _regs.dor.motor_enabled[1]= false;
  _regs.dor.motor_enabled[2]= false;
  _regs.dor.motor_enabled[3]= false;
  _regs.dor.irq_dma_enabled= false;
  _regs.dor.reset= true; // Inicialment a 1.
  _regs.dor.drive_sel= 0;

  // MSR
  _regs.msr.rqm= true;
  _regs.msr.dio= false; // Escriure
  _regs.msr.non_dma= false;
  _regs.msr.command_busy= false;
  _regs.msr.drv_busy[3]= false;
  _regs.msr.drv_busy[2]= false;
  _regs.msr.drv_busy[1]= false;
  _regs.msr.drv_busy[0]= false;

  // DSR
  // FALTEN COSES !!
  _regs.dsr.drate= 2; // 250Kbps
  
} // end init_regs


static void
init_state (void)
{

  int i;

  
  _state.drive_polling= true;
  _state.implied_seek= false;
  _state.fifo_thr= 1;
  _state.cmd_state= WAIT_CMD;
  for ( i= 0; i < 4; ++i )
    {
      _state.files[i].f= NULL;
      _state.files[i].current_C= 0;
      _state.files[i].head_ready= false;
    }
  for ( i= 0; i < 4; ++i )
    _state.op_state[i].op= OP_NONE;
  _state.int_state.state= INT_NONE;
  _state.srt= 0x00;
  _state.hut= 0x00;
  _state.hlt= 0x00;
  _state.use_dma= true;
  _state.cmd_args.N= 0;
  _state.dma_state.op= DMA_OP_NONE;
  
} // end init_state


static void
reset_state (void)
{

  int i;

  
  _state.drive_polling= true;
  _state.implied_seek= false;
  _state.fifo_thr= 1;
  _state.cmd_state= WAIT_CMD;
  for ( i= 0; i < 4; ++i )
    {
      _state.files[i].current_C= 0;
      _state.files[i].head_ready= false;
    }
  for ( i= 0; i < 4; ++i )
    _state.op_state[i].op= OP_NONE;
  _state.int_state.state= INT_NONE;
  _state.srt= 0x00;
  _state.hut= 0x00;
  _state.hlt= 0x00;
  _state.use_dma= true;
  _state.cmd_args.N= 0;
  _state.dma_state.op= DMA_OP_NONE;
  
} // end reset_state


static void
reset_soft (void)
{

  int i;

  
  // NOTES IMPORTANTS !!!!
  //
  // Note2: Emulators will often set the Disk Change flag to "true"
  // after a reset, but this does not happen on real hardware -- it is
  // a shared bug in all the emulators that do it. Another shared bug
  // is that most emulators do not fire an IRQ6 if disk polling mode
  // is off.
  
  // MSR
  _regs.msr.rqm= true;
  _regs.msr.dio= false; // Escriure
  _regs.msr.non_dma= false;
  _regs.msr.command_busy= false;
  _regs.msr.drv_busy[3]= false;
  _regs.msr.drv_busy[2]= false;
  _regs.msr.drv_busy[1]= false;
  _regs.msr.drv_busy[0]= false;

  // DOR
  _regs.dor.motor_enabled[0]= false;
  _regs.dor.motor_enabled[1]= false;
  _regs.dor.motor_enabled[2]= false;
  _regs.dor.motor_enabled[3]= false;

  // DSR
  // NOTA!! El reset no afecta al data rate
  //_regs.dsr.drate= 2; // 250Kbps
  
  // Estat intern
  // NOTA!! drive_polling no es pot modificar
  if ( _state.drive_polling )
    {
      _state.int_state.state= INT_POLLING4;
      _state.int_state.st0= 0xc0 | _regs.dor.drive_sel;
      _state.int_state.pcn= 0x00;
    }
  else _state.int_state.state= INT_NONE;
  _state.cmd_state= WAIT_CMD;
  for ( i= 0; i < 4; ++i )
    _state.op_state[i].op= OP_NONE;
  _state.srt= 0x00;
  _state.hut= 0x00;
  _state.hlt= 0x00;
  _state.use_dma= true;
  _state.cmd_args.N= 0;
  
  // FIFO
  // NOTA!!! Igual cal impedir que es reseteje en funció del LOCK
  _fifo.N= 0;
  _fifo.p= 0;
  
  // Coses timing
  // --> Para esperes
  _timing.cctoProcByte= 0;
  _timing.cctoReadResult= 0;
  for ( i= 0; i < 4; ++i )
    _timing.cctoOp[i]= 0;
  // --> Reajusta variables timing
  update_cc_variable ();

  // Reseteja senyal DMA
  PC_dma_dreq ( 2, false );
  
  // Force un escaló.
  if ( _regs.dor.irq_dma_enabled )
    {
      PC_ic_irq ( 6, true );
      PC_ic_irq ( 6, false );
    }
  
} // end reset_soft


static void
sense_interrupt (void)
{

  uint8_t st0,pcn;


  // Entra en mode commandament
  _regs.msr.command_busy= true;

  // Processa INT
  st0= _state.int_state.st0;
  pcn= _state.int_state.pcn;
  switch ( _state.int_state.state )
    {
    case INT_POLLING4: _state.int_state.state= INT_POLLING3; break;
    case INT_POLLING3: _state.int_state.state= INT_POLLING2; break;
    case INT_POLLING2: _state.int_state.state= INT_POLLING1; break;
    case INT_POLLING1: _state.int_state.state= INT_NONE; break;
    case INT_RECALIBRATE:
    case INT_SEEK:
      _state.int_state.state= INT_NONE;
      _regs.msr.drv_busy[_state.int_state.drv]= false;
      break;
    default:
      _warning ( _udata,
                 "fd - sense interrupt status: no hi havia cap interrupció.");
      st0= 0x80;
      pcn= 0x00;
      _state.int_state.state= INT_NONE;
      break;
    }
  
  // Reseteja fifo i ompli
  _fifo.p= 0; _fifo.N= 2;
  _fifo.v[0]= st0;
  _fifo.v[1]= pcn;
  
  // Passa a estat resultat
  _regs.msr.rqm= false; // <-- Esperem un temps per a permetre llegir
  _regs.msr.dio= true; // <-- Llegir
  _state.cmd_state= READ_RESULTS;
  _timing.cctoReadResult= _timing.cc__read_result;
  
} // end sense_interrupt


static void
recalibrate_wait_arg (void)
{
  
  _state.cmd_state= WAIT_RECALIBRATE_ARG;
  _regs.msr.command_busy= true;
  _regs.msr.rqm= true; // Permet que envien un altre
  _regs.msr.dio= false; // <-- Escriure
  
} // end recalibrate_wait_arg


static void
recalibrate_begin (
                   const uint8_t drv
                   )
{

  int desp;

  
  // Aquest comandament no té fase de resultat. El comandament es
  // llança en background i sols es pot acavar amb el sense interrupt.
  _state.cmd_state= WAIT_CMD;
  _regs.msr.command_busy= false;
  _regs.msr.rqm= true; // Permet que envien un altre
  _regs.msr.dio= false; // <-- Escriure

  // Prepara el comandament real.
  if ( _regs.msr.drv_busy[drv] || _state.op_state[drv].op != OP_NONE )
    _warning ( _udata,
               "FD: ignorant RECALIBRATE en %d perquè ja estava ocupat",
               drv );
  else
    {
      _regs.msr.drv_busy[drv]= true;
      _state.op_state[drv].op= OP_RECALIBRATE;
      _state.op_state[drv].N= 0;
      // Si està en 0 faig com si es moguera 1 (INVENT!!!)
      if ( _state.files[drv].current_C > 79 ) desp= 79;
      else if ( _state.files[drv].current_C == 0 ) desp= 1;
      else desp= _state.files[drv].current_C;
      _timing.cctoOp[drv]= desp*_timing.cc__srt;
    }
    
} // end recalibrate_begin


static void
recalibrate_op (
                const int drv
                )
{

  uint8_t pcn,st0;
  

  // Operació
  if ( _state.files[drv].current_C > 79 ) _state.files[drv].current_C-= 79;
  else _state.files[drv].current_C= 0;
  _state.files[drv].head_ready= false;
  _timing.cctoHUT[drv]= 0;
  
  // Llança interrupció.
  st0= 0x20 | ((uint8_t) drv);
  pcn= (uint8_t) _state.files[drv].current_C;
  if ( _regs.dor.irq_dma_enabled )
    {
      if ( _state.int_state.state != INT_NONE )
        _warning ( _udata,
                   "FD: No s'ha pogut llançar una interrupció de"
                   " recalibrate %d perquè no s'ha netejat una"
                   " interrupció anterior", drv );
      else
        {
          _state.int_state.state= INT_RECALIBRATE;
          _state.int_state.st0= st0;
          _state.int_state.pcn= pcn;
          _state.int_state.drv= drv;
          PC_ic_irq ( 6, true );
          PC_ic_irq ( 6, false );
        }
    }
  
} // end recalibrate_op


static void
read_id_wait_arg (void)
{
  
  _state.cmd_state= WAIT_READ_ID_ARG;
  _regs.msr.command_busy= true;
  _regs.msr.rqm= true; // Permet que envien un altre
  _regs.msr.dio= false; // <-- Escriure
  
} // end read_id_wait_arg


static void
read_id_begin (
               const uint8_t arg
               )
{

  int drv,H,i,tmp,num_sectors;
  uint8_t st0,st1,new_S;
  

  // Processa argument
  H= (arg>>2)&0x1;
  drv= arg&0x3;
  
  // Aquest comandament no té fase de resultat. El comandament es
  // llança en background però acava en 
  _state.cmd_state= EXEC_PHASE;
  _regs.msr.rqm= false; // No es deixen enviar més dades
  
  // Prepara el comandament real.
  if ( _regs.msr.drv_busy[drv] || _state.op_state[drv].op != OP_NONE )
    {
      _warning ( _udata,
                 "FD: ignorant READ ID en %d perquè ja estava ocupat",
                 drv );
      goto error;
    }
  else if ( !_regs.dor.motor_enabled[drv] ||
            _state.files[drv].f == NULL ||
            (H==1 && _state.files[drv].num_H==1) )
    goto error;
  else
    {

      // Calcula els sectors que ha de llegir fins trobar un sector de dades
      // --> Primer calcule els bytes mínim que deuria tindre el
      //     fitxer per tindre un sector en esta posició.
      tmp= _state.files[drv].current_C*
        _state.files[drv].num_H*_state.files[drv].num_S;
      if ( H == 1 ) tmp+= _state.files[drv].num_S;
      tmp*= 512;
      // --> Si és major o igual no hi ha cap sector per tant tindrà
      //     que pegar la volta.
      if ( tmp >= _state.files[drv].f->nbytes )
        {
          num_sectors= _state.files[drv].num_S;
          new_S= 0xFF; // Ha pegat la volta
        }
      // --> En cas contrari el primer sector del track
      else
        {
          num_sectors= 1;
          new_S= 0;
        }

      // Desactiva HUT si hi ha algun en procés.
      _timing.cctoHUT[drv]= 0;
      
      // Estimació (totalment INVENT!!!) del temps que tarda en llegir
      // el READ ID.
      //
      // La aprox que vaig a seguir és:
      //    HLT + 512*(NUM_SECTORS-1) + 4 + HUT - 1.5 us
      //
      _timing.cctoOp[drv]=
        (_state.files[drv].head_ready ? 0 : _timing.cc__hlt) +
        bytes2cc ( 512*(num_sectors-1) + 4 );
      
      // Acaba de prepara operació.
      // ARGUMENTS:
      //   -> arg original
      //   -> new_S precalculat (0xFF vol dir que ha pegat la volta)
      _regs.msr.drv_busy[drv]= true;
      _state.op_state[drv].op= OP_READ_ID;
      _state.op_state[drv].N= 2;
      _state.op_state[drv].args[0]= arg;
      _state.op_state[drv].args[1]= new_S;
      
    }

  return;

 error:
  // Reseteja fifo i ompli
  st0= 0x40 | ((uint8_t) drv) | (((uint8_t) H)<<2);
  st1= 0x05;
  _fifo.p= 0; _fifo.N= 7;
  _fifo.v[0]= st0;
  _fifo.v[1]= st1;
  for ( i= 2; i < 7; ++i )
    _fifo.v[i]= 0x00;
  
  // Passa a estat resultat
  _regs.msr.rqm= false; // <-- Esperem un temps per a permetre llegir
  _regs.msr.dio= true; // <-- Llegir
  _state.cmd_state= READ_RESULTS;
  _timing.cctoReadResult= _timing.cc__read_result;
  
  // Genera interrupció.
  if ( _regs.dor.irq_dma_enabled )
    {
      PC_ic_irq ( 6, true );
      PC_ic_irq ( 6, false );
    }
  
} // end read_id_begin


static void
read_id_op (
            const int drv
            )
{

  int sel_H,nbytes;
  uint8_t new_S,st0,st1,st2,C,H,R,N;
  
  
  // Agarra arguments
  sel_H= (_state.op_state[drv].args[0]>>2)&0x1;
  assert ( drv == (_state.op_state[drv].args[0]&0x3) );
  new_S= _state.op_state[drv].args[1];

  // Torne a fer comprovacions per si mentre estava fent-se la
  // operació s'ha modificat l'estat del diskette. En eixe cas torne
  // error.
  if ( _state.files[drv].f == NULL )
    {
      st0= 0x40 | ((uint8_t) drv) | ((uint8_t) (sel_H<<2));
      st1= 0x05;
      st2= 0x00;
      C= H= R= N= 0;
    }
  else
    {
      // Situació en que no hi ha sectors en este cilindre. Recomprove
      // altra vegada!!!
      nbytes= _state.files[drv].current_C*
        _state.files[drv].num_H*_state.files[drv].num_S;
      if ( sel_H == 1 )
        nbytes+= _state.files[drv].num_S;
      if ( new_S != 0xFF ) nbytes+= new_S;
      nbytes*= 512;
      if ( new_S == 0xFF || nbytes >= _state.files[drv].f->nbytes )
        {
          st0= 0x40 | ((uint8_t) drv) | ((uint8_t) (sel_H<<2));
          st1= 0x85;
          st2= 0x00;
          C= H= R= N= 0;
        }
      // Ho donem per bo.
      else
        {
          st0= 0x00 | ((uint8_t) drv) | ((uint8_t) (sel_H<<2));
          st1= 0x00;
          st2= 0x00;
          C= _state.files[drv].current_C;
          H= sel_H;
          R= new_S+1;
          N= 0x02; // 512 bytes
        }
    }
  
  // Reseteja fifo i ompli
  _fifo.p= 0; _fifo.N= 7;
  _fifo.v[0]= st0;
  _fifo.v[1]= st1;
  _fifo.v[2]= st2;
  _fifo.v[3]= C;
  _fifo.v[4]= H;
  _fifo.v[5]= R;
  _fifo.v[6]= N;

  // Genera interrupció.
  if ( _regs.dor.irq_dma_enabled )
    {
      PC_ic_irq ( 6, true );
      PC_ic_irq ( 6, false );
    }
  
  // Passa a estat resultat
  _regs.msr.drv_busy[drv]= false;
  _regs.msr.rqm= false; // <-- Esperem un temps per a permetre llegir
  _regs.msr.dio= true; // <-- Llegir
  _state.cmd_state= READ_RESULTS;
  _timing.cctoReadResult= _timing.cc__read_result;

  // Fes un HUT
  _state.files[drv].head_ready= true;
  _timing.cctoHUT[drv]= _timing.cc__hut;
  
} // end read_id_op


static void
specify_wait_arg1 (void)
{
  
  _state.cmd_state= WAIT_SPECIFY_ARG1;
  _regs.msr.command_busy= true;
  _regs.msr.rqm= true; // Permet que envien un altre
  _regs.msr.dio= false; // <-- Escriure
  
} // end specify_wait_arg1


static void
specify_first_arg (
                   const uint8_t data
                   )
{

  // Processa primer argument
  _state.srt= data>>4;
  _state.hut= data&0xF;
  update_cc_variable ();
  
  // Prepara següent byte
  _state.cmd_state= WAIT_SPECIFY_ARG2;
  _regs.msr.command_busy= true;
  _regs.msr.rqm= true; // Permet que envien un altre
  _regs.msr.dio= false; // <-- Escriure
  
} // end specify_first_arg


static void
specify_second_arg (
                   const uint8_t data
                   )
{

  // Processa segon argument
  _state.hlt= data>>1;
  _state.use_dma= ((data&0x01)==0x00);
  update_cc_variable ();
  
  // Acaba sense tornar resultats
  _fifo.N= 0;
  _state.cmd_state= WAIT_CMD;
  _regs.msr.command_busy= false;
  _regs.msr.rqm= true; // Permet que envien un altre
  _regs.msr.dio= false; // <-- Escriure
  
} // end specify_second_arg


static void
read_data_wait_arg1 (
                     const uint8_t cmd
                     )
{
  
  _state.cmd_state= WAIT_READ_DATA_ARGS;
  _state.cmd_args.v[0]= cmd;
  _state.cmd_args.N= 1;
  _regs.msr.command_busy= true;
  _regs.msr.rqm= true; // Permet que envien un altre
  _regs.msr.dio= false; // <-- Escriure
  
} // end read_data_wait_arg1


static void
read_data_wait_args (
                     const uint8_t arg
                     )
{

  _state.cmd_args.v[_state.cmd_args.N++]= arg;
  _state.cmd_state=
    _state.cmd_args.N==8 ? WAIT_READ_DATA_ARG8 : WAIT_READ_DATA_ARGS;
  _regs.msr.command_busy= true;
  _regs.msr.rqm= true; // Permet que envien un altre
  _regs.msr.dio= false; // <-- Escriure
  
} // end read_data_wait_args


static void
read_data_begin (
                 const uint8_t arg8
                 )
{

  bool MT;
  uint8_t HDS,DS,C,H,R,N,EOT,st0,st1;
  long offset,track_offset;
  int cc_to_op;
  
  
  // Obté arguments
  MT= (_state.cmd_args.v[0]&0x80)!=0;
  HDS= (_state.cmd_args.v[1]>>2)&0x1;
  DS= _state.cmd_args.v[1]&0x3;
  C= _state.cmd_args.v[2];
  H= _state.cmd_args.v[3];
  R= _state.cmd_args.v[4];
  N= _state.cmd_args.v[5];
  EOT= _state.cmd_args.v[6];
  //GPL= _state.cmd_args.v[7]; <-- Irrellevant
  //DTL= arg8;   <-- Irrellevant

  // Entra en mode execució
  _state.cmd_state= EXEC_PHASE;
  _regs.msr.rqm= false; // No es deixen enviar més dades

  // Comprovacions inicials
  if ( HDS != H || DS != _regs.dor.drive_sel ) { st1= 0x00; goto error; }
  if ( _state.files[DS].f == NULL ||
       C >= _state.files[DS].num_C ||
       H >= _state.files[DS].num_H ||
       R == 0 ||
       (R-1) >= _state.files[DS].num_S ||
       !_regs.dor.motor_enabled[DS] ||
       _regs.msr.drv_busy[DS] )
    { st1= 0x4; goto error; }
  // NOTA!!! Esta comprovació es questionable. Què caldria fer amb els
  // sectors "buits"? Llegir-los co 0s o dir que no estan marcats??
  offset=
    ((long) C)*((long) _state.files[DS].num_H)*
    ((long) _state.files[DS].num_S);
  if ( H == 1 ) offset+= (long) _state.files[DS].num_S;
  track_offset= offset;
  offset+= (long) (R-1);
  offset*= (long) 512;
  track_offset*= (long) 512;
  if ( offset >= _state.files[DS].f->nbytes ) { st1= 0x01; goto error; }

  // Actualitza HUT
  cc_to_op= 0;
  _timing.cctoHUT[DS]= 0;
  if ( !_state.files[DS].head_ready ) cc_to_op+= _timing.cc__hlt;

  // Seek si cal
  if ( C != _state.files[DS].current_C )
    {
      if ( _state.implied_seek )
        {
          // NOTA!! En el st0 cal indicar que s'ha fet un seek implícit
          PC_MSG("READ DATA IMPLIED SEEK!!! FALTA TAMBÉ EL BIT EN EL ST0");
          exit ( EXIT_FAILURE );
          // cc_to_op+= ?????????
        }
      else { st1= 0x00; goto error; }
    }

  // CONPROVACIÓ PROVISIONAL
  if ( N != 0x02 )
    {
      PC_MSG("READ_DATA SOLS IMPLEMENTA SECTORS DE 512!!");
      exit(EXIT_FAILURE);
    }
  
  // Acaba de preparar operació.
  _regs.msr.drv_busy[DS]= true;
  _fifo.N= 0; _fifo.p= 0; // Prepara FIFO
  if ( _state.use_dma )
    {
      _state.op_state[DS].op= OP_READ_DATA_DMA;
      _timing.cctoOp[DS]= cc_to_op + _timing.cc__byte;
      _state.dma_state.drv= DS;
      _state.dma_state.N= 0;
      _state.dma_state.p= 0;
      _state.dma_state.current_sec= R-1;
      if ( MT && H == 0 ) // Faig com si els dos capçals foren un únic
                          // track
        _state.dma_state.end_sec=
          _state.files[DS].num_S*_state.files[DS].num_H;
      // NOTA!! Pareix que EOT sempre fa referència a un sector dins
      // d'un H, és a dir, quan MT==1 && H==1 el EOT no és absolut a
      // tot. Donat que en track_offset sí que tinc en compte el
      // H. MT==1 && H==1 es comporta com MT==0
      else
        _state.dma_state.end_sec= _state.files[DS].num_S;
      // Com em guarde l'últim +1 contant des de 0.
      if ( EOT < _state.dma_state.end_sec )
        _state.dma_state.end_sec= EOT;
      _state.dma_state.track_offset= track_offset;
      _state.dma_state.C= C;
      _state.dma_state.H= H;
      _state.dma_state.S= R-1;
      _state.dma_state.op= DMA_OP_READ_DATA;
    }
  else
    {
      PC_MSG("READ DATA SENSE DMA!!!!");
      exit ( EXIT_FAILURE );
    }
  
  return;

  // NOTA!! Cal ficar abans el valor de st1
 error:
  // Reseteja fifo i ompli
  st0= 0x40 | ((uint8_t) DS) | (((uint8_t) H)<<2);
  _fifo.p= 0; _fifo.N= 7;
  _fifo.v[0]= st0;
  _fifo.v[1]= st1;
  _fifo.v[2]= 0x00;
  _fifo.v[3]= _state.files[DS].current_C; // Si falla que es veja el
                                          // cilindre verdader
  _fifo.v[4]= H;
  _fifo.v[5]= R;
  _fifo.v[6]= N;
  
  // Passa a estat resultat
  _regs.msr.rqm= false; // <-- Esperem un temps per a permetre llegir
  _regs.msr.dio= true; // <-- Llegir
  _state.cmd_state= READ_RESULTS;
  _timing.cctoReadResult= _timing.cc__read_result;

  // Genera interrupció.
  if ( _regs.dor.irq_dma_enabled )
    {
      PC_ic_irq ( 6, true );
      PC_ic_irq ( 6, false );
    }
  
} // end read_data_begin


// De st0 no cal ficar el DRV i H
static void
read_data_dma_result (
                      uint8_t       st0,
                      const uint8_t st1
                      )
{

  // Reseteja fifo i ompli
  st0|= ((uint8_t) _state.dma_state.drv) | (((uint8_t) _state.dma_state.H)<<2);
  _fifo.p= 0; _fifo.N= 7;
  _fifo.v[0]= st0;
  _fifo.v[1]= st1;
  _fifo.v[2]= 0x00;
  _fifo.v[3]= _state.dma_state.C;
  _fifo.v[4]= _state.dma_state.H;
  _fifo.v[5]= _state.dma_state.S+1;
  _fifo.v[6]= 0x02; // Hardcodejat a 512!!!
  
  // Passa a estat resultat
  _regs.msr.drv_busy[_state.dma_state.drv]= false;
  _regs.msr.rqm= false; // <-- Esperem un temps per a permetre llegir
  _regs.msr.dio= true; // <-- Llegir
  _state.cmd_state= READ_RESULTS;
  _timing.cctoReadResult= _timing.cc__read_result;
  
  // Genera interrupció.
  if ( _regs.dor.irq_dma_enabled )
    {
      PC_ic_irq ( 6, true );
      PC_ic_irq ( 6, false );
    }
  
  // Deshabilita DREQ per si de cas
  PC_dma_dreq ( 2, false );
  
  // Llança HUT
  _timing.cctoHUT[_state.dma_state.drv]= _timing.cc__hut;
  
  // Lleva DMA
  _state.dma_state.op= DMA_OP_NONE;

  // Para les operacions (pot ser cridat des de fora de la operació)
  _state.op_state[_state.dma_state.drv].op= OP_NONE;
  _timing.cctoOp[_state.dma_state.drv]= 0;
  
} // end read_data_dma_result


static void
read_data_dma_op (
                  const int drv
                  )
{

  uint8_t st1,data;
  long offset;
  int ret;
  
  
  // Llig següent sector si no queden bytes
  if ( _state.dma_state.p == _state.dma_state.N ) // Pot ser buit o ple
    {

      // Comprova EOT
      // Açò es tracta com un error però en realitat és una manera
      // correcta d'acavar.
      if ( _state.dma_state.current_sec == _state.dma_state.end_sec )
        { st1= 0x80; goto error; }
      
      // Comprovacions i càlcul offset
      if ( _state.files[drv].f == NULL ) { st1= 0x00; goto error; }
      offset= _state.dma_state.track_offset +
        ((long)512) * ((long) _state.dma_state.current_sec);
      if ( offset >= _state.files[drv].f->nbytes ) { st1= 0x01; goto error; }

      // Intenta llegir
      ret= PC_file_seek ( _state.files[drv].f, offset );
      if ( ret != 0 ) { st1= 0x04; goto error; }
      ret= PC_file_read ( _state.files[drv].f,
                          _state.dma_state.buf,
                          SECTOR_SIZE );
      if ( ret != 0 ) { st1= 0x04; goto error; }
      _state.dma_state.N= 512;
      _state.dma_state.p= 0;

      // Incrementa comptadors
      ++_state.dma_state.current_sec;
      // --> Els >= és per si de cas ens han modificat l'estructura
      if ( ++_state.dma_state.S >= _state.files[drv].num_S )
        {
          _state.dma_state.S= 0;
          if ( ++_state.dma_state.H >= _state.files[drv].num_H )
            {
              _state.dma_state.H= 0;
              ++_state.dma_state.C; // ??????????? No deuria passar a
                                    // no ser que ens modifiquen el
                                    // fitxer a meitat
            }
        }
    }

  // Clava en el FIFO
  data= _state.dma_state.buf[_state.dma_state.p++];
  if ( _fifo.N == FIFO_SIZE ) { st1= 0x10; goto error; }
  _fifo.v[(_fifo.p+_fifo.N)%FIFO_SIZE]= data;
  ++_fifo.N;
  
  // Demana DREQ
  if ( _fifo.N == 16-_state.fifo_thr ||
       _state.dma_state.p == _state.dma_state.N )
    PC_dma_dreq ( 2, true );

  // Repeteix operació
  _state.op_state[drv].op= OP_READ_DATA_DMA;
  _timing.cctoOp[drv]= _timing.cc__byte;
  
  return;
  
  // NOTA!! Cal ficar abans el valor de st1
 error:
  read_data_dma_result ( 0x40, st1 );
  
} // end read_data_dma_op


static void
seek_wait_arg1 (void)
{
  
  _state.cmd_state= WAIT_SEEK_ARG1;
  _regs.msr.command_busy= true;
  _regs.msr.rqm= true; // Permet que envien un altre
  _regs.msr.dio= false; // <-- Escriure
  _state.cmd_args.N= 0;
  
} // end seek_wait_arg1


static void
seek_wait_arg2 (
                const uint8_t arg1
                )
{

  _state.cmd_state= WAIT_SEEK_ARG2;
  _regs.msr.rqm= true; // Permet que envien un altre
  _regs.msr.dio= false; // <-- Escriure
  _state.cmd_args.v[0]= arg1;
  _state.cmd_args.N= 1;
  
} // end seek_wait_arg2


static void
seek_begin (
            const uint8_t arg2
            )
{

  int drv,desp;
  uint8_t new_C;


  // Arguments
  drv= _state.cmd_args.v[0]&0x3;
  new_C= arg2;
  
  // Aquest comandament no té fase de resultat. El comandament es
  // llança en background i sols es pot acavar amb el sense interrupt.
  _state.cmd_state= WAIT_CMD;
  _regs.msr.command_busy= false;
  _regs.msr.rqm= true; // Permet que envien un altre
  _regs.msr.dio= false; // <-- Escriure
  
  // Prepara el comandament real.
  if ( _regs.msr.drv_busy[drv] || _state.op_state[drv].op != OP_NONE )
    _warning ( _udata,
               "FD: ignorant SEEK en %d perquè ja estava ocupat",
               drv );
  else
    {
      _regs.msr.drv_busy[drv]= true;
      _state.op_state[drv].op= OP_SEEK;
      _state.op_state[drv].N= 1;
      _state.op_state[drv].args[0]= new_C;
      // Si està en 0 faig com si es moguera 1 (INVENT!!!)
      desp= new_C - _state.files[drv].current_C;
      if ( desp < 0 ) desp= -desp;
      // Si està ja fique uns pocs cicles aleatoris
      _timing.cctoOp[drv]= (desp == 0) ? 10 : (desp*_timing.cc__srt);
    }
    
} // end seek_begin


static void
seek_op (
         const int drv
         )
{

  uint8_t pcn,st0;
  

  // NOTA!!! He decidit no comprovar si hi ha o no disket. Vaig a fer
  // com si fora un recalibrate. És a dir, que es pot moure encara que
  // no hi hasca disquet.

  
  // Operació
  _state.files[drv].current_C= _state.op_state[drv].args[0];
  _state.files[drv].head_ready= false;
  _timing.cctoHUT[drv]= 0;
  
  // Llança interrupció.
  st0= 0x20 | ((uint8_t) drv);
  pcn= (uint8_t) _state.files[drv].current_C;
  if ( _regs.dor.irq_dma_enabled )
    {
      if ( _state.int_state.state != INT_NONE )
        _warning ( _udata,
                   "FD: No s'ha pogut llançar una interrupció de"
                   " seek %d perquè no s'ha netejat una"
                   " interrupció anterior", drv );
      else
        {
          _state.int_state.state= INT_SEEK;
          _state.int_state.st0= st0;
          _state.int_state.pcn= pcn;
          _state.int_state.drv= drv;
          PC_ic_irq ( 6, true );
          PC_ic_irq ( 6, false );
        }
    }
  
} // end seek_op


static void
process_byte (void)
{

  uint8_t data;
  
  
  // Comprova que hi han dades en la FIFO
  if ( _fifo.N == 0 )
    {
      _warning ( _udata,
                 "No es poden processar de la FD.FIFO perquè està buida" );
      return;
    }

  // Extrau dades
  data= _fifo.v[_fifo.p];
  _fifo.p= (_fifo.p+1)%FIFO_SIZE;
  --_fifo.N;

  // Processa
  switch ( _state.cmd_state )
    {

      // Esperant comandament
    case WAIT_CMD:
      switch ( data )
        {

          // SPECIFY
        case 0x03: specify_wait_arg1 (); break;
            
          // RECALIBRATE
        case 0x07: recalibrate_wait_arg (); break;
          // SENSE INTERRUPT STATUS
        case 0x08: sense_interrupt (); break;

          // SEEK
        case 0x0f: seek_wait_arg1 (); break;
          
          // READ ID
        case 0x0a:
        case 0x4a: read_id_wait_arg (); break;

          // READ DATA
        case 0x06:
        case 0x26:
        case 0x46:
        case 0x56:
        case 0x86:
        case 0xa6:
        case 0xc6:
        case 0xe6: read_data_wait_arg1 ( data ); break;
          
        default:
          printf ( "[EE] fd.c - unknown command %02X\n", data );
          exit ( EXIT_FAILURE );
        }
      break;

      // Esperant argument recalibrate
    case WAIT_RECALIBRATE_ARG: recalibrate_begin ( data ); break;
      // Esperant argument read id
    case WAIT_READ_ID_ARG: read_id_begin ( data ); break;
      // Esperant primer argument specify
    case WAIT_SPECIFY_ARG1: specify_first_arg ( data ); break;
      // Esperant segon argument specify
    case WAIT_SPECIFY_ARG2: specify_second_arg ( data ); break;
      // Esperant arguments read data
    case WAIT_READ_DATA_ARGS: read_data_wait_args ( data ); break;
      // Esperant últim argument read data
    case WAIT_READ_DATA_ARG8: read_data_begin ( data ); break;
      // Esperant primer argument seek
    case WAIT_SEEK_ARG1: seek_wait_arg2 ( data ); break;
      // Esperant segon argument seek
    case WAIT_SEEK_ARG2: seek_begin ( data ); break;
      
    default:
      printf ( "[EE] fd.c - _state.cmd_state %d no suportat\n",
               _state.cmd_state );
      exit ( EXIT_FAILURE );
    }
  
} // end process_byte


static void
run_op (
        const int drv
        )
{

  int op;


  op= _state.op_state[drv].op;
  _state.op_state[drv].op= OP_NONE;
  switch ( op )
    {
    case OP_RECALIBRATE: recalibrate_op ( drv ); break;
    case OP_READ_ID: read_id_op ( drv ); break;
    case OP_READ_DATA_DMA: read_data_dma_op ( drv ); break;
    case OP_SEEK: seek_op ( drv ); break;
    default:
      printf ( "[EE] fd.c - run_op - WTF!!!!\n" );
      exit ( EXIT_FAILURE );
    }
  
} // end run_op


static void
clock (
       const bool update_cc2event
       )
{

  int cc,clocks,i;


  _in_clock= true;
  
  // Processa cicles
  cc= PC_Clock-_timing.cc_used;
  if ( cc > 0 ) { _timing.cc+= cc; _timing.cc_used+= cc; }

  // Processa clocks
  clocks= _timing.cc;
  _timing.cc= 0;

  // HUT (0 vol dir que no estem espreant un HUT)
  for ( i= 0; i < 4; ++i )
    if ( _timing.cctoHUT[i] > 0 )
      {
        _timing.cctoHUT[i]-= clocks;
        if ( _timing.cctoHUT[i] <= 0 )
          {
            _timing.cctoHUT[i]= 0; // NOTA!! Important fer-ho abans
            _state.files[i].head_ready= false;
          }
      }
  
  // Reset (0 vol dir que no estem esperant un reset)
  if ( _timing.cctoReset > 0 )
    {
      _timing.cctoReset-= clocks;
      if ( _timing.cctoReset <= 0 )
        {
          _timing.cctoReset= 0; // NOTA!! Important fer-ho abans
          reset_soft ();
        }
    }

  // Processa bytes pendents fifo (0 vol dir que no estem esperant a
  // processar)
  if ( _timing.cctoProcByte > 0 )
    {
      _timing.cctoProcByte-= clocks;
      if ( _timing.cctoProcByte <= 0 )
        {
          _timing.cctoProcByte= 0; // NOTA!! Important fer-ho abans
          process_byte ();
        }
    }
  
  // Esperant a llegir següent byte FIFO (0 vol dir que no estem
  // esperant a llegir)
  if ( _timing.cctoReadResult > 0 )
    {
      _timing.cctoReadResult-= clocks;
      if ( _timing.cctoReadResult <= 0 )
        {
          _timing.cctoReadResult= 0; // NOTA!! Important fer-ho abans
          _regs.msr.rqm= true; // Permet llegir
        }
    }

  // Esperant a què s'execute una operació
  for ( i= 0; i < 4; ++i )
    if ( _timing.cctoOp[i] > 0 )
      {
        _timing.cctoOp[i]-= clocks;
        if ( _timing.cctoOp[i] <= 0 )
          {
            _timing.cctoOp[i]= 0; // NOTA!! Important fer-ho abans
            run_op ( i );
          }
      }
  
  // Actualitza cctoEvent
  if ( update_cc2event )
    update_cc_to_event ();

  _in_clock= false;
  
} // end clock


static void
fifo_write (
            const uint8_t data
            )
{
  
  // Si no està permès no faces res
  if ( _state.cmd_state == RESET_STATE )
    _warning ( _udata, "FD.FIFO= %02X. S'ha intentat escriure un"
               " byte en la FIFO del floppy durant el RESET_STATE",
               data );
  else if ( !_regs.msr.rqm || _regs.msr.dio )
    _warning ( _udata, "FD.FIFO= %02X. S'ha intentat escriure un"
               " byte en la FIFO del floppy, però no està llest per"
               " a rebre dades", data );
  // Inserta en el FIFO
  else
    {
      
      // Descarta bytes si és necessari.
      if ( _fifo.N == FIFO_SIZE )
        {
          _warning ( _udata,
                     "FD.FIFO= %02X. FIFO PLENA!! S'ha descartat el byte %02X",
                     data, _fifo.v[_fifo.p] );
          _fifo.p= (_fifo.p+1)%FIFO_SIZE;
          --_fifo.N;
        }
      
      // Inserta en la FIFO.
      _fifo.v[(_fifo.p+_fifo.N)%FIFO_SIZE]= data;
      ++_fifo.N;
      _regs.msr.rqm= false; // <-- Fins que no es processe no es permeten més
      // NOTA!! No tinc clar gens el temps que tarda en escriure i
      // processar un byte de la FIFO, però basant-me en el manual del
      // controlador vaig a assumir que és: 5+90+60 = 155 ns
      _timing.cctoProcByte= _timing.cc__proc_byte;
      
    }
  
} // end fifo_write


static uint8_t
fifo_read (void)
{

  uint8_t ret;
  
  
  ret= 0xff; // Valor per defecte.
  
  // Si no està permès no faces res
  if ( _state.cmd_state == RESET_STATE )
    _warning ( _udata, "Read FD.FIFO. S'ha intentat llegir un"
               " byte de la FIFO del floppy durant el RESET_STATE" );
  else if ( !_regs.msr.rqm || !_regs.msr.dio )
    _warning ( _udata, "Read FD.FIFO. S'ha intentat llegir un"
               " byte de la FIFO del floppy, però no està llesta per"
               " llegir dades" );
  else if ( _fifo.N == 0 )
    _warning ( _udata, "Read FD.FIFO. S'ha intentat llegir un"
               " byte de la FIFO del floppy, però no hi han dades que llegir" );
  // Inserta en el FIFO
  else
    {

      // NOTA!! Vaig a distinguir la fase de resultats de les
      // d'execució.
      if ( _state.cmd_state == READ_RESULTS )
        {
          
          // Llig.
          ret= _fifo.v[_fifo.p];
          _fifo.p= (_fifo.p+1)%FIFO_SIZE;
          --_fifo.N;

          // Si és l'últim passa a fase d'execució
          if ( _fifo.N == 0 )
            {
              _state.cmd_state= WAIT_CMD;
              _regs.msr.rqm= true;
              _regs.msr.dio= false; // <-- Escriure
              _regs.msr.command_busy= false;
            }
          // En cas contrari força una altra espera
          // NOTA!!! Aquest comportament té sentit si l'usuari està
          // llegint continuament els resultats de la FIFO.
          else
            {
              _regs.msr.rqm= false;
              _timing.cctoReadResult= _timing.cc__read_result;
            }
          
        }

      // Altres fases
      else
        {
          printf ( "[EE] fd.c - CAL IMPLEMENTAR CAS FASE NORMAL"
                   " PC_fifo_fd_read" );
          exit ( EXIT_FAILURE );
        }
      
    }
  
  return ret;
  
} // end fifo_read


static uint8_t
dma_read (void)
{
  
  uint8_t ret;
  
  
  if ( _state.dma_state.op == DMA_OP_READ_DATA )
    {
      if ( _fifo.N == 0 )
        {
          _warning ( _udata,
                     "FD: s'ha intentat llegir un byte de la FIFO"
                     " en mode DMA però no hi havien bytes disponibles." );
          read_data_dma_result ( 0x40, 0x10 );
          ret= 0xff;
        }
      else
        {
          ret= _fifo.v[_fifo.p];
          _fifo.p= (_fifo.p+1)%FIFO_SIZE;
          if ( --_fifo.N == 0 ) PC_dma_dreq ( 2, false );
        }
    }
  else
    {
      _warning ( _udata,
                 "FD: s'ha intentat llegir un byte de la FIFO en mode"
                 " DMA però no està en mode DMA" );
      ret= 0xff;
    }
  
  return ret;
  
} // end dma_read


static void
fifo_write_trace (
                  const uint8_t data
                  )
{

  _fifo_access ( _regs.dor.drive_sel, data, false,
                 _state.cmd_state==EXEC_PHASE, false, _udata );
  fifo_write ( data );
  
} // end fifo_write_trace


static uint8_t
fifo_read_trace (void)
{

  uint8_t ret;
  bool is_exec_phase;
  

  is_exec_phase= (_state.cmd_state==EXEC_PHASE);
  ret= fifo_read ();
  _fifo_access ( _regs.dor.drive_sel, ret, true, is_exec_phase, false, _udata );

  return ret;
  
} // end fifo_read_trace


static uint8_t
dma_read_trace (void)
{

  uint8_t ret;
  int drv;
  

  drv= _state.dma_state.drv;
  ret= dma_read ();
  _fifo_access ( drv, ret, true, true, true, _udata );
  
  return ret;
  
} // end dma_read_trace




/**********************/
/* FUNCIONS PÚBLIQUES */
/**********************/

void
PC_fd_init (
            PC_Warning          *warning,
            PC_FloppyFIFOAccess *fifo_access,
            void                *udata,
            const PC_Config     *config
            )
{

  int i;

  
  // Callbacks
  _warning= warning;
  _fifo_access= fifo_access;
  _udata= udata;
  _config= config;
  
  // Inicialitzacions diverses
  init_regs ();
  init_state ();
  _fifo.N= 0;
  _fifo.p= 0;
  
  // Timing
  _timing.cc= 0;
  _timing.cc_used= 0;
  _timing.cctoReset= 0;
  for ( i= 0; i < 4; ++i )
    _timing.cctoHUT[i]= 0;
  _timing.cctoProcByte= 0;
  _timing.cctoReadResult= 0;
  for ( i= 0; i < 4; ++i )
    _timing.cctoOp[i]= 0;
  // NOTA!! No tinc clar gens el temps que tarda en escriure i
  // processar un byte de la FIFO, però basant-me en el manual del
  // controlador vaig a assumir que és: 5+90+60 = 155 ns
  // Gastaré el mateix temps per llegir els resultats.
  _timing.cc__proc_byte= (155*PC_ClockFreq)/1000000000;
  if ( _timing.cc__proc_byte == 0 ) _timing.cc__proc_byte= 1;
  // Per a read result ficaré el mateix timing
  _timing.cc__read_result= (155*PC_ClockFreq)/1000000000;
  if ( _timing.cc__read_result == 0 ) _timing.cc__read_result= 1;
  update_cc_variable ();
  update_cc_to_event ();

  // Altres (Importants) Oo
  _fifo_write= fifo_write;
  _fifo_read= fifo_read;
  _dma_read= dma_read;
  _in_clock= false;
  
} // end PC_fd_init


void
PC_fd_reset (void)
{

  int i;

  
  clock ( false );

  // Inicialitzacions diverses
  init_regs ();
  reset_state ();
  _fifo.N= 0;
  _fifo.p= 0;
  
  // Timing
  _timing.cctoReset= 0;
  for ( i= 0; i < 4; ++i )
    _timing.cctoHUT[i]= 0;
  _timing.cctoProcByte= 0;
  _timing.cctoReadResult= 0;
  for ( i= 0; i < 4; ++i )
    _timing.cctoOp[i]= 0;
  update_cc_variable ();
  
  update_cc_to_event ();
  
} // end PC_fd_reset


void
PC_fd_set_mode_trace (
                      const bool val
                      )
{

  if ( val && _fifo_access )
    {
      _fifo_read= fifo_read_trace;
      _fifo_write= fifo_write_trace;
      _dma_read= dma_read_trace;
    }
  else
    {
      _fifo_read= fifo_read;
      _fifo_write= fifo_write;
      _dma_read= dma_read;
    }
  
} // end PC_fd_set_mode_trace


int
PC_fd_next_event_cc (void)
{

  int tmp;


  tmp= _timing.cctoEvent - _timing.cc;
  assert ( tmp > 0 );
  
  return tmp;
  
} // end PC_fd_next_event_cc


void
PC_fd_end_iter (void)
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
  
} // end PC_fd_end_iter


void
PC_fd_dor_write (
                 const uint8_t data
                 )
{

  bool new_reset;

  
  clock ( false );

  // Motors
  _regs.dor.motor_enabled[3]= ((data&0x80)!=0);
  _regs.dor.motor_enabled[2]= ((data&0x40)!=0);
  _regs.dor.motor_enabled[1]= ((data&0x20)!=0);
  _regs.dor.motor_enabled[0]= ((data&0x10)!=0);
  // IRQ/DMA
  _regs.dor.irq_dma_enabled= ((data&0x08)!=0);
  // Reset
  new_reset= ((data&0x04)!=0);
  // NOTA!!! Al passar de 0 -> 1 cal esperar 4 microsegons (en
  // realitat menys depenent del data rate) abans de manualment tornar
  // a passar de 1 -> 0, açò últim és el que provoca el RESET, que pot
  // aplegar a tardar fins . No vaig a emular la primera espera però
  // si el que tarda en fer la segona, perquè pot produir una excepció
  // IRQ.
  if ( !new_reset && _regs.dor.reset ) _state.cmd_state= RESET_STATE;
  // NOTA!! Interprete que esta fase és la que apareix en el manual
  // amb t31 (2micro segons)
  else if ( new_reset && !_regs.dor.reset) // Ix del reset estat.
    _timing.cctoReset= (2*PC_ClockFreq)/1000000;
  _regs.dor.reset= new_reset;
  // Drive select
  _regs.dor.drive_sel= data&0x3;
  
  update_cc_to_event ();
  
} // end PC_fd_dor_write


uint8_t
PC_fd_msr_read (void)
{

  uint8_t ret;

  
  clock ( true );
  ret=
    (_regs.msr.rqm ? 0x80 : 0x00) |
    (_regs.msr.dio ? 0x40 : 0x00) |
    (_regs.msr.non_dma ? 0x20 : 0x00) |
    (_regs.msr.command_busy ? 0x10 : 0x00) |
    (_regs.msr.drv_busy[3] ? 0x08 : 0x00) |
    (_regs.msr.drv_busy[2] ? 0x04 : 0x00) |
    (_regs.msr.drv_busy[1] ? 0x02 : 0x00) |
    (_regs.msr.drv_busy[0] ? 0x01 : 0x00)
    ;

  return ret;
  
} // end PC_fd_msr_read


void
PC_fd_fifo_write (
                  const uint8_t data
                  )
{

  clock ( false );
  _fifo_write ( data );
  update_cc_to_event ();
  
} // end PC_fd_fifo_write


uint8_t
PC_fd_fifo_read (void)
{

  uint8_t ret;

  
  clock ( false );
  ret= _fifo_read ();
  update_cc_to_event ();
  
  return ret;
  
} // end PC_fd_fifo_read


void
PC_fd_ccr_write (
                 const uint8_t data
                 )
{

  clock ( false );

  // NOTA!! Este registre sols gasta els 2 bits inferiors i en
  // realitat són un alias del dsr.
  _regs.dsr.drate= data&0x3;
  update_cc_variable ();
  
  update_cc_to_event ();
  
} // end PC_fd_ccr_write


PC_Error
PC_fd_insert_floppy (
                     PC_File   *file,
                     const int  drv
                     )
{
  
  PC_Error ret;
  
  
  clock ( false );

  // NOTA!!! Vaig a implementar les operacions de manera que no passa
  // res si fem un canvi a meitat.
  _state.files[drv].f= NULL;
  _state.files[drv].head_ready= false;
  
  // Comprova si la grandària és correcta i assigna grandàries.
  if ( file->nbytes < 0 || file->nbytes%512 != 0 )
    ret= PC_FD_WRONG_SIZE;
  else
    {
      switch ( _config->diskettes[drv] )
        {
        case PC_DISKETTE_NONE:
          ret= PC_FD_WRONG_SIZE;
          break;
        case PC_DISKETTE_360K:
          if ( file->nbytes <= 368640 )
            {
              _state.files[drv].f= file;
              _state.files[drv].num_C= 40;
              _state.files[drv].num_H= 2;
              _state.files[drv].num_S= 9;
              ret= PC_NOERROR;
            }
          else ret= PC_FD_WRONG_SIZE;
          break;
        case PC_DISKETTE_1M2:
          if ( file->nbytes <= 368640 )
            {
              _state.files[drv].f= file;
              _state.files[drv].num_C= 40;
              _state.files[drv].num_H= 2;
              _state.files[drv].num_S= 9;
              ret= PC_NOERROR;
            }
          else if ( file->nbytes <= 1228800 )
            {
              _state.files[drv].f= file;
              _state.files[drv].num_C= 80;
              _state.files[drv].num_H= 2;
              _state.files[drv].num_S= 15;
              ret= PC_NOERROR;
            }
          else ret= PC_FD_WRONG_SIZE;
          break;
        case PC_DISKETTE_720K:
          if ( file->nbytes <= 737280 )
            {
              _state.files[drv].f= file;
              _state.files[drv].num_C= 80;
              _state.files[drv].num_H= 1;
              _state.files[drv].num_S= 18;
              ret= PC_NOERROR;
            }
          else ret= PC_FD_WRONG_SIZE;
          break;
        case PC_DISKETTE_1M44:
          if ( file->nbytes <= 737280 )
            {
              _state.files[drv].f= file;
              _state.files[drv].num_C= 80;
              _state.files[drv].num_H= 1;
              _state.files[drv].num_S= 18;
              ret= PC_NOERROR;
            }
          else if ( file->nbytes <= 1474560 )
            {
              _state.files[drv].f= file;
              _state.files[drv].num_C= 80;
              _state.files[drv].num_H= 2;
              _state.files[drv].num_S= 18;
              ret= PC_NOERROR;
            }
          else ret= PC_FD_WRONG_SIZE;
          break;
        default:
          printf("fd.c - WTF !!!!\n");
          exit ( EXIT_FAILURE );
        }
    }
  
  update_cc_to_event ();
  
  return ret;
  
} // end PC_fd_insert_floppy


void
PC_fd_dma_signal (
                  const PC_DMA_Signal signal
                  )
{
  
  if ( !_in_clock ) clock ( false );
  
  if ( signal == PC_DMA_SIGNAL_DACK )
    {
      if ( _state.dma_state.op == DMA_OP_READ_DATA )
        {
          // Desactiva DREQ si ja hem enviat l'últim byte.
          if ( _state.dma_state.current_sec == _state.dma_state.end_sec &&
               _state.dma_state.p == SECTOR_SIZE )
            PC_dma_dreq ( 2, false );
        }
      else
        {
          printf("[EE] PC_fd_dma_signal %d!!!!!\n",signal);
          exit(EXIT_FAILURE);
        }
    }
  else // PC_DMA_SIGNAL_TC
    {
      assert ( signal == PC_DMA_SIGNAL_TC );
      if ( _state.dma_state.op == DMA_OP_NONE )
        _warning ( _udata, "FD: s'ha ignorat un DMA TC signal perquè"
                   " ja ha acabat la operació de DMA" );
      else if ( _state.dma_state.op == DMA_OP_READ_DATA )
        read_data_dma_result ( 0x00, 0x00 );
      else
        {
          printf("[EE] PC_fd_dma_signal %d!!!!!\n",signal);
          exit(EXIT_FAILURE);
        }
    }
  
  if ( !_in_clock ) update_cc_to_event ();
  
} // end PC_fd_dma_signal


uint8_t
PC_fd_dma_read (void)
{

  uint8_t ret;
  
  
  clock ( false );
  ret= _dma_read ();
  update_cc_to_event ();
  
  return ret;
  
} // end PC_fd_dma_read
