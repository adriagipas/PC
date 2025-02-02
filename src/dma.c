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
 *  dma.c - Implementació dels controladors de DMA (DMA ISA).
 *
 */

/*
 *  NOTA!!! Els temps del DMA ISA depen de SYSCLK. SYSCLK és PCICLK/4,
 *  però PCICLK pot ser 30MHz o 33MHz. Com que tampoc pareix súper
 *  important, de moment vaig a assumir que sempre és 30MHz, és a dir,
 *  SYSCLK 7.5MHz.
 */

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "PC.h"




/**********/
/* MACROS */
/**********/

#define DIV 75

#define DREQ_LAT_SIZE 100

#define ADDR_MASK 0xFFFFFF




/*********/
/* ESTAT */
/*********/

// Callbacks.
static PC_Warning *_warning;
static PC_DMATransfer8 *_dma_transfer8;
static PC_DMATransfer16 *_dma_transfer16;
static void *_udata;

// Funcions tracejables
static void (*_mem_write8) (const int chn,
                            const uint32_t addr,
                            const uint8_t data);
static uint8_t (*_mem_read8) (const int chn,
                              const uint32_t addr);
static uint16_t (*_mem_read16) (const int chn,
                                const uint32_t addr);

// Estat dels canals
static struct
{
  enum {
    DEMAND=0,
    SINGLE,
    BLOCK,
    CASCADE
  }        transfer_mode;
  bool     inc; // S'incrementa l'adreça
  bool     autoinit;
  enum {
    VERIFY,
    WRITE,
    READ,
    UNK
  }        transfer_type;
  uint16_t addr;
  uint16_t base_addr;
  uint16_t counter;
  uint16_t base_counter;
  uint8_t  low_page;
  uint8_t  base_low_page;
} _chns[8];

// Aquest registre funciona de màscara de les senyals DREQ. Un 1
// impideix que es processe un DREQ.
static uint8_t _mask;

// Indica l'estat de les senyals DREQ. 1 indica petició (implementació
// meua).
static uint8_t _dreq;

// Similar a DREQ però indica els flags TC (terminació).
static uint8_t _tc;

// Fliflop
static uint8_t _flipflop[2];

// Prioritat
static int _prio[2][4]; // El primer grup sempre té més prioritat.

// Estat d'una transferència
static struct
{
  
  int  chn;
  bool running;
  int  cc;
  
} _transfer;

// DREQ LATENCY
// NOTA!!! El primer de la FIFO sempre té el cc més baixet
static struct
{
  struct
  {
    int  chn_id;
    bool val;
    int  cc;
  }   v[DREQ_LAT_SIZE];
  int p;
  int N;
} _dreq_lat;

// Timing
static struct
{

  int cc_used;
  int cc;
  int cctoEvent;

  // Per a passar de CPU clocks a SYSCLOCK
  int cc_mul;
  int cc__byte;
  
} _timing;

// Indica que estem en meitat d'un clock.
static bool _in_clock;

// Indica que estem en mode JIT.
static bool _use_jit;

// Indica que estem en mode trace
static bool _trace_mode;




/*********************/
/* FUNCIONS PRIVADES */
/*********************/

static void
update_cc_to_event (void)
{
  
  int cc,tmp;
  
  
  // Per defecte 1s
  _timing.cctoEvent= PC_ClockFreq;
  // Latència DREQ
  if ( _dreq_lat.N > 0 )
    {
      tmp= _dreq_lat.v[_dreq_lat.p].cc;
      assert ( tmp > 0 );
      if ( tmp < _timing.cctoEvent ) _timing.cctoEvent= tmp;
    }
  // Esperant transfer
  if ( _transfer.running && _transfer.cc < _timing.cctoEvent )
    _timing.cctoEvent= _transfer.cc;
  
  // Actualitza PC_NextEventCC
  cc= PC_dma_next_event_cc ();
  cc+= PC_Clock; // Medim sempre des de que PC_Clock és 0
  if ( cc < PC_NextEventCC )
    PC_NextEventCC= cc;
  
} // end update_cc_to_event


static void
reset_group (
             const int id
             )
{

  int i,beg,end;

  
  beg= id*4;
  end= (id+1)*4;
  for ( i= beg; i < end; ++i )
    {
      _chns[i].transfer_mode= DEMAND;
      _chns[i].inc= true;
      _chns[i].autoinit= false;
      _chns[i].transfer_type= VERIFY;
      _chns[i].base_addr= 0x0000; // INVENT !!!
      _chns[i].addr= 0x0000; // INVENT !!!
      _chns[i].base_counter= 0x0000; // INVENT !!!
      _chns[i].counter= 0x0000; // INVENT !!!
      _chns[i].base_low_page= 0x00; // INVENT !!!
      _chns[i].low_page= 0x00; // INVENT !!!
    }
  // Desactiva
  if ( id == 0 )
    {
      _mask|= 0x0f;
      _dreq&= 0xf0;
      _tc&= 0xf0;
    }
  else
    {
      _mask|= 0xf0;
      _dreq&= 0x0f;
      _tc&= 0x0f;
    }
  _flipflop[id]= 0;
  // NOTA!! Açò no està clar si al resetejar es reseteja, però de
  // moment prioritat fixa!!!
  for ( i= 0; i < 4; ++i )
    _prio[id][i]= i + id*4;
  
  // Para transferències actives
  if ( _transfer.running &&
       ((id == 0 && _transfer.chn < 4) || (id == 1 && _transfer.chn >= 4) ) )
    {
      _transfer.running= false;
      // PARAR EVENT !!!!!
    }
  
} // end reset_group


static void
dack_signal (
             const int chn
             )
{

  switch ( chn )
    {
    case 1: PC_sb16_dma_signal ( PC_DMA_SIGNAL_DACK ); break;
    case 2: PC_fd_dma_signal ( PC_DMA_SIGNAL_DACK ); break;

    case 5: PC_sb16_dma16_signal ( PC_DMA_SIGNAL_DACK ); break;
    default:
      PC_MSGF("dma.c - dack - CHN%d",chn);
      exit ( EXIT_FAILURE );
    }
  
} // end dack_signal


static void
tc_signal (
           const int chn
           )
{

  switch ( chn )
    {
    case 1: PC_sb16_dma_signal ( PC_DMA_SIGNAL_TC ); break;
    case 2: PC_fd_dma_signal ( PC_DMA_SIGNAL_TC ); break;

    case 5: PC_sb16_dma16_signal ( PC_DMA_SIGNAL_TC ); break;
    default:
      PC_MSGF("dma.c - TC - CHN%d",chn);
      exit ( EXIT_FAILURE );
    }
  _tc|= 1<<chn;
  
} // end tc_signal


static void
check_signals (void)
{

  int i,j,chn;
  uint8_t tmp;
  

  if ( _transfer.running ) return;
  
  tmp= _dreq&(~_mask);
  for ( i= 0; i < 2; ++i )
    for ( j= 0; j < 4; ++j )
      {
        chn= _prio[i][j];
        if ( (tmp&(1<<chn)) != 0 )
          {
            dack_signal ( chn );
            _transfer.running= true;
            _transfer.chn= chn;
            // MBDMAx[FAST]  CAL IMPLEMENTAR !!!! Per defecte false
            _transfer.cc= _timing.cc__byte;
          }
      }
  
} // end check_signals


static void
mem_write8 (
            const int      chn,
            const uint32_t addr,
            const uint8_t  data
            )
{
  PC_CPU.mem_write8 ( _udata, (uint64_t) addr, data );
} // end mem_write8


static void
mem_jit_write8 (
                const int      chn,
                const uint32_t addr,
                const uint8_t  data
                )
{
  PC_CPU_JIT->mem_write8 ( _udata, (uint64_t) addr, data );
} // end mem_jit_write8


static void
mem_write8_trace (
                  const int      chn,
                  const uint32_t addr,
                  const uint8_t  data
                  )
{

  _dma_transfer8 ( chn, addr, data, false, _udata );
  mem_write8 ( chn, addr, data );
  
} // end mem_write8_trace


static void
mem_jit_write8_trace (
                      const int      chn,
                      const uint32_t addr,
                      const uint8_t  data
                      )
{

  _dma_transfer8 ( chn, addr, data, false, _udata );
  mem_jit_write8 ( chn, addr, data );
  
} // end mem_jit_write8_trace


static uint8_t
mem_read8 (
           const int      chn,
           const uint32_t addr
           )
{
  return PC_CPU.mem_read8 ( _udata, (uint64_t) addr );
} // end mem_read8


static uint8_t
mem_jit_read8 (
                const int      chn,
                const uint32_t addr
                )
{
  return PC_CPU_JIT->mem_read8 ( _udata, (uint64_t) addr, true );
} // end mem_jit_read8


static uint8_t
mem_read8_trace (
                 const int      chn,
                 const uint32_t addr
                 )
{

  uint8_t ret;

  
  ret= mem_read8 ( chn, addr );
  _dma_transfer8 ( chn, addr, ret, true, _udata );

  return ret;
  
} // end mem_read8_trace


static uint8_t
mem_jit_read8_trace (
                     const int      chn,
                     const uint32_t addr
                     )
{

  uint8_t ret;

  
  ret= mem_jit_read8 ( chn, addr );
  _dma_transfer8 ( chn, addr, ret, true, _udata );
  
  return ret;
  
} // end mem_jit_read8_trace


static uint16_t
mem_read16 (
            const int      chn,
            const uint32_t addr
            )
{
  return PC_CPU.mem_read16 ( _udata, (uint64_t) addr );
} // end mem_read16


static uint16_t
mem_jit_read16 (
                const int      chn,
                const uint32_t addr
                )
{
  return PC_CPU_JIT->mem_read16 ( _udata, (uint64_t) addr );
} // end mem_jit_read16


static uint16_t
mem_read16_trace (
                  const int      chn,
                  const uint32_t addr
                  )
{

  uint16_t ret;

  
  ret= mem_read16 ( chn, addr );
  _dma_transfer16 ( chn, addr, ret, true, _udata );

  return ret;
  
} // end mem_read16_trace


static uint16_t
mem_jit_read16_trace (
                      const int      chn,
                      const uint32_t addr
                      )
{

  uint16_t ret;
  
  
  ret= mem_jit_read16 ( chn, addr );
  _dma_transfer16 ( chn, addr, ret, true, _udata );
  
  return ret;
  
} // end mem_jit_read16_trace


static void
write_byte (
           const int      chn,
           const uint32_t addr
           )
{

  uint8_t byte;


  // Obté el byte
  switch ( chn )
    {
    case 2: byte= PC_fd_dma_read (); break;
    default:
      PC_MSGF("dma.c - write_byte - DMA.%d",chn);
      exit(EXIT_FAILURE);
    }

  // Escriu el byte
  _mem_write8 ( chn, addr, byte );
  
} // end write_byte


static void
read_byte (
           const int      chn,
           const uint32_t addr
           )
{

  uint8_t byte;


  // Obté el byte
  byte= _mem_read8 ( chn, addr );

  // Escriu el byte
  switch ( chn )
    {
    case 1: PC_sb16_dma_write ( byte ); break;
    default:
      PC_MSGF("dma.c - read_byte - DMA.%d",chn);
      exit(EXIT_FAILURE);
    }
  
} // end read_byte


static void
read_word (
           const int      chn,
           const uint32_t addr
           )
{

  uint16_t word;


  // Obté la paraula
  word= _mem_read16 ( chn, addr );
  
  // Escriu el byte
  switch ( chn )
    {
      
    case 5: PC_sb16_dma16_write ( word ); break;
        
    default:
      PC_MSGF("dma.c - read_word - DMA.%d",chn);
      exit(EXIT_FAILURE);
    }
  
} // end read_word


static void
run_transfer (void)
{

  uint32_t addr;
  bool tc;
  
  
  // NOTA!! No deuria de passar que es processen més d'una run de
  // colp, però així és més robust.
  do {
    
    // Transferència 8bit
    if ( _transfer.chn < 4 )
      {

        // Adreça
        addr=
          (((uint32_t) _chns[_transfer.chn].low_page)<<16) |
          ((uint32_t) _chns[_transfer.chn].addr);

        // Transferència
        switch ( _chns[_transfer.chn].transfer_type )
          {
          case WRITE: write_byte ( _transfer.chn, addr ); break;
          case READ: read_byte ( _transfer.chn, addr ); break;
          default:
            PC_MSGF("dma.c - run_transfer DMA.%d:"
                    " TRANSFER_TYPE %d",
                    _transfer.chn,_chns[_transfer.chn].transfer_type);
            exit(EXIT_FAILURE);
          }

        // Incrementa/Decrementa adreça
        if ( _chns[_transfer.chn].inc )
          {
            ++_chns[_transfer.chn].addr;
            /* // NO SÉ SI TINC QUE FER-HO
            if ( _chns[_transfer.chn].addr == 0x0000 )
              ++_chns[_transfer.chn].low_page;
            */
          }
        else
          {
            --_chns[_transfer.chn].addr;
            /* // NO SÉ SI TINC QUE FER-HO
            if ( _chns[_transfer.chn].addr == 0xFFFF )
              --_chns[_transfer.chn].low_page;
            */
          }
        
      }
    else // Transferència 16bit
      {

        // Adreça (Es fa un shift de l'adreçai s'ignora el bit
        // inferior de la pàgina)
        addr=
          (((uint32_t) (_chns[_transfer.chn].low_page&0xfe))<<16) |
          (((uint32_t) _chns[_transfer.chn].addr)<<1);
        
        // Transferència
        switch ( _chns[_transfer.chn].transfer_type )
          {
          case READ: read_word ( _transfer.chn, addr ); break;
          default:
            PC_MSGF("dma.c - run_transfer DMA.%d:"
                    " TRANSFER_TYPE %d",
                    _transfer.chn,_chns[_transfer.chn].transfer_type);
            exit(EXIT_FAILURE);
          }

        // Incrementa/Decrementa adreça
        if ( _chns[_transfer.chn].inc )
          {
            ++_chns[_transfer.chn].addr;
            /*
            if ( _chns[_transfer.chn].addr == 0x0000 )
              //_chns[_transfer.chn].low_page+= 2; // Big inferior s'ignora
              ++_chns[_transfer.chn].low_page;
            */
          }
        else
          {
            --_chns[_transfer.chn].addr;
            /*
            if ( _chns[_transfer.chn].addr == 0xFFFF )
              //_chns[_transfer.chn].low_page-= 2; // Big inferior s'ignora
              --_chns[_transfer.chn].low_page;
            */
          }
        
      }

    // Decrementa counter
    --_chns[_transfer.chn].counter;
    tc= false;
    if ( _chns[_transfer.chn].counter == 0xFFFF ) // TC
      {
        tc= true; // Sempre o sols quan no hi ha autonit ???
        tc_signal ( _transfer.chn );
        if ( _chns[_transfer.chn].autoinit )
          {
            _chns[_transfer.chn].low_page= _chns[_transfer.chn].base_low_page;
            _chns[_transfer.chn].addr= _chns[_transfer.chn].base_addr;
            _chns[_transfer.chn].counter= _chns[_transfer.chn].base_counter;
          }
        else _mask|= (1<<_transfer.chn);
      }
    
    // Transfer mode
    switch ( _chns[_transfer.chn].transfer_mode )
      {
      case SINGLE: // Desactiva
        _transfer.running= false;
        _transfer.cc= 0;
        break;
      case DEMAND: // Desactiva
        if ( tc )
          {
            _transfer.running= false;
            _transfer.cc= 0;
          }
        break;
      default:
        PC_MSGF("dma.c - run_transfer : TRANSFER_MODE %d",
                _chns[_transfer.chn].transfer_mode);
        exit(EXIT_FAILURE);
      }
    
  } while ( _transfer.running && _transfer.cc <= 0 );
  
  // Si no està running torna a provar
  if ( !_transfer.running ) check_signals ();
  
} // end run_transfer


static void
clock (
       const bool update_cc2event
       )
{

  int cc,clocks,end,p;
  

  _in_clock= true;
  
  // Processa cicles
  cc= PC_Clock - _timing.cc_used;
  if ( cc > 0 ) { _timing.cc+= cc; _timing.cc_used+= cc; }

  // Processa clocks
  clocks= _timing.cc;
  _timing.cc= 0;
  
  // Processa les latències.
  if ( _dreq_lat.N > 0 )
    {
      end= (_dreq_lat.p+_dreq_lat.N)%DREQ_LAT_SIZE;
      p= _dreq_lat.p;
      while ( p != end )
        {
          _dreq_lat.v[p].cc-= clocks;
          if ( _dreq_lat.v[p].cc <= 0 )
            {
              if ( _dreq_lat.v[p].val ) _dreq|= (1<<_dreq_lat.v[p].chn_id);
              else
                {
                  _dreq&= ~(1<<_dreq_lat.v[p].chn_id);
                  if ( _transfer.running &&
                       _transfer.chn == _dreq_lat.v[p].chn_id &&
                       _chns[_transfer.chn].transfer_mode == DEMAND )
                    {
                      _transfer.running= false;
                      _transfer.cc= 0;
                    }
                }
              check_signals ();
              // Com estan ordenades mai apareixeran huecos
              _dreq_lat.p= (_dreq_lat.p+1)%DREQ_LAT_SIZE;
              --_dreq_lat.N;
            }
          p= (p+1)%DREQ_LAT_SIZE;
        }
    }

  // Transferència
  if ( _transfer.running )
    {
      _transfer.cc-= clocks;
      if ( _transfer.cc <= 0 )
        run_transfer ();
    }
  
  // Actualitza cctoEvent
  if ( update_cc2event )
    update_cc_to_event ();

  _in_clock= false;
  
} // end clock




/**********************/
/* FUNCIONS PÚBLIQUES */
/**********************/

void
PC_dma_init (
             PC_Warning       *warning,
             PC_DMATransfer8  *dma_transfer8,
             PC_DMATransfer16 *dma_transfer16,
             void             *udata,
             const PC_Config  *config
             )
{

  int tmp_cc;

  
  // Callbacks.
  _warning= warning;
  _dma_transfer8= dma_transfer8;
  _dma_transfer16= dma_transfer16;
  _udata= udata;
  
  // Reseteja.
  _transfer.running= false;
  reset_group ( 0 );
  reset_group ( 1 );

  // Altres
  _dreq_lat.p= 0;
  _dreq_lat.N= 0;
  
  // Timing.
  _timing.cc= 0;
  _timing.cc_used= 0;
  _timing.cctoEvent= 0;
  assert ( PC_ClockFreq%100000 == 0 );
  _timing.cc_mul= PC_ClockFreq/100000;
  tmp_cc= 8*_timing.cc_mul; // 8 SYSCLK de latència
  _timing.cc__byte= tmp_cc/DIV + ((tmp_cc%DIV)!=0);
  update_cc_to_event ();

  // Altres (Importants)
  _mem_write8= mem_write8;
  _mem_read8= mem_read8;
  _mem_read16= mem_read16;
  _in_clock= false;
  _use_jit= false;
  _trace_mode= false;
  
} // end PC_dma_init


void
PC_dma_reset (void)
{

  clock ( false );
  
  // Reseteja.
  _transfer.running= false;
  reset_group ( 0 );
  reset_group ( 1 );
  
  // Altres
  _dreq_lat.p= 0;
  _dreq_lat.N= 0;
  
  update_cc_to_event ();
  
} // end PC_dma_reset


void
PC_dma_set_mode_jit (
                     const bool val
                     )
{
  
  _use_jit= val;
  PC_dma_set_mode_trace ( _trace_mode );
  
} // end PC_dma_set_mode_jit


void
PC_dma_set_mode_trace (
                       const bool val
                       )
{

  _trace_mode= val;
  if ( val && _dma_transfer8 )
    {
      _mem_write8= _use_jit ? mem_jit_write8_trace : mem_write8_trace;
      _mem_read8= _use_jit ? mem_jit_read8_trace : mem_read8_trace;
      _mem_read16= _use_jit ? mem_jit_read16_trace : mem_read16_trace;
    }
  else
    {
      _mem_write8= _use_jit ? mem_jit_write8 : mem_write8;
      _mem_read8= _use_jit ? mem_jit_read8 : mem_read8;
      _mem_read16= _use_jit ? mem_jit_read16 : mem_read16;
    }
  
} // end PC_dma_set_mode_trace


int
PC_dma_next_event_cc (void)
{

  int tmp;


  tmp= _timing.cctoEvent - _timing.cc;
  assert ( tmp > 0 );
  
  return tmp;
  
} // end PC_dma_next_event_cc


void
PC_dma_end_iter (void)
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
  
} // end PC_dma_end_iter


void
PC_dma_dmc_write (
                  const int dmaid
                  )
{

  clock ( false );

  reset_group ( dmaid );

  update_cc_to_event ();
  
} // end PC_dma_dmc_write


void
PC_dma_dclm_write (
                   const int dmaid
                   )
{
  PC_MSGF ( "(DCLM - DMA Clear Mask Register)"
           " Bits de la màscara 'cleared' DMA%d, s'accepten DMA"
           " requests per a tots els canals",
           dmaid+1 );
  exit(EXIT_FAILURE);
} // end PC_dma_dclm_write


void
PC_dma_dcm_write (
                  const int     dmaid,
                  const uint8_t data
                  )
{

  int chn;


  clock ( false );

  chn= (data&0x3) + dmaid*4;
  switch ( (data>>6)&0x3 )
    {
    case 0: _chns[chn].transfer_mode= DEMAND; break;
    case 1: _chns[chn].transfer_mode= SINGLE; break;
    case 2: _chns[chn].transfer_mode= BLOCK; break;
    case 3: _chns[chn].transfer_mode= CASCADE; break;
    }
  _chns[chn].inc= ((data&0x20)==0x00);
  _chns[chn].autoinit= ((data&0x10)!=0x00);
  switch ( (data>>2)&0x3 )
    {
    case 0: _chns[chn].transfer_type= VERIFY; break;
    case 1: _chns[chn].transfer_type= WRITE; break;
    case 2: _chns[chn].transfer_type= READ; break;
    case 3:
      _chns[chn].transfer_type= UNK;
      _warning ( _udata,
                 "S'ha configurat DMA.%d (ISA) amb un tipus"
                 " de transferència il·legal",
                 chn );
      break;
    }

  // Avisos sobre el canal 4 i el mode cascade
  if ( chn == 4 && _chns[chn].transfer_mode != CASCADE )
    _warning ( _udata,
               "S'ha configurat DMA.4 (ISA) amb un mode"
               " diferente de CASCADE !!!" );
  else if ( chn != 4 && _chns[chn].transfer_mode == CASCADE )
    _warning ( _udata,
               "S'ha configurat DMA.%d (ISA) amb mode"
               " CASCADE !!!",
               chn );
  
  update_cc_to_event ();
  
} // end PC_dma_dcm_write


void
PC_dma_wsmb_write (
                   const int     dmaid,
                   const uint8_t data
                   )
{

  int chn;

  
  clock ( false );

  chn= (data&0x3) + dmaid*4;
  if ( (data&0x04) != 0 ) _mask|= (1<<chn); // Disable
  else                    _mask&= ~(1<<chn); // Enable
  check_signals ();
  
  update_cc_to_event ();
  
} // end PC_dma_wsmb_write


void
PC_dma_dcbp_write (
                   const int dmaid
                   )
{

  clock ( true );
  
  _flipflop[dmaid]= 0;
  
} // end PC_dma_dcbp_write


uint8_t
PC_dma_status (
               const int dmaid
               )
{

  uint8_t ret;

  
  clock ( true );

  if ( dmaid == 0 )
    {
      ret=
        ((_dreq&0x0f)<<4) |
        (_tc&0x0f)
        ;
      _tc&= 0xf0;
    }
  else
    {
      ret=
        (_dreq&0xf0) |
        ((_tc&0xf0)>>4)
        ;
      _tc&= 0x0f;
    }
  
  return ret;
  
} // end PC_dma_status


void
PC_dma_dbaddr_write (
                     const int     chn_id, // 0 .. 7
                     const uint8_t data
                     )
{

  int gid;
  
  
  clock ( true );

  gid= (chn_id>>2);
  if ( _flipflop[gid] == 0 )
    {
      _chns[chn_id].base_addr&= 0xFF00;
      _chns[chn_id].base_addr|= (uint16_t) data;
      _chns[chn_id].addr&= 0xFF00;
      _chns[chn_id].addr|= (uint16_t) data;
    }
  else
    {
      _chns[chn_id].base_addr&= 0x00FF;
      _chns[chn_id].base_addr|= (((uint16_t) data)<<8);
      _chns[chn_id].addr&= 0x00FF;
      _chns[chn_id].addr|= (((uint16_t) data)<<8);
    }
  _flipflop[gid]^= 1;
  
} // end PC_dma_dbaddr_write


uint8_t
PC_dma_dbaddr_read (
                    const int chn_id // 0 .. 7
                    )
{

  int gid;
  uint8_t ret;
  
  
  clock ( true );

  gid= (chn_id>>2);
  if ( _flipflop[gid] == 0 ) ret= (uint8_t) (_chns[chn_id].addr&0xFF);
  else                       ret= (uint8_t) (_chns[chn_id].addr>>8);
  _flipflop[gid]^= 1;

  return ret;
  
} // end PC_dma_dbaddr_read


void
PC_dma_dbcnt_write (
                    const int     chn_id,
                    const uint8_t data
                    )
{

  int gid;
  
  
  clock ( true );
  
  gid= (chn_id>>2);
  if ( _flipflop[gid] == 0 )
    {
      _chns[chn_id].base_counter&= 0xFF00;
      _chns[chn_id].base_counter|= (uint16_t) data;
      _chns[chn_id].counter&= 0xFF00;
      _chns[chn_id].counter|= (uint16_t) data;
    }
  else
    {
      _chns[chn_id].base_counter&= 0x00FF;
      _chns[chn_id].base_counter|= (((uint16_t) data)<<8);
      _chns[chn_id].counter&= 0x00FF;
      _chns[chn_id].counter|= (((uint16_t) data)<<8);
    }
  _flipflop[gid]^= 1;
  
} // end PC_dma_dbcnt_write


uint8_t
PC_dma_dbcnt_read (
                   const int chn_id
                   )
{
  
  int gid;
  uint8_t ret;

  
  clock ( true );

  gid= (chn_id>>2);
  if ( _flipflop[gid] == 0 ) ret= (uint8_t) (_chns[chn_id].counter&0xFF);
  else                       ret= (uint8_t) (_chns[chn_id].counter>>8);
  _flipflop[gid]^= 1;

  return ret;
  
} // end PC_dma_dbcnt_read


void
PC_dma_dlpage_write (
                     const int     chn_id,
                     const uint8_t data
                     )
{

  clock ( true );
  
  _chns[chn_id].base_low_page= data;
  _chns[chn_id].low_page= data;
  
} // end PC_dma_dlpage_write


uint8_t
PC_dma_dlpage_read (
                    const int chn_id
                    )
{

  uint8_t ret;
  
  
  clock ( true );
  
  ret= _chns[chn_id].low_page;
  
  return ret;
  
} // end PC_dma_dlpage_read


void
PC_dma_dreq (
             const int  chn_id,
             const bool val
             )
{

  int n,p,end;

  
  if ( !_in_clock ) clock ( false );

  if ( false /* MBDMAx[FAST]  CAL IMPLEMENTAR!!!! Per defecte false */ )
    {
      if ( val ) _dreq|= (1<<chn_id);
      else       _dreq&= ~(1<<chn_id);
      if ( !_in_clock ) check_signals ();
    }
  else if ( _dreq_lat.N == DREQ_LAT_SIZE )
    _warning ( _udata,
               "S'ha ignorat DMA.%d DREQ perquè no cap en el buffer"
               " que he implementat!!!!", chn_id );
  else
    {
      // IMPORTANT !!!!! Tal i com ho he implementat la latència em
      // fastidia si la senyal és de desactivar!!! De fet.... Mirant
      // el tema del delay igual sols es quan s'activa. Per tant, al
      // desactivar vaig a desactivar immediatament i a desactivar
      // tots els pendents.
      if ( !val )
        {
          _dreq&= ~(1<<chn_id);
          if ( _transfer.running &&
               _transfer.chn == chn_id &&
               _chns[_transfer.chn].transfer_mode == DEMAND )
            {
              _transfer.running= false;
              _transfer.cc= 0;
            }
          p= _dreq_lat.p;
          end= (_dreq_lat.p+_dreq_lat.N)%DREQ_LAT_SIZE;
          while ( p != end )
            {
              if ( _dreq_lat.v[p].chn_id == chn_id )
                _dreq_lat.v[p].val= false;
              p= (p+1)%DREQ_LAT_SIZE;
            }
          if ( !_in_clock ) check_signals ();
        }
      else
        {
          n= (_dreq_lat.p+_dreq_lat.N)%DREQ_LAT_SIZE;
          ++_dreq_lat.N;
          _dreq_lat.v[n].chn_id= chn_id;
          _dreq_lat.v[n].val= val;
          _dreq_lat.v[n].cc= _timing.cc__byte; // Com un byte 8 cicles
        }
    }
  
  if ( !_in_clock ) update_cc_to_event ();
  
} // end PC_dma_dreq


void
PC_dma_dcom_write (
                   const int     dmaid,
                   const uint8_t data
                   )
{

  clock ( true );

  if ( (data&0xD4) != 0 )
    {
      PC_MSGF ( "DMA - PC_dma_dcom_write(GID:%d,DATA:%X)",
                dmaid, data );
      exit ( EXIT_FAILURE );
    }
  
} // end PC_dma_dcom_write


void
PC_dma_dr_write (
                 const int     dmaid,
                 const uint8_t data
                 )
{

  clock ( true );

  if ( (data&0x04) != 0 )
    {
      PC_MSGF ( "DMA - "
                "PC_dma_dr_write(GID:%d,DATA:%X) - Set",
                dmaid, data );
      exit ( EXIT_FAILURE );
    }
  
} // end PC_dma_dr_write
