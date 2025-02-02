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
 *  rtc.c - Implementa el Real Time Clock del chipset 430TX.
 *
 */
/*
 * FALTEN!!!!!!!!
 *
 *
 *  - El registre PCI: RTCCFG
 *
 *  - Diversos ports I/O:
 *     * RTCEI, RTCED
 */


#include <assert.h>
#include <stddef.h>
#include <stdlib.h>

#include "PC.h"




/**********/
/* MACROS */
/**********/

#define IF_IRQF 0x80
#define IF_PF   0x40
#define IF_AF   0x20
#define IF_UF   0x10




/*********/
/* ESTAT */
/*********/

// Callbacks.
static PC_Warning *_warning;
static PC_CMOSRAMAccess *_cmos_ram_access;
static PC_GetCurrentTime *_get_current_time;
static void *_udata;

static bool _use_year_century;

// RAM
static uint8_t *_ram[2]; // Cada bank és de 128 bytes.

// Estat registres I/O
static struct
{

  uint8_t addr; // Índex dins del standard RAM bank access

} _io;

// Registres.
static struct
{
  struct
  {
    bool uip;
    enum {
      NORMAL_OP= 0,
      INVALID
    }    div_mode;
    int  pie_rate;
  } a;
  struct
  {
    bool update;
    bool pie;
    bool aie;
    bool uie;
    bool use_binary;
    bool use_24h;
    bool dse;
  } b;
} _regs;

// Timing
static struct
{

  int cc_used;
  int cc; // Cicles que portem en l'actual update
  int cctoEvent;
  int ccpermicro;
  
  long cc_pie;
  long ccperPIE;
  long pie_fact;
  
  int cctoSetUIP;
  int cctoStartUpdate;
  int cctoFinishUpdate;
  
  int update_pos;
  
} _timing;

// Funcions lectura tracejables.
static uint8_t (*_rtcd_read) (void);
static void (*_rtcd_write) (const uint8_t data);




/*********************/
/* FUNCIONS PRIVADES */
/*********************/

static void
reset_regs (void)
{

  _ram[0][0xb]&= 0x87;
  _ram[0][0xc]= 0x00;
  
} // end reset_regs


static uint8_t
dec2bcd (
         const uint8_t val
         )
{
  return (((val/10)<<4) | (val%10))&0xFF;
} // end dec2bcd


static uint8_t
bcd2dec (
         const uint8_t val
         )
{
  return (val>>4)*10 + (val&0xF);
} // end bcd2dec


static void
check_irq (void)
{

  uint8_t val;


  val= _ram[0][0x0c];
  if ( ((val&IF_PF)!=0 && _regs.b.pie) ||
       ((val&IF_AF)!=0 && _regs.b.aie) ||
       ((val&IF_UF)!=0 && _regs.b.uie) )
    {
      _ram[0][0x0c]|= IF_IRQF;
      PC_ic_irq ( 8, true );
    }
  else
    {
      _ram[0][0x0c]&= ~IF_IRQF;
      PC_ic_irq ( 8, false );
    }
  
} // end check_irq


// Actualitza desde cero l'estat de cctoPie i ccperPIE. Interessant
// durant la inicialització o quan es modifica el registre A o B.
static void
init_cc_to_pie (void)
{

  static const struct
  {
    long fact; // Cal multiplicar els cicles per açò.
    long dur; // Cal multiplicar açò per ccpermicro per tindre
  } MODES[]= {
    { 0, 0 }, // none
    { 4, 15625 }, // 3.90625 ms
    { 2, 15625 }, // 7.8125 ms
    { 128, 15625 }, // 122.070 µs
    { 64, 15625 }, // 244.141 µs
    { 32, 15625 }, // 488.281 µs
    { 16, 15625 }, // 976.5625 µs
    { 8, 15625 }, // 1.953125 ms
    { 4, 15625 }, // 3.90625 ms
    { 2, 15625 }, // 7.8125 ms
    { 1, 15625 }, // 15.625 ms
    { 1, 31250 }, // 31.25 ms
    { 1, 62500 }, // 62.5 ms
    { 1, 125000 }, // 125 ms
    { 1, 250000 }, // 250 ms
    { 1, 500000 }, // 500 ms
  };

  long tmp,fact;
  

  // NOTA!!! Açò és aproximat perquè en realitat no té perquè estar
  // sincronitzat. Vull dir ací recalcu-le cc_pie com si estaguera a
  // 0, però igual tenia uns quants cicles ja avançats de l'actual
  // cicle. Per tant vaig a modificar sols si s'ha modificat el
  // pie_fact.
  if ( _regs.b.pie && _regs.a.pie_rate != 0 )
    {
      fact= MODES[_regs.a.pie_rate].fact;
      if ( _timing.pie_fact != fact )
        {
          _timing.pie_fact= fact;
          _timing.ccperPIE=
            MODES[_regs.a.pie_rate].dur * (long) _timing.ccpermicro;
          tmp= ((long) _timing.update_pos) * _timing.pie_fact;
          while ( tmp > _timing.ccperPIE )
            tmp-= _timing.ccperPIE;
          _timing.cc_pie= tmp;
        }
    }
  else _timing.pie_fact= 0;
  
} // end init_cc_to_pie


static void
update_reg_a (void)
{

  uint8_t val;


  val= _ram[0][0x0a];
  switch ( (val>>4)&0x7 )
    {
    case 0: _regs.a.div_mode= INVALID; break;
    case 2: _regs.a.div_mode= NORMAL_OP; break;
    default:
      printf ( "[EE] [RTC] (update_reg_a) Division Chain Select"
               " (DVx) no suportat: %d",
               (val>>4)&0x7 );
      exit(EXIT_FAILURE);
    }
  _regs.a.pie_rate= val&0xF;
  if ( _regs.a.pie_rate == 0 )
    _ram[0][0x0c]&= ~IF_PF;
  
} // end update_reg_a


static void
update_reg_b (void)
{

  uint8_t val,tmp,aux;
  bool use_binary,use_24h;
  int i;
  

  val= _ram[0][0x0b];
  _regs.b.update= ((val&0x80)==0);
  _regs.b.pie= ((val&0x40)!=0);
  _regs.b.aie= ((val&0x20)!=0);
  _regs.b.uie= ((val&0x10)!=0);
  if ( (val&0x08)!=0 )
    PC_MSG ( "RTC - UPDATE_REG_B -"
             " Square Wave Enable no implementat" );
  use_binary= ((val&0x04)!=0);
  if ( use_binary != _regs.b.use_binary )
    {
      if ( use_binary )
        {
          // Seconds ... Minutes Alarm
          for ( i= 0x00; i < 0x04; ++i )
            _ram[0][i]= bcd2dec ( _ram[0][i] );
          // Hours ... Hours Alarm
          for ( i= 0x04; i < 0x06; ++i )
            {
              tmp= _ram[0][i];
              _ram[0][i]= (tmp&0x80) | bcd2dec ( tmp&0x7F );
            }
          // Day of Week ... Year
          for ( i= 0x06; i < 0x0a; ++i )
            _ram[0][i]= bcd2dec ( _ram[0][i] );
        }
      else
        {
          // Seconds ... Minutes Alarm
          for ( i= 0x00; i < 0x04; ++i )
            _ram[0][i]= dec2bcd ( _ram[0][i] );
          // Hours ... Hours Alarm
          for ( i= 0x04; i < 0x06; ++i )
            {
              tmp= _ram[0][i];
              _ram[0][i]= (tmp&0x80) | dec2bcd ( tmp&0x7F );
            }
          // Day of Week ... Year
          for ( i= 0x06; i < 0x0a; ++i )
            _ram[0][i]= dec2bcd ( _ram[0][i] );
        }
    }
  _regs.b.use_binary= use_binary;
  use_24h= ((val&0x02)!=0);
  if ( _regs.b.use_24h != use_24h )
    {
      if ( use_24h )
        for ( i= 0x04; i < 0x06; ++i )
          {
            tmp= _ram[0][i];
            aux= tmp&0x7F;
            if ( !_regs.b.use_binary )
              aux= bcd2dec ( aux );
            if ( tmp&0x80 ) aux+= 11;
            else            --aux;
            if ( !_regs.b.use_binary )
              aux= dec2bcd ( aux );
            _ram[0][i]= aux;
          }
      else
        for ( i= 0x04; i < 0x06; ++i )
          {
            tmp= _ram[0][i];
            if ( !_regs.b.use_binary )
              tmp= bcd2dec ( tmp );
            aux= tmp%12 + 1;
            if ( !_regs.b.use_binary )
              aux= dec2bcd ( aux );
            if ( tmp > 11 ) aux|= 0x80;
            _ram[0][i]= aux;
          }
    }
  _regs.b.use_24h= use_24h;
  _regs.b.dse= ((val&0x01)!=0);
  
} // end update_reg_b


static void
init_io (void)
{

  _io.addr= 0;
  
} // end init_io


static uint8_t
rtcd_read (void)
{

  uint8_t ret;

  
  // NOTA!!! Probablement cal implementar moltes coses que depenen
  // d'altres registres.
  if ( _io.addr < 0x0E )
    {
      if ( _io.addr < 0x0A &&
           _regs.a.uip &&
           _timing.update_pos >= _timing.cctoStartUpdate )
        ret= 0xFF;
      else
        {
          switch ( _io.addr )
            {
            case 0x00 ... 0x09: ret= _ram[0][_io.addr]; break;
            case 0x0a: ret= (_regs.a.uip ? 0x80 : 0x00) | _ram[0][0x0a]; break;
            case 0x0b: ret= _ram[0][0x0b]; break;
            case 0x0c:
              ret= _ram[0][0x0c]&0xF0;
              _ram[0][0x0c]= 0x00;
              check_irq ();
              break;
            case 0x0d: ret= _ram[0][0x0d]&0xBF; break;
            default:
              PC_MSGF ( "[RTC] Llegint addr %02X de"
                        " standard RAM bank. Cal implementar!!!",
                        _io.addr );
              ret= 0xFF;
            }
        }
    }
  else ret= _ram[0][_io.addr];
  
  return ret;
  
} // end rtcd_read


static void
rtcd_write (
            const uint8_t data
            )
{

  // NOTA!!! Probablement cal implementar moltes coses que depenen
  // d'altres registres.
  if ( _io.addr < 0x0E )
    {
      if ( !(_io.addr < 0x0A &&
             _regs.a.uip &&
             _timing.update_pos >= _timing.cctoStartUpdate) )
        {
          switch ( _io.addr )
            {
            case 0x00 ... 0x09: _ram[0][_io.addr]= data; break;
            case 0x0a:
              _ram[0][0x0a]= data&0x7F;
              update_reg_a ();
              check_irq ();
              init_cc_to_pie ();
              break;
            case 0x0b:
              _ram[0][0x0b]= data;
              update_reg_b ();
              check_irq ();
              init_cc_to_pie ();
              break;
            case 0x0c:
              _ram[0][0x0c]= data&0xF0;
              check_irq ();
              break;
            case 0x0d:
              if ( data&0x80 )
                PC_MSG ( "[RTC] (rtcd_write) Valid RAM and"
                         " Time Bit no implementat" );
              _ram[0][0x0d]= data&0xBF;
              break;
            default:
              PC_MSGF ( "[RTC] Escrivint %02X en addr %02X"
                        " de standard RAM bank."
                        " Cal implementar!!!", data, _io.addr );
              exit(EXIT_FAILURE);
            }
        }
    }
  else _ram[0][_io.addr]= data;
  
} // end rtcd_write


static uint8_t
rtcd_read_trace (void)
{

  uint8_t ret;
  

  ret= rtcd_read ();
  _cmos_ram_access ( true, _io.addr, ret );

  return ret;
  
} // end rtcd_read_trace


static void
rtcd_write_trace (
                  const uint8_t data
                  )
{
  rtcd_write ( data );
  _cmos_ram_access ( false, _io.addr, data );
} // end rtcd_write_trace


// Compatibilitat amb qemu
static void
init_qemu_compatibility (
                         const PC_Config *config
                         )
{

  static const uint32_t RAM_SIZE_MB[PC_RAM_SIZE_SENTINEL]=
    { 4, 8, 16, 24, 32, 48, 64, 96, 128, 192, 256 };

  static const uint8_t QEMU_BOOT_ORDER_DEV[PC_QEMU_BOOT_ORDER_SENTINEL]=
    { 0, 1, 2, 3 };

  uint32_t ram_size;
  

  
  if ( !(config->flags&PC_CFG_QEMU_COMPATIBLE) ) return;

  // Grandària RAM
  // --> 0x15,0x16 en 1KB (640K)
  _ram[0][0x15]= 0x80;
  _ram[0][0x16]= 0x02;
  // --> 0x17,0x18 en 1KB (1MB --> ???)
  ram_size= ((RAM_SIZE_MB[config->ram_size]-1)*1024);
  _ram[0][0x17]= (uint8_t) (ram_size&0xFF);
  _ram[0][0x18]= (uint8_t) ((ram_size>>8)&0xFF);
  // --> 0x30,0x31,0x34,0x35
  if ( config->ram_size > PC_RAM_SIZE_24MB )
    {
      // Açò vendria a ser, memòria adicional a 16Mb medit en blocs de
      // 64KB.
      // Utilitza posicions de la CMOS reservats.
      ram_size= ((RAM_SIZE_MB[config->ram_size]-16)*1024)/64;
      _ram[0][0x34]= (uint8_t) (ram_size&0xFF); // CMOS_MEM_EXTMEM2_LOW
      _ram[0][0x35]= (uint8_t) ((ram_size>>8)&0xFF); // CMOS_MEM_EXTMEM2_HIGH
      // Fique els 16 MB en el low ???????? 0x3c00
      // of extended memory found above 1Mb at POST, medit en blocs de 1K
      _ram[0][0x30]= 0x00;
      _ram[0][0x31]= 0x3c;
    }
  else
    {
      _ram[0][0x34]= 0; // CMOS_MEM_EXTMEM2_LOW
      _ram[0][0x35]= 0; // CMOS_MEM_EXTMEM2_HIGH
      // of extended memory found above 1Mb at POST, medit en blocs de 1K
      ram_size= (RAM_SIZE_MB[config->ram_size]-1)*1024;
      _ram[0][0x30]= (uint8_t) (ram_size&0xFF); // CMOS_MEM_EXTMEM_LOW
      _ram[0][0x31]= (uint8_t) ((ram_size>>8)&0xFF); // CMOS_MEM_EXTMEM_HIGH
    }

  // Boot order.
  _ram[0][0x38]= (QEMU_BOOT_ORDER_DEV[config->qemu_boot_order.order[2]]<<4);
  _ram[0][0x3d]=
    (QEMU_BOOT_ORDER_DEV[config->qemu_boot_order.order[1]]<<4) |
    QEMU_BOOT_ORDER_DEV[config->qemu_boot_order.order[0]];
  if ( !(config->qemu_boot_order.check_floppy_sign) )
    _ram[0][0x38]|= 0x01;

  // Diskettes
  _ram[0][0x10]=
    (((uint8_t) config->diskettes[0])<<4) |
    ((uint8_t) config->diskettes[1]);
  
} // end init_qemu_compatibility


static void
init_date_time (
                const PC_Config *config
                )
{

  uint8_t ss,mm,hh,day_week,day_month,month,tmp;
  int year;


  _get_current_time ( _udata, &ss, &mm, &hh, &day_week,
                      &day_month, &month, &year );
  // segons
  if ( ss > 59 )
    {
      _warning ( _udata,
                 "[RTC] (get_current_time) valor incorrecte segons: %d",
                 ss );
      ss= 0;
    }
  _ram[0][0x00]= _regs.b.use_binary ? ss : dec2bcd ( ss );
  // minuts
  if ( mm > 59 )
    {
      _warning ( _udata,
                 "[RTC] (get_current_time) valor incorrecte minuts: %d",
                 mm );
      mm= 0;
    }
  _ram[0][0x02]= _regs.b.use_binary ? mm : dec2bcd ( mm );
  // Hores
  if ( hh > 23 )
    {
      _warning ( _udata,
                 "[RTC] (get_current_time) valor incorrecte hores: %d",
                 hh );
      hh= 0;
    }
  if ( _regs.b.use_24h )
    _ram[0][0x04]= _regs.b.use_binary ? hh : dec2bcd ( hh );
  else
    {
      tmp= hh%12 + 1;
      if ( hh > 11 )
        _ram[0][0x04]= 0x80 | (_regs.b.use_binary ? tmp : dec2bcd ( tmp ));
      else
        _ram[0][0x04]= _regs.b.use_binary ? tmp : dec2bcd ( tmp );
    }
  // Week
  if ( day_week < 1 || day_week > 7 )
    {
      _warning ( _udata,
                 "[RTC] (get_current_time) valor incorrecte"
                 " dia de la semana: %d",
                 day_week );
      day_week= 1;
    }
  if ( day_week == 1 ) day_week= 7;
  else                 --day_week;
  _ram[0][0x06]= _regs.b.use_binary ? day_week : dec2bcd ( day_week );
  // day of month
  if ( day_month < 1 || day_month > 31 )
    {
      _warning ( _udata,
                 "[RTC] (get_current_time) valor incorrecte"
                 " dia del mes: %d",
                 day_month );
      day_month= 1;
    }
  _ram[0][0x07]= _regs.b.use_binary ? day_month : dec2bcd ( day_month );
  // month
  if ( month < 1 || month > 12 )
    {
      _warning ( _udata,
                 "[RTC] (get_current_time) valor incorrecte"
                 " del mes: %d",
                 month );
      month= 1;
    }
  _ram[0][0x08]= _regs.b.use_binary ? month : dec2bcd ( month );
  // year
  tmp= year%100;
  _ram[0][0x09]= _regs.b.use_binary ? tmp : dec2bcd ( tmp );
  if ( config->flags&PC_CFG_QEMU_COMPATIBLE )
    {
      tmp= year/100;
      _ram[0][0x32]= _regs.b.use_binary ? tmp : dec2bcd ( tmp );
    }
  
} // end init_date_time


static void
update_cc_to_event (void)
{

  bool update_enabled;
  int cc,tmp;
  long tmp2;
  
  
  // Per defecte (1s)
  _timing.cctoEvent= PC_ClockFreq;
  // Cicle update.
  update_enabled= (_regs.a.div_mode==NORMAL_OP && _regs.b.update);
  if ( update_enabled && (_regs.b.aie || _regs.b.uie) )
    {
      tmp= _timing.cctoFinishUpdate - _timing.update_pos;
      assert ( tmp > 0 );
      if ( tmp < _timing.cctoEvent ) _timing.cctoEvent= tmp;
    }
  // PIE
  if ( _regs.b.pie && _regs.a.pie_rate != 0 )
    {
      tmp2= _timing.ccperPIE-_timing.cc_pie;
      assert ( tmp2 > 0 );
      tmp= (int) (tmp2/_timing.pie_fact) + ((tmp2%_timing.pie_fact)!=0);
      if ( tmp < _timing.cctoEvent ) _timing.cctoEvent= tmp;
    }
  
  // Actualitza PC_NextEventCC
  cc= PC_rtc_next_event_cc ();
  cc+= PC_Clock; // Medim sempre des de que PC_Clock és 0
  if ( cc < PC_NextEventCC )
    PC_NextEventCC= cc;
  
} // end update_cc_to_event


static void
run_update (void)
{

  uint8_t ss,mm,hh,day_week,day_month,month,year_l,year_h,tmp,aday_month;
  uint16_t year;
  bool inc_month,leap_year;

  
  // Llig
  ss= _ram[0][0x00];
  mm= _ram[0][0x02];
  hh= _ram[0][0x04];
  day_week= _ram[0][0x06];
  day_month= _ram[0][0x07];
  month= _ram[0][0x08];
  year_l= _ram[0][0x09];
  year_h= _ram[0][0x32];

  // Descodifica hh (decimal 24h)
  if ( !_regs.b.use_24h )
    {
      tmp= (hh&0x7F);
      if ( !_regs.b.use_binary ) tmp=  bcd2dec ( tmp );
      if ( hh&0x80 ) hh= tmp+11;
      else           hh= tmp-1;
    }
  else if ( !_regs.b.use_binary )
    hh= bcd2dec ( hh );
  // Descodifica altres camps
  if ( !_regs.b.use_binary )
    {
      ss= bcd2dec ( ss );
      mm= bcd2dec ( mm );
      day_week= bcd2dec ( day_week );
      day_month= bcd2dec ( day_month );
      month= bcd2dec ( month );
      year_l= bcd2dec ( year_l );
      year_h= bcd2dec ( year_h );
    }
  // Inicialitza any
  if ( _use_year_century )
    year= (((uint16_t) year_h)<<8) | ((uint16_t) year_l);
  else
    {
      if ( year_l > 80 ) year= 1900 + (uint16_t) year_l;
      else               year= 2000 + (uint16_t) year_l;
    }
  
  // Actualitza.
  if ( ss < 59 ) ++ss;
  else
    {
      ss= 0;
      if ( mm < 59 ) ++mm;
      else
        {
          mm= 0;
          if ( hh < 23 ) ++hh;
          else
            {
              hh= 0;
              if ( day_week == 7 ) day_week= 1;
              else                 ++day_week;
              // Day month
              // -> Febrer
              if ( month == 2 )
                {
                  leap_year= ((year%4==0) && ((year%100!=0) || (year%400==0)));
                  if ( (leap_year && day_month < 29) ||
                       (!leap_year && day_month < 28) )
                    { inc_month= false; ++day_month; }
                  else { inc_month= true; day_month= 1; }
                }
              // -> 30 dies
              else if ( month == 4 || month == 6 || month == 9 ||
                        month == 11 )
                {
                  if ( day_month < 30 ) { inc_month= false; ++day_month; }
                  else                  { inc_month= true; day_month= 1; }
                }
              // -> 31 dies
              else
                {
                   if ( day_month < 31 ) { inc_month= false; ++day_month; }
                   else                  { inc_month= true; day_month= 1; }
                }
              // Month
              if ( inc_month )
                {
                  if ( month < 12 ) ++month;
                  else
                    {
                      month= 1;
                      ++year;
                      if ( _use_year_century )
                        {
                          year_l= (uint8_t) (year&0xFF);
                          year_h= (uint8_t) (year>>8);
                        }
                      else year_l= (uint8_t) (year&0xFF);
                    }
                }
            }
        }
    }

  // Aplica DSE
  if ( _regs.b.dse )
    {
      // One is on the first Sunday in April, where time increments
      // from 1:59:59 AM to 3:00:00 AM.
      if ( month == 4 && day_week == 1 && day_month <= 7 &&
           hh == 2 && mm == 0 && ss == 0 )
        hh= 3;
      // the last Sunday in October when the time first reaches
      // 1:59:59 AM, it is changed to 1:00:00 AM.
      else if ( month == 10 && day_week == 1 && day_month >= 25 &&
                hh == 2 && mm == 0 && ss == 0 )
        hh= 1;
    }
  
  // Codifica hh
  if ( !_regs.b.use_24h )
    {
      tmp= (hh%12) + 1;
      if ( !_regs.b.use_binary ) tmp= dec2bcd ( tmp );
      if ( hh > 11 ) tmp|= 0x80;
      hh= tmp;
    }
  else if ( !_regs.b.use_binary )
    hh= dec2bcd ( hh );
  // Codifica altres camps
  if ( !_regs.b.use_binary )
    {
      ss= dec2bcd ( ss );
      mm= dec2bcd ( mm );
      day_week= dec2bcd ( day_week );
      day_month= dec2bcd ( day_month );
      month= dec2bcd ( month );
      year_l= dec2bcd ( year_l );
      year_h= dec2bcd ( year_h );
    }

  // Desa.
  _ram[0][0x00]= ss;
  _ram[0][0x02]= mm;
  _ram[0][0x04]= hh;
  _ram[0][0x06]= day_week;
  _ram[0][0x07]= day_month;
  _ram[0][0x08]= month;
  _ram[0][0x09]= year_l;
  _ram[0][0x32]= year_h;

  // ACtiva flags interrupció.
  _ram[0][0x0c]|= IF_UF;
  aday_month= _ram[0][0x0d]&0x3F;
  if ( _ram[0][0x01] == ss && _ram[0][0x03] == mm && _ram[0][0x05] == hh &&
       (aday_month == 0 || aday_month == day_month) )
    _ram[0][0x0c]|= IF_AF;
  
} // end run_update


static void
clock (
       const bool update_cc2event
       )
{

  int cc,tmp;
  bool update_enabled;

  
  // Processa cicles
  cc= PC_Clock-_timing.cc_used;
  if ( cc > 0 ) { _timing.cc+= cc; _timing.cc_used+= cc; }

  // PIE.
  if ( _regs.b.pie && _regs.a.pie_rate != 0 )
    {
      _timing.cc_pie+= ((long) _timing.cc)*_timing.pie_fact;
      while ( _timing.cc_pie >= _timing.ccperPIE )
        {
          _timing.cc_pie-= _timing.ccperPIE;
          _ram[0][0x0c]|= IF_PF;
        }
    }
  
  // Fes update (ho fa sempre, però sols actualitza si està activat)
  update_enabled= (_regs.a.div_mode==NORMAL_OP && _regs.b.update);
  while ( _timing.cc )
    {
      // Ficant UIP
      if ( _timing.update_pos < _timing.cctoSetUIP )
        {
          tmp= _timing.cctoSetUIP-_timing.update_pos;
          if ( _timing.cc >= tmp )
            {
              _timing.cc-= tmp;
              _timing.update_pos= _timing.cctoSetUIP;
              if ( update_enabled ) _regs.a.uip= true;
            }
          else
            {
              _timing.update_pos+= _timing.cc;
              _timing.cc= 0;
            }
        }
      // Start update
      else if ( _timing.update_pos < _timing.cctoStartUpdate )
        {
          tmp= _timing.cctoStartUpdate-_timing.update_pos;
          if ( _timing.cc >= tmp )
            {
              _timing.cc-= tmp;
              _timing.update_pos= _timing.cctoStartUpdate;
            }
          else
            {
              _timing.update_pos+= _timing.cc;
              _timing.cc= 0;
            }
        }
      // Finish update
      else
        {
          tmp= _timing.cctoFinishUpdate-_timing.update_pos;
          if ( _timing.cc >= tmp )
            {
              _timing.cc-= tmp;
              _timing.update_pos= 0;
              if ( update_enabled )
                {
                  _regs.a.uip= false;
                  run_update ();
                }
            }
          else
            {
              _timing.update_pos+= _timing.cc;
              _timing.cc= 0;
            }
        }
    }
  
  // Actualitza estat IRQ.
  check_irq ();
  
  // Actualitza cc2event
  if ( update_cc2event )
    update_cc_to_event ();
  
} // end clock




/**********************/
/* FUNCIONS PÚBLIQUES */
/**********************/

void
PC_rtc_init (
             PC_Warning        *warning,
             PC_GetCurrentTime *get_current_time,
             PC_GetCMOSRAM     *get_cmos_ram,
             PC_CMOSRAMAccess  *cmos_ram_access,
             void              *udata,
             const PC_Config   *config
             )
{
  
  // Callbacks.
  _warning= warning;
  _get_current_time= get_current_time;
  _cmos_ram_access= cmos_ram_access;
  _udata= udata;

  // Config.
  _use_year_century= (config->flags&PC_CFG_QEMU_COMPATIBLE)!=0;
  
  // RAM.
  _ram[0]= get_cmos_ram ( _udata );
  _ram[1]= _ram[0] + 128;
  
  // Funcions lectura CMOS/RAM
  _rtcd_read= rtcd_read;
  _rtcd_write= rtcd_write;
  
  // Estat.
  init_io ();

  // Inicialitza valors en la CMOS.
  _ram[0][0xa]&= 0x7F; // En el init fique este bit d'estat a 0
  reset_regs ();
  update_reg_a ();
  update_reg_b ();
  // UPDATEJA C i D
  init_qemu_compatibility ( config );
  init_date_time ( config );

  // Timers.
  _timing.cc_used= 0;
  _timing.cc= 0;
  assert ( PC_ClockFreq%1000000 == 0 );
  _timing.ccpermicro= PC_ClockFreq/1000000;
  _timing.cctoSetUIP= _timing.ccpermicro*(1000000-(244+1984));
  _timing.cctoStartUpdate= _timing.ccpermicro*(1000000-1984);
  _timing.cctoFinishUpdate= PC_ClockFreq;
  _timing.cctoEvent= 0;
  _timing.update_pos= 0;
  _timing.cc_pie= 0;
  _timing.ccperPIE= 0;
  _timing.pie_fact= 0;
  init_cc_to_pie ();
  update_cc_to_event ();
  
} // end PC_rtc_init


void
PC_rtc_write_rtci (
                   const uint8_t data
                   )
{

  clock ( true );
  
  _io.addr= data&0x7F;
  
} // end PC_rtc_write_rtci


uint8_t
PC_rtc_rtci_read (void)
{

  clock ( true );

  return _io.addr&0x7F;
  
} // end PC_rtc_rtci_read


uint8_t
PC_rtc_rtcd_read (void)
{

  clock ( true );
  
  return _rtcd_read ();
  
} // end PC_rtc_read_rtcd


void
PC_rtc_rtcd_write (
                   const uint8_t data
                   )
{

  clock ( false );
  
  _rtcd_write ( data );
  update_cc_to_event ();
  
} // end PC_rtc_rtcd_write


void
PC_rtc_set_mode_trace (
                       const bool val
                       )
{

  if ( val && _cmos_ram_access != NULL )
    {
      _rtcd_read= rtcd_read_trace;
      _rtcd_write= rtcd_write_trace;
    }
  else
    {
      _rtcd_read= rtcd_read;
      _rtcd_write= rtcd_write;
    }
  
} // end PC_rtc_set_mode_trace


int
PC_rtc_next_event_cc (void)
{

  int tmp;


  tmp= _timing.cctoEvent - _timing.cc;
  assert ( tmp > 0 );

  return tmp;
  
} // end PC_rtc_next_event_cc


void
PC_rtc_end_iter (void)
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
  
} // end PC_rtc_end_iter
