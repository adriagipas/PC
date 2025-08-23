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
 *  piix4_ide.c - Implementa la part del PIIX4 que s'encarrega de
 *                implementar el controlador IDE.
 *
 */
/*
 * NOTES!!! De moment en els CD-ROM no suporte ni interleave ni
 * encolament.
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

#define PCICMD_IOSE 0x0001

#define IDETIM_IDE 0x8000

#define SEC_SIZE 512
#define BUF_SIZE 0x10000 // MÀXIM SUPORTAT
#define MAX_LB_SIZE 2352

#define ERR_AMNF  0x01
#define ERR_TKZNF 0x02
#define ERR_ABRT  0x04
#define ERR_MCR   0x08
#define ERR_IDNF  0x10
#define ERR_MC    0x20
#define ERR_UNC   0x40
#define ERR_BBK   0x80

// Bytes màxims suportats per a un packet (el comandament)
#define PACKET_CMD_SIZE 12

// La grandària la aniré fent més gran segons necessite. Fique 19 per
// a ficar el 0.
#define CD_SENSE_DATA_SIZE 19

#define CD_SENSE_KEY_NONE            0x00
#define CD_SENSE_KEY_NOT_READY       0x02
#define CD_SENSE_KEY_MEDIUM_ERROR    0x03
#define CD_SENSE_KEY_ILLEGAL_REQUEST 0x05
#define CD_SENSE_KEY_ABORTED_COMMAND 0x0b

#define CD_ADD_SENSE_NONE                            0x0000
#define CD_ADD_SENSE_NO_SEEK_COMPLETE                0x0200
#define CD_ADD_SENSE_PARAMETER_LIST_LENGTH_ERROR     0x1a00
#define CD_ADD_SENSE_INVALID_FIELD_IN_CDB            0x2400
#define CD_ADD_SENSE_PARAMETER_VALUE_INVALID         0x2602
#define CD_ADD_SENSE_CAN_NOT_READ_UNK_FORMAT         0x3001
#define CD_ADD_SENSE_CAN_NOT_READ_INCOMPATIBE_FORMAT 0x3002
#define CD_ADD_SENSE_MEDIUM_NOT_PRESENT              0x3a00
#define CD_ADD_SENSE_OVERLAPPED_COMMANDS             0x4e00
#define CD_ADD_SENSE_ILLEGAL_MODE_FOR_THIS_TRACK     0x6400
#define CD_ADD_SENSE_AUDIO_PLAY_OPERATION_ABORTED    0xb900

#define MSF_IS_VALID(A) ((A).m<60 && (A).s<60 && (A).f<75)

#define CD_AUDIO_STATUS_NOVALID     0x00
#define CD_AUDIO_STATUS_IN_PROGRESS 0x11
#define CD_AUDIO_STATUS_PAUSED      0x12
#define CD_AUDIO_STATUS_COMPLETED   0x13
#define CD_AUDIO_STATUS_ERROR       0x14
#define CD_AUDIO_STATUS_NONE        0x15




/*************/
/* CONSTANTS */
/*************/

static const uint16_t VID= 0x8086;
static const uint16_t DID= 0x7111;
static const uint8_t RID= 0x00;

// CLASSC
static const uint8_t BASEC= 0x01; // Mass storage device
static const uint8_t SCC= 0x01; // IDE controller
static const uint8_t PI= 0x80; // Capable of IDE bus master operation.

static const uint8_t HEDT= 0x00;




/*********/
/* TIPUS */
/*********/

typedef struct
{
  uint8_t m;
  uint8_t s;
  uint8_t f;
} MSF;

typedef struct
{
  uint8_t lbalo; // En CHS és el sector
  uint8_t lbamid; // En CHS cylinder low
  uint8_t lbahi; // En CHS cylinder high
  uint8_t lbaextra; // En CHS Head
  bool    use_lba;
} hdd_addr_t;

typedef struct
{
  PC_File          *f; // No deuria ser NULL
  struct
  {
    int S;
    int C;
    int H;
  }                 size;
} hdd_t;

typedef struct
{

  // Globals
  uint8_t density_code;
  int     lblock_size;
  int     number_of_blocks; // ¿¿??? Típicament 0
  int     sectors_per_block; // Derivat
  
  // CD-ROM Parameters
  // NOTA!! Els paràmetres d'aquesta pàgina de moment els vaig a
  // ignorar. Si alguna vegada es modifiquen intentaré esbrinar si
  // importa el canvi.
  struct
  {
    uint8_t itm; // Inactivity Timer Multiplier.
    uint8_t MSFS_per_MSFM; // Segons per minut
    uint8_t MSFF_per_MSFS; // Sectors per segon
  }       cdrom_parameters;

  // CD-ROM Audio Control Parameters
  struct
  {
    bool    immed; // Immediate
    bool    sotc; // Stop On Track Crossing
    uint8_t chn_port0; // Out Port 0 Channel Select
    uint8_t chn_port1; // Out Port 1 Channel Select
    uint8_t vol_port0; // Output Port 0 Volume
    uint8_t vol_port1; // Output Port 0 Volume
  }       cdrom_audio_control_parameters;
  
} cdrom_mode_t;

typedef struct
{
  
  PC_CDRom     *cd; // No serà NULL.
  uint8_t       sense_data[CD_SENSE_DATA_SIZE];
  cdrom_mode_t  mode;
  bool          busy;
  bool          playing;
  bool          paused; // Paused implica playing.
  bool          locked;
  struct
  {
    uint8_t      v[MAX_LB_SIZE];
    unsigned int p;
    unsigned int l;
  }             buflb; // Buffer logical block
  struct
  {
    int     status;
    uint8_t v[CD_SEC_SIZE];
    int     p;
    int     l;
    MSF     current;
    MSF     end;
  }             audio; // Estat playing
  uint8_t       subchn_Q[CD_SUBCH_SIZE];
  
} cdrom_t;

typedef struct
{
  PC_IDEDeviceType type;
  struct
  {
    bool err;
    bool drq;
    bool srv;
    bool df;
    bool rdy;
    bool bsy;
  }                stat;
  bool             intrq;
  struct
  {
    bool     waiting;
    bool     drq_value; // Valor quan s'acava la tranferència.
    int      remain_cc; // Cicles que falten
    uint16_t buf[BUF_SIZE/2]; // NOTA!! està en el endianisme de la
                              // màquina on es compila.
    int      begin,end;
    enum {
      PT_NORMAL,
      PT_READ_SECTORS,
      PT_WRITE_SECTORS,
      PT_WRITE_SELECT_CD,
      PT_PACKET,
      PT_READ_CD, // Lectura normal
      PT_READ_CDLB
    }        mode;
    
    // Opcionals segons operacions
    int      current_sec; // Contant des de 0
    int      end_sec; // Sectors a continuació de l'últim

    // Opcionals per a packet
    int      packet_byte_count; // Bytes màxims per cada pio transfer
                                // (no inclou el propi comandament)

    // Opcionals per CD Logical Blocks
    struct
    {
      uint16_t remain;
      int      byte_count;
    }        cdlb;
    
  }                pio_transfer;
  hdd_t            hdd; // Per als dispositius HDD.
  cdrom_t          cdrom; // Per als dispositius CDROM.
} drv_t;




/*************/
/* CONSTANTS */
/*************/

static const cdrom_mode_t CD_DEFAULT_MODE= {

  // Global
  .density_code= 0x00,
  .lblock_size= 2048,
  .number_of_blocks= 0,
  .sectors_per_block= 1,

  // CD-Rom Parameters
  .cdrom_parameters= {
    .itm= 0x0d, // 8 minuts
    .MSFS_per_MSFM= 0x3c, // 60 segons per minut
    .MSFF_per_MSFS= 0x4b // 75 sectors per segon
  },
  
  .cdrom_audio_control_parameters= {
    .immed= true, // ¿¿Per defecte immediat??
    .sotc= false, // ¿¿Per defecte no para quan traspassa??
    .chn_port0= 0x01, // Left audio
    .chn_port1= 0x02, // Right audio
    .vol_port0= 0xff, // Màxim volum
    .vol_port1= 0xff  // Màxim volum
  }
  
};




/*********/
/* ESTAT */
/*********/

// Callbacks.
static PC_Warning *_warning;
static void *_udata;

// Registres PCI
static struct
{
  uint16_t pcicmd;
  uint32_t bmiba;
  uint16_t idetim[2];
} _pci_regs;

// Dispositius
static struct
{
  int        ind; // Selecciona el dispositiu
  hdd_addr_t addr;
  uint8_t    features;
  uint8_t    sector_count;
  drv_t      drv[2];
  uint8_t    error;
  struct
  {
    bool nien;
    bool srst;
  }          ctrl;
} _dev[2];

// CDROM connectat a la SB16
static drv_t *_sound_dev;
static int _sound_dev_ide;

// Timing
static struct
{

  int cc_used;
  int cc;
  int ccpersector;
  int cctoEvent;
  
} _timing;




/***************/
/* DEFINICIONS */
/***************/

static void read_sectors_iter (const int ide,drv_t *drv);
static void read_cdlb_iter (const int ide,drv_t *drv);
static void write_sectors_iter (const int ide,drv_t *drv);
static void run_packet_command (const int ide,drv_t *drv);
static void cd_transfer_finish (const int ide,drv_t *drv);
static void set_signature (const int ide,const drv_t *drv);
static void cdrom_write_select_data (const int ide,drv_t *drv);
static void cd_successful_command_completion
(const int ide,drv_t *drv,const bool cdrom_busy);



/*********************/
/* FUNCIONS PRIVADES */
/*********************/

// Torna -1 (a<b), 0 (a==b), 1 (a>b)
static int
cmp_msf (
         const MSF a,
         const MSF b
         )
{

  if ( a.m < b.m ) return -1;
  else if ( a.m > b.m ) return 1;
  else if ( a.s < b.s ) return -1;
  else if ( a.s > b.s ) return 1;
  else if ( a.f < b.f ) return -1;
  else if ( a.f == b.f ) return 0;
  else return 1;
  
} // end cmp_msf


static MSF
cdpos2msf (
           const CD_Position pos
           )
{

  MSF ret;


  ret.m= 10*(pos.mm>>4) + (pos.mm&0xf);
  ret.s= 10*(pos.ss>>4) + (pos.ss&0xf);
  ret.f= 10*(pos.sec>>4) + (pos.sec&0xf);

  return ret;
  
} // end cdpos2msf


static void
inc_msf (
         MSF *const a
         )
{
  
  if ( ++(a->f) == 75 )
    {
      a->f= 0;
      if ( ++(a->s) == 60 )
        {
          a->s= 0;
          ++(a->m);
        }
    }
  
} // end inc_msf


static uint32_t
cdpos2offset (
              const CD_Position pos
              )
{
  
  uint32_t ret;
  
  
  ret=
    (10*((uint32_t)(pos.mm>>4)) + ((uint32_t) (pos.mm&0xf)))*60*75 +
    (10*((uint32_t)(pos.ss>>4)) + ((uint32_t) (pos.ss&0xf)))*75 +
    (10*((uint32_t)(pos.sec>>4)) + ((uint32_t) (pos.sec&0xf)))
    ;
  
  return ret;
  
} // end cdpos2offset


static CD_Position
offset2cdpos (
              uint32_t offset
              )
{

  uint32_t tmp;
  CD_Position ret;
  
  
  // MM
  tmp= offset/(60*75);
  ret.mm= (uint8_t) ((tmp/10)*0x10 + tmp%10);
  
  // SS
  offset%= 60*75;
  tmp= offset/75;
  ret.ss= (uint8_t) ((tmp/10)*0x10 + tmp%10);
  
  // SEC
  offset%= 75;
  ret.sec= (uint8_t) ((offset/10)*0x10 + offset%10);
  
  return ret;
  
} // end offset2cdpos


// Torna offset mesurat en sectors
static uint32_t
hdd_addr_get_offset (
                     const hdd_addr_t *addr
                     )
{

  uint32_t ret;

  
  if ( addr->use_lba )
    ret=
      ((uint32_t) addr->lbalo) |
      (((uint32_t) addr->lbamid)<<8) |
      (((uint32_t) addr->lbahi)<<16) |
      (((uint32_t) (addr->lbaextra&0xF))<<24)
      ;
  else
    {
      ret= 0;
      PC_MSG("piix4_ide.c - "
             "hdd_addr_get_offset - FALTA CHS!!!");
      exit(EXIT_FAILURE);
    }
  
  return ret;
  
} // end hdd_addr_get_offset


// L'offset fa referència a sectors
static void
hdd_addr_set_offset (
                     hdd_addr_t     *addr,
                     const uint32_t  offset
                     )
{

  if ( addr->use_lba )
    {
      addr->lbalo= (uint8_t) (offset&0xFF);
      addr->lbamid= (uint8_t) ((offset>>8)&0xFF);
      addr->lbahi=  (uint8_t) ((offset>>16)&0xFF);
      addr->lbaextra=  (uint8_t) ((offset>>24)&0x0F);
    }
  else
    {
      PC_MSG("piix4_ide.c - "
             "hdd_addr_set_offset - FALTA CHS!!!");
      exit(EXIT_FAILURE);
    }
    
} // end hdd_addr_set_offset


static void
update_cc_to_event (void)
{

  int cc,tmp,i,j;
  
  
  // Per defecte 1s
  _timing.cctoEvent= PC_ClockFreq;
  // Transferències de sectors
  for ( i= 0; i < 2; ++i )
    for ( j= 0; j < 2; ++j )
      if ( _dev[i].drv[j].type != PC_IDE_DEVICE_TYPE_NONE )
        if ( _dev[i].drv[j].pio_transfer.waiting )
          {
            tmp= _dev[i].drv[j].pio_transfer.remain_cc;
            assert ( tmp > 0 );
            if ( tmp < _timing.cctoEvent ) _timing.cctoEvent= tmp;
          }
  
  // Actualitza PC_NextEventCC
  cc= PC_piix4_ide_next_event_cc ();
  cc+= PC_Clock; // Medim sempre des de que PC_Clock és 0
  if ( cc < PC_NextEventCC )
    PC_NextEventCC= cc;
  
} // end update_cc_to_event


static void
update_irq (void)
{

  int i;
  bool val;
  const drv_t *drv;
  
  
  for ( i= 0; i < 2; ++i )
    {
      if ( _dev[i].ctrl.nien ) val= false;
      else
        {
          drv= &_dev[i].drv[_dev[i].ind];
          val= drv->type == PC_IDE_DEVICE_TYPE_NONE ? false : drv->intrq;
        }
      PC_ic_irq ( 14+i, val );
    }
  
} // end update_irq


static void
write_idetim (
              const int ch,
              uint16_t  data
              )
{
  
  _pci_regs.idetim[ch]= data;
  if ( data&0x4000 )
    PC_MSGF("IDE%d - IDETIM - Slave IDE Timing Register Enable",ch);
  PC_MSGF("IDE%d - IDETIM - IORDY Sample Point: %X",ch,((data>>12)&0x3));
  PC_MSGF("IDE%d - IDETIM - Recovery Time: %X",ch,((data>>8)&0x3));
  if ( data&0x0080 )
    PC_MSGF("IDE%d - IDETIM - DMA Timing Enable Only",ch);
  if ( data&0x0040 )
    PC_MSGF("IDE%d - IDETIM - Prefetch and Posting Enable",ch);
  if ( data&0x0020 )
    PC_MSGF("IDE%d - IDETIM - IORDY Sample Point Enable Drive Select 1",ch);
  if ( data&0x0010 )
    PC_MSGF("IDE%d - IDETIM - Fast Timing Bank Drive Select 1",ch);
  if ( data&0x0008 )
    PC_MSGF("IDE%d - IDETIM - DMA Timing Enable Only",ch);
  if ( data&0x0004 )
    PC_MSGF("IDE%d - IDETIM - Prefetch and Posting Enable",ch);
  if ( data&0x0002 )
    PC_MSGF("IDE%d - IDETIM - IORDY Sample Point Enable Drive Select 0",ch);
  if ( data&0x0001 )
    PC_MSGF("IDE%d - IDETIM - Fast Timing Bank Drive Select 0",ch);
  
} // end write_idetim


static void
ide_sector_number_lbalo_write (
                               const int     ide,
                               const uint8_t data
                               )
{
  _dev[ide].addr.lbalo= data;
} // end ide_sector_number_lbalo_write


static uint8_t
ide_sector_number_lbalo_read (
                              const int ide
                              )
{

  uint8_t ret;
  
  
  ret= _dev[ide].addr.lbalo;
  
  return ret;
  
} // end ide_sector_number_lbalo_read


static void
ide_cylinder_low_lbamid_write (
                               const int     ide,
                               const uint8_t data
                               )
{
  _dev[ide].addr.lbamid= data;
} // end ide_cylinder_low_lbamid_write


static uint8_t
ide_cylinder_low_lbamid_read (
                              const int ide
                              )
{

  uint8_t ret;
  
  
  ret= _dev[ide].addr.lbamid;

  return ret;
  
} // end ide_cylinder_low_lbamid_read


static void
ide_cylinder_high_lbahi_write (
                               const int     ide,
                               const uint8_t data
                               )
{
  _dev[ide].addr.lbahi= data;
} // end ide_cylinder_high_lbahi_write


static uint8_t
ide_cylinder_high_lbahi_read (
                              const int ide
                              )
{

  uint8_t ret;
  
  
  ret= _dev[ide].addr.lbahi;
  
  return ret;
  
} // end ide_cylinder_high_lbahi_read


static void
ide_drive_head_write (
                      const int     ide,
                      const uint8_t data
                      )
{

  drv_t *drv;
  
  
  drv= &_dev[ide].drv[_dev[ide].ind];
  if ( drv->type != PC_IDE_DEVICE_TYPE_NONE )
    drv->intrq= false;
  _dev[ide].ind= (data&0x10)==0 ? 0 : 1;
  _dev[ide].addr.use_lba= ((data&0x40)!=0);
  _dev[ide].addr.lbaextra= data;
  update_irq ();
  
} // end ide_drive_head_write


static uint8_t
ide_drive_head_read (
                     const int ide
                     )
{

  uint8_t ret;


  ret=
    0xa0 |
    (_dev[ide].addr.lbaextra&0x0F) |
    (_dev[ide].ind==0 ? 0x00 : 0x10) |
    (_dev[ide].addr.use_lba ? 0x40 : 0x00)
    ;
  
  return ret;
  
} // end ide_drive_head_read


static uint8_t
ide_stat_read (
               const int ide
               )
{

  uint8_t ret;
  drv_t *drv;
  
  
  // En cas de no existir torne tots els flags a 0. D'aquesta manera
  // la BIOS aplega a la conclusió que no està conectat.
  if ( _dev[ide].drv[_dev[ide].ind].type == PC_IDE_DEVICE_TYPE_NONE )
    ret= 0x00;
  else
    {
      drv= &_dev[ide].drv[_dev[ide].ind];
      drv->intrq= false;
      PC_MSGF("IDE%d - STAT - SRV",ide);
      ret=
        0x00 |
        (drv->stat.err ? 0x01 : 0x00) |
        (drv->stat.drq ? 0x08 : 0x00) |
        (drv->stat.srv ? 0x10 : 0x00) |
        (drv->stat.df  ? 0x20 : 0x00) |
        (drv->stat.rdy ? 0x40 : 0x00) |
        (drv->stat.bsy ? 0x80 : 0x00)
        ;
      update_irq ();
    }
  
  return ret;
  
} // end ide_stat_read


static void
ide_reset (
           const int ide
           )
{

  drv_t *drv;
  
  
  drv= &_dev[ide].drv[_dev[ide].ind];
  if ( drv->type != PC_IDE_DEVICE_TYPE_NONE )
    {
      drv->stat.err= false; // ?????
      drv->stat.drq= false;
      drv->stat.srv= false;
      drv->stat.df= false;
      drv->stat.rdy= drv->type!=PC_IDE_DEVICE_TYPE_CDROM;
      drv->stat.bsy= false;
      drv->intrq= false; // ????
      drv->pio_transfer.waiting= false;
      drv->pio_transfer.drq_value= false;
      drv->pio_transfer.remain_cc= 0;
      drv->pio_transfer.begin= 0;
      drv->pio_transfer.end= 0;
      drv->pio_transfer.mode= PT_NORMAL;
    }
  update_irq ();
  
} // end ide_reset


static void
ide_control_write (
                   const int     ide,
                   const uint8_t data
                   )
{

  bool new_srst;


  if ( _dev[ide].drv[0].type==PC_IDE_DEVICE_TYPE_NONE &&
       _dev[ide].drv[1].type==PC_IDE_DEVICE_TYPE_NONE )
    return;
  
  _dev[ide].ctrl.nien= ((data&0x02)!=0);
  new_srst= (data&0x04)!=0;
  if ( _dev[ide].ctrl.srst && !new_srst )
    ide_reset ( ide );
  _dev[ide].ctrl.srst= new_srst;
  if ( (data&0x80)!=0 )
    PC_MSGF("IDE%d - Control - HOB ????",ide);
  update_irq ();
  
} // end ide_control_write


static uint8_t
ide_error_read (
                const int ide
                )
{

  drv_t *drv;
  uint8_t ret;
  
  
  drv= &_dev[ide].drv[_dev[ide].ind];
  if ( drv->type == PC_IDE_DEVICE_TYPE_NONE )
    _warning ( _udata, "s'ha intentat llegir registre error d'IDE%d.%d",
               ide, _dev[ide].ind );
  ret= _dev[ide].error;
  
  return ret;
  
} // end ide_error_read


static uint16_t
ide_data_read (
               const int ide
               )
{
  
  drv_t *drv;
  uint16_t ret;
  

  drv= &_dev[ide].drv[_dev[ide].ind];
  if ( drv->type == PC_IDE_DEVICE_TYPE_NONE )
    {
      _warning ( _udata, "s'ha intentat llegit dades d'IDE%d.%d però"
                 " no hi ha ningun disc dur connectat",
                 ide, _dev[ide].ind );
      ret= 0xffff;
    }
  else
    {
      if ( drv->pio_transfer.waiting ||
           drv->pio_transfer.begin >= drv->pio_transfer.end )
        {
          _warning ( _udata, "s'ha intentat llegit dades d'IDE%d.%d però"
                     " no hi han dades preparades",
                     ide, _dev[ide].ind );
          ret= 0xffff;
        }
      else
        {
          ret= drv->pio_transfer.buf[drv->pio_transfer.begin++];
          if ( drv->pio_transfer.begin == drv->pio_transfer.end )
            {
              drv->stat.drq= false;
              switch ( drv->pio_transfer.mode )
                {
                case PT_READ_SECTORS: read_sectors_iter ( ide, drv ); break;
                case PT_READ_CD: cd_transfer_finish ( ide, drv ); break;
                case PT_READ_CDLB: read_cdlb_iter ( ide, drv ); break;
                default: break;
                }
            }
        }
    }
  
  return ret;
  
} // end ide_data_read


static void
ide_data_write (
                const int ide,
                uint16_t  data
                )
{
  
  drv_t *drv;
  
  
  drv= &_dev[ide].drv[_dev[ide].ind];
  if ( drv->type == PC_IDE_DEVICE_TYPE_NONE )
    _warning ( _udata, "s'ha intentat escriure dades en IDE%d.%d però"
               " no hi ha ningun disc dur connectat",
               ide, _dev[ide].ind );
  else
    {
      if ( drv->pio_transfer.waiting ||
           drv->pio_transfer.begin == drv->pio_transfer.end )
        _warning ( _udata, "s'ha intentat escriure dades en IDE%d.%d però"
                   " no hi ha espai",
                   ide, _dev[ide].ind );
      else
        {
#if PC_BE
          data= PC_SWAP16(data);
#endif
          drv->pio_transfer.buf[drv->pio_transfer.begin++]= data;
          if ( drv->pio_transfer.begin == drv->pio_transfer.end )
            {
              drv->stat.drq= false;
              switch ( drv->pio_transfer.mode )
                {
                case PT_WRITE_SECTORS: write_sectors_iter ( ide, drv ); break;
                case PT_WRITE_SELECT_CD:
                  cdrom_write_select_data ( ide, drv );
                  break;
                case PT_PACKET: run_packet_command ( ide, drv ); break;
                default: break;
                }
            }
        }
    }
  
} // end ide_data_write


static void
identify_device (
                 drv_t *drv
                 )
{
  
  int i;
  uint32_t sectors;
  
  
  // Inicialitza comandament
  drv->stat.bsy= true;
  drv->stat.rdy= true;
  drv->stat.df= false;
  // NOTA!! No entenc molt bé com va drq, però vaig a fer que
  // abans de la trasnferència estiga a false, després a true, i
  // quan es buide a fals altra vegada.
  drv->stat.drq= false;
  drv->stat.err= false;
  
  // Plena sector
  
  // --> Word 0: General configuration
  drv->pio_transfer.buf[0]= 0x0040; // 6 - not removable controller
                                    // and/or device
  // --> Word 1: Number of cylinders (logical)
  drv->pio_transfer.buf[1]= (uint16_t) drv->hdd.size.C;
  // --> Word 2: Reserved.
  drv->pio_transfer.buf[2]= 0x0000;
  // --> Word 3: Number of logical heads
  drv->pio_transfer.buf[3]= (uint16_t) drv->hdd.size.H;
  // --> Word 4-5: Retired.
  for ( i= 4; i <= 5; ++i ) drv->pio_transfer.buf[i]= 0x0000;
  // --> Word 6: Number of logical sectors per logical track
  drv->pio_transfer.buf[6]= (uint16_t) drv->hdd.size.S;
  // --> Word 7-9: Retired.
  for ( i= 7; i <= 9; ++i ) drv->pio_transfer.buf[i]= 0x0000;
  // --> Words 10-19: Serial number
  for ( i= 10; i < 15; ++i ) drv->pio_transfer.buf[i]= 0x5858; // 'XX'
  for ( i= 15; i <= 19; ++i ) drv->pio_transfer.buf[i]= 0x2020; // '  '
  // --> Word 20-21: Retired.
  // --> Word 22: Obsolete.
  for ( i= 20; i <= 22; ++i ) drv->pio_transfer.buf[i]= 0x0000;
  // --> Word 23-26: Firmware revision
  drv->pio_transfer.buf[23]= 0x312e; // '1.'
  drv->pio_transfer.buf[24]= 0x3020; // '0 '
  for ( i= 25; i <= 26; ++i ) drv->pio_transfer.buf[i]= 0x2020; // '  '
  // --> Words 27-46: Model number
  drv->pio_transfer.buf[27]= 0x4844; // 'HD'
  drv->pio_transfer.buf[28]= 0x442d; // 'D-'
  drv->pio_transfer.buf[29]= 0x4154; // 'AT'
  drv->pio_transfer.buf[30]= 0x4120; // 'A '
  drv->pio_transfer.buf[31]= 0x6d65; // 'me'
  drv->pio_transfer.buf[32]= 0x6d75; // 'mu'
  drv->pio_transfer.buf[33]= 0x5043; // 'PC'
  for ( i= 34; i <= 46; ++i ) drv->pio_transfer.buf[i]= 0x2020; // '  '
  // --> Word 47: READ/WRITE MULTIPLE support.
  drv->pio_transfer.buf[47]= 256; // ????
  // --> Word 48: Reserved.
  drv->pio_transfer.buf[48]= 0x0000;
  
  // --> Word 50: Capabilities (2)
  drv->pio_transfer.buf[50]= 0x4000; // Sense Standby timer
  // --> Word 51: PIO data transfer mode number
  drv->pio_transfer.buf[51]= 0x0000; // PIO MODE 0 ???
  // --> Word 52: Retired
  drv->pio_transfer.buf[52]= 0x0000;
  // --> Word 53: Field validity
  //   Fields 54-58 vàlids, suporta DMA i Ultra DMA
  drv->pio_transfer.buf[53]= 0x0007;
  // --> Word 54: Number of current logical cylinders
  drv->pio_transfer.buf[54]= (uint16_t) drv->hdd.size.C;
  // --> Word 55: Number of current logical heads
  drv->pio_transfer.buf[55]= (uint16_t) drv->hdd.size.H;
  // --> Word 56: Number of current logical sectors per logical track
  drv->pio_transfer.buf[56]= (uint16_t) drv->hdd.size.S;
  // --> Word (58:57): Current capacity in sectors
  sectors= drv->hdd.size.C*drv->hdd.size.H*drv->hdd.size.S;
  drv->pio_transfer.buf[57]= (uint16_t) (sectors&0xFFFF);
  drv->pio_transfer.buf[58]= (uint16_t) (sectors>>16);
  
  // --> Word (61:60): Current capacity in sectors
  sectors= drv->hdd.size.C*drv->hdd.size.H*drv->hdd.size.S;
  drv->pio_transfer.buf[60]= (uint16_t) (sectors&0xFFFF);
  drv->pio_transfer.buf[61]= (uint16_t) (sectors>>16);

  // --> Words 69-74: Reserved
  for ( i= 69; i <= 74; ++i ) drv->pio_transfer.buf[i]= 0x0000;

  // --> Words 76-79: Reserved
  for ( i= 76; i <= 79; ++i ) drv->pio_transfer.buf[i]= 0x0000;
  // --> Word 80: Major version number
  drv->pio_transfer.buf[80]= 0x0000;
  // --> Word 81: Minor version number
  drv->pio_transfer.buf[81]= 0x0017; // ATA/ATAPI-4 T13 1153D revision 17

  // --> Words 92-126: Reserved
  for ( i= 92; i <= 126; ++i ) drv->pio_transfer.buf[i]= 0x0000;

  // --> Words 129-159: Vendor specific
  for ( i= 129; i <= 159; ++i ) drv->pio_transfer.buf[i]= 0x0000;
  // --> Words 160-255: Reserved
  for ( i= 160; i <= 255; ++i ) drv->pio_transfer.buf[i]= 0x0000;
  
  // FALTEN !!!!!!
  drv->pio_transfer.buf[49]= 0;
  drv->pio_transfer.buf[59]= 0;
  for ( i= 62; i <= 68; ++i ) drv->pio_transfer.buf[i]= 0;
  drv->pio_transfer.buf[75]= 0;
  for ( i= 82; i <= 91; ++i ) drv->pio_transfer.buf[i]= 0;
  for ( i= 127; i <= 128; ++i ) drv->pio_transfer.buf[i]= 0;
  PC_MSG("IDENTIFY DEVICE"
         " Cal acabar d'implementar !!!");
  
  // Prepara transferència
  drv->pio_transfer.waiting= true;
  drv->pio_transfer.drq_value= true;
  drv->pio_transfer.remain_cc= _timing.ccpersector;
  drv->pio_transfer.begin= 0;
  drv->pio_transfer.end= SEC_SIZE/2;
  drv->pio_transfer.mode= PT_NORMAL;
  
} // end identify_device


static void
identify_packet_device_cd (
                           drv_t *drv
                           )
{
  
  int i;

  
  // Inicialitza comandament
  drv->stat.bsy= true;
  drv->stat.rdy= true;
  drv->stat.df= false;
  // NOTA!! No entenc molt bé com va drq, però vaig a fer que
  // abans de la trasnferència estiga a false, després a true, i
  // quan es buide a fals altra vegada.
  drv->stat.drq= false;
  drv->stat.err= false;
  
  // Plena sector
  
  // --> Word 0: General configuration
  drv->pio_transfer.buf[0]= 0x8580; // 15-14 - ATAPI device
                                    // 12-8  - CD-ROM
                                    // 6-5 - 00: Device shall set DRQ
                                    //       to one within 3 ms of receiving
                                    //       PACKET command???
                                    // 1-0 - 00: 12 byte command packet
  // --> Word 1-9: Reserved.
  for ( i= 1; i <= 9; ++i ) drv->pio_transfer.buf[i]= 0x0000;
  // --> Words 10-19: Serial number
  for ( i= 10; i < 15; ++i ) drv->pio_transfer.buf[i]= 0x5858; // 'XX'
  for ( i= 15; i <= 19; ++i ) drv->pio_transfer.buf[i]= 0x2020; // '  '
  // --> Word 20-22: Reserved.
  for ( i= 20; i <= 22; ++i ) drv->pio_transfer.buf[i]= 0x0000;
  // --> Word 23-26: Firmware revision
  drv->pio_transfer.buf[23]= 0x312e; // '1.'
  drv->pio_transfer.buf[24]= 0x3020; // '0 '
  for ( i= 25; i <= 26; ++i ) drv->pio_transfer.buf[i]= 0x2020; // '  '
  // --> Words 27-46: Model number
  drv->pio_transfer.buf[27]= 0x4344; // 'CD'
  drv->pio_transfer.buf[28]= 0x2d52; // '-R'
  drv->pio_transfer.buf[29]= 0x4f4d; // 'OM'
  drv->pio_transfer.buf[30]= 0x2020; // '  '
  drv->pio_transfer.buf[31]= 0x6d65; // 'me'
  drv->pio_transfer.buf[32]= 0x6d75; // 'mu'
  drv->pio_transfer.buf[33]= 0x5043; // 'PC'
  for ( i= 34; i <= 46; ++i ) drv->pio_transfer.buf[i]= 0x2020; // '  '
  // --> Word 47-48: Reserved.
  for ( i= 47; i <= 48; ++i ) drv->pio_transfer.buf[i]= 0x0000;
  // --> Word 49: Capabilities
  drv->pio_transfer.buf[49]= 0x0f00; // De moment elimine DMA
                                     // interleaved, queuing i
                                     // overlap.
  // --> Word 50: Reserved.
  drv->pio_transfer.buf[50]= 0x0000;
  // --> Word 51: PIO data transfer mode number
  drv->pio_transfer.buf[51]= 0x0000; // PIO MODE 0 ???
  // --> Word 52: Reserved.
  drv->pio_transfer.buf[52]= 0x0000;
  //   Fields 54-58 vàlids, suporta DMA i Ultra DMA
  drv->pio_transfer.buf[53]= 0x0007;
  // --> Word 54-62: Reserved.
  for ( i= 54; i <= 62; ++i ) drv->pio_transfer.buf[i]= 0x0000;
  
  // --> Words 73-74: Reserved
  for ( i= 73; i <= 74; ++i ) drv->pio_transfer.buf[i]= 0x0000;

  // --> Words 76-79: Reserved
  for ( i= 76; i <= 79; ++i ) drv->pio_transfer.buf[i]= 0x0000;
  // --> Word 80: Major version number
  drv->pio_transfer.buf[80]= 0x0000;
  // --> Word 81: Minor version number
  drv->pio_transfer.buf[81]= 0x0017; // ATA/ATAPI-4 T13 1153D revision 17

  // --> Words 89-126: Reserved
  for ( i= 89; i <= 126; ++i ) drv->pio_transfer.buf[i]= 0x0000;

  // --> Words 129-159: Vendor specific
  for ( i= 129; i <= 159; ++i ) drv->pio_transfer.buf[i]= 0x0000;
  // --> Words 160-255: Reserved
  for ( i= 160; i <= 255; ++i ) drv->pio_transfer.buf[i]= 0x0000;
  
  // FALTEN !!!!!!
  for ( i= 63; i <= 72; ++i ) drv->pio_transfer.buf[i]= 0;
  drv->pio_transfer.buf[75]= 0;
  for ( i= 82; i <= 88; ++i ) drv->pio_transfer.buf[i]= 0;
  for ( i= 127; i <= 128; ++i ) drv->pio_transfer.buf[i]= 0;
  PC_MSG("IDENTIFY PACKET DEVICE CD"
         " Cal acabar d'implementar !!!");
  
  // Prepara transferència
  drv->pio_transfer.waiting= true;
  drv->pio_transfer.drq_value= true;
  drv->pio_transfer.remain_cc= _timing.ccpersector;
  drv->pio_transfer.begin= 0;
  drv->pio_transfer.end= SEC_SIZE/2;
  drv->pio_transfer.mode= PT_NORMAL;
  
} // end identify_packet_device_cd


static void
read_sectors_iter (
                   const int  ide,
                   drv_t     *drv
                   )
{
  
  long offset,sec_offset;
  int ret;
  
  
  assert ( drv->hdd.f != NULL );
  
  // Si no queden sectors torna.
  // La primera vegada açò no deuria de passar mai!!
  if ( drv->pio_transfer.current_sec == drv->pio_transfer.end_sec )
    {
      drv->pio_transfer.mode= PT_NORMAL; // <-- En realitat no cal
      return;
    }

  // Inicialitza comandament
  drv->stat.bsy= true;
  drv->stat.rdy= true;
  drv->stat.df= false;
  // NOTA!! perquè funcione en READ_SECTORS, és important que DRQ
  // estiga a true també abans de la transferència. Es ficarà a false
  // quan es llisquen les dades.
  drv->stat.drq= true;
  drv->stat.err= false;
  
  // Calcula offset
  sec_offset= (long) hdd_addr_get_offset ( &_dev[ide].addr );
  sec_offset+= (long) drv->pio_transfer.current_sec;
  offset= sec_offset * (long) SEC_SIZE;
  
  // Plena sector
  // --> Comprovacions inicials
  if ( (offset+SEC_SIZE) > drv->hdd.f->nbytes ) goto error;
  // --> Intenta llegir
  ret= PC_file_seek ( drv->hdd.f, offset );
  if ( ret != 0 ) goto error;
  ret= PC_file_read ( drv->hdd.f,
                      (uint8_t *) &(drv->pio_transfer.buf[0]),
                      SEC_SIZE );
  if ( ret != 0 ) goto error;
  // --> Com torna uint16_t, cal fer un swap si estem en una màquina BE
#if PC_BE
  int i;
  for ( i= 0; i < SEC_SIZE/2; ++i )
    drv->pio_transfer.buf[i]= PC_SWAP16(drv->pio_transfer.buf[i]);
#endif
  // --> Incrementa el sector
  ++(drv->pio_transfer.current_sec);
  
  // Prepara transferència
  drv->pio_transfer.waiting= true;
  drv->pio_transfer.drq_value= true;
  drv->pio_transfer.remain_cc= _timing.ccpersector;
  drv->pio_transfer.begin= 0;
  drv->pio_transfer.end= SEC_SIZE/2;
  
  return;

 error:
  _dev[ide].error= ERR_ABRT;
  hdd_addr_set_offset ( &_dev[ide].addr, sec_offset );
  drv->stat.bsy= false;
  drv->stat.rdy= false;
  drv->stat.df= false; // No hi ha device fault en el meu emulador
  drv->stat.drq= false;
  drv->stat.err= true;
  drv->intrq= true;
  update_irq ();
  
} // end read_sectors_iter


static void
read_sectors (
              const int  ide,
              drv_t     *drv
              )
{
  
  // NOTA!!!! No està gens clar com es comporta la transferència de
  // més d'uns ector. Mirant la descripció de READ MULTIPLE pareix
  // prou clar que es transfereixen sector a sector. De fet al final
  // de cada sector el bsy es fica a fals (vore rutina de OSdev). El
  // que no queda clar és quan comença la lectura del següent
  // sector. Com que no queda gens clar, el que vaig a fer és assumir
  // que es concatenen. És a dir, fins que no es llig l'últim byte no
  // es llança la següent operació.
  //
  // NOTA!! Cada sector el vaig a tractar com si fora un comandament.
  
  // Prepara mode read
  drv->pio_transfer.mode= PT_READ_SECTORS;
  drv->pio_transfer.current_sec= 0;
  drv->pio_transfer.end_sec= _dev[ide].sector_count;
  if ( drv->pio_transfer.end_sec == 0 )
    drv->pio_transfer.end_sec= 256;

  // Llança la lectura
  read_sectors_iter ( ide, drv );
  
} // end read_sectors


static void
write_sectors_iter (
                    const int  ide,
                    drv_t     *drv
                    )
{

  long offset,sec_offset;
  int ret;
  
  
  assert ( drv->hdd.f != NULL );

  // Inicialitza comandament
  drv->stat.bsy= true;
  drv->stat.rdy= true;
  drv->stat.df= false;
  drv->stat.drq= false;
  drv->stat.err= false;
  
  // Calcula offset
  sec_offset= (long) hdd_addr_get_offset ( &_dev[ide].addr );
  sec_offset+= (long) drv->pio_transfer.current_sec;
  offset= sec_offset * (long) SEC_SIZE;
  
  // Escriu sector
  // --> Comprovacions inicials
  if ( (offset+SEC_SIZE) > drv->hdd.f->nbytes ) goto error;
  // --> Intenta escriure
  ret= PC_file_seek ( drv->hdd.f, offset );
  if ( ret != 0 ) goto error;
  ret= PC_file_write ( drv->hdd.f,
                       (uint8_t *) &(drv->pio_transfer.buf[0]),
                       SEC_SIZE );
  if ( ret != 0 ) goto error;
  // --> Incrementa el sector
  ++(drv->pio_transfer.current_sec);
  
  // Si no queden sectors evitem que es torne a cridar a esta rutina.
  if ( drv->pio_transfer.current_sec == drv->pio_transfer.end_sec )
    {
      drv->pio_transfer.mode= PT_NORMAL;
      drv->pio_transfer.drq_value= false;
    }
  else // Prepara següent interacció.
    {
      drv->pio_transfer.begin= 0;
      drv->pio_transfer.end= SEC_SIZE/2;
      drv->pio_transfer.drq_value= true;
    }
  
  // Prepara transferència
  drv->pio_transfer.waiting= true;
  drv->pio_transfer.remain_cc= _timing.ccpersector;
  
  return;
  
 error:
  _dev[ide].error= ERR_ABRT;
  hdd_addr_set_offset ( &_dev[ide].addr, sec_offset );
  drv->stat.bsy= false;
  drv->stat.rdy= false;
  drv->stat.df= false; // No hi ha device fault en el meu simulador
  drv->stat.drq= false;
  drv->stat.err= true;
  drv->intrq= true;
  update_irq ();
  
} // end write_sectors_iter


static void
write_sectors (
               const int  ide,
               drv_t     *drv
               )
{
  
  // NOTA!! No queda molt clar com es fa l'escriptura de múltipes
  // sectors. Vaig a fer una implementació anàloga a la lectura. És a
  // dir, vaig a fer totalment seqüèncial la transferència. El que
  // faig només cmençar és esperar als bytes i execute una iteració
  // quan es plena.
  
  // Prepara mode read
  drv->pio_transfer.mode= PT_WRITE_SECTORS;
  drv->pio_transfer.current_sec= 0;
  drv->pio_transfer.end_sec= _dev[ide].sector_count;
  if ( drv->pio_transfer.end_sec == 0 )
    drv->pio_transfer.end_sec= 256;
  drv->pio_transfer.begin= 0;
  drv->pio_transfer.end= SEC_SIZE/2;
  
  // Prepara per a rebre comandaments. En realitat sols cal ficar el
  // DRQ a true, la resta deuria d'estar ja bé.
  drv->stat.bsy= false; // El ficarem a true quan es faça la transferència.
  drv->stat.rdy= true;
  drv->stat.df= false;
  drv->stat.drq= true; // Es fica a cert per a acceptar dades.
  drv->stat.err= false;
  
} // end write_sectors


static void
cdrom_reset (
             drv_t *drv
             )
{
  
  memset ( drv->cdrom.sense_data, 0, sizeof(drv->cdrom.sense_data) );
  drv->cdrom.mode= CD_DEFAULT_MODE;
  drv->cdrom.busy= false;
  drv->cdrom.buflb.p= 0;
  drv->cdrom.buflb.l= 0;
  drv->cdrom.playing= false;
  drv->cdrom.paused= false;
  drv->cdrom.locked= false;
  drv->cdrom.audio.p= 0;
  drv->cdrom.audio.l= 0;
  drv->cdrom.audio.status= CD_AUDIO_STATUS_NONE;
  memset ( drv->cdrom.subchn_Q, 0, sizeof(drv->cdrom.subchn_Q) );
  
} // end cdrom_reset


static void
cdrom_set_sense (
                 drv_t         *drv,
                 const uint8_t  sense_key,
                 const uint16_t add_sense
                 )
{

  int i;

  
  drv->cdrom.sense_data[0]= 0x80 | 0x70;
  drv->cdrom.sense_data[1]= 0x00;
  // Information
  for ( i= 3; i <= 6; ++i ) drv->cdrom.sense_data[i]= 0x00;
  drv->cdrom.sense_data[2]= sense_key;
  drv->cdrom.sense_data[7]= 18-7;
  // Command inf
  for ( i= 8; i <= 11; ++i ) drv->cdrom.sense_data[i]= 0x00;
  drv->cdrom.sense_data[12]= (uint8_t) (add_sense>>8);
  drv->cdrom.sense_data[13]= (uint8_t) (add_sense);
  // Altres
  for ( i= 14; i <= 17; ++i ) drv->cdrom.sense_data[i]= 0x00;
  drv->cdrom.sense_data[18]= 0x00; // No entra.
  
} // end cdrom_set_sense


static void
cdrom_abort (
             const int      ide,
             drv_t         *drv,
             const uint8_t  sense_key,
             const uint16_t add_sense
             )
{

  cdrom_set_sense ( drv, sense_key, add_sense );
  _dev[ide].error= (sense_key<<4) | 0x04; // ABRT
  _dev[ide].sector_count&= 0xf8;
  _dev[ide].sector_count|= 0x03; // Set I/O i C/D
  drv->stat.bsy= false;
  drv->stat.rdy= false;
  drv->stat.df= false;
  drv->stat.srv= true; // ??? No suporte overlap.
  drv->stat.drq= false;
  drv->stat.err= true; // CHK
  drv->cdrom.busy= false;
  drv->pio_transfer.mode= PT_NORMAL;
  drv->intrq= true;
  update_irq ();
  
} // end cdrom_abort


static bool
cdrom_check_is_ready (
                      const int   ide,
                      drv_t      *drv,
                      const bool  check_media
                      )
{

  if ( drv->cdrom.cd->current == NULL && check_media )
    {
      cdrom_abort ( ide, drv, CD_SENSE_KEY_NOT_READY,
                    CD_ADD_SENSE_MEDIUM_NOT_PRESENT );
      return false;
    }
  if ( drv->cdrom.busy )
    {
      _warning ( _udata, "s'ha intentat executar un comandament en "
                 "IDE%d.%d mentre està busy",
                 ide, _dev[ide].ind );
      cdrom_abort ( ide, drv, CD_SENSE_KEY_NOT_READY,
                    CD_ADD_SENSE_OVERLAPPED_COMMANDS );
      return false;
    }
  
  return true;
  
} // end cdrom_check_is_ready


// NOTA!! Ha d'estar ready.
static bool
cdrom_seek (
            const int       ide,
            drv_t          *drv,
            const uint32_t  lb_addr
            )
{
  
  uint32_t offset;
  int amm,ass,asect,i;
  const CD_Info *info;
  
  
  // IMPORTANT!!! Molts CDs comencen les dades a partir del segon 2
  // (pot ser un error meu). No tinc clar a què fa referència
  // lb_addr. Opcions:
  //
  //  1) Local al track 0
  //  2) Absolut però ignore eixos 2s inicials que no tinc clar com van
  //  3) Absolut incloent els 2s inicials
  //
  // He decidit apostar pel (2).
  
  assert ( drv->cdrom.cd->current != NULL );

  // Calcula offset inicial.
  offset= 0;
  info= drv->cdrom.cd->info;
  if ( info->ntracks > 0 && info->tracks[0].nindexes > 0 )
    for ( i= 0; i < info->tracks[0].nindexes; i++ )
      if ( info->tracks[0].indexes[i].id == 1 )
        {
          offset= cdpos2offset ( info->tracks[0].indexes[i].pos );
          break;
        }
  
  // Calcula posició.
  offset+= lb_addr*drv->cdrom.mode.sectors_per_block;
  amm= (int) (offset/(60*75));
  offset%= 60*75;
  ass= (int) (offset/75);
  asect= (int) (offset%75);
  
  // Fa seek.
  if ( !CD_disc_seek ( drv->cdrom.cd->current, amm, ass, asect ) )
    {
      cdrom_abort ( ide, drv, CD_SENSE_KEY_MEDIUM_ERROR,
                    CD_ADD_SENSE_NO_SEEK_COMPLETE );
      return false;
    }
  drv->cdrom.buflb.p= 0;
  drv->cdrom.buflb.l= 0;
  
  return true;
  
} // end cdrom_seek


// NOTA!! Ha d'estar ready.
static bool
cdrom_readlb (
              const int  ide,
              drv_t     *drv
              )
{

  static uint8_t buf[CD_SEC_SIZE];
  
  bool audio,crc_ok;
  unsigned int i;
  uint8_t mode;
  
  
  assert ( drv->cdrom.cd->current != NULL );

  if ( drv->cdrom.mode.lblock_size != 2048 )
    {
      PC_MSG("piix4_ide.c - cdrom_readlb: Logical Block Size != 2048");
      exit(EXIT_FAILURE);
    }

  // Llig
  if ( !CD_disc_read_q ( drv->cdrom.cd->current, drv->cdrom.subchn_Q,
                         &crc_ok, false ) )
    {
      cdrom_abort ( ide, drv, CD_SENSE_KEY_MEDIUM_ERROR,
                    CD_ADD_SENSE_CAN_NOT_READ_UNK_FORMAT );
      return false;
    }
  if ( !CD_disc_read ( drv->cdrom.cd->current, buf, &audio, true ) )
    {
      cdrom_abort ( ide, drv, CD_SENSE_KEY_MEDIUM_ERROR,
                    CD_ADD_SENSE_CAN_NOT_READ_UNK_FORMAT );
      return false;
    }
  
  // Còpia.
  if ( audio )
    {
      PC_MSG("piix4_ide.c - cdrom_readlb: audio sector");
      exit(EXIT_FAILURE);
    }
  mode= buf[15];
  if ( mode == 1 )
    {
      for ( i= 16; i < 2064; i++ )
        drv->cdrom.buflb.v[i-16]= buf[i];
      drv->cdrom.buflb.p= 0;
      drv->cdrom.buflb.l= 2048;
    }
  else if ( mode == 2 )
    {
      if ( (buf[0x12]&0x20) == 0 ) // Form 1
        {
          for ( i= 0x18; i < 0x818; i++ )
            drv->cdrom.buflb.v[i-0x18]= buf[i];
          drv->cdrom.buflb.p= 0;
          drv->cdrom.buflb.l= 2048;
        }
      else
        {
          PC_MSG("piix4_ide.c - cdrom_readlb: FORM 2");
          exit(EXIT_FAILURE);
        }
    }
  else
    {
      PC_MSGF("piix4_ide.c - cdrom_readlb: mode %02X",mode);
      exit(EXIT_FAILURE);
    }
  
  return true;
  
} // end cdrom_readlb


static void
cdrom_stop_playing (
                    drv_t *drv,
                    int    new_status
                    )
{
  
  drv->cdrom.playing= false;
  drv->cdrom.paused= false;
  drv->cdrom.audio.status= new_status;
  //drv->cdrom.busy= false;
  
} // end cdrom_stop_playing


static bool
cdrom_check_range (
                   const int  ide,
                   drv_t     *drv,
                   const MSF  start,
                   const MSF  end
                   )
{

  MSF last_pos;
  const CD_Info *info;
  
  
  assert ( drv->cdrom.cd->current != NULL );
  
  // Comprova que els MSF siguen correctes
  if ( !MSF_IS_VALID ( start ) ) goto error;
  if ( !MSF_IS_VALID ( end ) ) goto error;

  // Comprova els límits.
  if ( cmp_msf ( start, end ) == 1 ) goto error;
  info= drv->cdrom.cd->info;
  last_pos= cdpos2msf ( info->tracks[info->ntracks-1].pos_last_sector );
  inc_msf ( &last_pos );
  if ( cmp_msf ( end, last_pos ) == 1 ) goto error;
  
  return true;
  
 error:
  cdrom_abort ( ide, drv, CD_SENSE_KEY_ILLEGAL_REQUEST,
                CD_ADD_SENSE_PARAMETER_VALUE_INVALID );
  return false;
  
} // end cdrom_check_msf_range


static void
cdrom_start_playing (
                     drv_t     *drv,
                     const MSF  start,
                     const MSF  end
                     )
{

  drv->cdrom.busy= 0;
  drv->cdrom.playing= true;
  drv->cdrom.paused= false;
  drv->cdrom.audio.p= 0;
  drv->cdrom.audio.l= 0;
  drv->cdrom.audio.current= start;
  drv->cdrom.audio.end= end;
  drv->cdrom.audio.status= CD_AUDIO_STATUS_IN_PROGRESS;
  
} // end cdrom_start_playing


// Torna cert si tot ha anat bé.
static bool
cdrom_play_load_next_sector (
                             const int  ide,
                             drv_t     *drv
                             )
{

  bool audio,crc_ok;

  
  if ( _sound_dev->cdrom.mode.cdrom_audio_control_parameters.sotc )
    {
      PC_MSG ( "CDROM - SOTC==true" );
      exit ( EXIT_FAILURE );
    }
  
  // Comprova que tenim CD
  if ( drv->cdrom.cd->current == NULL )
    {
      _warning ( _udata,
                 "no es pot continuar reproduint el CD perquè "
                 "s'ha expulsat inesperadament" );
      cdrom_abort ( ide, drv, CD_SENSE_KEY_NOT_READY,
                    CD_ADD_SENSE_MEDIUM_NOT_PRESENT );
      goto stop;
    }

  // Comprova que no hem aplegat al final.
  // En realitat no té sentit que siga > (1), però per si de cas
  if ( cmp_msf ( drv->cdrom.audio.current, drv->cdrom.audio.end ) != -1 )
    {
      if ( !_sound_dev->cdrom.mode.cdrom_audio_control_parameters.immed )
        {
          PC_MSG ( "CDROM - Immed==false" );
          exit ( EXIT_FAILURE );
        }
      goto success;
    }

  // Llig.
  if ( !CD_disc_seek ( drv->cdrom.cd->current,
                       (int) (drv->cdrom.audio.current.m),
                       (int) (drv->cdrom.audio.current.s),
                       (int) (drv->cdrom.audio.current.f) ) )
    {
      cdrom_abort ( ide, drv, CD_SENSE_KEY_MEDIUM_ERROR,
                    CD_ADD_SENSE_NO_SEEK_COMPLETE );
      goto stop;
    }
  if ( !CD_disc_read_q ( drv->cdrom.cd->current, drv->cdrom.subchn_Q,
                         &crc_ok, false ) )
    {
      cdrom_abort ( ide, drv, CD_SENSE_KEY_MEDIUM_ERROR,
                    CD_ADD_SENSE_CAN_NOT_READ_UNK_FORMAT );
      return false;
    }
  if ( !CD_disc_read ( drv->cdrom.cd->current,
                       drv->cdrom.audio.v, &audio, false ) )
    {
      cdrom_abort ( ide, drv, CD_SENSE_KEY_MEDIUM_ERROR,
                    CD_ADD_SENSE_CAN_NOT_READ_UNK_FORMAT );
      goto stop;
    }
  if ( !audio )
    {
      cdrom_abort ( ide, drv, CD_SENSE_KEY_ILLEGAL_REQUEST,
                    CD_ADD_SENSE_ILLEGAL_MODE_FOR_THIS_TRACK );
      goto stop;
    }
  inc_msf ( &(drv->cdrom.audio.current) );
  drv->cdrom.audio.p= 0;
  drv->cdrom.audio.l= CD_SEC_SIZE;
  
  return true;

 success:
  cdrom_stop_playing ( _sound_dev, CD_AUDIO_STATUS_COMPLETED );
  return false;
 stop:
  cdrom_stop_playing ( _sound_dev, CD_AUDIO_STATUS_ERROR );
  return false;
  
} // end cdrom_play_load_next_sector


static uint8_t
cdrom_get_media_type (
                      const drv_t *drv
                      )
{

  uint8_t ret;


  // NOTA!!! No tinc door open emulat. Tampoc emule mini-disc.
  if ( drv->cdrom.cd->current == NULL )
    ret= 0x70; // Door closed, no disc present
  else
    {
      switch ( drv->cdrom.cd->info->type )
        {
          
          // 120 mm CD-DA audio only, door closed or caddy inserted
        case CD_DISK_TYPE_AUDIO:
          ret= 0x02;
          break;
          
          // 120 mm CD-ROM data only, door closed or caddy inserted
        case CD_DISK_TYPE_MODE1:
        case CD_DISK_TYPE_MODE2:
          ret= 0x01;
          break;
          
          // 120 mm CD-ROM data and audio combined, door closed or
          // caddy inserted
        case CD_DISK_TYPE_MODE1_AUDIO:
        case CD_DISK_TYPE_MODE2_AUDIO:
          ret= 0x03;
          break;

          // Desconegut.
        case CD_DISK_TYPE_UNK:
          ret= 0x00;
          break;

        default:
          PC_MSGF("piix4_ide.c - cdrom_get_media_type"
                  " - tipus de CD desconegut: %d\n",drv->cdrom.cd->info->type);
          ret= 0x00;
          
        }
    }

  return ret;
  
} // end cdrom_get_media_type


static void
cdrom_write_select_data (
                         const int  ide,
                         drv_t     *drv
                         )
{

  const uint8_t *data;
  int remain, page_length;
  uint8_t page_code;
  cdrom_mode_t *mode;

  
  // Prepara.
  data= (const uint8_t *) &drv->pio_transfer.buf;
  remain= drv->pio_transfer.packet_byte_count;
  mode= &(drv->cdrom.mode);

  // Ignora capçalera.
  if ( remain < 8 )
    {
      cdrom_abort ( ide, drv, CD_SENSE_KEY_ILLEGAL_REQUEST,
                    CD_ADD_SENSE_PARAMETER_LIST_LENGTH_ERROR );
      return;
    }
  remain-= 8;
  data+= 8;
  
  // Processa pàgines
  while ( remain >= 2 )
    {

      // Capçalera pàgina.
      page_code= data[0]&0x3F;
      page_length= (int) ((unsigned int) data[1]);
      data+= 2;
      remain-= 2;
      if ( page_length > remain )
        {
          cdrom_abort ( ide, drv, CD_SENSE_KEY_ILLEGAL_REQUEST,
                        CD_ADD_SENSE_PARAMETER_LIST_LENGTH_ERROR );
          return;
        }
      remain-= page_length;

      // Processa capçalera
      switch ( page_code )
        {

        case 0x0e: // CD-Rom audio control page
          if ( page_length < 14 )
            {
              cdrom_abort ( ide, drv, CD_SENSE_KEY_ILLEGAL_REQUEST,
                            CD_ADD_SENSE_PARAMETER_LIST_LENGTH_ERROR );
              return;
            }
          else if ( page_length > 14 )
            {
              PC_MSG("piix_ide - cdrom_write_select_data"
                     " - PAGE_CODE=0E no suporta grandària != 14!!");
              exit ( EXIT_FAILURE );
            }
          mode->cdrom_audio_control_parameters.immed= (data[0]&0x04)!=0;
          mode->cdrom_audio_control_parameters.sotc= (data[0]&0x02)!=0;
          mode->cdrom_audio_control_parameters.chn_port0= data[6]&0xF;
          mode->cdrom_audio_control_parameters.vol_port0= data[7];
          mode->cdrom_audio_control_parameters.chn_port1= data[8]&0xF;
          mode->cdrom_audio_control_parameters.vol_port1= data[9];
          break;
          
        default:
          PC_MSGF("piix_ide - cdrom_write_select_data"
                  " - PAGE_CODE=%02X!!",page_code);
          exit(EXIT_FAILURE);
        }
      data+= page_length;
      
    }

  cd_successful_command_completion ( ide, drv, false );
  
} // end cdrom_write_select_data


static void
cd_successful_command_completion (
                                  const int   ide,
                                  drv_t      *drv,
                                  const bool  cdrom_busy
                                  )
{
  
  _dev[ide].sector_count&= 0xf8;
  _dev[ide].sector_count|= 0x03; // Set I/O i C/D
  drv->stat.bsy= false;
  drv->stat.rdy= false;
  drv->stat.srv= true;
  drv->stat.drq= false;
  drv->stat.err= false; // CHK
  drv->cdrom.busy= cdrom_busy;
  drv->intrq= true;
  update_irq ();
  
} // end cd_successful_command_completion


static void
cd_test_unit_ready (
                    const int  ide,
                    drv_t     *drv
                    )
{
  
  assert ( drv->type == PC_IDE_DEVICE_TYPE_CDROM );
  assert ( drv->cdrom.cd != NULL );

  // Not ready.
  if ( drv->cdrom.cd->current == NULL || drv->cdrom.busy )
    {

      // SENSE
      cdrom_set_sense ( drv, CD_SENSE_KEY_NOT_READY,
                        drv->cdrom.busy ? CD_ADD_SENSE_OVERLAPPED_COMMANDS :
                        CD_ADD_SENSE_MEDIUM_NOT_PRESENT );
      
      // ATA
      _dev[ide].error= (CD_SENSE_KEY_NOT_READY<<4);
      _dev[ide].sector_count&= 0xf8;
      _dev[ide].sector_count|= 0x03; // Set I/O i C/D
      drv->stat.bsy= false;
      drv->stat.rdy= false;
      drv->stat.df= false;
      drv->stat.srv= false; // ??? No suporte overlap.
      drv->stat.drq= false;
      drv->stat.err= true; // CHK
      drv->intrq= true;
      update_irq ();
      
    }
  else cd_successful_command_completion ( ide, drv, false );
  
} // end cd_test_unit_ready


static void
read_cdlb_iter (
                const int  ide,
                drv_t     *drv
                )
{

  bool available,data_empty;
  int nbytes;
  unsigned int i;
  uint16_t data;
  
  
  // Comprova disponibilitat
  available=
    (drv->cdrom.buflb.p < drv->cdrom.buflb.l) ||
    (drv->pio_transfer.cdlb.remain > 0)
    ;
  if ( !available )
    {
      drv->pio_transfer.mode= PT_NORMAL;
      // SI NO HO INTERPRETE MAL!! En el Packet sí que hi ha un IRQ
      // adicional per a indicar comandament finalitzat correctament.
      cd_successful_command_completion ( ide, drv, false );
      return;
    }

  // Intena omplir buffer PIO
  nbytes= 0;
  data_empty= true;
  data= 0x0000;
  drv->pio_transfer.begin= 0;
  drv->pio_transfer.end= 0;
  while ( nbytes < drv->pio_transfer.cdlb.byte_count && available )
    {

      // Si no queden bytes carrega
      if ( drv->cdrom.buflb.p >= drv->cdrom.buflb.l )
        {
          if ( drv->pio_transfer.cdlb.remain > 0 )
            {
              if ( !cdrom_readlb ( ide, drv ) )
                return;
              --drv->pio_transfer.cdlb.remain;
            }
          else available= false;
        }

      // Còpia
      for ( i= drv->cdrom.buflb.p;
            i != drv->cdrom.buflb.l &&
              nbytes < drv->pio_transfer.cdlb.byte_count;
            i++, nbytes++ )
        {
          if ( data_empty ) data= (uint16_t) (drv->cdrom.buflb.v[i]);
          else
            {
              data|= ((uint16_t) (drv->cdrom.buflb.v[i]))<<8;
              drv->pio_transfer.buf[drv->pio_transfer.end++]= data;
            }
          data_empty= !data_empty;
        }
      drv->cdrom.buflb.p= i;
      
    }
  if ( !data_empty ) drv->pio_transfer.buf[drv->pio_transfer.end++]= data;
  assert ( drv->pio_transfer.end > 0 );
  
  // Prepara comandament
  _dev[ide].sector_count&= 0xf8;
  _dev[ide].sector_count|= 0x02; // Set I/O - Transfer to host
  _dev[ide].addr.lbamid= (uint8_t) (nbytes&0xFF);
  _dev[ide].addr.lbahi= (uint8_t) (nbytes>>8);
  drv->stat.bsy= true;
  drv->stat.rdy= true;
  drv->stat.df= false; // DMRD (De moment no gaste DMA)
  // NOTA!! és important que DRQ
  // estiga a true també abans de la transferència. Es ficarà a false
  // quan es llisquen les dades.
  drv->stat.srv= false; // ??? No suporte overlap.
  drv->stat.drq= true;
  drv->stat.err= false; // CHK

  // Acaba de preparar transferència
  drv->pio_transfer.waiting= true;
  drv->pio_transfer.drq_value= true;
  drv->pio_transfer.remain_cc= (nbytes*_timing.ccpersector)/SEC_SIZE;
  if ( drv->pio_transfer.remain_cc == 0 )
    drv->pio_transfer.remain_cc= 1;
  
} // end read_cdlb_iter


static void
cd_read64K (
            const int      ide,
            drv_t         *drv,
            const uint8_t *packet
            )
{
  
  uint32_t lb_addr;
  uint16_t lb_length;

  
  assert ( drv->type == PC_IDE_DEVICE_TYPE_CDROM );
  assert ( drv->cdrom.cd != NULL );
  if ( !cdrom_check_is_ready ( ide, drv, true ) ) return;
  
  // Obté arguments.
  lb_addr=
    (((uint32_t) packet[2])<<24) |
    (((uint32_t) packet[3])<<16) |
    (((uint32_t) packet[4])<<8) |
    ((uint32_t) packet[5])
    ;
  lb_length= (((uint16_t) packet[7])<<8) | ((uint16_t) packet[8]);
  if ( lb_length == 0 )
    {
      cdrom_abort ( ide, drv, CD_SENSE_KEY_ILLEGAL_REQUEST,
                    CD_ADD_SENSE_PARAMETER_VALUE_INVALID );
      return;
    }
  
  // Llig.
  if ( !cdrom_seek ( ide, drv, lb_addr ) ) return;
  drv->pio_transfer.mode= PT_READ_CDLB;
  drv->pio_transfer.cdlb.remain= lb_length;
  drv->pio_transfer.cdlb.byte_count= 
    (((uint16_t) _dev[ide].addr.lbamid) |
     (((uint16_t) _dev[ide].addr.lbahi)<<8))&0xFFFE
    ;
  if ( drv->pio_transfer.cdlb.byte_count == 0 )
    {
      _warning ( _udata, "s'ha intentat executar READ (10) en "
                 "IDE%d.%d amb byte_count&0xFFFE == 0",
                 ide, _dev[ide].ind );
      cdrom_abort ( ide, drv, CD_SENSE_KEY_ILLEGAL_REQUEST,
                    CD_ADD_SENSE_PARAMETER_VALUE_INVALID );
      return;
    }
  drv->cdrom.buflb.p= 0;
  drv->cdrom.buflb.l= 0;
  read_cdlb_iter ( ide, drv );
  
} // end cd_read64K


static void
cd_transfer_finish (
                    const int  ide,
                    drv_t     *drv
                    )
{

  drv->pio_transfer.mode= PT_NORMAL;
  cd_successful_command_completion ( ide, drv, false );
  
} // end cd_transfer_finish


static void
cd_request_sense (
                  const int      ide,
                  drv_t         *drv,
                  const uint8_t *packet
                  )
{
  
  int i;
  uint8_t length;

  
  assert ( drv->type == PC_IDE_DEVICE_TYPE_CDROM );
  assert ( drv->cdrom.cd != NULL );
  if ( !cdrom_check_is_ready ( ide, drv, false ) ) return;
  
  // Preara mode lectura
  drv->pio_transfer.mode= PT_READ_CD;
  if ( drv->pio_transfer.packet_byte_count == 0 )
    {
      _warning ( _udata, "s'ha intentat executar REQUEST SENSE en "
                 "IDE%d.%d amb byte_count == 0",
                 ide, _dev[ide].ind );
      cdrom_abort ( ide, drv, CD_SENSE_KEY_ILLEGAL_REQUEST,
                    CD_ADD_SENSE_PARAMETER_VALUE_INVALID );
      return;
    }
  length= packet[4];
  if ( length > 18 ) length= 18;
  if ( drv->pio_transfer.packet_byte_count != length )
    {
      _warning ( _udata, "s'ha intentat executar REQUEST SENSE en "
                 "IDE%d.%d amb byte_count (%d) != %u",
                 ide, _dev[ide].ind, drv->pio_transfer.packet_byte_count,
                 length );
      cdrom_abort ( ide, drv, CD_SENSE_KEY_ILLEGAL_REQUEST,
                    CD_ADD_SENSE_PARAMETER_VALUE_INVALID );
      return;
    }
  drv->pio_transfer.begin= 0;
  drv->pio_transfer.end= (length+1)/2;
  for ( i= 0; i < 18; ++i )
    ((uint8_t *) &drv->pio_transfer.buf)[i]= drv->cdrom.sense_data[i];
  drv->pio_transfer.waiting= true;
  drv->pio_transfer.drq_value= true; // Crec que no cal perquè fique
                                     // el DRQ ja a true
  // NOTA!! No sé quan tarda així que ho faré proporcional.
  drv->pio_transfer.remain_cc=
    (drv->pio_transfer.packet_byte_count*_timing.ccpersector)/SEC_SIZE;
  if ( drv->pio_transfer.remain_cc == 0 )
    drv->pio_transfer.remain_cc= 1;
  
  // Prepara.
  _dev[ide].sector_count&= 0xf8;
  _dev[ide].sector_count|= 0x02; // Set I/O (to the host)
  drv->stat.bsy= false;
  drv->stat.df= false; // DMRD (De moment no gaste DMA)
  drv->stat.srv= false; // ??? No suporte overlap.
  drv->stat.drq= true; // Falta la transmissió.
  drv->stat.err= false; // CHK
  drv->cdrom.busy= true;

} // end cd_request_sense


static void
cd_inquiry (
            const int      ide,
            drv_t         *drv,
            const uint8_t *packet
            )
{
  
  uint8_t length;
  uint8_t *buf;

  
  assert ( drv->type == PC_IDE_DEVICE_TYPE_CDROM );
  assert ( drv->cdrom.cd != NULL );

  if ( (packet[1]&0x01) != 0 )
    {
      _warning ( _udata, "s'ha intentat executar INQUIRY en "
                 "IDE%d.%d amb EVPD!=0",
                 ide, _dev[ide].ind );
      cdrom_abort ( ide, drv, CD_SENSE_KEY_ILLEGAL_REQUEST,
                    CD_ADD_SENSE_INVALID_FIELD_IN_CDB );
      return;
    }
  
  // Preara mode lectura
  drv->pio_transfer.mode= PT_READ_CD;
  if ( drv->pio_transfer.packet_byte_count == 0 )
    {
      _warning ( _udata, "s'ha intentat executar INQUIRY en "
                 "IDE%d.%d amb byte_count == 0",
                 ide, _dev[ide].ind );
      cdrom_abort ( ide, drv, CD_SENSE_KEY_ILLEGAL_REQUEST,
                    CD_ADD_SENSE_PARAMETER_VALUE_INVALID );
      return;
    }
  length= packet[4];
  if ( length > 47 ) length= 47;
  if ( drv->pio_transfer.packet_byte_count != length )
    {
      _warning ( _udata, "s'ha intentat executar INQUIRY en "
                 "IDE%d.%d amb byte_count (%d) != %u",
                 ide, _dev[ide].ind, drv->pio_transfer.packet_byte_count,
                 length );
      cdrom_abort ( ide, drv, CD_SENSE_KEY_ILLEGAL_REQUEST,
                    CD_ADD_SENSE_PARAMETER_VALUE_INVALID );
      return;
    }
  drv->pio_transfer.begin= 0;
  drv->pio_transfer.end= (length+1)/2;
  buf= ((uint8_t *) &drv->pio_transfer.buf);
  buf[0]= 0x05; // Peripheral Qualifier??? Peripheral Device Type (05h)
  buf[1]= 0x80; // Removable
  buf[2]= 0x02; // ANSI Version (2)
  buf[3]= 0x02; // TIOP (Ficat a false), Response Format (2)
  buf[4]= 0x2a; // Additional Length
  buf[7]= 0x00; // SYNC (Ficat a false)
  // Vendor Name (8-byte ASCII code)
  buf[8]= 'M';
  buf[9]= 'E';
  buf[10]= 'M';
  buf[11]= 'U';
  buf[12]= 'P';
  buf[13]= 'C';
  buf[14]= ' ';
  buf[15]= ' ';
  // Product Inquiry Data (16-byte ASCII code)
  buf[16]= 'C';
  buf[17]= 'D';
  buf[18]= '-';
  buf[19]= 'R';
  buf[20]= 'O';
  buf[21]= 'M';
  buf[22]= ' ';
  buf[23]= 'B';
  buf[24]= 'A';
  buf[25]= 'S';
  buf[26]= 'I';
  buf[27]= 'C';
  buf[28]= ' ';
  buf[29]= ' ';
  buf[30]= ' ';
  buf[31]= ' ';
  // Revision Number (ASCII code) (“XXXX”)
  buf[32]= '0';
  buf[33]= '1';
  buf[34]= '0';
  buf[35]= '0';
  // Release Version (20h)
  buf[36]= 0x20;
  // Revision Date (10-byte ASCII code) (“YYYY/MM/DD”)
  buf[37]= '2';
  buf[38]= '0';
  buf[39]= '2';
  buf[40]= '3';
  buf[41]= '/';
  buf[42]= '1';
  buf[43]= '2';
  buf[44]= '/';
  buf[45]= '2';
  buf[46]= '3';
  // Padding
  buf[47]= 0x00;
  drv->pio_transfer.waiting= true;
  drv->pio_transfer.drq_value= true; // Crec que no cal perquè fique
                                     // el DRQ ja a true
  // NOTA!! No sé quan tarda així que ho faré proporcional.
  drv->pio_transfer.remain_cc=
    (drv->pio_transfer.packet_byte_count*_timing.ccpersector)/SEC_SIZE;
  if ( drv->pio_transfer.remain_cc == 0 )
    drv->pio_transfer.remain_cc= 1;
  
  // Prepara.
  _dev[ide].sector_count&= 0xf8;
  _dev[ide].sector_count|= 0x02; // Set I/O (to the host)
  drv->stat.bsy= false;
  drv->stat.df= false; // DMRD (De moment no gaste DMA)
  drv->stat.srv= false; // ??? No suporte overlap.
  drv->stat.drq= true; // Falta la transmissió.
  drv->stat.err= false; // CHK
  drv->cdrom.busy= true;
  
} // end cd_inquiry


static void
cd_mode_sense (
               const int      ide,
               drv_t         *drv,
               const uint8_t *packet,
               const bool     is10bytes
               )
{
  
  int pos,sense_length;
  uint16_t length;
  uint8_t pcf,page_code;
  uint8_t *data;
  const cdrom_mode_t *mode;
  
  
  assert ( drv->type == PC_IDE_DEVICE_TYPE_CDROM );
  assert ( drv->cdrom.cd != NULL );
  if ( !cdrom_check_is_ready ( ide, drv, false ) ) return;

  // Extrau arguments
  length=
    is10bytes ?
    ((((uint16_t) packet[7])<<8) | ((uint16_t) packet[8])) :
    ((uint16_t) packet[4])
    ;
  pcf= packet[2]>>5;
  page_code= packet[2]&0x3f;

  // Comparació byte_count i allocation_length
  if ( drv->pio_transfer.packet_byte_count != length )
    {
      _warning ( _udata, "s'ha intentat executar MODE SENSE en "
                 "IDE%d.%d amb byte_count (%d) != %u",
                 ide, _dev[ide].ind, drv->pio_transfer.packet_byte_count,
                 length );
      cdrom_abort ( ide, drv, CD_SENSE_KEY_ILLEGAL_REQUEST,
                    CD_ADD_SENSE_PARAMETER_VALUE_INVALID );
      return;
    }

  // Cas especial quan length == 0 (No retorna res)
  if ( length == 0 )
    {
      cd_successful_command_completion ( ide, drv, false );
      return;
    }

  // Prepara dades
  data= ((uint8_t *) &drv->pio_transfer.buf);
  pos= 0;
  // IMPORTANT!!! Sobre què ha d'apareixer a la capçalera hi ha
  // polèmica segons el manual!
  // --> Data Header
  pos= 2;
  data[pos++]= cdrom_get_media_type ( drv );
  data[pos++]= 0x00; // Reserved
  data[pos++]= 0x00; // Reserved
  data[pos++]= 0x00; // Reserved
  data[pos++]= 0x00; // Reserved
  data[pos++]= 0x00; // Reserved
  // --> Page(s)
  if ( page_code != 0x00 )
    {

      // Page control field
      switch ( pcf )
        {
        case 0: mode= &drv->cdrom.mode; break;
        case 1:
          PC_MSG("piix_ide - cd_mode_sense - PCF=1 not implemented!!");
          exit(EXIT_FAILURE);
          break;
        case 2: mode= &CD_DEFAULT_MODE; break;
        case 3:
          _warning ( _udata, "s'ha intentat executar MODE SENSE en "
                     "IDE%d.%d amb PCF=3", ide, _dev[ide].ind );
          cdrom_abort ( ide, drv, CD_SENSE_KEY_ILLEGAL_REQUEST,
                        CD_ADD_SENSE_PARAMETER_VALUE_INVALID );
          return;
        default: mode= NULL; // CALLA
        }

      // Escriu pages
      switch ( page_code )
        {
        case 0x0d: // CD-Rom Parameters
          data[pos++]= 0x0d;
          data[pos++]= 0x06;
          data[pos++]= 0x00; // Reserved
          data[pos++]= mode->cdrom_parameters.itm;
          data[pos++]= 0x00;
          data[pos++]= mode->cdrom_parameters.MSFS_per_MSFM;
          data[pos++]= 0x00;
          data[pos++]= mode->cdrom_parameters.MSFF_per_MSFS;
          break;
        case 0x0e: // CD-Rom audio control page
          data[pos++]= 0x0e;
          data[pos++]= 0x0e;
          data[pos++]=
            (mode->cdrom_audio_control_parameters.immed ? 0x04 : 0x00) |
            (mode->cdrom_audio_control_parameters.sotc ? 0x02 : 0x00)
            ;
          data[pos++]= 0x00; // Reserved
          data[pos++]= 0x00; // Reserved
          data[pos++]= 0x00; // Reserved
          data[pos++]= 0x00; // LBA per second of Audio Playback???
          data[pos++]= 0x00;
          data[pos++]= mode->cdrom_audio_control_parameters.chn_port0;
          data[pos++]= mode->cdrom_audio_control_parameters.vol_port0;
          data[pos++]= mode->cdrom_audio_control_parameters.chn_port1;
          data[pos++]= mode->cdrom_audio_control_parameters.vol_port1;
          data[pos++]= 0x00; // Reserved
          data[pos++]= 0x00; // Reserved
          data[pos++]= 0x00; // Reserved
          data[pos++]= 0x00; // Reserved
          break;
        default:
          PC_MSGF("piix_ide - cd_mode_sense - PAGE_CODE=%02X!!",
                 page_code);
          exit(EXIT_FAILURE);
        }
      // --> Sense length
      sense_length= pos-2;
      data[0]= (uint8_t) ((sense_length>>8)&0xff);
      data[1]= (uint8_t) (sense_length&0xff);
      
    }
  
  // Prepara mode lectura
  drv->pio_transfer.mode= PT_READ_CD;
  drv->pio_transfer.begin= 0;
  if ( pos < length ) length= pos;
  if ( drv->pio_transfer.packet_byte_count != length )
    {
      _warning ( _udata, "s'ha intentat executar MODE SENSE en "
                 "IDE%d.%d amb byte_count (%d) != %u (paquet)",
                 ide, _dev[ide].ind, drv->pio_transfer.packet_byte_count,
                 length );
      cdrom_abort ( ide, drv, CD_SENSE_KEY_ILLEGAL_REQUEST,
                    CD_ADD_SENSE_PARAMETER_VALUE_INVALID );
      return;
    }
  drv->pio_transfer.end= (length+1)/2;
  drv->pio_transfer.waiting= true;
  drv->pio_transfer.drq_value= true; // Crec que no cal perquè fique
                                     // el DRQ ja a true
  // NOTA!! No sé quan tarda així que ho faré proporcional.
  drv->pio_transfer.remain_cc=
    (drv->pio_transfer.packet_byte_count*_timing.ccpersector)/SEC_SIZE;
  if ( drv->pio_transfer.remain_cc == 0 )
    drv->pio_transfer.remain_cc= 1;

  // Prepara.
  _dev[ide].sector_count&= 0xf8;
  _dev[ide].sector_count|= 0x02; // Set I/O (to the host)
  drv->stat.bsy= false;
  drv->stat.df= false; // DMRD (De moment no gaste DMA)
  drv->stat.srv= false; // ??? No suporte overlap.
  drv->stat.drq= true; // Falta la transmissió.
  drv->stat.err= false; // CHK
  drv->cdrom.busy= true;
  
} // end cd_mode_sense


static void
cd_read_toc (
             const int      ide,
             drv_t         *drv,
             const uint8_t *packet
             )
{
  
  int pos,i,j;
  uint16_t length,toc_length;
  uint8_t start_track,format,adr_control;
  uint8_t *data;
  const CD_TrackInfo *tracki;
  const CD_IndexInfo *indexi;
  CD_Position cdpos;
  uint32_t offset;
  bool use_msf,crc_ok;
  MSF msf;

  
  assert ( drv->type == PC_IDE_DEVICE_TYPE_CDROM );
  assert ( drv->cdrom.cd != NULL );
  if ( !cdrom_check_is_ready ( ide, drv, true ) ) return;

  // Extrau arguments
  use_msf= (packet[1]&0x02)!=0;
  length= ((((uint16_t) packet[7])<<8) | ((uint16_t) packet[8]));
  start_track= packet[6];
  format= packet[9]>>6;
  
  // Comparació byte_count i allocation_length
  if ( length == 0 )
    {
      cdrom_abort ( ide, drv, CD_SENSE_KEY_ILLEGAL_REQUEST,
                    CD_ADD_SENSE_PARAMETER_VALUE_INVALID );
      return;
    }
  if ( drv->pio_transfer.packet_byte_count != length )
    {
      _warning ( _udata, "s'ha intentat executar READ TOC en "
                 "IDE%d.%d amb byte_count (%d) != %u",
                 ide, _dev[ide].ind, drv->pio_transfer.packet_byte_count,
                 length );
      cdrom_abort ( ide, drv, CD_SENSE_KEY_ILLEGAL_REQUEST,
                    CD_ADD_SENSE_PARAMETER_VALUE_INVALID );
      return;
    }
  
  // Comprova el start_track
  if ( format == 1 && start_track != 0 )
    {
      _warning ( _udata, "s'ha intentat executar READ TOC en "
                 "IDE%d.%d amb START_TRACK=%d i FORMAT=1",
                 ide, _dev[ide].ind, start_track,
                 length );
      cdrom_abort ( ide, drv, CD_SENSE_KEY_ILLEGAL_REQUEST,
                    CD_ADD_SENSE_PARAMETER_VALUE_INVALID );
      return;
    }
  if ( format != 1 )
    {
      if ( start_track == 0 ) start_track= 1;
      if ( start_track > drv->cdrom.cd->info->ntracks )
        {
          _warning ( _udata, "s'ha intentat executar READ TOC en IDE%d.%d amb "
                     "START_TRACK=%d fora de rang (1-%d)",
                     ide, _dev[ide].ind, start_track,
                     drv->cdrom.cd->info->ntracks );
          cdrom_abort ( ide, drv, CD_SENSE_KEY_ILLEGAL_REQUEST,
                        CD_ADD_SENSE_INVALID_FIELD_IN_CDB );
          return;
        }
    }

  // Simula l'accés per a llegir el toc i obtindre el Q.
  if ( !CD_disc_seek ( drv->cdrom.cd->current, 0, 1, 74 ) )
    {
      cdrom_abort ( ide, drv, CD_SENSE_KEY_MEDIUM_ERROR,
                    CD_ADD_SENSE_NO_SEEK_COMPLETE );
      return;
    }
  if ( !CD_disc_read_q ( drv->cdrom.cd->current, drv->cdrom.subchn_Q,
                         &crc_ok, false ) )
    {
      cdrom_abort ( ide, drv, CD_SENSE_KEY_MEDIUM_ERROR,
                    CD_ADD_SENSE_CAN_NOT_READ_UNK_FORMAT );
      return;
    }
  
  // Prepara dades
  data= ((uint8_t *) &drv->pio_transfer.buf);
  pos= 0;
  switch ( format )
    {
    case 0: // SCSI-2
      // --> Header
      pos= 2; // TOC Data Length al final
      data[pos++]= start_track;
      data[pos++]= (uint8_t) drv->cdrom.cd->info->ntracks;
      // --> Track Descriptors
      for ( i= (int) (start_track-1); i < drv->cdrom.cd->info->ntracks; i++ )
        {
          tracki= &drv->cdrom.cd->info->tracks[i];
          data[pos++]= 0x00; // Reserved
          adr_control=
            0x10 |
            (tracki->audio_four_channel ? 0x08 : 0x00) |
            (tracki->is_audio ? 0x00 : 0x04) |
            (tracki->digital_copy_allowed ? 0x02 : 0x00) |
            (tracki->audio_preemphasis ? 0x01 : 0x00)
            ;
          data[pos++]= adr_control;
          data[pos++]= (uint8_t) (i+1);
          data[pos++]= 0x00; // Reserved
          assert ( tracki->nindexes > 0 );
          for ( indexi= NULL, j= 0; j < tracki->nindexes; j++ )
            if ( tracki->indexes[j].id == 1 )
              indexi= &(tracki->indexes[j]);
          assert ( indexi != NULL );
          if ( use_msf )
            {
              msf= cdpos2msf ( indexi->pos );
              data[pos++]= 0x00;
              data[pos++]= msf.m;
              data[pos++]= msf.s;
              data[pos++]= msf.f;
            }
          else // Si és LBA no es té en compte el Lead-In.
            {
              offset= cdpos2offset ( indexi->pos );
              if ( offset < 150 )
                {
                  _warning ( _udata, "READ TOC en IDE%d.%d: l'offset (%u) del "
                             "TRACK=%02X és menor que <150 (es fixa a 0)",
                             ide, _dev[ide].ind, offset, tracki->id );
                  offset= 0;
                }
              else offset-= 150; // No es té en compte el Lead-In
              data[pos++]= (uint8_t) ((offset>>24)&0xff);
              data[pos++]= (uint8_t) ((offset>>16)&0xff);
              data[pos++]= (uint8_t) ((offset>>8)&0xff);
              data[pos++]= (uint8_t) (offset&0xff);
            }
        }
      // --> TOC Data length
      toc_length= pos-2;
      data[0]= (uint8_t) ((toc_length>>8)&0xff);
      data[1]= (uint8_t) (toc_length&0xff);
      break;
    case 1: // Sessions
      if ( drv->cdrom.cd->info->nsessions > 1 )
        {
          PC_MSG("piix_ide - cd_read_toc -"
                 " FORMAT=1 not implementat per a multisession!!");
          exit(EXIT_FAILURE);
        }
      // --> Header
      pos= 2; // TOC Data Length al final
      data[pos++]= 1; // First Session Number
      data[pos++]= 1; // Last Session Number
      // --> TOC Track Descriptor (En realitat 1 per a l'última sessió??)
      data[pos++]= 0x00; // Reserved
      // D'on apareix la informació??? Primer track última sessió???
      // NO ESTÀ CLAR
      tracki= &drv->cdrom.cd->info->tracks[0];
      adr_control= 
            0x10 |
            (tracki->audio_four_channel ? 0x08 : 0x00) |
            (tracki->is_audio ? 0x00 : 0x04) |
            (tracki->digital_copy_allowed ? 0x02 : 0x00) |
            (tracki->audio_preemphasis ? 0x01 : 0x00)
            ;
      data[pos++]= adr_control;
      data[pos++]= 1; // First track number in last complete session
      data[pos++]= 0; // Reserved
      // Logical Block Address of First Track in Last Session
      assert ( tracki->nindexes > 0 );
      for ( indexi= NULL, j= 0; j < tracki->nindexes; j++ )
        if ( tracki->indexes[j].id == 1 )
          indexi= &(tracki->indexes[j]);
      assert ( indexi != NULL );
      if ( use_msf )
        {
          msf= cdpos2msf ( indexi->pos );
          data[pos++]= 0x00;
          data[pos++]= msf.m;
          data[pos++]= msf.s;
          data[pos++]= msf.f;
        }
      else // Si és LBA no es té en compte el Lead-In.
        {
          offset= cdpos2offset ( indexi->pos );
          if ( offset < 150 )
            {
              _warning ( _udata, "READ TOC en IDE%d.%d: l'offset (%u) del "
                         "TRACK=%02X és menor que <150 (es fixa a 0)",
                         ide, _dev[ide].ind, offset, tracki->id );
              offset= 0;
            }
          else offset-= 150; // No es té en compte el Lead-In
          data[pos++]= (uint8_t) ((offset>>24)&0xff);
          data[pos++]= (uint8_t) ((offset>>16)&0xff);
          data[pos++]= (uint8_t) ((offset>>8)&0xff);
          data[pos++]= (uint8_t) (offset&0xff);
        }
      // --> TOC Data length
      toc_length= pos-2;
      data[0]= (uint8_t) ((toc_length>>8)&0xff);
      data[1]= (uint8_t) (toc_length&0xff);
      break;
    case 2: // All Subchannel Q code information
      PC_MSG("piix_ide - cd_read_toc - FORMAT=2 "
             "revisar implementació!!");
      exit(EXIT_FAILURE);
      assert ( drv->cdrom.cd->info->nsessions > 0 );
      // --> Header
      pos= 2; // TOC Data Length al final
      data[pos++]= 1;
      data[pos++]= (uint8_t) drv->cdrom.cd->info->nsessions;
      // --> Track Descriptors
      offset= 0;
      for ( i= 0; i < drv->cdrom.cd->info->ntracks; i++ )
        {
          tracki= &drv->cdrom.cd->info->tracks[i];
          data[pos++]= 0x00; // Reserved
          adr_control=
            0x10 |
            (tracki->audio_four_channel ? 0x08 : 0x00) |
            (tracki->is_audio ? 0x00 : 0x04) |
            (tracki->digital_copy_allowed ? 0x02 : 0x00) |
            (tracki->audio_preemphasis ? 0x01 : 0x00)
            ;
          data[pos++]= adr_control;
          data[pos++]= 0x00; // TNO
          data[pos++]= (uint8_t) (i+1); // POINTER
          cdpos= offset2cdpos ( offset );
          data[pos++]= cdpos.mm; // MIN
          data[pos++]= cdpos.ss; // SEC
          data[pos++]= cdpos.sec; // FRAC
          data[pos++]= 0x00;
          assert ( tracki->nindexes > 0 );
          for ( indexi= NULL, j= 0; j < tracki->nindexes; j++ )
            if ( tracki->indexes[j].id == 1 )
              indexi= &(tracki->indexes[j]);
          assert ( indexi != NULL );
          data[pos++]= indexi->pos.mm; // P-MIN
          data[pos++]= indexi->pos.ss; // P-SEC
          data[pos++]= indexi->pos.sec; // P-FRAC
          /* REVISAR...
          if ( offset < 150 )
            {
              _warning ( _udata, "READ TOC en IDE%d.%d: l'offset (%u) del "
                         "TRACK=%02X és menor que <150 (es fixa a 0)",
                         ide, _dev[ide].ind, offset, tracki->id );
              offset= 0;
            }
          else offset-= 150; // No es té en compte el Lead-In
          */
        }
      // --> Fique Led out???
      // --> TOC Data length
      toc_length= pos-2;
      data[0]= (uint8_t) ((toc_length>>8)&0xff);
      data[1]= (uint8_t) (toc_length&0xff);
      break;
    default: // Reserved
      _warning ( _udata, "s'ha intentat executar READ TOC en "
                 "IDE%d.%d amb FORMAT=%d", ide, _dev[ide].ind, format );
      cdrom_abort ( ide, drv, CD_SENSE_KEY_ILLEGAL_REQUEST,
                    CD_ADD_SENSE_PARAMETER_VALUE_INVALID );
      return;
    }

  // Prepara mode lectura
  drv->pio_transfer.mode= PT_READ_CD;
  drv->pio_transfer.begin= 0;
  if ( pos < length )
    {
      length= pos;
      drv->pio_transfer.packet_byte_count= pos;
      _dev[ide].addr.lbahi= (uint8_t) ((length>>8)&0xff);
      _dev[ide].addr.lbamid= (uint8_t) (length&0xff);
    }
  drv->pio_transfer.end= (length+1)/2;
  drv->pio_transfer.waiting= true;
  drv->pio_transfer.drq_value= true; // Crec que no cal perquè fique
                                     // el DRQ ja a true
  // NOTA!! No sé quan tarda així que ho faré proporcional.
  drv->pio_transfer.remain_cc=
    (drv->pio_transfer.packet_byte_count*_timing.ccpersector)/SEC_SIZE;
  if ( drv->pio_transfer.remain_cc == 0 )
    drv->pio_transfer.remain_cc= 1;
  
  // Prepara.
  _dev[ide].sector_count&= 0xf8;
  _dev[ide].sector_count|= 0x02; // Set I/O (to the host)
  drv->stat.bsy= false;
  drv->stat.df= false; // DMRD (De moment no gaste DMA)
  drv->stat.srv= false; // ??? No suporte overlap.
  drv->stat.drq= true; // Falta la transmissió.
  drv->stat.err= false; // CHK
  drv->cdrom.busy= true;
  
} // end cd_read_toc


static void
cd_read_cdrom_capacity (
                        const int      ide,
                        drv_t         *drv,
                        const uint8_t *packet
                        )
{
  
  uint8_t *data;
  uint32_t offset;
  
  
  assert ( drv->type == PC_IDE_DEVICE_TYPE_CDROM );
  assert ( drv->cdrom.cd != NULL );
  if ( !cdrom_check_is_ready ( ide, drv, true ) ) return;

  // Comparació byte_count
  if ( drv->pio_transfer.packet_byte_count != 8 )
    {
      _warning ( _udata, "s'ha intentat executar READ CD-ROM CAPACITY  en "
                 "IDE%d.%d amb byte_count (%d) != 8",
                 ide, _dev[ide].ind, drv->pio_transfer.packet_byte_count );
      cdrom_abort ( ide, drv, CD_SENSE_KEY_ILLEGAL_REQUEST,
                    CD_ADD_SENSE_PARAMETER_VALUE_INVALID );
      return;
    }

  // NOTA!! No tinc molt clar com calcular açò. Vaig a assumir
  // logicalblocks de 2048 i tornar el LBA de l'últim sector (incloent
  // audio) + 1. Això sí, amb LBA no tenim en compte el leading in.
  offset= cdpos2offset ( drv->cdrom.cd->info->tracks
                         [drv->cdrom.cd->info->ntracks-1].pos_last_sector )
    + 1
    ;
  if ( offset < 150 )
    {
      _warning ( _udata, "READ CDROM CAPACITY en"
                 " IDE%d.%d: l'offset (%u) de l'últim "
                 "segment és menor que <150 (es fixa a 0)",
                 ide, offset );
      offset= 0;
    }
  else offset-= 150; // No es té en compte el Lead-In
  
  // Prepara dades
  data= ((uint8_t *) &drv->pio_transfer.buf);
  // --> Logical block address (Última adreça vàlida)
  data[0]= (uint8_t) ((offset>>24)&0xff);
  data[1]= (uint8_t) ((offset>>16)&0xff);
  data[2]= (uint8_t) ((offset>>8)&0xff);
  data[3]= (uint8_t) (offset&0xff);
  // --> Logical block length (De moment fixat a 2048)
  offset= 2048;
  data[4]= (uint8_t) ((offset>>24)&0xff);
  data[5]= (uint8_t) ((offset>>16)&0xff);
  data[6]= (uint8_t) ((offset>>8)&0xff);
  data[7]= (uint8_t) (offset&0xff);
  
  // Prepara mode lectura
  drv->pio_transfer.mode= PT_READ_CD;
  drv->pio_transfer.begin= 0;
  drv->pio_transfer.end= 8/2;
  drv->pio_transfer.waiting= true;
  drv->pio_transfer.drq_value= true; // Crec que no cal perquè fique
                                     // el DRQ ja a true
  // NOTA!! No sé quan tarda així que ho faré proporcional.
  drv->pio_transfer.remain_cc=
    (drv->pio_transfer.packet_byte_count*_timing.ccpersector)/SEC_SIZE;
  if ( drv->pio_transfer.remain_cc == 0 )
    drv->pio_transfer.remain_cc= 1;

  // Prepara.
  _dev[ide].sector_count&= 0xf8;
  _dev[ide].sector_count|= 0x02; // Set I/O (to the host)
  drv->stat.bsy= false;
  drv->stat.df= false; // DMRD (De moment no gaste DMA)
  drv->stat.srv= false; // ??? No suporte overlap.
  drv->stat.drq= true; // Falta la transmissió.
  drv->stat.err= false; // CHK
  drv->cdrom.busy= true;
  
} // end cd_read_cdrom_capacity


static void
cd_stop_play (
             const int  ide,
             drv_t     *drv
             )
{

  int new_status;

  
  assert ( drv->type == PC_IDE_DEVICE_TYPE_CDROM );
  assert ( drv->cdrom.cd != NULL );
  if ( !cdrom_check_is_ready ( ide, drv, false ) ) return;

  // IMPORTANT!!! Si estaguera fent un SCAN açò el que fa és iniciar
  // el PLAY on s'ha quedat el scan. NO ESTÀ FET!
  
  // Executa
  // NOTA!!! No em queda clar quin és l'estat després d'un
  // stop. Completed o paused??
  new_status=
    drv->cdrom.playing ? CD_AUDIO_STATUS_COMPLETED : drv->cdrom.audio.status;
  cdrom_stop_playing ( drv, new_status );
  cd_successful_command_completion ( ide, drv, false );
  
} // end cd_stop_play


static void
cd_seek_ext (
             const int      ide,
             drv_t         *drv,
             const uint8_t *packet
             )
{
  
  uint32_t lb_addr;
  
  
  assert ( drv->type == PC_IDE_DEVICE_TYPE_CDROM );
  assert ( drv->cdrom.cd != NULL );
  if ( !cdrom_check_is_ready ( ide, drv, true ) ) return;
  
  // Obté arguments.
  lb_addr=
    (((uint32_t) packet[2])<<24) |
    (((uint32_t) packet[3])<<16) |
    (((uint32_t) packet[4])<<8) |
    ((uint32_t) packet[5])
    ;
  
  // Executa.
  if ( !cdrom_seek ( ide, drv, lb_addr ) ) return;
  cd_successful_command_completion ( ide, drv, false );
  
} // end cd_seek_ext


static void
cd_play_audio_msf (
                   const int      ide,
                   drv_t         *drv,
                   const uint8_t *packet
                   )
{

  MSF start,end;
  
  
  assert ( drv->type == PC_IDE_DEVICE_TYPE_CDROM );
  assert ( drv->cdrom.cd != NULL );
  if ( !cdrom_check_is_ready ( ide, drv, true ) ) return;

  // No sé què passa si ja està reproduint. Vaig a assumir que es
  // reinicia.
  if ( drv->cdrom.playing )
    _warning ( _udata, "s'ha intentat executar PLAY AUDIO MSF en "
               "IDE%d.%d quan ja s'estava reproduint so",
               ide, _dev[ide].ind );
  
  // IMPORTANT!!! En alguns manuals parla sobre que si M o S són FF
  // vol dir relatiu en compte d'absolut. De moment vaig a ignorar-ho.
  // Obté valors i comprova si tot va bé.
  start.m= packet[3];
  start.s= packet[4];
  start.f= packet[5];
  if ( start.m == 0xFF && start.s == 0xFF && start.f == 0xFF )
    {
      // En aquest cas reproduïria des de la posició actual.
      PC_MSG ( "piix4_ide - cd_play_audio_msf"
               " - MSF d'inici especial no implementat" );
      exit ( EXIT_FAILURE );
    }
  end.m= packet[6];
  end.s= packet[7];
  end.f= packet[8];
  if ( !cdrom_check_range ( ide, drv, start, end ) ) return;

  // Executa
  cdrom_start_playing ( drv, start, end );
  if ( !drv->cdrom.mode.cdrom_audio_control_parameters.immed )
    {
      PC_MSG ( "piix4_ide - CDROM - Immed==false" );
      exit ( EXIT_FAILURE );
    }
  cd_successful_command_completion ( ide, drv, true );
  
} // end cd_play_audio_msf


static void
cd_mode_select (
                const int      ide,
                drv_t         *drv,
                const uint8_t *packet,
                const bool     is10bytes
                )
{
  
  uint16_t length;
  bool sp;
  
  
  assert ( drv->type == PC_IDE_DEVICE_TYPE_CDROM );
  assert ( drv->cdrom.cd != NULL );
  if ( !cdrom_check_is_ready ( ide, drv, false ) ) return;

  // Extrau arguments
  length=
    is10bytes ?
    ((((uint16_t) packet[7])<<8) | ((uint16_t) packet[8])) :
    ((uint16_t) packet[4])
    ;
  sp= (packet[1]&0x01)!=0;
  if ( sp )
    {
      PC_MSG("piix4_ide.c - cd_mode_select - SP==true");
      exit ( EXIT_FAILURE );
    }

  // Comparació byte_count i allocation_length
  if ( drv->pio_transfer.packet_byte_count != length )
    {
      _warning ( _udata, "s'ha intentat executar MODE SELECT en "
                 "IDE%d.%d amb byte_count (%d) != %u",
                 ide, _dev[ide].ind, drv->pio_transfer.packet_byte_count,
                 length );
      cdrom_abort ( ide, drv, CD_SENSE_KEY_ILLEGAL_REQUEST,
                    CD_ADD_SENSE_PARAMETER_VALUE_INVALID );
      return;
    }

  // Cas especial quan length == 0.
  if ( length == 0 )
    {
      cd_successful_command_completion ( ide, drv, false );
      return;
    }

  // Prepara mode escriptura
  drv->pio_transfer.mode= PT_WRITE_SELECT_CD;
  drv->pio_transfer.begin= 0;
  drv->pio_transfer.end= (length+1)/2;
  drv->pio_transfer.drq_value= true; // Crec que no cal perquè fique
                                     // el DRQ ja a true
  drv->pio_transfer.waiting= false;

  // Prepara.
  _dev[ide].sector_count&= 0xf8; // I/O <- 0, C/D <- 0
  //_dev[ide].sector_count|= 0x00; // Set I/O (to the host)
  drv->stat.bsy= false;
  drv->stat.df= false; // DMRD (De moment no gaste DMA)
  drv->stat.srv= false; // ???
  drv->stat.drq= true; // Falta la transmissió.
  drv->stat.err= false; // CHK
  drv->cdrom.busy= true;
  drv->intrq= true;
  update_irq ();
  
} // end cd_mode_select


static void
cd_read_sub_channel (
                     const int      ide,
                     drv_t         *drv,
                     const uint8_t *packet
                     )
{

  bool use_msf,sub_q;
  uint8_t format;
  int pos,/*track_number,*/sub_chn_length;
  uint16_t length;
  uint8_t *data;
  
  
  assert ( drv->type == PC_IDE_DEVICE_TYPE_CDROM );
  assert ( drv->cdrom.cd != NULL );
  if ( !cdrom_check_is_ready ( ide, drv, false ) ) return;
  
  // Extrau arguments
  use_msf= (packet[1]&0x02)!=0;
  sub_q= (packet[2]&0x40)!=0;
  format= packet[3];
  //track_number= (int) ((unsigned int) packet[6]);
  length= ((((uint16_t) packet[7])<<8) | ((uint16_t) packet[8]));

  // Comparació byte_count i allocation_length
  if ( drv->pio_transfer.packet_byte_count != length )
    {
      _warning ( _udata, "s'ha intentat executar MODE SENSE en "
                 "IDE%d.%d amb byte_count (%d) != %u",
                 ide, _dev[ide].ind, drv->pio_transfer.packet_byte_count,
                 length );
      cdrom_abort ( ide, drv, CD_SENSE_KEY_ILLEGAL_REQUEST,
                    CD_ADD_SENSE_PARAMETER_VALUE_INVALID );
      return;
    }
  
  // Prepara dades
  data= ((uint8_t *) &drv->pio_transfer.buf);
  pos= 0;
  data[pos++]= 0x00; // Reserved
  data[pos++]= drv->cdrom.audio.status;
  drv->cdrom.audio.status= CD_AUDIO_STATUS_NONE;
  if ( !sub_q )
    {
      data[pos++]= 0;
      data[pos++]= 0;
    }
  else
    {
      pos+= 2; // Ací va la grandària de la resta
      switch ( format )
        {
          
        case 0x01: // CD-ROM current position.
          data[pos++]= 0x01; // Code
          data[pos++]=
            (drv->cdrom.subchn_Q[1]<<4) | // ADR
            (drv->cdrom.subchn_Q[1]>>4) // CTRL
            ;
          data[pos++]= drv->cdrom.subchn_Q[2]; // Track number
          data[pos++]= drv->cdrom.subchn_Q[3]; // Index number
          if ( use_msf )
            {
              data[pos++]= 0x00;
              data[pos++]= drv->cdrom.subchn_Q[8]; // M - Absolute
              data[pos++]= drv->cdrom.subchn_Q[9]; // S - Absolute
              data[pos++]= drv->cdrom.subchn_Q[10]; // F - Absolute
              data[pos++]= 0x00;
              data[pos++]= drv->cdrom.subchn_Q[4]; // M - Relative
              data[pos++]= drv->cdrom.subchn_Q[5]; // S - Relative
              data[pos++]= drv->cdrom.subchn_Q[6]; // F - Relative
            }
          else
            {
              PC_MSG("piix4_ide.c -"
                     " cd_read_sub_channel - FORMAT=01 use_msf=False");
              exit(EXIT_FAILURE);
            }
          break;
          
        default:
          PC_MSGF("piix4_ide.c -"
                  " cd_read_sub_channel - FORMAT=%02X!!",
                  format);
          exit(EXIT_FAILURE);
        }
      // --> Sense length
      sub_chn_length= pos-4;
      data[2]= (uint8_t) ((sub_chn_length>>8)&0xff);
      data[3]= (uint8_t) (sub_chn_length&0xff);
    }

  // Prepara mode lectura
  drv->pio_transfer.mode= PT_READ_CD;
  drv->pio_transfer.begin= 0;
  if ( pos < length ) length= pos;
  if ( drv->pio_transfer.packet_byte_count != length )
    {
      _warning ( _udata, "s'ha intentat executar READ SUB CHANNEL en "
                 "IDE%d.%d amb byte_count (%d) != %u (paquet)",
                 ide, _dev[ide].ind, drv->pio_transfer.packet_byte_count,
                 length );
      cdrom_abort ( ide, drv, CD_SENSE_KEY_ILLEGAL_REQUEST,
                    CD_ADD_SENSE_PARAMETER_VALUE_INVALID );
      return;
    }
  drv->pio_transfer.end= (length+1)/2;
  drv->pio_transfer.waiting= true;
  drv->pio_transfer.drq_value= true; // Crec que no cal perquè fique
                                     // el DRQ ja a true
  // NOTA!! No sé quan tarda així que ho faré proporcional.
  drv->pio_transfer.remain_cc=
    (drv->pio_transfer.packet_byte_count*_timing.ccpersector)/SEC_SIZE;
  if ( drv->pio_transfer.remain_cc == 0 )
    drv->pio_transfer.remain_cc= 1;

  // Prepara.
  _dev[ide].sector_count&= 0xf8;
  _dev[ide].sector_count|= 0x02; // Set I/O (to the host)
  drv->stat.bsy= false;
  drv->stat.df= false; // DMRD (De moment no gaste DMA)
  drv->stat.srv= false; // ??? No suporte overlap.
  drv->stat.drq= true; // Falta la transmissió.
  drv->stat.err= false; // CHK
  drv->cdrom.busy= true;
  
} // end cd_read_sub_channel


static void
cd_prevent_allow_medium_removal (
                                 const int      ide,
                                 drv_t         *drv,
                                 const uint8_t *packet
                                 )
{

  assert ( drv->type == PC_IDE_DEVICE_TYPE_CDROM );
  assert ( drv->cdrom.cd != NULL );
    
  // Executa
  drv->cdrom.locked= (packet[4]&0x01)!=0;
  cd_successful_command_completion ( ide, drv, false );
  
} // end cd_prevent_allow_medium_removal


static void
cd_pause_resume (
                 const int      ide,
                 drv_t         *drv,
                 const uint8_t *packet
                 )
{

  bool resume;

  
  assert ( drv->type == PC_IDE_DEVICE_TYPE_CDROM );
  assert ( drv->cdrom.cd != NULL );
  if ( !cdrom_check_is_ready ( ide, drv, false ) ) return;
  if ( !drv->cdrom.playing )
    {
      cdrom_abort ( ide, drv, CD_SENSE_KEY_ABORTED_COMMAND,
                    CD_ADD_SENSE_AUDIO_PLAY_OPERATION_ABORTED );
      return;
    }
  
  // Executa
  resume= (packet[8]&0x01)!=0;
  drv->cdrom.paused= !resume;
  drv->cdrom.audio.status=
    resume ? CD_AUDIO_STATUS_IN_PROGRESS : CD_AUDIO_STATUS_PAUSED;
  cd_successful_command_completion ( ide, drv, false );
  
} // end cd_pause_resume


static void
cd_start_stop_unit (
                    const int      ide,
                    drv_t         *drv,
                    const uint8_t *packet
                    )
{

  assert ( drv->type == PC_IDE_DEVICE_TYPE_CDROM );
  assert ( drv->cdrom.cd != NULL );
  //if ( !cdrom_check_is_ready ( ide, drv, false ) ) return;
  
  // Executa
  PC_MSG("piix4_ide - run_packet_command -"
         " START/STOP UNIT Command (1B)");
  
  cd_successful_command_completion ( ide, drv, false );
  
} // end cd_start_stop_unit


static void
run_packet_command (
                    const int  ide,
                    drv_t     *drv
                    )
{

  const uint8_t *data;
  uint8_t cmd;
  

  data= (const uint8_t *) &drv->pio_transfer.buf;
  cmd= data[0];
  // Inicialitza.
  drv->stat.bsy= true;
  drv->stat.rdy= false;
  drv->stat.df= false;
  drv->stat.drq= false;
  drv->stat.err= false;
  switch ( cmd )
    {
    case 0x00: cd_test_unit_ready ( ide, drv ); break;

    case 0x03: cd_request_sense ( ide, drv, data ); break;
      
    case 0x12: cd_inquiry ( ide, drv, data ); break;

    case 0x15: cd_mode_select ( ide, drv, data, false ); break;
        
    case 0x1a: cd_mode_sense ( ide, drv, data, false ); break;
    case 0x1b: cd_start_stop_unit ( ide, drv, data ); break;
      
    case 0x1e: cd_prevent_allow_medium_removal ( ide, drv, data ); break;

    case 0x25: cd_read_cdrom_capacity ( ide, drv, data ); break;
      
    case 0x28: cd_read64K ( ide, drv, data ); break;

    case 0x2b: cd_seek_ext ( ide, drv, data ); break;
      
    case 0x42: cd_read_sub_channel ( ide, drv, data ); break;
    case 0x43: cd_read_toc ( ide, drv, data ); break;
      
    case 0x47: cd_play_audio_msf ( ide, drv, data ); break;

    case 0x4b: cd_pause_resume ( ide, drv, data ); break;
      
    case 0x4e: cd_stop_play ( ide, drv ); break;

    case 0x55: cd_mode_select ( ide, drv, data, true ); break;
      
    case 0x5a: cd_mode_sense ( ide, drv, data, true ); break;
      
    default:
      PC_MSGF("run_packet_command -"
             " comandament desconegut: %02X\n",cmd);
      exit(EXIT_FAILURE);
    }
  
} // end run_packet_command


static void
packet (
        const int  ide,
        drv_t     *drv
        )
{

  bool ovl,dma;
  uint8_t tag;

  
  // NOTA!! De moment no suporte ni overlaping ni encolament.

  // Comprovacions.
  ovl= (_dev[ide].features&0x2)!=0;
  if ( ovl )
    _warning ( _udata, "s'ha intentat executar PACKET en IDE%d.%d amb"
               " 'Overlapping' però el dispositiu no ho suporta",
               ide, _dev[ide].ind );
  dma= (_dev[ide].features&0x1)!=0;
  if ( dma )
    {
      PC_MSG("packet amb DMA");
      exit ( EXIT_FAILURE );
    }
  tag= _dev[ide].sector_count>>3;
  if ( tag != 0 )
    _warning ( _udata, "s'ha intentat executar PACKET en IDE%d.%d amb"
               " una etiqueta (%02X) però el dispositiu no suporta etiquetes",
               ide, _dev[ide].ind, tag );
  
  // Prepara mode read
  drv->pio_transfer.mode= PT_PACKET;
  drv->pio_transfer.packet_byte_count=
    ((uint16_t) _dev[ide].addr.lbamid) |
    (((uint16_t) _dev[ide].addr.lbahi)<<8)
    ;
  drv->pio_transfer.begin= 0;
  drv->pio_transfer.end= PACKET_CMD_SIZE/2;
  
  // Prepara per a rebre comandaments.
  _dev[ide].sector_count&= 0xf8;
  _dev[ide].sector_count|= 0x1; // Transfer of command packet.
  drv->stat.bsy= false; // El ficarem a true quan comence a
                        // executar-se el comandament. Eventualment es
                        // pot tornar a ficar a false esperant a més
                        // dades.
  drv->stat.rdy= false; // DRDY. Crec que irrelevant ???
  drv->stat.df= false; // DMRD.
  drv->stat.srv= false;  // Si no hi ha overlap depèn del
                         // commandament. En principi es fica 1 quan
                         // acaba un comandament i està llest per a
                         // rebre un altre.
  drv->stat.drq= true; // Es fica a cert per a acceptar dades.
  drv->stat.err= false; // CHK.
  
} // end packet


static void
device_reset (
              const int  ide,
              drv_t     *drv
              )
{

  assert ( drv->type == PC_IDE_DEVICE_TYPE_CDROM );
  _dev[ide].error= 0x01; // Diagnostic result. Bàsicament diu que passa
                         // el test
  set_signature ( ide, drv );
  
  // CAL PARAR QUALSEVOL COMANDAMENT QUE ESTIGA EXECUTANT-SE.
  drv->cdrom.busy= false;
  drv->pio_transfer.mode= PT_NORMAL;
  drv->pio_transfer.waiting= false;
  // IMPORTANT!! Del TombRaider deduisc que aquest reset no para la
  // reproducció.
  //cdrom_stop_playing ( drv, CD_AUDIO_STATUS_NONE );
  
  // Estat
  drv->intrq= false;
  drv->stat.err= false; // bit 0
  drv->stat.drq= false; // bit 3
  drv->stat.df= false; // bit 5
  drv->stat.bsy= false;
  drv->stat.rdy= false;
  update_irq ();
  
} // end device_reset


static void
ide_command (
             const int     ide,
             const uint8_t data
             )
{
  
  drv_t *drv;
  
  
  drv= &_dev[ide].drv[_dev[ide].ind];
  
  // NOTA!! Vaig a executar els comandaments immediatament.
  drv->intrq= false;
  drv->stat.err= false;
  _dev[ide].error= 0x00;
  switch ( data )
    {
      // NOP
    case 0x00:
      // NOTA!!! Depenent del "Subcommand code" aborta coles, però no
      // suporte això.
      goto abort;
      break;
      
      // RESET
    case 0x08:
      switch ( drv->type )
        {
        case PC_IDE_DEVICE_TYPE_NONE:
        case PC_IDE_DEVICE_TYPE_HDD:
          goto abort;
        case PC_IDE_DEVICE_TYPE_CDROM:
          device_reset ( ide, drv );
          break;
        default:
          PC_MSGF("reset: %d",drv->type);
          exit(EXIT_FAILURE);
        }
      break;
      
      // READ SECTORS
    case 0x20:
    case 0x21:
      if ( drv->type == PC_IDE_DEVICE_TYPE_HDD )
        read_sectors ( ide, drv );
      else
        {
          PC_MSGF("read_sectors: %d",drv->type);
          exit(EXIT_FAILURE);
        }
      break;

      // WRITE SECTORS
    case 0x30:
    case 0x31:
      if ( drv->type == PC_IDE_DEVICE_TYPE_HDD )
        write_sectors ( ide, drv );
      else
        {
          PC_MSGF("write_sectors: %d",drv->type);
          exit(EXIT_FAILURE);
        }
      break;

      // PACKET
    case 0xa0:
      switch ( drv->type )
        {
        case PC_IDE_DEVICE_TYPE_NONE:
        case PC_IDE_DEVICE_TYPE_HDD:
          goto abort;
        case PC_IDE_DEVICE_TYPE_CDROM:
          packet ( ide, drv );
          break;
        default:
          PC_MSGF("packet: %d",drv->type);
          exit(EXIT_FAILURE);
        }
      break;
      // IDENTIFY PACKET DEVICE
    case 0xa1:
      switch ( drv->type )
        {
        case PC_IDE_DEVICE_TYPE_NONE:
        case PC_IDE_DEVICE_TYPE_HDD:
          goto abort;
        case PC_IDE_DEVICE_TYPE_CDROM:
          identify_packet_device_cd ( drv );
          break;
        default:
          PC_MSGF("identify_packet_device: %d",drv->type);
          exit(EXIT_FAILURE);
        }
      break;
      
      // IDENTIFY DEVICE
    case 0xec:
      switch ( drv->type )
        {
        case PC_IDE_DEVICE_TYPE_NONE:
          goto abort;
        case PC_IDE_DEVICE_TYPE_HDD:
          identify_device ( drv );
          break;
        case PC_IDE_DEVICE_TYPE_CDROM:
          set_signature ( ide, drv );
          goto abort;
        default:
          PC_MSGF("identify_device: %d",drv->type);
          exit(EXIT_FAILURE);
        }
      break;
      
    default:
      printf("[EE] IDE%d - Comandament desconegut %02X\n",ide,data);
      exit(EXIT_FAILURE);
    }
  update_irq ();
  
  return;
  
 abort:
  drv->stat.err= true;
  _dev[ide].error|= ERR_ABRT;
  drv->stat.rdy= drv->type!=PC_IDE_DEVICE_TYPE_CDROM;
  update_irq ();
  
} // end ide_command




/*******/
/* PCI */
/*******/

static uint8_t
pci_read8 (
           const uint8_t addr
           )
{

  uint8_t ret;

  
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
      // Reserved
    case 0x0f ... 0x1f: ret= 0x00; break;

      // Reserved
    case 0x24 ... 0x3f: ret= 0x00; break;
      
    default:
      _warning ( _udata,
                 "PCI:PIIX4_IDE.read8 - addreça no implementada %02X\n",
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

      // Reserved
    case 0x08 ... 0x0f: ret= 0x0000; break;

      // Reserved
    case 0x12 ... 0x1f: ret= 0x0000; break;

      // IDETIM - IDE TIMING REGISTER
    case 0x20: ret= _pci_regs.idetim[0]; break;
    case 0x21: ret= _pci_regs.idetim[1]; break;

    default: goto warn;
    }
  
  return ret;
  
 warn:
  _warning ( _udata,
             "PCI:PIIX4_IDE.read16 - addreça no implementada %02X\n",
             addr );
  return 0xffff;
  
} // end pci_read16


static uint32_t
pci_read32 (
            const uint8_t addr
            )
{

  uint32_t ret;
  
  
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

      // Reserved
    case 0x04 ... 0x07: ret= 0x00000000; break;
      // BMIBA
    case 0x08: ret= _pci_regs.bmiba; break;
      // Reserved
    case 0x09 ... 0x0f: ret= 0x00000000; break;
      
    default:
      _warning ( _udata,
                 "PCI:PIIX4_IDE.read32 - addreça no implementada %02X\n",
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
      // Reserved
    case 0x0f ... 0x1f: break;

      // Reserved
    case 0x24 ... 0x3f: break;
      
    default:
      _warning ( _udata,
                 "PCI:PIIX4_IDE.write8 - addreça no implementada %02X\n",
                 addr );
    }
  
} // end pci_write8


static void
pci_write16 (
             const uint8_t  addr,
             const uint16_t data
             )
{

  switch ( addr )
    {

      // VID
    case 0x00: break;
      // DID
    case 0x01: break;
      //PCICMD
    case 0x02:
      _pci_regs.pcicmd= data&0x021F;
      if ( data&0x04 )
        _warning ( _udata,
                   "pci_write16 (PIIX4 IDE) - s'ha intentat habilitar"
                   " el Bus Master Function Enable, però no està implementat" );
      break;

      // SCC i BASEC;
    case 0x05: break;

      // Reserved
    case 0x08 ... 0x0f: break;
      
      // Reserved
    case 0x12 ... 0x1f: break;

      // IDETIM - IDE TIMING REGISTER
    case 0x20: write_idetim ( 0, data ); break;
    case 0x21: write_idetim ( 1, data ); break;
      
    default:
      _warning ( _udata,
                 "PCI:PIIX4_IDE.write16 - addreça no implementada %02X\n",
                 addr );
    }
  
} // end pci_write16


static void
pci_write32 (
             const uint8_t  addr,
             const uint32_t data
             )
{

  switch ( addr )
    {

    case 0x00: break;
      
      // Reserved
    case 0x04 ... 0x07: break;
      // BMIBA
    case 0x08: _pci_regs.bmiba= (data&0xFFFFFFF0)|0x1; break;
      // Reserved
    case 0x09 ... 0x0f: break;
      
    default:
      _warning ( _udata,
                 "PCI:PIIX4_IDE.write32 - addreça no implementada %02X\n",
                 addr );
    }
  
} // end pci_write32


const PC_PCIFunction PC_PIIX4_PCIFunction_ide=
  {
    pci_read8,
    pci_read16,
    pci_read32,
    pci_write8,
    pci_write16,
    pci_write32,
    "82371AB (PIIX4) - IDE Controller"
  };

static void
init_pci_regs (void)
{

  _pci_regs.pcicmd= 0x0000;
  _pci_regs.bmiba= 0x00000001;
  _pci_regs.idetim[0]= 0x0000;
  _pci_regs.idetim[1]= 0x0000;
  
} // end init_pci_regs


static void
clock (
       const bool update_cc2event
       )
{

  int i,j,cc,cc_proc;
  bool check_irq;
  drv_t *drv;
  
  
  // Processa cicles
  cc= PC_Clock-_timing.cc_used;
  if ( cc > 0 ) { _timing.cc+= cc; _timing.cc_used+= cc; }
  cc_proc= _timing.cc;
  _timing.cc= 0;
  
  // Transferències de sectors.
  check_irq= false;
  for ( i= 0; i < 2; ++i )
    for ( j= 0; j < 2; ++j )
      {
        drv= &_dev[i].drv[j];
        if ( drv->type != PC_IDE_DEVICE_TYPE_NONE )
          if ( drv->pio_transfer.waiting )
            {
              drv->pio_transfer.remain_cc-= cc_proc;
              if ( drv->pio_transfer.remain_cc <= 0 )
                {
                  drv->pio_transfer.remain_cc= 0;
                  drv->pio_transfer.waiting= false;
                  drv->stat.bsy= false;
                  drv->stat.drq= drv->pio_transfer.drq_value;
                  drv->intrq= true;
                  check_irq= true;
                }
            }
      }
  if ( check_irq ) update_irq ();
  
  // Actualitza cctoEvent
  if ( update_cc2event )
    update_cc_to_event ();
  
} // end clock


// Torna cert si la grandària és acceptable
static bool
calc_hdd_size (
               hdd_t *hdd
               )
{

  long size,tmp;
  
  
  if ( hdd->f == NULL ) return true;

  // Per simplificar sols accepte tracks de 63 sectors, i cilindres el
  // més gran posible fins 1024.
  if ( hdd->f->nbytes%SEC_SIZE != 0 ) return false;
  size= hdd->f->nbytes/SEC_SIZE;
  if ( size > (long) (63*1024) )
    {
      tmp= size/((long) (63*1024));
      hdd->size.H= (tmp>255) ? 255 : (uint16_t) tmp;
      hdd->size.C= 1024;
      hdd->size.S= 63;
    }
  else
    {
      hdd->size.H= 1;
      if ( size > 63 )
        {
          hdd->size.C= (uint16_t) (size/63);
          hdd->size.S= 63;
        }
      else
        {
          hdd->size.C= 1;
          hdd->size.S= (uint8_t) size;
        }
    }
  
  return true;
  
} // end calc_hdd_size


static void
set_signature (
               const int    ide,
               const drv_t *drv
               )
{

  switch ( drv->type )
    {
    case PC_IDE_DEVICE_TYPE_CDROM:
      _dev[ide].sector_count= 0x01;
      _dev[ide].addr.lbalo= 0x01;
      _dev[ide].addr.lbamid= 0x14;
      _dev[ide].addr.lbahi= 0xEB;
      _dev[ide].addr.lbaextra= _dev[ide].ind==0 ? 0x00 : 0x10;
      _dev[ide].addr.use_lba= false;
      break;
    default: // NO PACKET
      _dev[ide].sector_count= 0x01;
      _dev[ide].addr.lbalo= 0x01;
      _dev[ide].addr.lbamid= 0x00;
      _dev[ide].addr.lbahi= 0x00;
      _dev[ide].addr.lbaextra= 0x00;
      _dev[ide].addr.use_lba= false;
    }
  
} // end set_signature




/**********************/
/* FUNCIONS PÚBLIQUES */
/**********************/

PC_Error
PC_piix4_ide_init (
                   PC_IDEDevice  ide_devices[2][2],
                   PC_Warning   *warning,
                   void         *udata
                   )
{

  int i,j;
  
  
  _warning= warning;
  _udata= udata;

  _sound_dev= NULL;
  _sound_dev_ide= 0;
  for ( i= 0; i < 2; ++i )
    {
      _dev[i].ind= 0;
      _dev[i].features= 0x00;
      _dev[i].sector_count= 0x00;
      memset ( &_dev[i].addr, 0, sizeof(_dev[i].addr) );
      _dev[i].addr.use_lba= false;
      _dev[i].error= 0;
      for ( j= 0; j < 2; ++j )
        {
          _dev[i].drv[j].type= ide_devices[i][j].type;
          if ( _dev[i].drv[j].type != PC_IDE_DEVICE_TYPE_NONE )
            {
              _dev[i].drv[j].stat.err= false;
              _dev[i].drv[j].stat.drq= false;
              _dev[i].drv[j].stat.srv= false;
              _dev[i].drv[j].stat.df= false;
              _dev[i].drv[j].stat.rdy=
                _dev[i].drv[j].type != PC_IDE_DEVICE_TYPE_CDROM;
              _dev[i].drv[j].stat.bsy= false;
              _dev[i].drv[j].intrq= false;
              _dev[i].drv[j].pio_transfer.waiting= false;
              _dev[i].drv[j].pio_transfer.drq_value= false;
              _dev[i].drv[j].pio_transfer.remain_cc= 0;
              memset ( _dev[i].drv[j].pio_transfer.buf, 0, SEC_SIZE );
              _dev[i].drv[j].pio_transfer.begin= 0;
              _dev[i].drv[j].pio_transfer.end= 0;
              switch ( _dev[i].drv[j].type )
                {
                case PC_IDE_DEVICE_TYPE_HDD:
                  _dev[i].drv[j].hdd.f= ide_devices[i][j].hdd.f;
                  _dev[i].drv[j].hdd.size.H= 0;
                  _dev[i].drv[j].hdd.size.C= 0;
                  _dev[i].drv[j].hdd.size.S= 0;
                  if ( !calc_hdd_size ( &_dev[i].drv[j].hdd ) )
                    return PC_HDD_WRONG_SIZE;
                  break;
                case PC_IDE_DEVICE_TYPE_CDROM:
                  _dev[i].drv[j].cdrom.cd= ide_devices[i][j].cdrom.cdrom;
                  cdrom_reset ( &_dev[i].drv[j] );
                  _sound_dev= &_dev[i].drv[j];
                  _sound_dev_ide= i;
                  break;
                default:
                }
              set_signature ( i, &_dev[i].drv[j] );
            }
        }
    }
  
  init_pci_regs ();
  
  // Timing.
  _timing.cc_used= 0;
  _timing.cc= 0;
  _timing.cctoEvent= 0;
  // NOTA!! L'únic timing que vaig a emular de la transferència PIO és
  // la preparació d'un sector en el buffer. No tinc molt clar quan
  // tarda açò i si tarda el mateix quan ve del disc o són metadades,
  // però he decidit fer que tarde aproximadament 16MB/s.
  _timing.ccpersector= (SEC_SIZE*PC_ClockFreq) / (16*1024*1024);
  update_cc_to_event ();
  
  return PC_NOERROR;
  
} // end PC_piix4_ide_init


void
PC_piix4_ide_reset (void)
{

  int i,j;

  
  clock ( false );

  for ( i= 0; i < 2; ++i )
    {
      _dev[i].ind= 0;
      _dev[i].features= 0x00;
      _dev[i].sector_count= 0x00;
      memset ( &_dev[i].addr, 0, sizeof(_dev[i].addr) );
      _dev[i].addr.use_lba= false;
      _dev[i].error= 0;
      for ( j= 0; j < 2; ++j )
         if ( _dev[i].drv[j].type != PC_IDE_DEVICE_TYPE_NONE )
            {
              _dev[i].drv[j].stat.err= false;
              _dev[i].drv[j].stat.drq= false;
              _dev[i].drv[j].stat.srv= false;
              _dev[i].drv[j].stat.df= false;
              _dev[i].drv[j].stat.rdy=
                _dev[i].drv[j].type != PC_IDE_DEVICE_TYPE_CDROM;
              _dev[i].drv[j].stat.bsy= false;
              _dev[i].drv[j].intrq= false;
              _dev[i].drv[j].pio_transfer.waiting= false;
              _dev[i].drv[j].pio_transfer.drq_value= false;
              _dev[i].drv[j].pio_transfer.remain_cc= 0;
              _dev[i].drv[j].pio_transfer.begin= 0;
              _dev[i].drv[j].pio_transfer.end= 0;
              switch ( _dev[i].drv[j].type )
                {
                case PC_IDE_DEVICE_TYPE_CDROM:
                  cdrom_reset ( &_dev[i].drv[j] );
                  break;
                default:
                }
              set_signature ( i, &_dev[i].drv[j] );
            }
    }
  
  init_pci_regs ();
  
  // Timing.
  _timing.cctoEvent= 0;
  
  update_cc_to_event ();
  
} // end PC_piix4_ide_reset


int
PC_piix4_ide_next_event_cc (void)
{
  
  int tmp;
  
  
  tmp= _timing.cctoEvent - _timing.cc;
  assert ( tmp > 0 );

  return tmp;
  
} // end PC_piix4_ide_next_event_cc


void
PC_piix4_ide_end_iter (void)
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
  
} // end PC_piix4_ide_end_iter


bool
PC_piix4_ide_port_read8 (
                         const uint16_t  port,
                         uint8_t        *data
                         )
{
  
  uint16_t base,iport;
  bool ret;
  

  if ( !(_pci_regs.pcicmd&PCICMD_IOSE) ) return false;

  clock ( true );
  
  ret= false;
  
  // IDE0 registers
  if ( _pci_regs.idetim[0]&IDETIM_IDE )
    {
      switch ( port )
        {
          // Error Register
        case 0x01f1:
          *data= ide_error_read ( 0 );
          ret= true;
          break;
          // Sector Count Register
        case 0x01f2:
          if ( _dev[0].drv[0].type==PC_IDE_DEVICE_TYPE_NONE &&
               _dev[0].drv[1].type==PC_IDE_DEVICE_TYPE_NONE )
            *data= 0xff;
          else
            *data= _dev[0].sector_count;
          ret= true;
          break;
          // Sector Number Register (LBAlo)
        case 0x01f3:
          *data= ide_sector_number_lbalo_read ( 0 );
          ret= true;
          break;
        case 0x01f4: // Cylinder Low Register / (LBAmid)
          *data= ide_cylinder_low_lbamid_read ( 0 );
          ret= true;
          break;
        case 0x01f5: // Cylinder High Register / (LBAhi)
          *data= ide_cylinder_high_lbahi_read ( 0 );
          ret= true;
          break;
          // Drive/Head
        case 0x01f6:
          *data= ide_drive_head_read ( 0 );
          ret= true;
          break;
          // Status
        case 0x01f7:
          *data= ide_stat_read ( 0 );
          ret= true;
          break;
          
          // Alternate Status
        case 0x03f6:
          PC_MSG("IDE0 - Alternate Status - Cal fer"
                 " que no modifique a les interrupcions");
          *data= ide_stat_read ( 0 );
          ret= true;
          break;
          
        case 0x03f7:
          printf("[EE] READ (IDE0 CONTROL BLOCK.8) port:%X\n",
                 port);
          ret= true;
          *data= 0xff;
          exit(EXIT_FAILURE);
          break;
        }
    }
  if ( ret ) return ret;

  // IDE1 registers
  if ( _pci_regs.idetim[1]&IDETIM_IDE )
    {
      switch ( port )
        {
          // Error Register
        case 0x0171:
          *data= ide_error_read ( 1 );
          ret= true;
          break;
          // Sector Count Register
        case 0x0172:
          if ( _dev[1].drv[0].type==PC_IDE_DEVICE_TYPE_NONE &&
               _dev[1].drv[1].type==PC_IDE_DEVICE_TYPE_NONE )
            *data= 0xff;
          else
            *data= _dev[1].sector_count;
          ret= true;
          break;
          // Sector Number Register (LBAlo)
        case 0x0173:
          *data= ide_sector_number_lbalo_read ( 1 );
          ret= true;
          break;
        case 0x0174: // Cylinder Low Register / (LBAmid)
          *data= ide_cylinder_low_lbamid_read ( 1 );
          ret= true;
          break;
        case 0x0175: // Cylinder High Register / (LBAhi)
          *data= ide_cylinder_high_lbahi_read ( 1 );
          ret= true;
          break;
          // Drive/Head
        case 0x0176:
          *data= ide_drive_head_read ( 1 );
          ret= true;
          break;
          // Status
        case 0x0177:
          *data= ide_stat_read ( 1 );
          ret= true;
          break;
          
        case 0x0374 ... 0x0375:
          printf("[EE] READ (IDE1 CONTROL BLOCK.8) port:%X\n",
                 port);
          ret= true;
          *data= 0xff;
          exit(EXIT_FAILURE);
          break;

          // Alternate Status
        case 0x0376:
          PC_MSG("IDE1 - Alternate Status - Cal fer"
                 " que no modifique a les interrupcions");
          *data= ide_stat_read ( 1 );
          ret= true;
          break;
          
        case 0x0377:
          printf("[EE] READ (IDE1 CONTROL BLOCK.8) port:%X\n",
                 port);
          ret= true;
          *data= 0xff;
          exit(EXIT_FAILURE);
          break;
        }
    }
  if ( ret ) return ret;
  
  // BMIBA
  base= (_pci_regs.bmiba&0x0000FFF0);
  if ( port >= base && port < base+16 )
    {
      iport= port-base;
      switch ( iport )
        {
        default:
          _warning (  _udata,
                      "PC_piix4_ide_port_read8 -> unknown port %04X (%04X)",
                      port, iport );
          *data= 0xFF;
          
        }
      ret= true;
    }

  // IDE Port3 i Port4 no implementat
  switch ( port )
    {
    case 0x01e8 ... 0x01ef:
    case 0x0168 ... 0x016f:
      _warning ( _udata,
                 "PC_piix4_ide_port_read8 -> unknown IDE port %04X",
                 port );
      *data= 0xFF;
      ret= true;
      break;
    }
  
  return ret;
  
} // end PC_piix4_ide_port_read8


bool
PC_piix4_ide_port_read16 (
                          const uint16_t  port,
                          uint16_t       *data
                          )
{

  uint16_t base,iport;
  bool ret;
  

  if ( !(_pci_regs.pcicmd&PCICMD_IOSE) ) return false;
  ret= false;

  clock ( true );
  
  // IDE0 registers
  if ( _pci_regs.idetim[0]&IDETIM_IDE )
    {
      switch ( port )
        {
          // Data Register 
        case 0x01f0:
          *data= ide_data_read ( 0 );
          ret= true;
          break;
        case 0x01f1 ... 0x01f7:
          printf("[EE] READ (IDE0 COMMAND BLOCK.16) port:%X\n",
                 port);
          ret= true;
          *data= 0xffff;
          exit(EXIT_FAILURE);
          break;
        case 0x03f4 ... 0x03f7:
          printf("[EE] READ (IDE0 CONTROL BLOCK.16) port:%X\n",
                 port);
          ret= true;
          *data= 0xffff;
          exit(EXIT_FAILURE);
          break;
        }
    }
  if ( ret ) return ret;

  // IDE1 registers
  if ( _pci_regs.idetim[1]&IDETIM_IDE )
    {
      switch ( port )
        {
          // Data Register 
        case 0x0170:
          *data= ide_data_read ( 1 );
          ret= true;
          break;
        case 0x0171 ... 0x0177:
          printf("[EE] READ (IDE1 COMMAND BLOCK.16) port:%X\n",
                 port);
          ret= true;
          *data= 0xffff;
          exit(EXIT_FAILURE);
          break;
        case 0x0374 ... 0x0377:
          printf("[EE] READ (IDE1 CONTROL BLOCK.16) port:%X\n",
                 port);
          ret= true;
          *data= 0xffff;
          exit(EXIT_FAILURE);
          break;
        }
    }
  if ( ret ) return ret;
  
  // BMIBA
  base= (_pci_regs.bmiba&0x0000FFF0);
  if ( port >= base && port < base+16 )
    {
      iport= port-base;
      switch ( iport )
        {
        default:
          _warning (  _udata,
                      "PC_piix4_ide_port_read16 -> unknown port %04X (%04X)",
                      port, iport );
          *data= 0xFFFF;
          
        }
      ret= true;
    }
  else ret= false;
  
  return ret;
  
} // end PC_piix4_ide_port_read16


bool
PC_piix4_ide_port_read32 (
                          const uint16_t  port,
                          uint32_t       *data
                          )
{
  
  uint16_t base,iport;
  bool ret;
  

  if ( !(_pci_regs.pcicmd&PCICMD_IOSE) ) return false;

  clock ( true );
  
  ret= false;
  
  // IDE0 registers
  if ( _pci_regs.idetim[0]&IDETIM_IDE )
    {
      switch ( port )
        {
        case 0x01f0 ... 0x01f7:
          printf("[EE] READ (IDE0 COMMAND BLOCK.32) port:%X\n",
                 port);
          ret= true;
          *data= 0xffffffff;
          exit(EXIT_FAILURE);
          break;
        case 0x03f4 ... 0x03f7:
          printf("[EE] READ (IDE0 CONTROL BLOCK.32) port:%X\n",
                 port);
          ret= true;
          *data= 0xffffffff;
          exit(EXIT_FAILURE);
          break;
        }
    }
  if ( ret ) return ret;

  // IDE1 registers
  if ( _pci_regs.idetim[1]&IDETIM_IDE )
    {
      switch ( port )
        {
          // Data Register 
        case 0x0170:
          *data= (uint32_t) ide_data_read ( 1 );
          *data|= (((uint32_t) ide_data_read ( 1 ))<<16);
          ret= true;
          break;
          
        case 0x0171 ... 0x0177:
          printf("[EE] READ (IDE1 COMMAND BLOCK.32) port:%X\n",
                 port);
          ret= true;
          *data= 0xffffffff;
          exit(EXIT_FAILURE);
          break;
        case 0x0374 ... 0x0377:
          printf("[EE] READ (IDE1 CONTROL BLOCK.32) port:%X\n",
                 port);
          ret= true;
          *data= 0xffffffff;
          exit(EXIT_FAILURE);
          break;
        }
    }
  if ( ret ) return ret;
  
  // BMIBA
  base= (_pci_regs.bmiba&0x0000FFF0);
  if ( port >= base && port < base+16 )
    {
      iport= port-base;
      switch ( iport )
        {
        default:
          _warning (  _udata,
                      "PC_piix4_ide_port_read32 -> unknown port %04X (%04X)",
                      port, iport );
          *data= 0xFFFFFFFF;
          
        }
      ret= true;
    }
  else ret= false;
  
  return ret;

} // end PC_piix4_ide_port_read32


bool
PC_piix4_ide_port_write8 (
                         const uint16_t port,
                         const uint8_t  data
                         )
{

  uint16_t base,iport;
  bool ret;


  if ( !(_pci_regs.pcicmd&PCICMD_IOSE) ) return false;

  clock ( false );
  
  ret= false;
  
  // IDE0 registers
  if ( _pci_regs.idetim[0]&IDETIM_IDE )
    {
      switch ( port )
        {
        case 0x01f1: // Features Register
          if ( _dev[0].drv[0].type!=PC_IDE_DEVICE_TYPE_NONE ||
               _dev[0].drv[1].type!=PC_IDE_DEVICE_TYPE_NONE )
            _dev[0].features= data;
          ret= true;
          break;
        case 0x01f2: // Sector Count
          if ( _dev[0].drv[0].type!=PC_IDE_DEVICE_TYPE_NONE ||
               _dev[0].drv[1].type!=PC_IDE_DEVICE_TYPE_NONE )
            _dev[0].sector_count= data;
          ret= true;
          break;
        case 0x01f3: // Sector Number Register (LBAlo)
          ide_sector_number_lbalo_write ( 0, data );
          ret= true;
          break;
        case 0x01f4: // Cylinder Low Register / (LBAmid)
          ide_cylinder_low_lbamid_write ( 0, data );
          ret= true;
          break;
        case 0x01f5: // Cylinder High Register / (LBAhi)
          ide_cylinder_high_lbahi_write ( 0, data );
          ret= true;
          break;
        case 0x01f6: // Drive/Head
          ide_drive_head_write ( 0, data );
          ret= true;
          break;
        case 0x01f7: // Command Register 
          ide_command ( 0, data );
          ret= true;
          break;
        case 0x01f0:
          printf("[EE] WRITE (IDE0 COMMAND BLOCK.8)"
                 " port:%X data:%X\n", port, data );
          ret= true;
          exit(EXIT_FAILURE);
          break;
        case 0x03f6: // Device control
          ide_control_write ( 0, data );
          ret= true;
          break;
        case 0x03f7:
        case 0x03f4 ... 0x03f5:
          printf("[EE] WRITE (IDE0 CONTROL BLOCK.8)"
                 " port:%X data:%X\n", port, data );
          ret= true;
          exit(EXIT_FAILURE);
          break;
        }
    }
  if ( ret ) { update_cc_to_event (); return ret; }
  
  // IDE1 registers
  if ( _pci_regs.idetim[1]&IDETIM_IDE )
    {
      switch ( port )
        {
        case 0x0171: // Features Register
          if ( _dev[1].drv[0].type!=PC_IDE_DEVICE_TYPE_NONE ||
               _dev[1].drv[1].type!=PC_IDE_DEVICE_TYPE_NONE )
            _dev[1].features= data;
          ret= true;
          break;
        case 0x0172: // Sector Count
          if ( _dev[1].drv[0].type!=PC_IDE_DEVICE_TYPE_NONE ||
               _dev[1].drv[1].type!=PC_IDE_DEVICE_TYPE_NONE )
            _dev[1].sector_count= data;
          ret= true;
          break;
        case 0x0173: // Sector Number Register (LBAlo)
          ide_sector_number_lbalo_write ( 1, data );
          ret= true;
          break;
        case 0x0174: // Cylinder Low Register / (LBAmid)
          ide_cylinder_low_lbamid_write ( 1, data );
          ret= true;
          break;
        case 0x0175: // Cylinder High Register / (LBAhi)
          ide_cylinder_high_lbahi_write ( 1, data );
          ret= true;
          break;
        case 0x0176: // Drive/Head
          ide_drive_head_write ( 1, data );
          ret= true;
          break;
        case 0x0177: // Command Register
          ide_command ( 1, data );
          ret= true;
          break;
        case 0x0170:
          printf("[EE] WRITE (IDE1 COMMAND BLOCK.8)"
                 " port:%X data:%X\n", port, data );
          ret= true;
          exit(EXIT_FAILURE);
          break;
        case 0x0376: // Device control
          ide_control_write ( 1, data );
          ret= true;
          break;
        case 0x0377:
        case 0x0374 ... 0x0375:
          printf("[EE] WRITE (IDE1 CONTROL BLOCK.8)"
                 " port:%X data:%X\n", port, data );
          ret= true;
          exit(EXIT_FAILURE);
          break;
        }
    }
  if ( ret ) { update_cc_to_event (); return ret; }

  // IDE Port3 i Port4 no implementat
  switch ( port )
    {
    case 0x01e8 ... 0x01ef:
    case 0x0168 ... 0x016f:
      _warning ( _udata,
                 "PC_piix4_ide_port_write8 -> unknown IDE port %04X",
                 port );
      ret= true;
      break;
    }
  if ( ret ) { update_cc_to_event (); return ret; }
  
  // BMIBA
  base= (_pci_regs.bmiba&0x0000FFF0);
  if ( port >= base && port < base+16 )
    {
      iport= port-base;
      switch ( iport )
        {
        default:
          _warning (  _udata,
                      "PC_piix4_ide_port_write8 -> unknown port %04X (%04X)",
                      port, iport );
        }
      ret= true;
    }
  else ret= false;

  update_cc_to_event ();
  
  return ret;
  
} // end PC_piix4_ide_port_write8


bool
PC_piix4_ide_port_write16 (
                           const uint16_t port,
                           const uint16_t data
                           )
{
  
  uint16_t base,iport;
  bool ret;
  

  if ( !(_pci_regs.pcicmd&PCICMD_IOSE) ) return false;
  
  ret= false;

  clock ( false );
  
  // IDE0 registers
  if ( _pci_regs.idetim[0]&IDETIM_IDE )
    {
      switch ( port )
        {
          // Data Register 
        case 0x01f0:
          ide_data_write ( 0, data );
          ret= true;
          break;
          
        case 0x01f1 ... 0x01f7:
          printf("[EE] WRITE (IDE0 COMMAND BLOCK.16)"
                 " port:%X data:%X\n", port, data );
          ret= true;
          exit(EXIT_FAILURE);
          break;
        case 0x03f4 ... 0x03f7:
          printf("[EE] WRITE (IDE0 CONTROL BLOCK.16)"
                 " port:%X data:%X\n", port, data );
          ret= true;
          exit(EXIT_FAILURE);
          break;
        }
    }
  if ( ret ) { update_cc_to_event (); return ret; }

  // IDE1 registers
  if ( _pci_regs.idetim[1]&IDETIM_IDE )
    {
      switch ( port )
        {
          // Data Register 
        case 0x0170:
          ide_data_write ( 1, data );
          ret= true;
          break;
          
        case 0x0171 ... 0x0177:
          printf("[EE] WRITE (IDE1 COMMAND BLOCK.16)"
                 " port:%X data:%X\n", port, data );
          ret= true;
          exit(EXIT_FAILURE);
          break;
        case 0x0374 ... 0x0377:
          printf("[EE] WRITE (IDE1 CONTROL BLOCK.16)"
                 " port:%X data:%X\n", port, data );
          ret= true;
          exit(EXIT_FAILURE);
          break;
        }
    }
  if ( ret ) { update_cc_to_event (); return ret; }
  
  // BMIBA
  base= (_pci_regs.bmiba&0x0000FFF0);
  if ( port >= base && port < base+16 )
    {
      iport= port-base;
      switch ( iport )
        {
        default:
          _warning (  _udata,
                      "PC_piix4_ide_port_write16 -> unknown port %04X (%04X)",
                      port, iport );
        }
      ret= true;
    }
  else ret= false;

  update_cc_to_event ();
  
  return ret;
  
} // end PC_piix4_ide_port_write16


bool
PC_piix4_ide_port_write32 (
                           const uint16_t port,
                           const uint32_t data
                           )
{

  uint16_t base,iport;
  bool ret;
  

  if ( !(_pci_regs.pcicmd&PCICMD_IOSE) ) return false;

  clock ( false );
  
  ret= false;
  
  // IDE0 registers
  if ( _pci_regs.idetim[0]&IDETIM_IDE )
    {
      switch ( port )
        {
        case 0x01f0 ... 0x01f7:
          printf("[EE] WRITE (IDE0 COMMAND BLOCK.32)"
                 " port:%X data:%X\n", port, data );
          ret= true;
          exit(EXIT_FAILURE);
          break;
        case 0x03f4 ... 0x03f7:
          printf("[EE] WRITE (IDE0 CONTROL BLOCK.32)"
                 " port:%X data:%X\n", port, data );
          ret= true;
          exit(EXIT_FAILURE);
          break;
        }
    }
  if ( ret ) { update_cc_to_event (); return ret; }

  // IDE1 registers
  if ( _pci_regs.idetim[1]&IDETIM_IDE )
    {
      switch ( port )
        {
        case 0x0170 ... 0x0177:
          printf("[EE] WRITE (IDE1 COMMAND BLOCK.32)"
                 " port:%X data:%X\n", port, data );
          ret= true;
          exit(EXIT_FAILURE);
          break;
        case 0x0374 ... 0x0377:
          printf("[EE] WRITE (IDE1 CONTROL BLOCK.32)"
                 " port:%X data:%X\n", port, data );
          ret= true;
          exit(EXIT_FAILURE);
          break;
        }
    }
  if ( ret ) { update_cc_to_event (); return ret; }
  
  // BMIBA
  base= (_pci_regs.bmiba&0x0000FFF0);
  if ( port >= base && port < base+16 )
    {
      iport= port-base;
      switch ( iport )
        {
        default:
          _warning (  _udata,
                      "PC_piix4_ide_port_write32 -> unknown port %04X (%04X)",
                      port, iport );
        }
      ret= true;
    }
  else ret= false;

  update_cc_to_event ();
  
  return ret;
  
} // end PC_piix4_ide_port_write32


void
PC_piix4_ide_get_next_cd_audio_sample (
                                       int16_t *l,
                                       int16_t *r
                                       )
{

  int16_t sample0,sample1,sample_l,sample_r;
  cdrom_t *cd;
  
  
  if ( _sound_dev != NULL &&
       _sound_dev->cdrom.playing &&
       !_sound_dev->cdrom.paused )
    {
      cd= &(_sound_dev->cdrom);
      if ( cd->audio.p == cd->audio.l &&
           !cdrom_play_load_next_sector ( _sound_dev_ide, _sound_dev ) )
        {
          *l= 0;
          *r= 0;
        }
      else
        {
          
          // Llig mostres
          sample_l= (int16_t)
            (((uint16_t) cd->audio.v[cd->audio.p]) |
             (((uint16_t) cd->audio.v[cd->audio.p+1])<<8));
          sample_r= (int16_t)
            (((uint16_t) cd->audio.v[cd->audio.p+2]) |
             (((uint16_t) cd->audio.v[cd->audio.p+3])<<8));
          cd->audio.p+= 4;
          assert ( cd->audio.p <= cd->audio.l );
          
          // Obté mostra port0 (L)
          switch ( cd->mode.cdrom_audio_control_parameters.chn_port0 )
            {
            case 0: sample0= 0; break;
            case 1: sample0= sample_l; break;
            case 2: sample0= sample_r; break;
            default:
              sample0= 0;
              _warning ( _udata, "AUDIO CDROM - chn_port0:%d no suportat",
                         cd->mode.cdrom_audio_control_parameters.chn_port0 );
            }
          *l= (int16_t)
            ((((int) sample0)*
              (int) ((unsigned int) cd->
                     mode.cdrom_audio_control_parameters.vol_port0))/255);

          // Obté mostra port1 (R)
          switch ( cd->mode.cdrom_audio_control_parameters.chn_port1 )
            {
            case 0: sample1= 0; break;
            case 1: sample1= sample_l; break;
            case 2: sample1= sample_r; break;
            default:
              sample1= 0;
              _warning ( _udata, "AUDIO CDROM - chn_port1:%d no suportat",
                         cd->mode.cdrom_audio_control_parameters.chn_port1 );
            }
          *r= (int16_t)
            ((((int) sample1)*
              (int) ((unsigned int) cd->
                     mode.cdrom_audio_control_parameters.vol_port1))/255);
          
          
        }
    }
  else
    {
      *l= 0;
      *r= 0;
    }
  
} // end PC_piix4_ide_get_next_cd_audio_sample
