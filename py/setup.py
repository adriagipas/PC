from distutils.core import setup, Extension
import sys,subprocess

def check_version(current,required):
    a= [int(x) for x in current.split('.')]
    b= [int(x) for x in required.split('.')]
    length= min(len(a),len(b))
    for i in range(0,length):
        if a[i]>b[i]: return True
        elif a[i]<b[i]: return False
    return True

def pkgconfig_exe(lib,extra):
    ret= subprocess.check_output(['pkg-config',lib]+extra)
    return ret.decode('utf-8')

def pkgconfig(lib,req_version):
    try:
        version= pkgconfig_exe(lib,['--modversion'])
        if not check_version(version,req_version):
            sys.exit(('%s >= %s not found, current'+
                      ' version is %s')%(lib,req_version,version))
        libs= pkgconfig_exe(lib,['--libs'])
        libs= libs.replace('-l','').split()
        cflags= pkgconfig_exe(lib,['--cflags']).split()
        return libs,cflags
    except Exception as e:
        sys.exit()

glib_libs,glib_cflags= pkgconfig('glib-2.0','2.50')

module= Extension ( 'PC',
                    sources= [ 'pcmodule.c',
                               '../src/cdrom.c',
                               '../src/files.c',
                               '../src/mtxc.c',
                               '../src/piix4_power_management.c',
                               '../src/rtc.c',
                               '../src/svga_cirrus_clgd5446.c',
                               '../src/cpu.c',
                               '../src/ic.c',
                               '../src/piix4.c',
                               '../src/piix4_usb.c',
                               '../src/sound_blaster16.c',
                               '../src/timers.c',
                               '../src/dma.c',
                               '../src/io.c',
                               '../src/piix4_ide.c',
                               '../src/pmtimer.c',
                               '../src/sound.c',
                               '../src/fd.c',
                               '../src/main.c',
                               '../src/piix4_pci_isa_bridge.c',
                               '../src/ps2.c',
                               '../src/speaker.c',
                               'IA32/src/cpu.c',
                               'IA32/src/dis.c',
                               'IA32/src/interpreter.c',
                               'IA32/src/jit.c',
                               'CD/src/crc.c',
                               'CD/src/cue.c',
                               'CD/src/info.c',
                               'CD/src/iso.c',
                               'CD/src/new.c',
                               'CD/src/utils.c'],
                    depends= [ '../src/PC.h',
                               'IA32/src/IA32.h',
                               'IA32/src/interpreter_call_jmp.h',
                               'IA32/src/interpreter_lop.h',
                               'IA32/src/interpreter_set.h',
                               'IA32/src/jit_pag.h',
                               'IA32/src/interpreter_add.h',
                               'IA32/src/interpreter_cmp_sub.h',
                               'IA32/src/interpreter_mov.h',
                               'IA32/src/interpreter_shift_rols.h',
                               'IA32/src/interpreter_bits_bytes.h',
                               'IA32/src/interpreter_fpu.h',
                               'IA32/src/interpreter_mul_div.h',
                               'IA32/src/jit_compile.h',
                               'IA32/src/interpreter_cache.h',
                               'IA32/src/interpreter_io.h',
                               'IA32/src/interpreter_other.h',
                               'IA32/src/jit_exec.h',
                               'CD/src/CD.h',
                               'CD/src/crc.h',
                               'CD/src/cue.h',
                               'CD/src/iso.h',
                               'CD/src/utils.h'],
                    libraries= ['SDL']+glib_libs,
                    extra_compile_args= glib_cflags+['-UNDEBUG',
                                                     '-frounding-math',
                                                     '-Wno-unknown-pragmas'],
                    define_macros= [('__LITTLE_ENDIAN__',None),
                                    ('PC_DEBUG',None)],
                    include_dirs= ['../src', 'IA32/src', 'CD/src'])

setup ( name= 'PC',
        version= '1.0',
        description= 'Pentium PC simulator',
        ext_modules= [module] )
