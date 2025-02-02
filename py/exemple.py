import PC
import sys

if len(sys.argv)!=4:
    sys.exit('%s <BIOS> <VGABIOS> <HDD>'%sys.argv[0])
bios_fn= sys.argv[1]
vgabios_fn= sys.argv[2]
hdd_fn= sys.argv[3]

PC.init(open(bios_fn,'rb').read(),
        open(vgabios_fn,'rb').read(),
        hdd_fn,
        use_unix_epoch=True)
PC.jit_loop()
PC.close()
