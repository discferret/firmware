#! /usr/bin/env python

#############################################################################
# DiscFerret disc read test
#############################################################################

import sys, usb, struct, datetime, random, time
from discferret import *

#############################################################################

# swap the bits in a byte
def bitswap(num):
	val = 0
	for x in range(8):
		b = num&(1<<x) != 0
		val = val<<1 | b
	return val

#############################################################################

InitFPGA = True

#############################################################################

# open the discferret
dev = DiscFerret()
if not dev.open():
	print "Could not open device, is it connected?"
	sys.exit(-1)
else:
	print "Device opened successfully"

if InitFPGA:
	print "FPGA load: status code %d" % dev.fpgaLoadRBFFile("microcode.rbf")

# poll fpga status
print "Load complete. FPGA status:",
resp = dev.fpgaGetLoadStatus()
if resp == ERR_FPGA_NOT_CONF:
	print "FPGA is waiting for microcode load. Microcode load error."
	sys.exit(-1)
elif resp == ERR_OK:
	print "FPGA microcode is active. Microcode load succeeded."
else:
	print "FPGA status code unknown, is %d, wanted %d or %d" % (resp, ERR_OK, ERR_FPGA_NOT_CONF)
	sys.exit(-1)

print dev.getDeviceInfo()

print "set addr to zero, resp: %d" % dev.setRAMAddr(0)
print "current ram address: 0x%06X" % dev.getRAMAddr()
#dblk = range(256)
dblk = range(256) * (512*1024 / 256)
print "write %d byte block:" % len(dblk)
t1 = time.time()
dev.ramWrite(dblk)
te = time.time() - t1
print "elapsed clock time: %0.3f" % te
print "%d bytes in %0.3f secs = %0.3f bytes/sec" % (len(dblk), te, len(dblk) / te)
print "current ram address: 0x%06X" % dev.getRAMAddr()
print

print "set addr to zero, resp: %d" % dev.setRAMAddr(0)
print "current ram address: 0x%06X" % dev.getRAMAddr()
print "read %d byte block:" % len(dblk)
t1 = time.time()
k = dev.ramRead(len(dblk))
te = time.time() - t1
print "elapsed clock time: %0.3f" % te
print "%d bytes in %0.3f secs = %0.3f bytes/sec" % (len(dblk), te, len(dblk) / te)
print "current ram address: 0x%06X" % dev.getRAMAddr()

print "compare...",
if (list(k) == list(dblk[:len(k)])):
	print "ok"
else:
	print "err"

print
