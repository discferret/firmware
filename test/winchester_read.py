#! /usr/bin/env python

#############################################################################
# DiscFerret Winchester disc read test
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

# TODO: set to the maximum possible cylinder
curtrk = 1024

def seek(dev, trk):
	global curtrk
	print "current track %d" % curtrk
	if trk < curtrk:
		ntracks = curtrk - trk
		direction = STEP_CMD_TOWARDS_ZERO
		print "seek %d towards zero" % ntracks
	else:
		ntracks = trk - curtrk
		direction = STEP_CMD_AWAYFROM_ZERO
		print "seek %d away from zero" % ntracks

	print "seek to %d..." % trk
	while ntracks > 0:
		if (ntracks > STEP_COUNT_MASK):
			i = STEP_COUNT_MASK
		else:
			i = ntracks

		# do the seek
		dev.poke(STEP_CMD, direction | i)
		ntracks = ntracks - i

		#dev.debug_dump_status()
		# wait for STEPPING to be inactive, and SEEK COMPLETE to be active
		stat = STATUS2_STEPPING
		while ((stat & STATUS2_STEPPING) != 0) or (stat & STATUS2_DENSITY != STATUS2_DENSITY):
			stat = dev.peek(STATUS2)
	dev.debug_dump_status()
	curtrk = trk


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

# reset ram address
print "set ram addr to zero, resp: %d" % dev.setRAMAddr(0)
# get hardware status
dev.debug_dump_status()
print

# abort any currently running acquisition
print "abort acquisition: %d" % dev.poke(ACQCON, ACQCON_ABORT)
dev.debug_dump_status()
print

MFM_SYNC=False
if MFM_SYNC:
	# Demo of MFM-synched start/stop for 3.5in DSDD (720K) discs
	# set start/stop sync words to 0x4489
	print "set start mfm hi: resp %d" % dev.poke(MFM_SYNCWORD_START_H, 0x44)
	print "set start mfm lo: resp %d" % dev.poke(MFM_SYNCWORD_START_L, 0x89)
	print "set stop  mfm hi: resp %d" % dev.poke(MFM_SYNCWORD_STOP_H,  0x44)
	print "set stop  mfm lo: resp %d" % dev.poke(MFM_SYNCWORD_STOP_L,  0x89)

	print "set start mask hi: resp %d" % dev.poke(MFM_MASK_START_H, 0xFF)
	print "set start mask lo: resp %d" % dev.poke(MFM_MASK_START_L, 0xFF)
	print "set stop  mask hi: resp %d" % dev.poke(MFM_MASK_STOP_H,  0xFF)
	print "set stop  mask lo: resp %d" % dev.poke(MFM_MASK_STOP_L,  0xFF)

	print "set start event: resp %d" % dev.poke(ACQ_START_EVT, ACQ_EVENT_MFM)
	print "set start count: resp %d" % dev.poke(ACQ_START_NUM, 1)
	print "set stop  event: resp %d" % dev.poke(ACQ_STOP_EVT, ACQ_EVENT_MFM)
	print "set stop  count: resp %d" % dev.poke(ACQ_STOP_NUM, 31)
	print "set mfm clock rate: resp %d" % dev.poke(MFM_CLKSEL, MFM_CLKSEL_250KBPS)
else:
	# start event: index pulse, second instance
	print "set start event: resp %d" % dev.poke(ACQ_START_EVT, ACQ_EVENT_INDEX)
	print "set start count: resp %d" % dev.poke(ACQ_START_NUM, 1)
	# stop event: index pulse, first instance
	print "set stop  event: resp %d" % dev.poke(ACQ_STOP_EVT, ACQ_EVENT_INDEX)
	print "set stop  count: resp %d" % dev.poke(ACQ_STOP_NUM, 0)

# HSTMD thresholds don't need to be set

# set step rate to fastest (250us)
steprate=250
print "set step rate: resp %d" % dev.poke(STEP_RATE, steprate/250)

# set HSIO direction to all inputs
print "set hsio dir all-in: resp %d" % dev.poke(HSIO_DIR, 0xff)

# select winchester drive 0
print "select: resp %d" % dev.poke(DRIVE_CONTROL, DRIVE_CONTROL_DS0)
time.sleep(3)

# seek to track zero
seek(dev, 0)
time.sleep(1)

# seek to desired track
seek(dev, 100)
time.sleep(1)

# start acquisition
print "start acq: resp %d" % dev.poke(ACQCON, ACQCON_START)
# dump status
dev.debug_dump_status()

foo=True
while foo:
	# wait for acquisition to start
	stat = 0
	while (stat & STATUS1_ACQSTATUS_MASK) != STATUS1_ACQ_ACQUIRING:
		stat = dev.peek(STATUS1)
#	print "acquisition started."
	dev.debug_dump_status()
	stat = STATUS1_ACQ_ACQUIRING
	while (stat & STATUS1_ACQSTATUS_MASK) == STATUS1_ACQ_ACQUIRING:
		stat = dev.peek(STATUS1)
	foo=False
print "acquisition completed."
dev.debug_dump_status()
nbytes = dev.getRAMAddr()-1
print "acquired %d bytes of data" % (nbytes)

# now save the RAM data
dev.setRAMAddr(0)
ramdata = dev.ramRead(nbytes)

f=open("dat.bin", "wb")
g=open("dat", "wt")
h=open("dat.his", "wt")
s=""
# binary dump
for x in ramdata:
	s = s + struct.pack('B', x)
f.write(s)
f.close()

# text dump
s=""
carry=0
pos=0
cdat=[]
for x in ramdata:
	if (x&0x7f)==0:
		carry += 128
	else:
		s = s + ("%d %d\n" % (pos, carry + (x & 0x7f)))
		cdat.append(carry + (x & 0x7f))
		carry = 0
		pos = pos + 1
g.write(s)
g.close()

# histogram
s=""
maxv=0
for x in cdat:
	if (x > maxv):
		maxv = x
histogram=[0]*(maxv+1)
for x in cdat:
	histogram[x] = histogram[x] + 1
pos = 0
for x in histogram:
	s = s + ("%d %d\n" % (pos, x))
	pos = pos + 1
h.write(s)
h.close()

# deselect all drives
print "deselect: resp %d" % dev.poke(DRIVE_CONTROL, 0)

print
