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

"""
print "current ram address: 0x%06X" % dev.getRAMAddr()
print "set addr to zero, resp: %d" % dev.setRAMAddr(0)
print "current ram address: 0x%06X" % dev.getRAMAddr()
print "read 40 byte block:\n   ",
print dev.ramRead(40)
print "current ram address: 0x%06X" % dev.getRAMAddr()
print

print "set addr to zero, resp: %d" % dev.setRAMAddr(0)
print "current ram address: 0x%06X" % dev.getRAMAddr()
print "write incrementing sequence: %d" % dev.ramWrite(range(32))
print "  ",
print range(32)
print "current ram address: 0x%06X" % dev.getRAMAddr()
print

# POKE DB: Poke DeBug. Used to trigger SignalTap.
print "poke 0xDB=0x55: %d" % dev.poke(0xDB, 0x55)
print "current ram address: 0x%06X" % dev.getRAMAddr()
print "set addr to zero, resp: %d" % dev.setRAMAddr(0)
print "current ram address: 0x%06X" % dev.getRAMAddr()
print "read 40 byte block:\n   ",
print dev.ramRead(40)
print "current ram address: 0x%06X" % dev.getRAMAddr()
print

print "set addr to zero, resp: %d" % dev.setRAMAddr(0)
print "current ram address: 0x%06X" % dev.getRAMAddr()
a = list()
for i in range(32):
	a.append(int(random.getrandbits(8)))
print "write random sequence: %d" % dev.ramWrite(a)
print "  ",
print a
print "current ram address: 0x%06X" % dev.getRAMAddr()
print

# POKE DB: Poke DeBug. Used to trigger SignalTap.
print "poke 0xDB=0x55: %d" % dev.poke(0xDB, 0x55)
print "current ram address: 0x%06X" % dev.getRAMAddr()
print "set addr to zero, resp: %d" % dev.setRAMAddr(0)
print "current ram address: 0x%06X" % dev.getRAMAddr()
print "read 40 byte block:\n   ",
print dev.ramRead(40)
print "current ram address: 0x%06X" % dev.getRAMAddr()
print
"""

# reset ram address
print "set ram addr to zero, resp: %d" % dev.setRAMAddr(0)
# get hardware status
dev.debug_dump_status()
print

# abort any currently running acquisition
print "abort acquisition: %d" % dev.poke(ACQCON, ACQCON_ABORT)
dev.debug_dump_status()
print

SYNC_MODE="hstmd"
if SYNC_MODE == "mfm":
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
elif SYNC_MODE == "hstmd":
	# start event: HSTMD
	# 1rpm = 1 rev in 60 secs. convert to Hz.
	# 300RPM rotation speed, 10 sectors, 500us resolution
	# then multiply by 0.75 to find worst-case minimum sector time
	threshold = ((60.0/300.0)/10.0) * 0.75
	print "hstmd threshold: %f secs" % threshold
	threshold = int(threshold / 500.0e-6)
	print "hstmd threshold ival: %d counts" % threshold
	print "set hstmd start thr: %d" % dev.poke(ACQ_HSTMD_THR_START, threshold)
	print "set hstmd stop  thr: %d" % dev.poke(ACQ_HSTMD_THR_STOP, threshold)
	# start event: HSTMD detect, second instance
	print "set start event: resp %d" % dev.poke(ACQ_START_EVT, ACQ_EVENT_IMMEDIATE | ACQ_EVENT_WAIT_HSTMD)
	print "set start count: resp %d" % dev.poke(ACQ_START_NUM, 1)
	# stop event: index pulse, 12th instance
	print "set stop  event: resp %d" % dev.poke(ACQ_STOP_EVT, ACQ_EVENT_INDEX)
	print "set stop  count: resp %d" % dev.poke(ACQ_STOP_NUM, 11)
else:
	# start event: index pulse, second instance
	print "set start event: resp %d" % dev.poke(ACQ_START_EVT, ACQ_EVENT_INDEX)
	print "set start count: resp %d" % dev.poke(ACQ_START_NUM, 1)
	# stop event: index pulse, first instance
	print "set stop  event: resp %d" % dev.poke(ACQ_STOP_EVT, ACQ_EVENT_INDEX)
	print "set stop  count: resp %d" % dev.poke(ACQ_STOP_NUM, 0)

# set step rate to 9ms
print "set step rate: resp %d" % dev.poke(STEP_RATE, 9000/250)

# select drive 0 (PC cable -- DS2=MOTEN iirc)
# -- PC drive A
#print "select: resp %d" % dev.poke(DRIVE_CONTROL, DRIVE_CONTROL_DS0 | DRIVE_CONTROL_DS2)
# -- BBC Micro drive 0
print "select: resp %d" % dev.poke(DRIVE_CONTROL, DRIVE_CONTROL_DS0 | DRIVE_CONTROL_MOTEN)
# -- Shotgun (Select All)
#print "select: resp %d" % dev.poke(DRIVE_CONTROL, DRIVE_CONTROL_DS0 | DRIVE_CONTROL_DS1 | DRIVE_CONTROL_DS2 | DRIVE_CONTROL_DS3 | DRIVE_CONTROL_MOTEN) # | DRIVE_CONTROL_SIDESEL)
time.sleep(3)

# seek to track zero
print "seek to zero: resp %d" % dev.poke(STEP_CMD, STEP_CMD_TOWARDS_ZERO | (90 & STEP_COUNT_MASK))
dev.debug_dump_status()
stat = STATUS2_STEPPING
while (stat & STATUS2_STEPPING) != 0:
	stat = dev.peek(STATUS2)
dev.debug_dump_status()
time.sleep(1)
"""
# seek to desired track
trk=40
print "seek to %d: resp %d" % (trk, dev.poke(STEP_CMD, STEP_CMD_AWAYFROM_ZERO | (trk & STEP_COUNT_MASK)))
dev.debug_dump_status()
stat = STATUS2_STEPPING
while (stat & STATUS2_STEPPING) != 0:
	stat = dev.peek(STATUS2)
dev.debug_dump_status()
time.sleep(1)

# seek to track zero
print "seek to zero: resp %d" % dev.poke(STEP_CMD, STEP_CMD_TOWARDS_ZERO | (90 & STEP_COUNT_MASK))
dev.debug_dump_status()
stat = STATUS2_STEPPING
while (stat & STATUS2_STEPPING) != 0:
	stat = dev.peek(STATUS2)
dev.debug_dump_status()
"""
# head settling time
time.sleep(1)

# get index freq
ixfrq = dev.peek(INDEX_FREQ_HI) << 8;
ixfrq = ixfrq + dev.peek(INDEX_FREQ_LO);
print "** measured index frequency: %d counts" % ixfrq
print "** equals: %d microseconds, or %f milliseconds (%f RPM)" % (ixfrq * 250, ixfrq * 0.250, (60/(ixfrq * 0.000250)))

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
		# TODO: maybe this needs to be 127 (see bad_cpc_disc scatter graph)
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
