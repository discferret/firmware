#! /usr/bin/env python

#############################################################################
# DiscFerret disc write test
#############################################################################

import sys, usb, struct, datetime, random, time
from discferret import *

#############################################################################

LoadFPGA = True

#############################################################################

# open the discferret
dev = DiscFerret()
if not dev.open():
	print "Could not open device, is it connected?"
	sys.exit(-1)
else:
	print "Device opened successfully"

if LoadFPGA:
	print "Initialising FPGA...",
	# FPGA LOAD INIT -- start a microcode load
	resp = dev.fpgaLoadBegin()
	if resp == ERR_OK:
		print "OK!",
	else:
		print "Failed with status code %d" % resp
		sys.exit(-1)

	# poll fpga status
	resp = dev.fpgaGetLoadStatus()
	if resp == ERR_FPGA_NOT_CONF:
		print "(FPGA is waiting for microcode load)"
	elif resp == ERR_OK:
		print "(FPGA microcode is active)"
	else:
		print "FPGA status code unknown, is %d, wanted %d or %d" % (resp, ERR_OK, ERR_FPGA_NOT_CONF)
		sys.exit(-1)

	# load RBF file
	try:
		f = open("microcode.rbf", "rb")
		rbfstr = f.read()
		if len(rbfstr) < 1:
			raise -1
		f.close()
	except:
		print "Microcode file read error"
		sys.exit(-1)

	print "RBF file contains %d data bytes" % len(rbfstr)

	# bitswap the RBF file
	rbf = list()
	for x in range(len(rbfstr)):
		rbf.append(bitswap(struct.unpack('B', rbfstr[x])[0]))

	# now send the RBF to the PIC
	pos = 0
	while pos < len(rbf):
		# if we have more than 62 bytes to send, then send the first 62
		if (len(rbf)-pos) > 62:
			i = 62
		else:
			i = (len(rbf)-pos)

		resp = dev.fpgaLoadBlock(rbf[pos:pos+i])
		if resp != ERR_OK:
			print "FPGA microcode block transfer failed at addr=%04X, err=%d" % (pos, resp)
			sys.exit(-1)

		# update pointer
		pos += i

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

# set step rate to 9ms
print "set step rate: resp %d" % dev.poke(STEP_RATE, 9000/250)

# reset ram address
print "set ram addr to zero, resp: %d" % dev.setRAMAddr(0)
# get hardware status
dev.debug_dump_status()
print

# build data block
dblk = []
dblk.append(0x01)					# SET WR GATE = 0x01
for x in range(((512*1024)/5)-128):	# leave some space for the header and trailer commands
#	dblk.append((120-5) | 0x80)		# WAIT 3uS
#	dblk.append((120-5) | 0x80)		# WAIT 3uS
	dblk.append((80-5) | 0x80)		# WAIT 2uS
	dblk.append((80-5) | 0x80)		# WAIT 2uS
	dblk.append((80-5) | 0x80)		# WAIT 2uS
	dblk.append((80-5) | 0x80)		# WAIT 2uS
	dblk.append(0x02)				# WRITE PULSE
dblk.append(0x00)					# SET WR GATE = 0x00
dblk.append(0x00)					# SET WR GATE = 0x00
dblk.append(0x7f)					# STOP

# write to ram
dev.setRAMAddr(0)
lp = len(dblk)
pos = 0
while lp > 0:
	if lp >= 61:
		data = dev.ramWrite(dblk[pos:pos+61])
		lp = lp - 61
		pos = pos + 61
	else:
		data = dev.ramWrite(dblk[pos:])
		lp = 0

# abort any currently running acquisition
print "abort acquisition: %d" % dev.poke(ACQCON, ACQCON_ABORT)
dev.debug_dump_status()
print

# select drive 0 (amstrad cable swaps DS0 and DS1)
print "select: resp %d" % dev.poke(DRIVE_CONTROL, DRIVE_CONTROL_DS0 | DRIVE_CONTROL_DS1 | DRIVE_CONTROL_DS2 | DRIVE_CONTROL_DS3 | DRIVE_CONTROL_MOTEN)# | DRIVE_CONTROL_SIDESEL)
time.sleep(3)
"""
# seek to track 40
print "seek to 40: resp %d" % dev.poke(STEP_CMD, STEP_CMD_AWAYFROM_ZERO | (10 & STEP_COUNT_MASK))
dev.debug_dump_status()
stat = STATUS2_STEPPING
while (stat & STATUS2_STEPPING) != 0:
	stat = dev.peek(STATUS2)
dev.debug_dump_status()
time.sleep(1)
"""
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
"""
# kick off the write
dev.setRAMAddr(0)
print "write: %d" % dev.poke(ACQCON, ACQCON_WRITE)
dev.debug_dump_status()
time.sleep(2)
stat = STATUS1_ACQ_WRITING
while ((stat & STATUS1_ACQSTATUS_MASK) == STATUS1_ACQ_WRITING) != 0:
	stat = dev.peek(STATUS2)
dev.debug_dump_status()

# deselect all drives
print "deselect: resp %d" % dev.poke(DRIVE_CONTROL, 0)
print

