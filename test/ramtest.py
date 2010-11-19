#! /usr/bin/env python

# DiscFerret ATE code, based on code by Julius Constante

import sys, struct, datetime, random, time, math
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

# Set to False to disable FPGA microcode loading (use when debugging over JTAG)
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

	print "Load complete.",

print "FPGA status:",
# poll fpga status
resp = dev.fpgaGetLoadStatus()
if resp == ERR_FPGA_NOT_CONF:
	print "FPGA is waiting for microcode load. Microcode load error."
	sys.exit(-1)
elif resp == ERR_OK:
	print "FPGA microcode is active. Microcode load succeeded."
else:
	print "FPGA status code unknown, is %d, wanted %d or %d" % (resp, ERR_OK, ERR_FPGA_NOT_CONF)
	sys.exit(-1)

print "DEVICE INFORMATION:"
print dev.getDeviceInfo()

#############################################################################

err=False
print
print "####################################"
print "# FPGA <==> MCU COMMUNICATION TEST #"
print "####################################"
print

# --- Fixed 0x55/0xAA register test ---
aval = dev.peek(FIXED55)
if (aval != 0x55):
	print "ERROR: Read from Fixed55 returned 0x%02X, wanted 0x55" % aval
	err = True
else:
	print "Read from Fixed55 passed."

aval = dev.peek(FIXEDAA)
if (aval != 0xAA):
	print "ERROR: Read from FixedAA returned 0x%02X, wanted 0xAA" % aval
	err = True
else:
	print "Read from FixedAA passed."
print

# --- Clock ticker test ---
aval = dev.peek(CLOCK_TICKER)
bval = dev.peek(CLOCK_TICKER)
if (aval == bval):
	# if we get two values the same, try again
	aval = dev.peek(CLOCK_TICKER)
# two values the same, twice in a row. the clock isn't running.
if (aval == bval):
	print "ERROR: FPGA clock oscillator not running. Got values %d and %d." % (aval, bval)
	err = True
else:
	print "FPGA oscillator running. Got values %d and %d." % (aval, bval)

# --- Clock ticker test (PLL) ---
aval = dev.peek(CLOCK_TICKER_PLL)
bval = dev.peek(CLOCK_TICKER_PLL)
if (aval == bval):
	# if we get two values the same, try again
	aval = dev.peek(CLOCK_TICKER_PLL)
# two values the same, twice in a row. the clock isn't running.
if (aval == bval):
	print "ERROR: FPGA PLL not running. Got values %d and %d." % (aval, bval)
	err = True
else:
	print "FPGA PLL running. Got values %d and %d." % (aval, bval)

# --- Scratchpad test ---
for testval in [0x55, 0xaa, 0x00, 0xff, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0]:
	dev.poke(SCRATCHPAD, testval)
	pval = dev.peek(SCRATCHPAD)
	ival = dev.peek(INVERSE_SCRATCHPAD)
	if (pval != testval):
		print "ERROR: Wrote 0x%02X to Scratchpad, got pval 0x%02X back." % (testval, pval)
		err=True
	if (ival != (testval ^ 0xff)):
		print "ERROR: Wrote 0x%02X to Scratchpad, got ival 0x%02X back." % (testval, ival)
		err=True

if err:
	print "Errors occurred, aborting test sequence."
	sys.exit(-1)
else:
	print "All tests passed."
	print

#############################################################################

err = False
print
print "########################"
print "# ADDRESS COUNTER TEST #"
print "########################"
print
for val in [0, 0x07ffff, 123456, 0x055555, 0x02aaaa]:
	print "Set address to 0x%06X - response %d..." % (val, dev.setRAMAddr(val)),
	ra = dev.getRAMAddr()
	if ra != val:
		print "Readback error! Got 0x%06X" % ra
		err = True
		break
	else:
		print "RAM addr set OK!"

# TODO: test autoincrement for read and write

if err:
	print "Error occurred, aborting test sequence."
	sys.exit(-1)
else:
	print "All tests passed."
	print

#############################################################################

err = False
print
print "###################"
print "# STATIC RAM TEST #"
print "###################"
print

for RAMLEN in [512*1024, 256*1024, 128*1024, 64*1024, 32*1024, 16*1024, 8*1024, 4*1024, 2*1024, 1024, 512, 256, 128, 64, 32, 16, 8, 4, 2]:
	# RAM length
	# TODO: run the RAM test for 512K, then if it fails, cycle back. This will
	# reveal the location of faulty address bits :)
	#RAMLEN=1024*512
	# maximum address counter size
	MAXADDR=(1024*512)-1

	print "set addr to zero, resp: %d" % dev.setRAMAddr(0)
	ra = dev.getRAMAddr()
	if (ra != 0):
		print "ERROR: RAM address was not zero after Set-to-Zero!"
		sys.exit(-1)
	print

	print "Generating %d bytes of randomness..." % RAMLEN,
	buf = []
	for x in range(RAMLEN):
		buf.append(int(random.getrandbits(8)))
	print "done!"
	print

	print "Writing %d bytes of data to RAM..." % len(buf)
	i = len(buf)
	j = 0
	while i > 0:
		if (i > 32):
			dev.ramWrite(buf[j:j+32])
			j = j + 32
			i = i - 32
		else:
			dev.ramWrite(buf[j:j+i])
			j = j + i
			i = 0

	ra = dev.getRAMAddr()
	print "RAM pointer = 0x%06X (%d)" % (ra, ra)
	if (ra != (RAMLEN & MAXADDR)):
		print "ERROR! I wanted 0x%06X" % (RAMLEN & MAXADDR)
		sys.exit(-1)

	# TODO: check flags!
	dev.debug_dump_status()

	## read back test
	print
	print "Reading %d bytes of data from RAM..." % RAMLEN
	print "set addr to zero, resp: %d" % dev.setRAMAddr(0)
	ra = dev.getRAMAddr()
	if (ra != 0):
		print "ERROR: RAM address was not zero after Set-to-Zero!"
		sys.exit(-1)
	dev.debug_dump_status()

	i = RAMLEN
	rbuf = []
	while (i > 0):
		if (i > 32):
			x = dev.ramRead(32)
			i = i - 32
		else:
			x = dev.ramRead(i)
			i = 0
		rbuf.extend(x)
	print "done."

	ra = dev.getRAMAddr()
	print "RAM pointer = 0x%06X (%d)" % (ra, ra)
	if (ra != ((RAMLEN+1) & MAXADDR)):
		print "ERROR! I wanted 0x%06X" % ((RAMLEN+1) & MAXADDR)
		sys.exit(-1)

	print "len buf:  %d" % len(buf)
	print "len rbuf: %d" % len(rbuf)

	print "buffer compare..."
	cerr = 0
	for x in range(len(buf)):
		if (rbuf[x] != buf[x]):
			print "COMPARE ERROR at 0x%06X" % x
			print "\tWrote 0x%02X, read 0x%02X" % (buf[x], rbuf[x])
			cerr = cerr + 1
			if (cerr == 10):
				print "too many compare errors"
				break
	if cerr == 0:
		print "compare OK! RAMLen = %d bytes" % RAMLEN
		break
	print

if (RAMLEN < 512*1024):
	print "RAM test failure: address line A%d" % (math.log(RAMLEN, 2))
else:
	print "RAM test successful."
print
