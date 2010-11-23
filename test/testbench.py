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

#############################################################################

err = False
print
print "######################"
print "# FPGA LOOPBACK TEST #"
print "######################"
print

for mode in range(2):
	if mode == 0:
		rbffile = "SioATE_pin2sio.rbf"
	elif mode == 1:
		rbffile = "SioATE_sio2pin.rbf"
	else:
		print "Hmm, invalid mode! M=%d" % mode
		err = True
		break

	# Load SioATE
	print "Loading %s..." % rbffile
	resp = dev.fpgaLoadRBFFile(rbffile)
	print "FPGA load: status code %d" % resp
	if (resp != ERR_OK):
		print "Error loading FPGA microcode. Check FCNSTAT, FCNCONF, FCDONE, FCDCLK and FCDATA0."
		err = True
		break

	# Poll FPGA status
	resp = dev.fpgaGetLoadStatus()
	if resp == ERR_FPGA_NOT_CONF:
		print "FPGA is waiting for microcode load.. LOAD FAILED!"
		err = True
		break
	elif resp == ERR_OK:
		print "FPGA microcode is active.. LOAD SUCCEEDED!"
	else:
		print "FPGA status code unknown, is %d, wanted %d or %d" % (resp, ERR_OK, ERR_FPGA_NOT_CONF)
		err = True
		break

	# Now do some SSQ requests
	print "Sending SSQ requests..."
	for i in range(0x400):
		# Send the SSQ request
		resp = dev.secretSquirrel(i, mode)
		if (resp == None):
			err = True
			print "Hardware error, mode %d, value 0x%03X: Secret Squirrel can't hear the FPGA!" % (mode, i)
			break
		elif (resp != i):
			err = True
			print "Error: mode %d, wanted 0x%03X, got 0x%03X" % (mode, i, resp)
			break
	if not err:
		print "Loopback test succeeded, pass %d of 2" % (mode + 1)

print

if err:
	print "Errors occurred, aborting test sequence."
	print
	sys.exit(-1)
else:
	print "All tests passed."
	print

# wait time for LED blink
time.sleep(5)

"""
# Start by kicking the FPGA into Load mode. This forces it to tristate all
# its I/O pins. We want to do short-and-open testing, so we most definitely
# do NOT want the FPGA interfering.
# This would be so much easier if the PIC spoke JTAG..... then we could put
# it and the FPGA on a JTAG chain and do proper boundary scan testing. For
# now, this will have to do.
dev.fpgaLoadBegin()

# We have several different mask triads -- these are for PORTB, D, and E in
# sequence, and determine which port pins are tested. The tristate masks are
# generated auto-magically :3
"""
"""
	 * PORTB
	 *   5: PMALL	--> Parallel Master Port Address Load Low
	 *   4: PMALH	--> Parallel Master Port Address Load High
	 *
	 * PORTD
	 *   7: PMD7	}
	 *   6: PMD6	}
	 *   5: PMD5	}
	 *   4: PMD4	}	Parallel Master Port
	 *   3: PMD3	}	Data Bus
	 *   2: PMD2	}
	 *   1: PMD1	}
	 *   0: PMD0	}
	 *
	 * PORTE
	 *   1: PMWR	--> Parallel Master Port Write
	 *   0: PMRD	--> Parallel Master Port Read
""""""
triads = [
		[0b00100000, 0b00000000, 0b00000000, "PMALL"],		# PMALL
		[0b00010000, 0b00000000, 0b00000000, "PMALH"],		# PMALH
		[0b00000000, 0b10000000, 0b00000000, "PMD7"],		# PMD7
		[0b00000000, 0b01000000, 0b00000000, "PMD6"],		# PMD6
		[0b00000000, 0b00100000, 0b00000000, "PMD5"],		# PMD5
		[0b00000000, 0b00010000, 0b00000000, "PMD4"],		# PMD4
		[0b00000000, 0b00001000, 0b00000000, "PMD3"],		# PMD3
		[0b00000000, 0b00000100, 0b00000000, "PMD2"],		# PMD2
		[0b00000000, 0b00000010, 0b00000000, "PMD1"],		# PMD1
		[0b00000000, 0b00000001, 0b00000000, "PMD0"],		# PMD0
		[0b00000000, 0b00000000, 0b00000010, "PMWR"],		# PMWR
		[0b00000000, 0b00000000, 0b00000001, "PMRD"]		# PMRD
		]

# Build the Available I/O mask
mask = [0,0,0]
for t in triads:
	for i in range(3):
		mask[i] |= t[i]

for t in triads:
	# calculate tristate bits
	tris = [~t[0], ~t[1], ~t[2]]
	hot = t
	cold = [0,0,0]

	# Run tests with the bit set and cleared
	hresp = dev.secretSquirrel(tris, hot)
	cresp = dev.secretSquirrel(tris, cold)

	# When bit is set, nothing should be pulling it down
	if (((hresp[0] & t[0]) != t[0]) or
		((hresp[1] & t[1]) != t[1]) or
		((hresp[2] & t[2]) != t[2])):
		# ... but something is!
		print "Triad ", t[3], " stuck low"
		print "Triad: ", t, "\nColdR: ", cresp, "\nHotRs: ", hresp
		print "Triad: ", t, "\nColdP: ", cold, "\nHotPl: ", hot

	# When bit is clear, no other bits should be clear (ideally!)
	if (((cresp[0] & mask[0]) != (mask[0] & ~t[0])) or
		((cresp[1] & mask[1]) != (mask[1] & ~t[1])) or
		((cresp[2] & mask[2]) != (mask[2] & ~t[2]))):
		print "Triad ", t[3], " shorted to other I/Os!"
		print "Triad: ", t, "\nColdR: ", cresp, "\nHotRs: ", hresp

print
"""
# We're done. Reset the Ferret.
print "Test complete. Resetting DiscFerret."
dev.resetDevice()
dev = None
devOpen = False
for i in range(5):
	dev = DiscFerret()
	if dev.open():
		devOpen = True
		break
	else:
		time.sleep(1)

if not devOpen:
	print "Could not open device, is it connected?"
	sys.exit(-1)
else:
	print "Device opened successfully"

#############################################################################

if LoadFPGA:
	print "Initialising FPGA...",
	print "FPGA load: status code %d" % dev.fpgaLoadRBFFile("microcode.rbf")

	# Poll FPGA status
	resp = dev.fpgaGetLoadStatus()
	if resp == ERR_FPGA_NOT_CONF:
		print "FPGA is waiting for microcode load.. LOAD FAILED!"
		sys.exit(-1)
	elif resp == ERR_OK:
		print "FPGA microcode is active.. LOAD SUCCEEDED!"
	else:
		print "FPGA status code unknown, is %d, wanted %d or %d" % (resp, ERR_OK, ERR_FPGA_NOT_CONF)
		sys.exit(-1)
	print "Load complete.",

#############################################################################

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
