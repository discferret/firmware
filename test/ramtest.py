#! /usr/bin/env python

# DiscFerret ATE code, based on code by Julius Constante

import sys, usb, struct, datetime, random, time

#############################################################################

# Control registers (write only)
DRIVE_CONTROL			= 0x04
ACQCON					= 0x05
ACQ_START_EVT			= 0x06
ACQ_STOP_EVT			= 0x07
ACQ_START_NUM			= 0x08
ACQ_STOP_NUM			= 0x09

ACQ_HSTMD_THR_START		= 0x10
ACQ_HSTMD_THR_STOP		= 0x11

MFM_SYNCWORD_START_L	= 0x20
MFM_SYNCWORD_START_H	= 0x21
MFM_SYNCWORD_STOP_L		= 0x22
MFM_SYNCWORD_STOP_H		= 0x23
MFM_MASK_START_L		= 0x24
MFM_MASK_START_H		= 0x25
MFM_MASK_STOP_L			= 0x26
MFM_MASK_STOP_H			= 0x27
MFM_CLKSEL				= 0x2F	# MFM clock select

SCRATCHPAD				= 0x30
INVERSE_SCRATCHPAD		= 0x31
FIXED55					= 0x32
FIXEDAA					= 0x33
CLOCK_TICKER			= 0x34
CLOCK_TICKER_PLL		= 0x35

STEP_RATE				= 0xF0	# step rate, 250us per count
STEP_CMD				= 0xFF	# step command, bit7=direction, rest=num steps

# Status registers (read only)
STATUS1					= 0x0E
STATUS2					= 0x0F

# -----
# Step Command bits
STEP_CMD_TOWARDS_ZERO	= 0x80
STEP_CMD_AWAYFROM_ZERO	= 0x00
STEP_COUNT_MASK			= 0x7F

# -----
# DRIVE_CONTROL bits
DRIVE_CONTROL_DENSITY	= 0x01
DRIVE_CONTROL_INUSE		= 0x02
DRIVE_CONTROL_DS0		= 0x04
DRIVE_CONTROL_DS1		= 0x08
DRIVE_CONTROL_DS2		= 0x10
DRIVE_CONTROL_DS3		= 0x20
DRIVE_CONTROL_MOTEN		= 0x40
DRIVE_CONTROL_SIDESEL	= 0x80

# -----
# ACQCON bits
ACQCON_WRITE			= 0x04
ACQCON_ABORT			= 0x02
ACQCON_START			= 0x01

# -----
# masks and events for ACQ_*_EVT registers
ACQ_EVENT_IMMEDIATE		= 0x00
ACQ_EVENT_INDEX			= 0x01
ACQ_EVENT_MFM			= 0x02
# "wait for HSTMD before acq" combination bit
ACQ_EVENT_WAIT_HSTMD	= 0x80

# -----
# legal MFM_CLKSEL values
MFM_CLKSEL_1MBPS		= 0x00
MFM_CLKSEL_500KBPS		= 0x01
MFM_CLKSEL_250KBPS		= 0x02
MFM_CLKSEL_125KBPS		= 0x03

# -----
# Status bits
STATUS1_ACQSTATUS_MASK	= 0x07
STATUS1_ACQ_WRITING		= 0x04
STATUS1_ACQ_WAITING		= 0x02
STATUS1_ACQ_ACQUIRING	= 0x01
STATUS1_ACQ_IDLE		= 0x00
STATUS2_INDEX			= 0x80
STATUS2_TRACK0			= 0x40
STATUS2_WRITE_PROTECT	= 0x20
STATUS2_DISC_CHANGE		= 0x10
STATUS2_DENSITY			= 0x08
STATUS2_STEPPING		= 0x04
STATUS2_RAM_EMPTY		= 0x02
STATUS2_RAM_FULL		= 0x01

#############################################################################

CMD_NOP				= 0
CMD_FPGA_INIT		= 1
CMD_FPGA_LOAD		= 2
CMD_FPGA_POLL		= 3
CMD_FPGA_POKE		= 4
CMD_FPGA_PEEK		= 5
CMD_RAM_ADDR_SET	= 6
CMD_RAM_ADDR_GET	= 7
CMD_RAM_WRITE		= 8
CMD_RAM_READ		= 9
CMD_GET_VERSION		= 0xFF

ERR_OK				= 0
ERR_HARDWARE_ERROR	= 1
ERR_INVALID_LEN		= 2
ERR_FPGA_NOT_CONF	= 3

#############################################################################

# The UsbPic class was written by Julius Constante and modified by Phil Pemberton
# to use Bulk Transfer Mode and control a DiscFerret

class DiscFerret:
	def __init__(self):
		vendor_id = 0x04d8
		product_id = 0xfbbb
		busses = usb.busses() # enumerate busses
		self.handle = None
		for bus in busses:
			devices = bus.devices
			for dev in devices:
				if dev.idVendor==vendor_id and dev.idProduct==product_id: # device matches
					self.dev = dev
					self.conf = self.dev.configurations[0]
					self.intf = self.conf.interfaces[0][0]
					self.endpoints = []
					for endpoint in self.intf.endpoints:
						self.endpoints.append(endpoint)
					return

	def open(self):
		if self.handle:
			self.handle = None
		try:
			self.handle = self.dev.open()
#			self.handle.detachKernelDriver(0)
#			self.handle.detachKernelDriver(1)
			self.handle.setConfiguration(self.conf)
			self.handle.claimInterface(self.intf)
			self.handle.setAltInterface(self.intf)
			return True
		except:
			return False

	def write(self, ep, buff, timeout = 1000):
		try:
			return self.handle.bulkWrite(ep, buff, timeout) #return bytes written
		except:
			return 0

	def read(self, ep, size, timeout = 1000):
		try:
			# the OR 0x80 forces a READ
			return self.handle.bulkRead(ep | 0x80, size, timeout) # return data read
		except:
			return []

	def getDeviceInfo(self):
		info = dict()
		# poll USB for product information
		if self.dev.iManufacturer != 0:
			info['manufacturer']	= self.handle.getString(self.dev.iManufacturer,	40)
		else:
			info['manufacturer']	= ''
		if self.dev.iProduct != 0:
			info['product']			= self.handle.getString(self.dev.iProduct,		40)
		else:
			info['product']			= ''
		if self.dev.iSerialNumber != 0:
			info['serialnumber']	= self.handle.getString(self.dev.iSerialNumber,	40)
		else:
			info['serialnumber']	= ''
		# poll the device with a GET_VERSION request
		self.write(1, [CMD_GET_VERSION])
		resp = self.read(0x81, 64)
		info['hardware_rev'] = str()
		for i in range(1, 5):
			info['hardware_rev'] += chr(resp[i])
		info['firmware_ver']	= (resp[5] << 8) + resp[6]
		info['microcode_type']	= (resp[7] << 8) + resp[8]
		info['microcode_ver']	= (resp[9] << 8) + resp[10]
		return info

	def fpgaLoadBegin(self):
		self.write(1, [CMD_FPGA_INIT])
		resp = self.read(0x81, 1)
		return resp[0]

	def fpgaGetLoadStatus(self):
		self.write(1, [CMD_FPGA_POLL])
		resp = self.read(0x81, 1)
		return resp[0]

	def fpgaLoadBlock(self, block):
		# TODO: raise exception if len(block) > 62
		packet = [CMD_FPGA_LOAD, len(block)]
		packet.extend(block)
		self.write(1, packet)
		resp = self.read(0x81, 1)
		return resp[0]

	def peek(self, addr):
		self.write(1, [CMD_FPGA_PEEK, ((addr >> 8) & 0xff), addr & 0xff])
		resp = self.read(0x81, 2)
		if resp[0] == ERR_OK:
			return resp[1]
		else:
			return None

	def poke(self, addr, data):
		self.write(1, [CMD_FPGA_POKE, ((addr >> 8) & 0xff), addr & 0xff, data & 0xff])
		resp = self.read(0x81, 1)
		if resp[0] == ERR_OK:
			return True
		else:
			return False

	def setRAMAddr(self, addr):
		self.write(1, [CMD_RAM_ADDR_SET, addr & 0xff, (addr >> 8) & 0xff, (addr >> 16) & 0xff])
		resp = self.read(0x81, 1)
		if resp[0] == ERR_OK:
			return True
		else:
			return False

	def getRAMAddr(self):
		self.write(1, [CMD_RAM_ADDR_GET])
		resp = self.read(0x81, 4)
		if resp[0] == ERR_OK:
			return resp[1] + (resp[2] << 8) + (resp[3] << 16)
		else:
			return False

	def ramWrite(self, block):
		# TODO: raise exception if len(block) > 61
		packet = [CMD_RAM_WRITE, len(block) & 0xff, (len(block) >> 8) & 0xff]
		packet.extend(block)
		self.write(1, packet)
		resp = self.read(0x81, 1)
		return resp[0]

	def ramRead(self, nbytes):
		# TODO: raise exception if nbytes > 64
		packet = [CMD_RAM_READ, nbytes & 0xff, (nbytes >> 8) & 0xff]
		self.write(1, packet)
		resp = self.read(0x81, nbytes+1)
		if resp[0] == ERR_OK:
			return resp[1:]
		else:
			return False

	def debug_dump_status(self):
		a = self.peek(STATUS1)
		b = self.peek(STATUS2)
		# STATUS1
		if (a & STATUS1_ACQSTATUS_MASK) == STATUS1_ACQ_WRITING:
			s = "writing, "
		elif (a & STATUS1_ACQSTATUS_MASK) == STATUS1_ACQ_WAITING:
			s = "waiting, "
		elif (a & STATUS1_ACQSTATUS_MASK) == STATUS1_ACQ_ACQUIRING:
			s = "acquiring, "
		elif (a & STATUS1_ACQSTATUS_MASK) == STATUS1_ACQ_IDLE:
			s = "idle, "
		else:
			s = "ILLEGAL, "

		# STATUS2
		if (b & STATUS2_INDEX):
			s = s + "INDEX "
		if (b & STATUS2_TRACK0):
			s = s + "TRACK0 "
		if (b & STATUS2_WRITE_PROTECT):
			s = s + "WRPROT "
		if (b & STATUS2_DISC_CHANGE):
			s = s + "DISC-CHANGE "
		if (b & STATUS2_DENSITY):
			s = s + "DENSITY "
		if (b & STATUS2_STEPPING):
			s = s + "STEPPING "
		if (b & STATUS2_RAM_EMPTY):
			s = s + "RAM-EMPTY "
		if (b & STATUS2_RAM_FULL):
			s = s + "RAM-FULL "
		if s[-1] == ' ':
			s = s[:-1]
		if s[-1] == ',':
			s = s[:-1]
		print "STATUS: %02X %02X [%s]" % (a,b,s)

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

	"""
	print "data written to ram: ",
	print buf[0:10]
	print "data read back:      ",
	print rbuf[0:10]
	print
	"""

