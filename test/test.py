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
STATUS1_ACQSTATUS_MASK	= 0x03
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
		if (a & STATUS1_ACQSTATUS_MASK) == STATUS1_ACQ_WAITING:
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

# open the discferret
dev = DiscFerret()
if not dev.open():
	print "Could not open device, is it connected?"
	sys.exit(-1)
else:
	print "Device opened successfully"

"""
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
"""
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

"""
print dev.getDeviceInfo()
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

MFM_SYNC=True
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

# set step rate to 6ms
print "set step rate: resp %d" % dev.poke(STEP_RATE, 6000/250)

# select drive 0 (PC cable -- DS2=MOTEN iirc)
print "select: resp %d" % dev.poke(DRIVE_CONTROL, DRIVE_CONTROL_DS2 | DRIVE_CONTROL_DS0 | DRIVE_CONTROL_SIDESEL)
time.sleep(3)
# seek to track zero
print "seek to zero: resp %d" % dev.poke(STEP_CMD, STEP_CMD_TOWARDS_ZERO | (90 & STEP_COUNT_MASK))
dev.debug_dump_status()
stat = STATUS2_STEPPING
while (stat & STATUS2_STEPPING) != 0:
	stat = dev.peek(STATUS2)
dev.debug_dump_status()
# seek to track 4
"""
print "seek to 4: resp %d" % dev.poke(STEP_CMD, STEP_CMD_AWAYFROM_ZERO | (4 & STEP_COUNT_MASK))
dev.debug_dump_status()
stat = STATUS2_STEPPING
while (stat & STATUS2_STEPPING) != 0:
	stat = dev.peek(STATUS2)
dev.debug_dump_status()
"""
# head settling time
time.sleep(1)

# start acquisition
print "start acq: resp %d" % dev.poke(ACQCON, ACQCON_START)
# dump status
dev.debug_dump_status()

# wait for acquisition to start
stat = 0
while (stat & STATUS1_ACQSTATUS_MASK) != STATUS1_ACQ_ACQUIRING:
	stat = dev.peek(STATUS1)
print "acquisition started."
dev.debug_dump_status()
stat = STATUS1_ACQ_ACQUIRING
while (stat & STATUS1_ACQSTATUS_MASK) == STATUS1_ACQ_ACQUIRING:
	stat = dev.peek(STATUS1)
print "acquisition completed."
dev.debug_dump_status()
nbytes = dev.getRAMAddr()-1
print "acquired %d bytes of data" % (nbytes)

# now save the RAM data
dev.setRAMAddr(0)
lp = nbytes
ramdata = []
while lp > 0:
	if lp >= 63:
		data = dev.ramRead(63)
		lp = lp - 63
	else:
		data = dev.ramRead(lp)
		lp = 0
	ramdata.extend(data)

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
