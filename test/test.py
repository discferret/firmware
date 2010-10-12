#! /usr/bin/env python

# DiscFerret ATE code, based on code by Julius Constante

import sys, usb, struct, datetime

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

	def write(self, ep, buff, timeout = 100):
		try:
			return self.handle.bulkWrite(ep, buff, timeout) #return bytes written
		except:
			return 0

	def read(self, ep, size, timeout = 100):
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

	def ramWrite(self, data):
		return False

	def ramRead(self, nbytes):
		return False

######################################################################################################

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

print dev.getDeviceInfo()

