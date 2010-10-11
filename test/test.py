#! /usr/bin/env python

# DiscFerret ATE code, based on code by Julius Constante

import sys, usb, struct, datetime

# The UsbPic class was written by Julius Constante and modified to use Bulk
# Transfer Mode by philpem.

class UsbPic:
	def __init__(self, vendor_id, product_id):
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
			return self.handle.bulkRead(ep, size, timeout) # return data read
		except:
			return []

	def getDeviceManufacturer(self):
		return self.handle.getString(1, 40)

	def getDeviceName(self):
		return self.handle.getString(2, 40)

######################################################################################################

def rwop(dev, ep, arr, readlen, timeout = 100):
	dev.write(1, arr, timeout)
	resp = dev.read(readlen, timeout)
	if len(resp) > 0:
		return resp
	else:
		print "Error: device read failed."
		sys.exit(-1)

def bitswap(num):
	val = 0
	for x in range(8):
		b = num&(1<<x) != 0
		val = val<<1 | b
	return val

#############################################################################

CMD_NOP				= 0
CMD_FPGA_INIT		= 1
CMD_FPGA_LOAD		= 2
CMD_FPGA_POLL		= 3
CMD_POKE			= 4
CMD_PEEK			= 5

ERR_OK				= 0
ERR_HARDWARE_ERROR	= 1
ERR_INVALID_LEN		= 2
ERR_FPGA_NOT_CONF	= 3

#############################################################################

# open the discferret
dev = UsbPic(0x04d8, 0xfbbb)
if not dev.open():
	print "Could not open device, is it connected?"
	sys.exit(-1)
else:
	print "Device opened: %s %s" % (dev.getDeviceManufacturer(), dev.getDeviceName())

print "Initialising FPGA... ",
# FPGA LOAD INIT -- start a microcode load
resp = rwop(dev, 1, [CMD_FPGA_INIT], 1, 1000)
if resp[0] == ERR_OK:
	print "OK! ",
else:
	print "Failed with status code %d" % resp[0]
	sys.exit(-1)

# poll fpga status
resp = rwop(dev, 1, [CMD_FPGA_POLL], 1, 1000)
if resp[0] == ERR_FPGA_NOT_CONF:
	print "(FPGA is waiting for microcode load)"
elif resp[0] == ERR_OK:
	print "(FPGA microcode is active)"
else:
	print "FPGA status code unknown, is %d, wanted %d or %d" % (resp[0], ERR_OK, ERR_FPGA_NOT_CONF)
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

	# packet header
	packet = [CMD_FPGA_LOAD, i]

	# data payload
	packet.extend(rbf[pos:pos+i])
	resp = rwop(dev, 1, packet, 1, 1000)
	if resp[0] != 0:
		print "FPGA microcode block transfer failed at addr=%04X, err=%d" % (pos, resp[0])
		sys.exit(-1)

	# update pointer
	pos += i

# poll fpga status
print "Load complete. FPGA status: ",
resp = rwop(dev, 1, [CMD_FPGA_POLL], 1, 1000)
if resp[0] == ERR_FPGA_NOT_CONF:
	print "FPGA is waiting for microcode load"
elif resp[0] == ERR_OK:
	print "FPGA microcode is active"
else:
	print "FPGA status code unknown, is %d, wanted %d or %d" % (resp[0], ERR_OK, ERR_FPGA_NOT_CONF)
	sys.exit(-1)


