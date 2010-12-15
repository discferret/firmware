#!/usr/bin/env python

from discferret import *
import sys

needhelp = False
if (len(sys.argv) == 2) and (sys.argv[1].lower() == '--help'):
	needhelp = True
elif len(sys.argv) == 1:
	needhelp = True
elif (len(sys.argv) < 3) and (sys.argv[1].lower() != '--info'):
	needhelp = True

if needhelp:
	print "Syntax: %s serialnumber hardwarerevision" % sys.argv[0]
	print "Example: %s 0I06 GB0L0801" % sys.argv[0]
	print "%s --help: display this help screen" % sys.argv[0]
	print "%s --info: display device info" % sys.argv[0]
	print
	sys.exit(-1)

# open the discferret
dev = DiscFerret()
if not dev.open():
	print "Could not open device, is it connected?"
	sys.exit(-1)
else:
	print "Device opened successfully"

if sys.argv[1].lower() == '--info':
	print "Device information:"
	devinfo = dev.getDeviceInfo()
	for k in devinfo:
		if k in ['microcode_ver', 'firmware_ver', 'microcode_type']:
			print "\t%s:\t0x%04X" % (k, devinfo[k])
		else:
			print "\t%s:\t%s" % (k, devinfo[k])
	print
	sys.exit(0)

# Make sure command line parameters are valid
if len(sys.argv[1]) > 4:
	print "ERROR: Board revision ID is too long (max 4 chars, got %d)" % len(sys.argv[2])
	sys.exit(-1)

if len(sys.argv[2]) > 12:
	print "ERROR: Serial number is too long (max 12 chars, got %d)" % len(sys.argv[1])
	sys.exit(-1)

# Get command line parameters
serialnum = sys.argv[2].upper().ljust(12, '\0')[:12]
hwrev = sys.argv[1].upper().ljust(4, '\0')[:4]

# Check the serial number against Windows Plug and Play requirements
i=0
for c in serialnum:
	# Byte values less than 0x20 are invalid.
	# Byte values greater than 0x7F are invalid.
	# Byte value 0x2C is invalid."
	if (c < '\x20') or (c > '\x7F') or (c == '\x2C'):
		print "ERROR: Invalid character in serial number string, offset %d" % (i+1)
		sys.exit(-1)
	i = i + 1

# Build the serial number block
snb = ['\0']*16
pos = 0
for i in hwrev:
	snb[pos] = i
	pos = pos + 1
pos = 4
for i in serialnum:
	snb[pos] = i
	pos = pos + 1

# Build a "Program Serial Number" packet
packet = [CMD_PROGRAM_SERIAL, 0xAC, 0xCE, 0x55, 0xE5] + snb
dev.write(1, packet)
resp = dev.read(0x81, 1)
if (resp[0] == 0):
	print "ID block successfully programmed."
elif (resp[0] == 1):
	print "ID block already programmed; existing values not changed."
elif (resp[0] == 4):
	print "Bad magic number. This should never happen!"
else:
	print "Unknown error code: %d" % resp[0]

print
