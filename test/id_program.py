#!/usr/bin/env python

from discferret import *
import sys

if len(sys.argv) < 3:
	print "Syntax: %s serialnumber hardwarerevision" % sys.argv[0]
	print "Example: %s 0I06 GB0L0801" % sys.argv[0]
	print
	sys.exit(-1)

# open the discferret
dev = DiscFerret()
if not dev.open():
	print "Could not open device, is it connected?"
	sys.exit(-1)
else:
	print "Device opened successfully"

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

print snb

# Build a "Program Serial Number" packet
packet = [CMD_PROGRAM_SERIAL, 0xAC, 0xCE, 0x55, 0xE5] + snb
dev.write(1, packet)
resp = dev.read(0x81, 1)
print "Programmed; response: %d" % resp[0]

print
print

