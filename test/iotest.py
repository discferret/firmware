#!/usr/bin/env python

from discferret import *
import time
import sys, termios, atexit
from select import select

# save the terminal settings
fd = sys.stdin.fileno()
new_term = termios.tcgetattr(fd)
old_term = termios.tcgetattr(fd)

# new terminal setting unbuffered
new_term[3] = (new_term[3] & ~termios.ICANON & ~termios.ECHO)

# switch to normal terminal
def set_normal_term():
    termios.tcsetattr(fd, termios.TCSAFLUSH, old_term)

# switch to unbuffered terminal
def set_curses_term():
    termios.tcsetattr(fd, termios.TCSAFLUSH, new_term)

def putch(ch):
    sys.stdout.write(ch)

def getch():
    return sys.stdin.read(1)

def getche():
    ch = getch()
    putch(ch)
    return ch

def kbhit():
    dr,dw,de = select([sys.stdin], [], [], 0)
    return dr <> []

# set up unbuffered terminal
atexit.register(set_normal_term)
set_curses_term()

# open the discferret
dev = DiscFerret()
if not dev.open():
	print "Could not open device, is it connected?"
	sys.exit(-1)
else:
	print "Device opened successfully"

# load microcode
print "Loading microcode.rbf..."
resp = dev.fpgaLoadRBFFile("microcode.rbf")
print "FPGA load: status code %d" % resp
if (resp != ERR_OK):
	print "Error loading FPGA microcode. Check FCNSTAT, FCNCONF, FCDONE, FCDCLK and FCDATA0."
	err = True
	sys.exit(-1)

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
print "Load complete."

######################
# Walking lamps test
print
print "Walking lamps test"
print "=================="
ok = True
while ok:
	for i in [DRIVE_CONTROL_DENSITY, DRIVE_CONTROL_INUSE, DRIVE_CONTROL_DS3, DRIVE_CONTROL_DS0, DRIVE_CONTROL_DS1, DRIVE_CONTROL_DS2, DRIVE_CONTROL_MOTEN, DRIVE_CONTROL_SIDESEL]:
		dev.poke(DRIVE_CONTROL, i)
		time.sleep(1)
		if kbhit():
			ok = False
			break

# turn the lamps off
dev.poke(DRIVE_CONTROL, 0)

# flush keyboard buffer
while kbhit():
	getch()

#######################
# Switch status check
print
print "Status check"
print "============"
ok = True
while ok:
	print "Status:",
	r = dev.peek(STATUS2)
	if r & STATUS2_DENSITY:			print "DENS",
	if r & STATUS2_INDEX:			print "INDX",
	if r & STATUS2_TRACK0:			print "TRK0",
	if r & STATUS2_WRITE_PROTECT:	print "WPRT",
	if r & STATUS2_DISC_CHANGE:		print "RDY ",
	# Clreol followed by CR
	print "\033[K\r",

	if kbhit():
		ok = False

# flush keyboard buffer
while kbhit():
	getch()

print
print

