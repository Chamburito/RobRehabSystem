#! /usr/bin/tclsh
# U.tcl - print U's.

# modems send U's in test mode, since they give a square wave.
# capital U is 0x55, 0b01010101
# 9600 baud U gives a 4800 Hz square wave.

# this stty isn't working when i run it from the script.
# exec stty 9600 -F /dev/ttyS0 clocal

# program just dumps to stdout at the baud that the output device
# is already set to.
# ttyS0 should be COM1 - the first port.
# tclsh ./U.tcl > /dev/ttyS0

while {1} {
    set UUU "UUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU"
    puts -nonewline $UUU
    flush stdout
}
