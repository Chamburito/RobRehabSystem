# modbus library
# ascii mode

set ob(plccmd) ""
set ob(toplc) ""
set ob(fromplc) ""
set ob(plcfd) ""

# proc dputs args {puts $args}
proc dputs args {}

# 9600 baud, even parity, 7 bits, one stop bit
proc mbopen {{port /dev/ttyS0} {mode "9600,e,7,1"}} {
	# open r+ for read/write
	set fd [open $port r+]

	fconfigure $fd -mode $mode

	# don't block on read, don't buffer output
	fconfigure $fd -blocking 0 -buffering none
	return $fd
}

proc mbclose {fd} {
	close $fd
}

# remove non-hex chars, force upper case HEX
# convert ": 12 ab cd ef<crlf>" to "12ABCDEF"

proc rmnonhex {str} {
	set str [string toupper $str]
	regsub -all {[^0-9a-fA-F]} $str "" str
	regsub -all {[a-f]} $str {[A-F]} str
	return $str
}

# insert blanks.
# convert "ABCDEF" to "AB CD EF "

proc insblanks {str} {
	regsub -all ".." $str "& " str
	return $str
}

# parsed tokenized ret data

proc parse_mb_str {str} {
	set slave [lindex $str 0]
	set cmd [lindex $str 1]
	set count [lindex $str 2]
	set data [lrange $str 3 end-1]
	set cksum [lindex $str end]
	return [list $slave $cmd $count $data $cksum]
}


# modbus lrc (longitudinal redundancy check) 8-bit cheskum
# sum hex bytes, mod 256, twos complement
# this is for the ASCII mode, the RTU (binary mode) uses a CRC

proc calclrc {str} {
	set sum 0
	set str [rmnonhex $str]
	set str [insblanks $str]
	foreach n $str {
		if {![string is xdigit $n]} continue
		scan $n %x bin
		incr sum $bin
	}
	set sum [expr {256 - ($sum % 256)}]
	set sum [format %2.2X $sum]
	return $sum
}

# takes a string terminated with a two-digit hex checksum
# runs calclrc, if the checksum binary values match returns 1, if not 0.

proc checklrcisok {str} {
	set sum 0
	set str [rmnonhex $str]
        set str [insblanks $str]
	set insum [lindex $str end]
	set str [lrange $str 0 end-1]
	set newsum [calclrc $str]
	expr {[scan %x $insum] == [scan %x $newsum]}
}

# s 000-3ff relay bit memory (like m)
# x 400-4ff inputs
# y 500-5ff outputs
# t 600-6ff timers (bit,word)
# m 800-dff memory

# m b000-b9ff memory
# c e00-ec7 counter (16 bit,word)
# c ec8-eff counter (32 bit,word)

# d 1000-1fff general memory word
# d 9000-1387 general memory word

# what about e, f, and file?

# convert named addressed to hex adddresses:
# s12 == 0x012
# x34 == 0x434
# y56 == 0x556
# etc

# note that this just adds the hex number to the base.
# so s410 == x10

proc ad {xnum} {
	set a(s) 0x000
	set a(x) 0x400
	set a(y) 0x500
	set a(t) 0x600
	set a(m) 0x800
	set a(c) 0xe00
	set a(d) 0x1000

	# set a(C) 0xec8
	# set a(D) 0x9000

	set first [string range $xnum 0 0]
	set rest [string range $xnum 1 end]
	set base $a($first)
	scan $rest %x addr
	format %04X [expr {$base + $addr}]
}

# commands

proc gen_read_coil_status {inaddr innpoints} {
	dputs "read_coil_status addr $inaddr bits $innpoints"
	set slave 01
	set read_coil_status 01
	set cmd $read_coil_status
	scan [ad $inaddr] %x addr
	set addr [format %04X $addr]
	set npoints [format %04X $innpoints]
	return "$slave $cmd $addr $npoints"
}

proc gen_read_input_status {inaddr innpoints} {
	dputs "read_input_status addr $inaddr bits $innpoints"
	set slave 01
	set read_input_status 02
	set cmd $read_input_status
	scan [ad $inaddr] %x addr
	set addr [format %04X $addr]
	set npoints [format %04X $innpoints]
	return "$slave $cmd $addr $npoints"
}

proc gen_read_holding_register {inaddr innwords} {
	dputs "read_holding_register addr $inaddr words $innwords"
	set slave 01
	set read_holding_register 03
	set cmd $read_holding_register
	scan [ad $inaddr] %x addr
	set addr [format %04X $addr]
	set nwords [format %04X $innwords]
	return "$slave $cmd $addr $nwords"
}

proc gen_force_single_coil {inaddr state} {
	dputs "force_single_coil addr $inaddr state $state"
	set slave 01
	set force_single_coil 05
	set cmd $force_single_coil
	scan [ad $inaddr] %x addr
	set addr [format %04X $addr]
	switch $state {
	0 {set val 0000}
	1 {set val FF00}
	default {error "force single coil state must be 0 or 1"}
	}
	return "$slave $cmd $addr $val"
}

proc gen_preset_single_register {inaddr inval} {
	dputs "preset_single_register addr $inaddr val $inval"
	set slave 01
	set preset_single_register 06
	set cmd $preset_single_register
	scan [ad $inaddr] %x addr
	set addr [format %04X $addr]
	set val [format %04X $inval]
	return "$slave $cmd $addr $val"
}

proc gen_force_multiple_coils {inaddr innpoints} {
}

proc gen_preset_multiple_register {inaddr innpoints} {
	dputs "force_single_coil addr $inaddr bits $innpoints"
	set slave 01
	set preset_multiple_register 10
	set cmd $preset_multiple_register
	scan [ad $inaddr] %x addr
	set addr [format %04X $addr]
	set npoints [format %04X $innpoints]
	return "$slave $cmd $addr $npoints"
}

proc gen_report_slave_id {} {
	dputs "report_slave_id"
	set slave 01
	set report_slave_id 11
	set cmd $report_slave_id
	return "$slave $cmd"
}

# set read_coil_status 01
# set read_input_status 02
# set read_holding_register 03
# set force_single_coil 05
# set preset_single_register 06
# set force_multiple_coils 15
# set preset_multiple_register 16
# set report_slave_id 17

# ==================================================

proc write_plc {fd str} {
	puts $fd $str
}

proc read_plc {fd} {
	read $fd
}

proc decode_plc_exception {{xcmd 80} {xerrcode 99}} {
    set cmd 80
    scan $xcmd %x cmd
    set ret "Command Succeeded: <$xcmd $xerrcode>"
    if {$cmd & 0x80} {
	switch $xerrcode {
	   "01" { set str "illegal command code" }
	   "02" { set str "illegal device address" }
	   "03" { set str "illegal device value" }
	   "07" { set str "checksum error" }
	   default { set str "unknown error" }
	}
	set ret "Command Failed: <$xcmd $xerrcode> $str"
    }
    return $ret
}

# ==================================================

proc do_open_plc {} {
    set ::ob(plcfd) [mbopen /dev/ttyS0]
}

proc do_close_plc {} {
    if {$::ob(plcfd) != ""} {
	mbclose $::ob(plcfd)
    }
    set ::ob(plcfd) ""
}

proc do_check_read_plc {cmd errcode} {
    if {$cmd == ""} {
	error "read_plc returned empty"
    }
}

proc do_write_plc {str} {
	if {![info exists ::ob(plcfd)]} {
		error "plc not open"
	}
	set sum [calclrc $str]
	set str "$str $sum"

	set ::ob(toplc) "$str"

	set str [rmnonhex $str]
	set str ":$str"

	write_plc $::ob(plcfd) $str

	after 100
}

proc do_read_plc {} {
	if {![info exists ::ob(plcfd)]} {
		error "plc not open"
	}
	set ret ""
	set ret [read_plc $::ob(plcfd)]

	# convert ":abcdef\n" to "ab cd ef "

	set ret [rmnonhex $ret]
	set ret [insblanks $ret]

	# note, errocode is first token of data.
	foreach {slave cmd count data cksum} [parse_mb_str $ret] break

	# do_check_read_plc $cmd $count

	set ::ob(fromplc) "<$slave> <$cmd> <$count> <$data> <$cksum>"
	set ::ob(plcerr) [decode_plc_exception $cmd $count]
}

proc do_check_read_plc {cmd errcode} {
	if {$cmd == ""} {
		error "read_plc returned empty"
	}
}

# ==================================================

proc do_read_coil_status {addr nbits} {
	set ::ob(plccmd) "read_coil_status $addr $nbits"
	set str [gen_read_coil_status $addr $nbits]
	do_write_plc $str
	do_read_plc
}

proc do_read_input_status {addr nbits} {
	set ::ob(plccmd) "read_input_status $addr $nbits"
	set str [gen_read_input_status $addr $nbits]
	do_write_plc $str
	do_read_plc
}

proc do_read_holding_register {addr nbits} {
	set ::ob(plccmd) "read_holding_register $addr $nbits"
	set str [gen_read_holding_register $addr $nbits]
	do_write_plc $str
	do_read_plc
}

proc do_force_single_coil {addr state} {
	set ::ob(plccmd) "force_single_coil $addr $state"
	set str [gen_force_single_coil $addr $state]
	do_write_plc $str
	do_read_plc
}

proc do_preset_single_register {addr nbits} {
	set ::ob(plccmd) "preset_single_register $addr $nbits"
	set str [gen_preset_single_register $addr $nbits]
	do_write_plc $str
	do_read_plc
}

proc do_force_multiple_coils {addr state} {
	set ::ob(plccmd) "force_multiple_coils $addr $state"
	set str [gen_force_multiple_coils $addr $state]
	do_write_plc $str
	do_read_plc
}

proc do_preset_multiple_register {addr nbits} {
	set ::ob(plccmd) "preset_multiple_register $addr $nbits"
	set str [gen_preset_multiple_register $addr $nbits]
	do_write_plc $str
	do_read_plc
}

proc do_report_slave_id {} {
	set ::ob(plccmd) "report_slave_id"
	set str [gen_report_slave_id]
	do_write_plc $str
	do_read_plc
}

proc do_print_read_write_plc {} {
	puts "Command: $::ob(plccmd)"
	puts "To PLC: $::ob(toplc)"
	puts "From PLC: $::ob(fromplc)"
	puts "Error: $::ob(plcerr)"
	puts ""
}
