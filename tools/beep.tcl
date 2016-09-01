# fill notearr with evenly tempered piano scale

proc initnotearr {} {
    global notearr

    set notestep [expr {pow(2.,(1./12.))}]

    # Hertz
    set lowA 27.5

    set curnote $lowA

    # 8 octaves
    foreach j {1 2 3 4 5 6 7 8} {
	# 12 notes per octave, using sharps
	foreach i {A A# B C C# D D# E F F# G G#} {
	    # puts "$i,$j $curnote"
	    set notearr($i,$j) $curnote
	    # curnote accumulates error, but it's not signifigant.
	    set curnote [expr {$curnote * $notestep}]
	}
    }
}

initnotearr

proc nbeep {i {note A} {len 50}} {
    global notearr

    switch $i {
    1 {exec beep -l $len -f $notearr($note,3) -D $len -n -l $len -f $notearr($note,4)}
    2 {exec beep -l $len -f $notearr($note,4) -D $len -n -l $len -f $notearr($note,3)}
    3 {exec beep -l $len -f $notearr($note,4) -D $len -n -l $len -f $notearr($note,6)}
    4 {exec beep -l $len -f $notearr($note,6) -D $len -n -l $len -f $notearr($note,4)}
    5 {exec beep -l $len -f $notearr($note,6) -D $len -n -l $len -f $notearr($note,8)}
    6 {exec beep -l $len -f $notearr($note,8) -D $len -n -l $len -f $notearr($note,6)}
    }
}
