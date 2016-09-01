#! /bin/sh
# convert binary to ascii.

ncols=${1:-3}

shift

# reads stdin.
# convert a file with ncols 8-byte elements to ncols %g ascii fields,
# each element separated by space and each line terminated by newline.

hexdump -e "$ncols/8 \"%g \" \"\n\"
