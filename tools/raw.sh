#! /bin/sh
./shm << EOF
g adc 0 32
g adcvolts 0 32
g dac 0 32
g dacvolts 0 32
g dio 0 32
q
EOF
