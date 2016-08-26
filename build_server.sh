#!/bin/bash

gcc -std=gnu99 -DROBREHAB_SERVER -D__USE_POSIX199309 -D_DEFAULT_SOURCE=__STRICT_ANSI__ \
    -D_SVID_SOURCE -DIP_NETWORK_LEGACY -DDEBUG -Isrc -Isrc/ip_network/ src/robrehab_system.c \
    src/robrehab_network.c src/ip_network/ip_network.c src/ip_network/async_ip_network.c \
    src/threads/thread_safe_data.c src/shm_control.c src/shared_memory/shm_unix.c \
    src/threads/threads_unix.c src/time/timing_unix.c -o RobRehabServer -lrt -lpthread
