#!/bin/bash

gcc -std=gnu99 -DROBREHAB_CONTROL -D__USE_POSIX199309 -D_DEFAULT_SOURCE=__STRICT_ANSI__ \
    -D_SVID_SOURCE -DDEBUG -Isrc -Isrc/actuator_control/ -Isrc/robot_control \
    -Isrc/data_io -Isrc/signal_io -Isrc/time -Isrc/threads -Isrc/shared_memory \
    src/robrehab_system.c src/robrehab_control.c src/threads/thread_safe_data.c \
    src/shm_control.c src/shared_memory/shm_unix.c src/threads/threads_unix.c src/debug/data_logging.c \
    src/time/timing_unix.c src/configuration.c src/motors.c src/curve_interpolation.c \
    src/kalman_filters.c src/matrices_blas.c src/actuators.c src/robots.c src/sensors.c \
    src/signal_processing.c -o RobRehabControl -ldl -lrt -lpthread -lblas -llapack
