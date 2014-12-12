#!/bin/bash

gcc src/server_main.c -I/usr/include/python3.4m -IRehab_Robot -lpython3.4m -lpthread -o src/RobRehabServer

cd src/

./RobRehabServer
