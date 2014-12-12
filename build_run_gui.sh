#!/bin/bash

g++ src/gui_main.cpp -I/usr/include/python3.4m -IRehab_Robot -lpython3.4m -lpthread -o src/RobRehabGui

cd src/

./RobRehabGui
