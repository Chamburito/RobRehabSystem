#! /usr/bin/python

import os
import sys

for line in sys.stdin.readlines():
  print line.replace('\n', '\r')
