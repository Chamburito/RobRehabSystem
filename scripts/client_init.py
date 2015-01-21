#-*-coding:cp1252-*-

from NetworkInterface import createConnection as createConnection
# Stabilish network connection to remote computer
from NetworkInterface import addClient as addClient
# Add an existing connection to a server connection clients list
from NetworkInterface import sendMessage as sendMessage
# Send string through defined connection
from NetworkInterface import receiveMessage as receiveMessage
# Get string from defined connection
from NetworkInterface import receiveLastMessage as receiveLastMessage
# Get last string received from defined connection
from NetworkInterface import connectionsNumber as connectionsNumber
# Stabilish network connection to remote computer
from NetworkInterface import clientsNumber as clientsNumber
# Get size of clients list for defined server index
from NetworkInterface import getAddress as getAddress
# Get host and port strings from defined connection
from NetworkInterface import closeConnection as closeConnection
# Close socket and end read/write threads for defined connection

import NetworkInterface

import sys

sys.argv = [ 'update.py' ]	# Prevent fatal error on TK window creation

# Predefined values

statusWord = { 'Ready to Switch ON' : 1, 'Switched ON' : 2, 'Operation Enabled' : 4, 'Fault' : 8, 'Voltage Enabled' : 16, \
                'Quick Stop' : 32, 'Switch on Disable' : 64, 'Remote NMT' : 512, 'Target Reached' : 1024, 'Setpoint Acknowlege' : 4096 }

controlWord = { 'Switch ON' : 1, 'Enable Voltage' : 2, 'Quick Stop' : 4, 'Enable Operation' : 8, 'New Setpoint' : 16, \
              'Change Immediatedly' : 32, 'Abs Rel' : 64, 'Fault Reset' : 128, 'Halt' : 256 }

operationMode = { 'Homming' : 0x06, 'Profile Velocity' : 0x03, 'Profile Position' : 0x01, 'Position' : 0xFF, \
                  'Velocity' : 0xFE, 'Current' : 0xFD, 'Master Encoder' : 0xFB, 'Step' : 0xFA }

dimensionIndex = { 'Position' : 0, 'Position Ref' : 1, 'Velocity' : 2, 'Velocity Ref' : 3, 'Current' : 4, 'Tension' : 5, 'Angle' : 6, 'Angle Ref' : 7, 'Force' : 8 }
dimensionUnit = { 'Position' : 'ppr', 'Position Ref' : 'ppr', 'Velocity' : 'rpm', 'Velocity Ref' : 'rpm', \
		 'Current' : '', 'Tension' : 'mV', 'Angle' : '°', 'Angle Ref' : '°', 'Force' : 'N' }
dimensionColor = { 'Position' : 'blue', 'Position Ref' : '#00007f', 'Velocity' : 'green', 'Velocity Ref' : '#007f00', 'Current' : 'red', 'Tension' : '#7f0000', \
		  'Angle' : '#7f7f00', 'Angle Ref' : '#007f7f', 'Force' : 'magenta' }

dimensionTranslation = { 'Position' : 'Posição', 'Position Ref' : 'Posição Alvo', 'Velocity' : 'Velocidade', 'Velocity Ref' : 'Velocidade Alvo', 'Current' : 'Corrente', \
			  'Angle Ref' : 'Ângulo Alvo', 'Tension' : 'Tensão', 'Angle' : 'Ângulo', 'Force' : 'Força' }

parameterIndex = { 'Rigidez' : 0, 'Amortecimento' : 1 }

controlMode = { 2 : 'Quadril', 1 : 'Joelho', 0 : 'Calcanhar' }
