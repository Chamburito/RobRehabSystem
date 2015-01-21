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
from enum import IntEnum

class StatusWord( IntEnum ): ReadyToSwitchON = 1; SwitchedON = 2; OperationEnabled = 4; Fault = 8; VoltageEnabled = 16
                             QuickStop = 32; SwitchOnDisable = 64; RemoteNMT = 512; TargetReached = 1024; SetpointAcknowlege = 4096

class ControlWord( IntEnum ): SwitchON = 1; EnableVoltage = 2; QuickStop = 4; EnableOperation = 8 
                              NewSetpoint = 16; ChangeImmediatedly = 32; AbsRel = 64, FaultReset = 128, Halt = 256

class OperationMode( IntEnum ): Homming = 0x06; ProfileVelocity = 0x03; ProfilePosition = 0x01 
                                Position = 0xFF; Velocity = 0xFE; Current = 0xFD; MasterEncoder = 0xFB; Step = 0xFA

class Dimension( IntEnum ): Position = 0; PositionRef = 1; Velocity = 2; VelocityRef = 3; Current = 4; Tension = 5; Angle = 6; AngleRef = 7; Force = 8

dimensionUnit = { Dimension.Position : 'ppr', Dimension.PositionRef : 'ppr', Dimension.Velocity : 'rpm', Dimension.VelocityRef : 'rpm', \
                  Dimension.Current : 'mA', Dimension.Tension : 'mV', Dimension.Angle : '°', Dimension.AngleRef : '°', Dimension.Force : 'N' }
dimensionColor = { Dimension.Position : 'blue', Dimension.Position Ref : '#00007f', Dimension.Velocity : 'green', Dimension.VelocityRef : '#007f00', \
                   Dimension.Current : 'red', Dimension.Tension : '#7f0000', Dimension.Angle : '#7f7f00', Dimension.AngleRef : '#007f7f', Dimension.Force : 'magenta' }

dimensionName = { Dimension.Position : 'Posição', Dimension.PositionRef : 'Posição Alvo', Dimension.Velocity : 'Velocidade', Dimension.VelocityRef : 'Velocidade Alvo', \
                  Dimension.Current : 'Corrente', Dimension.AngleRef : 'Ângulo Alvo', Dimension.Tension : 'Tensão', Dimension.Angle : 'Ângulo', Dimension.Force : 'Força' }
                         
class Parameter( IntEnum ): Stiffness = 0; Damping = 1

parameterName = { Parameter.Stiffness : 'Rigidez', Parameter.Damping : 'Amortecimento' }

controlMode = { 2 : 'Quadril', 1 : 'Joelho', 0 : 'Calcanhar' }
