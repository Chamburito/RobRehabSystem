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

class StatusWord( IntEnum ): 
   READY_TO_SWITCH_ON = 1; SWITCHED_ON = 2; OPERATION_ENABLED = 4; FAULT = 8; VOLTAGE_ENABLED = 16; 
   QUICK_STOP = 32; SWITCH_ON_DISABLE = 64; REMOTE_NMT = 512; TARGET_REACHED = 1024; SETPOINT_ACKNOWLEGE = 4096

class ControlWord( IntEnum ): 
   SWITCH_ON = 1; ENABLE_VOLTAGE = 2; QUICK_STOP = 4; ENABLE_OPERATION = 8; 
   NEW_SETPOINT = 16; CHANGE_IMMEDIATEDLY = 32; ABS_REL = 64; FAULT_RESET = 128; HALT = 256

class OperationMode( IntEnum ): 
   HOMMING = 0x06; PROFILE_VELOCITY = 0x03; PROFILE_POSITION = 0x01; 
   POSITION = 0xFF; VELOCITY = 0xFE; CURRENT = 0xFD; MASTER_ENCODER = 0xFB; STEP = 0xFA

class Dimension( IntEnum ): 
   POSITION = 0; POSITION_REF = 1; VELOCITY = 2; VELOCITY_REF = 3; CURRENT = 4; TENSION = 5; ANGLE = 6; ANGLE_REF = 7; FORCE = 8

dimensionUnit = { Dimension.POSITION : 'ppr', Dimension.POSITION_REF : 'ppr', Dimension.VELOCITY : 'rpm', Dimension.VELOCITY_REF : 'rpm', 
                  Dimension.CURRENT : 'mA', Dimension.TENSION : 'mV', Dimension.ANGLE : '°', Dimension.ANGLE_REF : '°', Dimension.FORCE : 'N' }

dimensionColor = { Dimension.POSITION : 'blue', Dimension.POSITION_REF : '#00007f', Dimension.VELOCITY : 'green', 
                   Dimension.VELOCITY_REF : '#007f00', Dimension.CURRENT : 'red', Dimension.TENSION : '#7f0000', 
                   Dimension.ANGLE : '#7f7f00', Dimension.ANGLE_REF : '#007f7f', Dimension.FORCE : 'magenta' }

dimensionName = { Dimension.POSITION : 'Posição', Dimension.POSITION_REF : 'Posição Alvo', Dimension.VELOCITY : 'Velocidade', 
                  Dimension.VELOCITY_REF : 'Velocidade Alvo', Dimension.CURRENT : 'Corrente', Dimension.TENSION : 'Tensão', 
                  Dimension.ANGLE_REF : 'Ângulo Alvo', Dimension.ANGLE : 'Ângulo', Dimension.FORCE : 'Força' }
                         
class Parameter( IntEnum ): STIFFNESS = 0; DAMPING = 1

parameterName = { Parameter.STIFFNESS : 'Rigidez', Parameter.DAMPING : 'Amortecimento' }

class Axis:
  
   def __init__( self ):
      self.dimensionValues = {}
      for dimension in list(Dimension):
         self.dimensionValues[ dimension ] = []
         self.dimensionValues[ dimension ].append( 0 )
          
      self.motionTimes = []
      self.motionTimes.append( execTime() * 1000 )
      
      self.controlEnabled = 0
      
      self.controlValues = {}
      for valueIndex in list(Parameter):
         self.controlValues[ valueIndex ] = 0
         
      self.statusValues = {}
      for valueIndex in list(StatusWord):
         self.statusValues[ valueIndex ] = 0
