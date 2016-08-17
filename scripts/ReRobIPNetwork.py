#-*-coding:cp1252-*-

from socket import *

import sys
#import json

DEFAULT_ADDRESS = 'localhost'

BUFFER_SIZE = 512

FLOAT_SIZE = 4

AXIS_VARS_NUMBER = 7
AXIS_DATA_SIZE = 7 * FLOAT_SIZE

JOINT_VARS_NUMBER = 8
JOINT_DATA_SIZE = 8 * FLOAT_SIZE

class ClientConnection:

  def __init__( self ):
    self.eventSocket = socket( AF_INET, SOCK_STREAM )
    self.axisSocket = socket( AF_INET, SOCK_DGRAM )
    self.jointSocket = socket( AF_INET, SOCK_DGRAM )

    self.isConnected = False

  def __del__( self ):
    if self.isConnected:
      self.eventSocket.close()
      self.axisSocket.close()
      self.jointSocket.close()

  def Connect( self, host ):
    try:
      self.eventSocket.connect( ( host, 50000 ) )
      self.axisSocket.connect( ( host, 50001 ) )
      self.jointSocket.connect( ( host, 50002 ) )
      self.isConnected = True
      print( 'client connected' )  
    except:
      exception = sys.exc_info()[ 0 ]
      print( exception )
      self.isConnected = False

  def RefreshInfo( self ):
    robotsInfo = {}
    if self.isConnected:
      messageBuffer = ' ' #bytearray( 1 )
      messageBuffer[ 0 ] = 0
      self.eventSocket.send( messageBuffer )
      robotsInfoString = self.eventSocket.recv( BUFFER_SIZE )
      robotsInfo = {}#json.loads( robotsInfoString.decode() )

    robotsList = robotsInfo.get( 'robots', [] )
    jointsList = robotsInfo.get( 'joints', [] )
    axesList = robotsInfo.get( 'axes', [] )

    return ( robotsList, jointsList, axesList )

  def SendCommand( self, targetIndex, commandNumber ):
    if self.isConnected:
      messageBuffer = '   ' #bytearray( 3 )
      messageBuffer[ 0 ] = 1
      messageBuffer[ 1 ] = targetIndex
      messageBuffer[ 2 ] = commandNumber
      self.eventSocket.send( messageBuffer )

  def CheckState( self, eventNumber ):
    return false
    #return self.eventSocket.recv( BUFFER_SIZE )

  def SendAxisData( self, axisID, position, velocity, stiffness ):
    if self.isConnected:
      messageBuffer = '   ' #bytearray( 3 )
      messageBuffer[ 0 ] = 1
      messageBuffer[ 1 ] = axisID
      messageBuffer[ 2 ] = int( '00010011', 2 )
      for setpoint in [ position, velocity, 0.0, 0.0, stiffness, 0.0, 0.0 ]:
        messageBuffer += struct.pack( 'f', setpoint )
      self.axisSocket.send( messageBuffer )

  def ReceiveAxisData( self, axisID ):
    measures = [ 0.0 for var in range( AXIS_VARS_NUMBER ) ]
    if self.isConnected:
      messageBuffer = self.axisSocket.recv( BUFFER_SIZE )
      axesNumber = messageBuffer[ 0 ]
      for axisIndex in range( axesNumber ):
        axisDataOffset = axisIndex * AXIS_DATA_SIZE + 1
        if messageBuffer[ axisDataOffset ] == axisID:
          for measureIndex in range( AXIS_VARS_NUMBER ):
            axisMeasureOffset = axisDataOffset + measureIndex * FLOAT_SIZE
            measures[ measureIndex ] = struct.unpack_from( 'f', messageBuffer, axisMeasureOffset )[ 0 ]

    return measures

  def SendJointData( self, jointID, position, stiffness ):
    if self.isConnected:
      messageBuffer = bytearray( 3 )
      messageBuffer[ 0 ] = 1
      messageBuffer[ 1 ] = jointID
      messageBuffer[ 2 ] = int( '00000101', 2 )
      for setpoint in [ position, 0.0, stiffness, 0.0, 0.0, 0.0, 0.0 ]:
        messageBuffer += struct.pack( 'f', setpoint )
      self.jointSocket.send( messageBuffer )

  def ReceiveJointData( self, jointID ):
    measures = [ 0.0 for var in range( JOINT_VARS_NUMBER ) ]
    if self.isConnected:
      messageBuffer = self.jointSocket.recv( BUFFER_SIZE )
      jointsNumber = messageBuffer[ 0 ]
      for jointIndex in range( jointsNumber ):
        jointDataOffset = jointIndex * JOINT_DATA_SIZE + 1
        if messageBuffer[ jointDataOffset ] == jointID:
          for measureIndex in range( JOINT_VARS_NUMBER ):
            jointMeasureOffset = jointDataOffset + measureIndex * FLOAT_SIZE
            measures[ measureIndex ] = struct.unpack_from( 'f', messageBuffer, jointMeasureOffset )[ 0 ]

    return measures
