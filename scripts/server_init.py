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

# Definição de classe que gerencia jogadores conectados a um mesmo servidor
class GameServer:
   
   dataClients = []
   
   def __init__( self, port ):
      self.port = port
      self.serverId = createConnection( '0.0.0.0', str(self.port), NetworkInterface.UDP )
      if self.serverId is not None:
         print( 'Game server id: ' + str(self.serverId) )
         ( host, port ) = getAddress( self.serverId )
         print( 'host: ' + str(host) + ' - port: ' + str(port) )

   def update( self ):
      elementPositions = {}
      elementAddresses = {}
      
      message = receiveMessage( self.serverId )
      if message is not None: 
         print( 'GameServer accept: ' + message )
         self.dataClients.append( int(message.split()[0]) )
         
      for client in self.dataClients:
         message = receiveMessage( client )
         if message is not None:
            print( 'GameServer receive: ' + message )
            clientInfo = message.split(':')
            
            if len(clientInfo) >= 8:
               if clientInfo[0] == 'Game Position':
                  for elementDataOffset in range( 1, 7, len(clientInfo) - 6 ):
                     print( 'GameServer updating element ' + str(elementDataOffset / 7 + 1) )
                     # Posições X e Z (plano horizontal) do objeto na Unity 
                     elementPositions[ clientInfo[ elementDataOffset ] ] = ( float(clientInfo[ elementDataOffset + 1 ]), float(clientInfo[ elementDataOffset + 3 ]) )
                     elementAddresses[ clientInfo[ elementDataOffset ] ] = getAddress( client )[0]
                     
                  for destinationClient in self.dataClients:
                     if destinationClient != client: sendMessage( destinationClient, message )

            elif len(clientInfo) >= 1:
               if clientInfo[0] == 'Game Request NetworkID':
                  print( 'Game client ' + str(client) + ' requesting network ID' )
                  sendMessage( client, 'Game NetworkID:' + str(client) )

      return ( elementPositions, elementAddresses )


# Definição de classe que fornece a clientes informações sobre motores em uso
class AxisServer:
  
   dataClients = []
   axisDataClients = {}
   
   # Variáveis para medição de desempenho (benchmark)
   logNSamples = 200
   logSamplesCount = 0
   messageArrivalTimes = []
   sendCallsNumber = []
   receiveCallsNumber = []
   
   def __init__( self, port ):
      self.port = port
      self.serverId = createConnection( '0.0.0.0', str(self.port), NetworkInterface.UDP )
      if self.serverId is not None:
         print( 'Axis server id: ' + str(self.serverId) )
         ( host, port ) = getAddress( self.serverId )
         print( 'host: ' + str(host) + ' - port: ' + str(port) )
         
      for i in range(self.logNSamples):                                       # benchmark
         self.messageArrivalTimes.append( 0 )                                 # benchmark
         self.sendCallsNumber.append( 0 )                               # benchmark
         self.receiveCallsNumber.append( 0 )                            # benchmark
         
   def update( self ):
     
      message = receiveMessage( self.serverId )
      if message is not None: 
         print( 'Axis Server accept: ' + message )
         self.dataClients.append( int(message.split()[0]) )
         
      axisDataMessage = 'Axis Data'
         
      for client in self.dataClients:
         if self.logSamplesCount < self.logNSamples: self.receiveCallsNumber[ self.logSamplesCount ] += 1                         # benchmark
         message = receiveMessage( client )
         if message is not None:
            # print( 'Axis Server receive: ' + message )
            clientData = message.split(':')
            if len(clientData) >= 4 and( len(clientData) - 1 ) % 3 == 0:
               if clientData[0] == 'Axis Data':
                  for axisDataOffset in range( 1, 3, len(clientData) - 2 ):
                     axisName = clientData[ axisDataOffset ]
                     self.axisDataClients[ axisName ] = client
                     
                  # Teste Latência
                  messageIndex = int(clientData[2])                                                                               # benchmark
                  if self.logSamplesCount < self.logNSamples and messageIndex < self.logNSamples:                                 # benchmark
                     if self.messageArrivalTimes[ messageIndex ] == 0: self.logSamplesCount += 1                                  # benchmark  
                     self.messageArrivalTimes[ messageIndex ] = int(execTime() * 1000)                                            # benchmark
                     #print( 'Message ' + str(messageIndex) + ' received at ' + str(self.messageArrivalTimes[ messageIndex ]) )    # benchmark
                     #print( '-> ' + message )                                                                                     # benchmark
                     #print( str(self.logSamplesCount) + ' messages received' )                                                    # benchmark
                     
                  self.sendCallsNumber[ self.logSamplesCount - 1 ] += 1                                                           # benchmark
                  sendMessage( client, 'Axis Feedback:PLAYER Calcanhar:' + str(messageIndex) + ':0' )                             # benchmark
					 
                     # Teste Feedback
                     #sendMessage( client, 'Axis Feedback:{0:s}:{1:g}:{2:g}'.format( axisName, 
						#					float(clientData[ axisDataOffset + 1 ]), float(clientData[ axisDataOffset + 2 ]) ) )
                     
                  newAxisData = message.replace( 'Axis Data', '', 1 )
                  if len(axisDataMessage + newAxisData) < NetworkInterface.BUFFER_SIZE:
                     axisDataMessage += newAxisData
                  else:
                     #print( 'Sending Axis Data: ' + axisDataMessage )
                     for destinationClient in self.dataClients:
                        if destinationClient not in self.axisDataClients.values(): sendMessage( destinationClient, axisDataMessage )
                     axisDataMessage = newAxisData

            if len(clientData) >= 4:
               if clientData[0] == 'Axis Feedback':
                  axisName = clientData[1]
                  #print( 'Receiver client ' + str(client) + ' returning feedback to axis ' + axisName )
                  sendMessage( self.axisDataClients[ axisName ], message )

      if axisDataMessage != 'Axis Data':
         #print( 'Sending Axis Data: ' + axisDataMessage )
         for destinationClient in self.dataClients:
            if destinationClient not in self.axisDataClients.values(): sendMessage( destinationClient, axisDataMessage )