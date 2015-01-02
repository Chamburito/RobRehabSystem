 #-*-coding:cp1252-*-

from tkinter import *
from tkinter import ttk

from time import time as execTime # benchmark

#from gameserver import GameServer as GameServer
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
            clientInfo = message.split(':')
            if len(clientInfo) >= 4 and( len(clientInfo) - 1 ) % 3 == 0:
               if clientInfo[0] == 'Axis Data':
                  for axisDataOffset in range( 1, 3, len(clientInfo) - 2 ):
                     axisName = clientInfo[ axisDataOffset ]
                     self.axisDataClients[ axisName ] = client
                     
                  # Teste Latência
                  messageIndex = int(clientInfo[2])                                                                               # benchmark
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
						#					float(clientInfo[ axisDataOffset + 1 ]), float(clientInfo[ axisDataOffset + 2 ]) ) )
                     
                  newAxisData = message.replace( 'Axis Data', '', 1 )
                  if len(axisDataMessage + newAxisData) < NetworkInterface.BUFFER_SIZE:
                     axisDataMessage += newAxisData
                  else:
                     #print( 'Sending Axis Data: ' + axisDataMessage )
                     for destinationClient in self.dataClients:
                        if destinationClient not in self.axisDataClients.values(): sendMessage( destinationClient, axisDataMessage )
                     axisDataMessage = newAxisData

            if len(clientInfo) >= 4:
               if clientInfo[0] == 'Axis Feedback':
                  axisName = clientInfo[1]
                  #print( 'Receiver client ' + str(client) + ' returning feedback to axis ' + axisName )
                  sendMessage( self.axisDataClients[ axisName ], message )

      if axisDataMessage != 'Axis Data':
         #print( 'Sending Axis Data: ' + axisDataMessage )
         for destinationClient in self.dataClients:
            if destinationClient not in self.axisDataClients.values(): sendMessage( destinationClient, axisDataMessage )
                        

# Criação da Janela principal
window = Tk()

# Variáveis de conexão em rede
currentServerPort = 10000
serverPort = IntVar()
serverPort.set( currentServerPort )
# Manipuladores de conexão servidor principal
serverId = None
# Referência para servidor de eixos
axisServer = None


# Método ativado no encerramento da janela principal (só para benchmark)
def close():
   print( 'Saving Log' )
   messageArrivalLog = open( 'data/message_arrival_times.txt', 'w' )
   sendCallsLog = open( 'data/send_calls_server.txt', 'w' )
   receiveCallsLog = open( 'data/receive_calls_server.txt', 'w' )
   for time in range( axisServer.logSamplesCount ):
      messageArrivalLog.write( str(axisServer.messageArrivalTimes[ time ]) + '\n' )
      sendCallsLog.write( str(axisServer.sendCallsNumber[ time ]) + '\n' )
      receiveCallsLog.write( str(axisServer.receiveCallsNumber[ time ]) + '\n' )
   messageArrivalLog.close()
   sendCallsLog.close()
   receiveCallsLog.close()
   
   window.destroy()
      
window.protocol( 'WM_DELETE_WINDOW', close )
# benchmark


def serverConnect():
   global serverId
   global axisServer
   global currentServerPort
   
   if ( serverId is None ) or ( axisServer is None ) or ( currentServerPort != serverPort.get() ):
      if serverId is not None: closeConnection( serverId )
   
      serverId = createConnection( '0.0.0.0', str(serverPort.get()), NetworkInterface.TCP )
      axisServer = AxisServer( serverPort.get() )
      currentServerPort = serverPort.get()
      
      if serverId is not None:
         print( 'server id: ' + str(serverId) )
         ( host, port ) = getAddress( serverId )
         print( 'host: ' + str(host) + ' - port: ' + str(port) )

serverConnect()

infoClients = []
gameServers = {}
axisInfo = {}
gameInfo = {}
elementPositions = {}
elementAddresses = {}
canvasElements = []

# Raio da representação do elemento no canvas
iconRadius = 20

# Cores disponíveis para desenho no canvas
availableColors = [ 'red', 'green', 'blue', 'cyan', 'yellow', 'magenta' ]

def updateValues():
   if axisServer is not None: axisServer.update()
   for serverName in gameServers.keys():
      ( elementPositions[ serverName ], elementAddresses[ serverName ] ) = gameServers[ serverName ].update()
      
   window.after( 1, updateValues )

def refreshInfo():
   global axisInfo
   #print( 'Cleaning axis info' )
   axisInfo = {}
   
   window.after( 20000, refreshInfo )

# Função de inicialização
def updateInfo():
  
   message = None
   if serverId is not None: message = receiveMessage( serverId )
   if message is not None: 
      print( 'Add Client: ' + message )
      newClientInfo = message.split()
      infoClients.append( int(newClientInfo[0]) )
      #sendMessage( infoClients[-1], 'Server here !' )
   
   for client in infoClients:
      message = receiveMessage( client )
      clientAddress = getAddress( client )
      if message is not None:
         print( 'Add Register: ' + message + ' - Received from: host: ' + clientAddress[0] + ' - port: ' + clientAddress[1] )
         dataList = message.split(':')
      
         if len(dataList) >= 1:
            messageHeader = str(dataList[0]).strip()
            
            if messageHeader == 'Axis Info':
               sendMessage( client, 'Axis Info Begin' )
               print( 'Axis Info: ' + str(axisInfo) )
               for axisName in axisInfo.keys():
                  sendMessage( client, axisName )
               sendMessage( client, 'Axis Info End' )
            
            elif messageHeader == 'Game Info':
               print( 'Game Info Requested' )
               sendMessage( client, 'Game Info Begin' )
               print( 'Game Info: ' + str(gameServers.keys()) )
               for serverName in gameServers.keys():
                  sendMessage( client, serverName + ': ' + str(gameServers[ serverName ].port) )
               sendMessage( client, 'Game Info End' )
      
            elif len(dataList) >= 2:
	      
               messageContent = str(dataList[1]).strip()
	      
               if messageHeader == 'Axis Keep':
                  axisInfo[ messageContent ] = client
               elif messageHeader == 'Axis Connect':
                  if messageContent in axisInfo:
                     axisClient = int(axisInfo[ messageContent ])
                     sendMessage( axisClient, 'Axis Connect:' + messageContent )
                  
               elif messageHeader == 'Game New':
                  print( 'Game Requested' )
                  print( 'Server Name: ' + messageContent )
                  if messageContent not in gameServers:
                     gameServers[ messageContent ] = GameServer( serverPort.get() + len(gameServers) + 1 )
                  
                     serverSelector[ 'values' ] = list( gameServers.keys() )
                     gameServersNumber.set( len( serverSelector[ 'values' ] ) )
                     if gameServersNumber.get() == 1: 
                        serverSelector.current( 0 )
                        gameServerPort.set( gameServers[ gameServerName.get() ].port )
                  
   window.after( 1000, updateInfo )

def updateScreen():

   for serverName in elementPositions.keys():
      
      if serverName == gameServerName.get():
         for tag in elementPositions[ serverName ].keys():
            x = ( elementPositions[ serverName ][ tag ][0] + 0.5 ) * int(clientCanvas['width'])
            y = ( -elementPositions[ serverName ][ tag ][1] + 0.5 ) * int(clientCanvas['height'])
            if tag == elementName.get():
               elementPosition.set( 'x: ' + str(int(x)) + ' - y: ' + str(int(y)) ) # X e Z na Unity
               elementAddress.set( elementAddresses[ serverName ][ tag ] )
            if tag in canvasElements:
               clientCanvas.coords( tag, x - iconRadius, y - iconRadius, x + iconRadius, y + iconRadius )
            else:
               fillColor = availableColors[ gameClientsNumber.get() % len(availableColors) ]
               clientCanvas.create_oval( [ x - iconRadius, y - iconRadius, x + iconRadius, y + iconRadius ], fill = fillColor, tags = tag )
               canvasElements.append( tag )
               gameClientsNumber.set( len(canvasElements) )
               elementSelector['values'] = canvasElements
               if len(canvasElements) == 1: elementSelector.current( 0 )
   
   window.after( 5, updateScreen )

# Propriedades da Janela
window.title('RobRehabServer')
window.resizable( width = False, height = False )
window.columnconfigure( 1, minsize = 400 )
window.columnconfigure( 2, minsize = 400 )
window.columnconfigure( 3, minsize = 400 )
      
# Frame de apresentação do programa
headerFrame = LabelFrame( window )
headerFrame.grid( row = 1, column = 1, columnspan = 3, pady = 5 )     
# Label do Título    
titleLabel = Label( headerFrame, width = 70, height = 5 )
titleLabel['text'] = 'CONTROLE DE IMPEDÂNCIA EXO-KANGUERA - EPOS CANOpen\nESCOLA DE ENGENHARIA DE SÃO CARLOS - EESC/USP\nLABORATÓRIO DE REABILITAÇÃO ROBÓTICA\n'
titleLabel['font'] = ('comic sans', 12, 'bold')
titleLabel.grid( row = 1, column = 2 )
# Logos
robRehabLogo = PhotoImage( file='rehab_resized.gif' )
robRehabLabel = ttk.Label( headerFrame, image = robRehabLogo )
robRehabLabel.grid( row = 1, column = 1, padx = 10, pady = 10 )
eescLogo = PhotoImage( file='eesc_resized.gif' )
eescLabel = ttk.Label( headerFrame, image = eescLogo )
eescLabel.grid( row = 1, column = 3, padx = 10, pady = 5 )

# Frame Auxiliar
dataFrame = LabelFrame( window )
dataFrame.rowconfigure( 1, minsize = 150 )
dataFrame.rowconfigure( 2, minsize = 150 )
dataFrame.rowconfigure( 3, minsize = 150 )
dataFrame.grid( row = 2, column = 1, padx = 20, pady = 20, sticky = (N, S, E, W) )

# Configurações de servidor
serverFrame = LabelFrame( dataFrame, text = 'Configurações de Servidor' )
serverFrame.grid( row = 1, column = 1, pady = 15, ipady = 10, sticky = N )
# Configuração de porta
Label( serverFrame, text = 'Porta:' ).pack( side = LEFT, padx = 10, anchor = E )
Entry( serverFrame, textvariable = serverPort, width = 10, fg = 'black', bg = 'white', justify = CENTER ).pack( side = LEFT, padx = 10 )
# Botão de ativação da conexão
ttk.Button( serverFrame, text = 'Reiniciar', command = serverConnect ).pack( side = LEFT, padx = 40 )

# Listagem de servidores ativos
gameServersFrame = LabelFrame( dataFrame, text = 'Servidores de jogo ativos' )
gameServersFrame.grid( row = 2, column = 1, padx = 20, pady = 10, sticky = N )
# Contagem de servidores ativos 
Label( gameServersFrame, text = 'Nº de servidores:' ).grid( row = 1, column = 1, columnspan = 2, padx = 15, pady = 10, sticky = E )
gameServersNumber = IntVar()
Label( gameServersFrame, width = 5, textvariable = gameServersNumber, fg = 'black', bg = 'white' ).grid( row = 1, column = 3, columnspan = 2, padx = 10, sticky = W )
# Seleção de servidor
Label( gameServersFrame, text = 'Seleção:' ).grid( row = 2, column = 1, padx = 15, pady = 10 )
gameServerName = StringVar()
gameServerPort = IntVar()
serverSelector = ttk.Combobox( gameServersFrame, textvariable = gameServerName, justify = CENTER )
def serverChange( event ):
   if gameServersNumber.get() > 0 and gameServerName.get() in gameServers:
      gameServerPort.set( gameServers[ gameServerName.get() ].port )
   for tag in canvasElements: clientCanvas.delete( tag )
   canvasElements.clear()
serverSelector.bind( '<<ComboboxSelected>>', serverChange )
serverSelector.grid( row = 2, column = 2, padx = 10 )
# Seleção de servidor
Label( gameServersFrame, text = 'Porta:' ).grid( row = 2, column = 3, padx = 15 )
Label( gameServersFrame, width = 10, textvariable = gameServerPort, fg = 'black', bg = 'white' ).grid( row = 2, column = 4, padx = 10 )

# Listagem de clientes de servidor selecionado
gameClientsFrame = LabelFrame( dataFrame, text = 'Clientes ativos' )
gameClientsFrame.grid( row = 3, column = 1, padx = 20, pady = 10, sticky = N )
# Contagem de clientes ativos 
Label( gameClientsFrame, text = 'Nº de clientes:' ).grid( row = 1, column = 1, padx = 15, pady = 10, sticky = E )
gameClientsNumber = IntVar()
Label( gameClientsFrame, width = 5, textvariable = gameClientsNumber, fg = 'black', bg = 'white' ).grid( row = 1, column = 2, padx = 10, sticky = W )
# Seleção de elemento
Label( gameClientsFrame, text = 'Seleção:' ).grid( row = 2, column = 1, padx = 15, pady = 10, sticky = E )
elementName = StringVar()
elementAddress = StringVar()
elementSelector = ttk.Combobox( gameClientsFrame, textvariable = elementName, justify = CENTER )
def clientChange( event ):
   if gameClientsNumber.get() > 0 and elementName.get() in elementPositions:
      elementAddress.set( elementAddresses[ elementName.get() ] )
elementSelector.bind( '<<ComboboxSelected>>', clientChange )
elementSelector.grid( row = 2, column = 2, padx = 10, sticky = W )
# Endereço IP do elemento
Label( gameClientsFrame, text = 'Endereço:' ).grid( row = 3, column = 1, padx = 15, pady = 10, sticky = E )
Label( gameClientsFrame, width = 20, textvariable = elementAddress, fg = 'black', bg = 'white' ).grid( row = 3, column = 2, padx = 10, sticky = W )
# Posição do elemento selecionado
Label( gameClientsFrame, text = 'Posição:' ).grid( row = 4, column = 1, padx = 15, pady = 10, sticky = E )
elementPosition = StringVar()
Label( gameClientsFrame, width = 20, textvariable = elementPosition, fg = 'black', bg = 'white' ).grid( row = 4, column = 2, padx = 10, sticky = W )

# Representação dos clientes no espaço do Jogo
clientCanvas = Canvas( window, width = 600, height = 600, bg = 'white', bd = 5, relief = RIDGE )
def elementSelect( event ):
   canvas = event.widget
   x = canvas.canvasx(event.x)
   y = canvas.canvasy(event.y)
   elementsFound = canvas.find_overlapping( x - iconRadius/2, y - iconRadius/2, x + iconRadius/2, y + iconRadius/2 )
   if len(elementsFound) > 0:
      elementName.set( canvas.gettags( elementsFound[0] )[0] )
      elementAddress.set( canvas.gettags( elementsFound[0] )[1] )
clientCanvas.bind( '<ButtonPress-1>', elementSelect )
clientCanvas.grid( row = 2, column = 2, columnspan = 2, padx = 20, pady = 20 )

# Regisra função de inicialização para ser executada em seguida
window.after( 5, updateValues )
window.after( 1000, updateInfo )
window.after( 100, updateScreen )
window.after( 10000, refreshInfo )

# Inclui os elementos gráficos criados no loop de atualização da janela
window.mainloop()
