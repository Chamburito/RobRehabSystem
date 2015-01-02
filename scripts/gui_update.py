#-*-coding:cp1252-*-

from tkinter import *
from tkinter import ttk

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg as FigureCanvas
from matplotlib.backends.backend_tkagg import NavigationToolbar2TkAgg as NavigationToolbar

from time import time as execTime # benchmark

# Criação da Janela principal
window = Tk()  

# Variáveis para medição de desempenho (benchmark)
logNSamples = 200
logOutSamplesCount = 0
logInSamplesCount = 0
messageDispatchTimes = []
messageResponseTimes = []
sendCallsNumber = []
receiveCallsNumber = []
for i in range( logNSamples ):
   messageDispatchTimes.append( 0 )
   messageResponseTimes.append( 0 )
   sendCallsNumber.append( 0 )
   receiveCallsNumber.append( 0 )

   
# Variáveis de conexão em rede
currentServerHost = 'localhost'
serverHost = StringVar()
serverHost.set( currentServerHost )
serverPort = 10000
# Manipuladores de conexão com servidor ou cliente
infoServerId = None
infoClientId = None
dataServerId = None
dataClientId = None

# Método ativado no encerramento da janela principal
def close():
   print( 'Destroying Window' )
   if dataServerId is not None: sendMessage( dataServerId, 'Axis End' )   
   
   print( 'Saving Log' )
   messageResponseLog = open( 'data/message_response_times.txt', 'w' )        # benchmark
   messageDispatchLog = open( 'data/message_dispatch_times.txt', 'w' )        # benchmark
   sendCallsLog = open( 'data/send_calls_client.txt', 'w' )                   # benchmark
   receiveCallsLog = open( 'data/receive_calls_client.txt', 'w' )             # benchmark
   for time in range( logOutSamplesCount ):                                   # benchmark
      messageResponseLog.write( str(messageResponseTimes[ time ]) + '\n' )    # benchmark
      messageDispatchLog.write( str(messageDispatchTimes[ time ]) + '\n' )    # benchmark
      sendCallsLog.write( str(sendCallsNumber[ time ]) + '\n' )               # benchmark
      receiveCallsLog.write( str(receiveCallsNumber[ time ]) + '\n' )         # benchmark
   messageResponseLog.close()                                                 # benchmark
   messageDispatchLog.close()                                                 # benchmark
   sendCallsLog.close()                                                       # benchmark
   receiveCallsLog.close()                                                    # benchmark
   
   window.destroy()
      
window.protocol( 'WM_DELETE_WINDOW', close )

# Define variável do id do Eixo sendo visualizada no momento 
axisIdView = IntVar()

# Nome e Identificador de Rede do Paciente
playerName = StringVar()
playerNetworkId = 'PLAYER'
playerName.set( playerNetworkId )

# Identificadores de Rede dos eixos
axisNetworkIds = {} 

# Número de Eixos sendo utilizadas
nAxis = 3

# Habilita ou não o uso de servidor remoto
remoteServerEnabled = -1
useRemoteServer = IntVar()
useRemoteServer.set( 0 )
# Função de conexão com servidor ( Computador recebendo os dados )
def setupConnection():
   global playerNetworkId
   global axisNetworkIds
   global remoteServerEnabled
   
   maxIdLength = 10
   playerNetworkId = playerName.get() if len(playerName.get()) <= maxIdLength else playerName.get()[:maxIdLength]
   for axisId in range( nAxis ):
      axisNetworkIds[ playerNetworkId + ' ' + controlMode[ axisId ] ] = axisId
   
   if useRemoteServer.get() != remoteServerEnabled:
      if useRemoteServer.get() == 1:
         setupClient()
      else:
         setupServer()
         
   remoteServerEnabled = useRemoteServer.get()
   
   
def setupServer():
   global infoServerId
   global dataServerId
   
   if infoServerId is None: infoServerId = createConnection( '0.0.0.0', str(serverPort), NetworkInterface.TCP )
   if dataServerId is None: dataServerId = createConnection( '0.0.0.0', str(serverPort), NetworkInterface.UDP )
   
   if infoClientId is not None: closeConnection( infoClientId )
   if dataClientId is not None: closeConnection( dataClientId )
      
   if infoServerId is not None:
      ( host, port ) = getAddress( infoServerId )
      print( 'host: ' + str(host) + ' - port: ' + str(port) )
   if dataServerId is not None:
      ( host, port ) = getAddress( dataServerId )
      print( 'host: ' + str(host) + ' - port: ' + str(port) )
      
   window.after( 100, updateClientInfo )
   window.after( 100, updateDataClient )
      
def setupClient():
   global infoClientId
   global dataClientId
   global infoServerId
   global dataServerId
   global currentServerHost
   
   if ( infoClientId is None ) or ( dataClientId is None ) or ( currentServerHost != serverHost.get() ):
      if infoClientId is not None: closeConnection( infoClientId )
      if dataClientId is not None: closeConnection( dataClientId )
      if infoServerId is not None: 
         closeConnection( infoServerId )
         infoServerId = None
      if dataServerId is not None:
         closeConnection( dataServerId )
         dataServerId = None
   
      infoClientId = createConnection( serverHost.get(), str(serverPort), NetworkInterface.TCP )
      dataClientId = createConnection( serverHost.get(), str(serverPort), NetworkInterface.UDP )
      
      currentServerHost = serverHost.get()
      
   if infoClientId is not None:
      ( host, port ) = getAddress( infoClientId )
      print( 'host: ' + str(host) + ' - port: ' + str(port) )
   if dataClientId is not None:
      ( host, port ) = getAddress( dataClientId )
      print( 'host: ' + str(host) + ' - port: ' + str(port) )
      
   window.after( 100, updateServerInfo )
   window.after( 100, sendData )


count = 0
# Função de inicialização
def init():
   
   global count
      
   for axisId in range( 0, nAxis ):
      readValues( axisId )
      setControlWord( axisId, controlWord[ 'Fault Reset' ], 1 )
      writeValues( axisId )
      setControlWord( axisId, controlWord[ 'Fault Reset' ], 0 )
      writeValues( axisId )
         
   initLabel[ 'text' ] += '.'
   
   if count >= 10:
      initLabel[ 'text' ] = 'COMUNICAÇÃO CANOpen INICIALIZADA'
      window.after( 100, updateValues )
   else:
      count += 1
      window.after( 500, init )


dimensionValue = []
for axisId in range(nAxis):
   dimensionValue.append( {} )
   for dimension in dimensionIndex.keys():
      dimensionValue[ axisId ][ dimension ] = []
#Função de atualização dos valores
def updateValues():   
   # Obtém valores dos frames da rede CAN
   for axisId in range(nAxis):
      readValues( axisId )
      for dimension in dimensionValue[ axisId ].keys():
         dimensionValue[ axisId ][ dimension ].append( getAxisValue( axisId, dimensionIndex[ dimension ] ) )
   
   # Atualiza valores a serem exibidos na tela
   for word in statusWord.keys():
      statusValue[ word ].set( getStatusWord( axisIdView.get(), statusWord[ word ] ) )
      
   # Registra função de atualização para ser executada novamente
   window.after( 5, updateValues )


axisPositions = []
axisVelocities = []
axisMotionTimes = []
for axisId in range( nAxis ):
   axisPositions.append( 0 )
   axisVelocities.append( 0 )
   axisMotionTimes.append( execTime() * 1000 )
   
# Função para enviar valores de eixos para o servidor central ou cliente conectado
def sendData():
   global logOutSamplesCount # benchmark
   
   if dataClientId is not None:

      axisDataMessage = 'Axis Data'
      
      axisName = 'PLAYER Calcanhar'                                                                                           # benchmark
      axisId = axisNetworkIds[ axisName ]                                                                                     # benchmark
      if logOutSamplesCount < logNSamples and logOutSamplesCount == logInSamplesCount:                                        # benchmark
         axisDataMessage += ':{0:s}:{1:g}:{2:g}'.format( axisName, logOutSamplesCount, axisVelocities[ axisId ] )             # benchmark
         messageDispatchTimes[ logOutSamplesCount ] = int(execTime() * 1000)                                                  # benchmark
         # print( 'Sending message ' + str(logOutSamplesCount) + ' at ' + str(messageDispatchTimes[ logOutSamplesCount ]) )     # benchmark
         # print( '-> ' + axisDataMessage )
         logOutSamplesCount += 1                                                                                              # benchmark
      elif logOutSamplesCount < logNSamples:
         # print( 'Waiting message ' + str(logOutSamplesCount - 1) )
         axisDataMessage += ':{0:s}:{1:g}:{2:g}'.format( axisName, logOutSamplesCount - 1, axisVelocities[ axisId ] )         # benchmark
      window.after( 5, receiveData )                                                                                          # benchmark

      for axisName in axisNetworkIds.keys():
         axisId = axisNetworkIds[ axisName ]
         if len(dimensionValue[ axisId ]) > 0:
            if( dimensionValue[ axisId ][ 'Velocity' ][-1] != axisVelocities[ axisId ] 
            or dimensionValue[ axisId ][ 'Position' ][-1] != axisPositions[ axisId ] ):
               axisPositions[ axisId ] = dimensionValue[ axisId ][ 'Position' ][-1]
               axisVelocities[ axisId ] = dimensionValue[ axisId ][ 'Velocity' ][-1]               
               axisDataMessage += ':{0:s}:{1:g}:{2:g}'.format( axisName, axisPositions[ axisId ], axisVelocities[ axisId ] )
               window.after( 10, receiveData )
            
      # print( 'Sending Axis Data: ' + axisDataMessage )
      if logInSamplesCount < logNSamples: sendCallsNumber[ logOutSamplesCount - 1 ] += 1                                      # benchmark
      sendMessage( dataClientId, axisDataMessage )
      
      window.after( 10, sendData ) 

# Função para receber valores de eixos do servidor central ou cliente conectado
def receiveData():
   global logInSamplesCount # benchmark
   
   # Comunicação via socket UDP
   message = None
   if dataClientId is not None:
      if logOutSamplesCount < logNSamples: receiveCallsNumber[ logOutSamplesCount ] += 1                                            # benchmark
      message = receiveMessage( dataClientId )
      if message is not None:
         # print( 'Received message: ' + message )
         dataList = message.split(':')
         if len(dataList) >= 4:
            if dataList[0] == 'Axis Feedback':
               messageIndex = int(dataList[2])                                                                                      # benchmark
               if logInSamplesCount < logOutSamplesCount and messageIndex < logNSamples:                                            # benchmark
                  if messageResponseTimes[ messageIndex ] == 0: logInSamplesCount += 1                                              # benchmark
                  messageResponseTimes[ messageIndex ] = int(execTime() * 1000)                                                     # benchmark
                  # print( 'Receiving message ' + str(messageIndex) + ' response at ' + str(messageResponseTimes[ messageIndex ]) )   # benchmark
               axisName = dataList[1]
               axisId = axisNetworkIds[ axisName ]
               positionReference = int(dataList[2]) + int(dataList[3]) * ( axisMotionTimes[ axisId ] - int(execTime() * 1000) ) / 1000
               setAxisValue( axisId, 1, positionReference ) # dimensionIndex[ 'Position Ref' ]
               if logInSamplesCount == logNSamples: print( 'Profiling End' )
               return
      
      window.after( 5, receiveData )
  
# Função de recebimento de dados do cliente
def updateDataClient():
   global dataClientId
  
   # Comunicação via socket UDP
   message = None
   if dataServerId is not None:
      message = receiveMessage( dataServerId )
      if message is not None:
         print( 'Add Client: ' + message )
         if dataClientId is not None: closeConnection( dataClientId )
         dataClientId = int( message.split()[0] )
         window.after( 100, updateConnectedData )
   
   window.after( 100, updateDataClient ) 


# Função de atualização de informações do servidor
def updateServerInfo():
  
   # Comunicação via socket TCP
   if infoServerId is not None:      
      for axis in controlMode.values():
         print( 'Axis Keep Alive' )
         sendMessage( infoServerId, 'Axis Keep:' + playerNetworkId + ' ' + axis )
               
   window.after( 10000, updateServerInfo )

# Função de atualização de informações do cliente
def updateClientInfo():
   global infoClientId 
   
   # Comunicação via socket TCP
   message = None   
   if infoServerId is not None:  
      message = receiveMessage( infoServerId )
      if message is not None: 
         print( 'Add Client: ' + message )
         newClientInfo = message.split()
         infoClientId = int(newClientInfo[0])
      
   if infoClientId is not None:
      message = receiveMessage( infoClientId )
      if message is not None:
         if message.strip() == 'Axis Info':
            print( 'Axis Info Requested' )
            sendMessage( infoClientId, 'Axis Info Begin' )
            print( 'Axis Info: ' + str(controlMode) )
            for axisName in controlMode.values():
               sendMessage( infoClientId, playerNetworkId + ' ' + axisName )
            sendMessage( infoClientId, 'Axis Info End' )
               
   window.after( 1000, updateClientInfo )


# Função de atualização da tela 
def updateScreen():
  
   time = len(dimensionValue[ axisIdView.get() ][ 'Position' ])
   global plotWidth
   
   # Plota valores de posição, velocidade, corrente e tensão no gráfico   
   begin = int(time * scroll.get()[0])
   end = int(time * scroll.get()[1])
   
   figure.clear()
   
   nPlots = 0
   for check in dimensionCheck.values():
      if check.get() == 1:
         nPlots += 1
     
   plotId = 0
   for dimension in dimensionValue[ axisIdView.get() ].keys():
      if dimensionCheck[ dimension ].get() == 1:
         plotId += 1
         plot = figure.add_subplot( nPlots, 1, plotId, anchor = 'E' )
         plot.set_ylabel( dimension, fontdict = { 'fontsize' : 8 } )
         plot.tick_params( 'y', labelsize = 8, labelright = 'on', labelleft = 'off' )
         plot.tick_params( 'x', labelsize = 0, labeltop = 'off', labelbottom = 'off' )
         #plot.autoscale( axis = 'y', tight = False )
         if time > plotWidth[0]:
            plot.set_xlim( left = begin, right = end )
         else:
            plot.set_xlim( left = 0, right = plotWidth[0] )
         interval = plotWidth[1] * dimensionScale[ dimension ].get()
         plot.set_ylim( top = interval / 2, bottom = -interval / 2 )
         plot.plot( dimensionValue[ axisIdView.get() ][ dimension ], color = dimensionColor[ dimension ] )
         #plot.axhline( 0, 0, 1, color = 'black' )
         
      if len(dimensionValue[ axisIdView.get() ][ dimension ]) > 0:
         dimensionEntry[ dimension ]['text'] = '{:.3g} {}'.format( dimensionValue[ axisIdView.get() ][ dimension ][-1], dimensionUnit[ dimension ] )
      
   if nPlots == 0:
      plot = figure.add_subplot( 111 )
      plot.tick_params( labelsize = 8, labelright = 'on', labelleft = 'off' )
      plot.axis( [ 0, plotWidth[0], -plotWidth[1] / 2, plotWidth[1] / 2 ] )
      #plot.axhline( 0, 0, 1, color = 'black' )

   figure.axes[-1].tick_params( 'x', labelsize = 8, labeltop = 'off', labelbottom = 'on' )
   
   # Desloca slider quando gráfico cresce além do tamanho do canvas
   if time > plotWidth[0]:
      if scroll.get()[1] == 1.0:
         scroll.set( 1.0 - plotWidth[0] / time, 1.0 )
      else:
         scroll.set( scroll.get()[0] * (time - 1) / time, (scroll.get()[0] * (time - 1) + plotWidth[0]) / time )
         
   mplCanvas.draw()
   mplCanvas.show()
   
   # Registra função de atualização para ser executada novamente
   window.after( 100, updateScreen )

# Função de Reset das falhas
def resetFault():

   setControlWord( axisIdView.get(), controlWord['Fault Reset'], 1 )
   writeValues( axisIdView.get() )
   setControlWord( axisIdView.get(), controlWord['Fault Reset'], 0 )
   writeValues( axisIdView.get() )

# Propriedades da Janela
window.title('RobRehabGui')
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
      
# Configurações de conexão
serverFrame = LabelFrame( window, text = 'Configurações de Conexão' )
serverFrame.grid( row = 2, column = 1, columnspan = 3, pady = 5, ipady = 10 )

# Configuração de identificador de rede ( nome do usuário )
Label( serverFrame, text = 'Usuário:' ).pack( side = LEFT, padx = 10, anchor = E )
Entry( serverFrame, textvariable = playerName, width = 20, fg = 'black', bg = 'white', justify = CENTER ).pack( side = LEFT, padx = 10 )
# Configuração de host
Label( serverFrame, text = 'Endereço (Domínio ou IP):' ).pack( side = LEFT, padx = 10, anchor = E )
Entry( serverFrame, textvariable = serverHost, width = 20, fg = 'black', bg = 'white', justify = CENTER ).pack( side = LEFT, padx = 10 )
# Configuração de porta
#Label( serverFrame, text = 'Porta:' ).pack( side = LEFT, padx = 10, anchor = E )
#Entry( serverFrame, textvariable = serverPort, width = 10, fg = 'black', bg = 'white', justify = CENTER ).pack( side = LEFT, padx = 10 )
# Ativação de uso de servidor remoto ( dados dos eixos vão para servidor principal )
Label( serverFrame, text = 'Utilizar Servidor Remoto' ).pack( side = LEFT, padx = 10, anchor = E )
Checkbutton( serverFrame, variable = useRemoteServer, fg = 'black' ).pack( side = LEFT )
# Botão de ativação da conexão
ttk.Button( serverFrame, text = 'Conectar', command = setupConnection ).pack( side = LEFT, padx = 40 )
      
# Label do status de inicialização      
initLabel = Label( window, height = 2, width = 60, relief = RIDGE )
initLabel['text'] = 'INICIALIZANDO COMUNICAÇÃO CANOpen COM AS EPOS'
initLabel.grid( row = 3, column = 1, columnspan = 3, pady = 5 )

#Frames de configuração das controladoras dos eixos
controlCheck = {}
controlValue = {}
for axisId in range(nAxis):
   axisFrame = LabelFrame( window, text = str(controlMode[ axisId ]) )
   axisFrame.grid( row = 4, column = axisId + 1, pady = 10 )
   Label( axisFrame, text = 'Visualização' ).grid( row = 1, column = 1, padx = 10 )
   Radiobutton( axisFrame, variable = axisIdView, value = axisId, fg = 'black' ).grid( row = 1, column = 2, sticky = W )
   Label( axisFrame, text = 'Controle' ).grid( row = 1, column = 3, padx = 10 )
   
   controlCheck[ axisId ] = IntVar()
   # Função de início/término do controle (Definida para cada Eixo)
   def toggleControl( axisId = axisId ):
      setAxisEnabled( axisId, controlCheck[ axisId ].get() )
      setControl( axisId, controlCheck[ axisId ].get(), axisId )
   
   Checkbutton( axisFrame, variable = controlCheck[ axisId ], command = toggleControl, fg = 'black' ).grid( row = 1, column = 4, sticky = W )
   
   pos = 0
   controlValue[ axisId ] = {}
   for parameter in sorted( parameterIndex.keys(), key = parameterIndex.get ):
      Label( axisFrame, text = parameter + ':', width = pos * 6 + 6, justify = RIGHT ).grid( row = 2, column = pos * 2 + 1, pady = 5 )

      controlValue[ axisId ][ parameter ] = DoubleVar()
      controlValue[ axisId ][ parameter ].set( getControlValue( axisId, parameterIndex[ parameter ] ) )
      # Função de evento da caixa de texto
      def setParameter( event, axisId = axisId, parameter = parameter ):
         setControlValue( axisId, parameterIndex[ parameter ], controlValue[ axisId ][ parameter ].get() )
         controlValue[ axisId ][ parameter ].set( getControlValue( axisId, parameterIndex[ parameter ] ) )
   
      parameterEntry = Entry( axisFrame, textvariable = controlValue[ axisId ][ parameter ], width = 5, fg = 'black', bg = 'white', justify = CENTER )
      parameterEntry.bind( ('<KeyPress-Return>', '<KeyRelease-Return>'), setParameter )
      parameterEntry.bind( '<FocusOut>', setParameter )
      parameterEntry.grid( row = 2, column = pos * 2 + 2, padx = 10, sticky = E )
      pos += 1

# Botão de Reset de falhas
resetButton = ttk.Button( window, text = 'Reset de Falhas', command = resetFault )
resetButton.grid( row = 5, column = 1, sticky = S )

# Frame de exibição dos valores de StatusWord 
statusFrame = LabelFrame( window, text = 'LEITURA DE STATUS WORDS' )#, labelanchor = 'n' )
statusFrame.grid( row = 6, column = 1, rowspan = 2, pady = 30, ipadx = 10, ipady = 5, sticky = N )   
# Caixas para exibição dos valores de Status Word
pos = 1
statusEntry = {}
statusValue = {}
for word in sorted( statusWord.keys(), key = statusWord.get ):
   Label( statusFrame, text = word + ':' ).grid( row = pos, column = 1, padx = 20, pady = 5, sticky = E )
   statusValue[ word ] = IntVar()
   statusEntry[ word ] = Label( statusFrame, textvariable = statusValue[ word ], width = 5, fg = 'black', bg = 'white', justify = CENTER, relief = SUNKEN )
   statusEntry[ word ].grid( row = pos, column = 2, padx = 10, sticky = W )
   pos += 1
   
# Frame de exibição dos valores medidos
dimensionFrame = LabelFrame( window, text = 'VALORES MEDIDOS' )#, labelanchor = 'n' )
dimensionFrame.grid( row = 5, column = 3, rowspan = 3, pady = 5 ) 
# Caixas para exibição dos valores medidos
pos = 1
dimensionEntry = {}
dimensionScale = {}
dimensionCheck = {}
for dimension in sorted( dimensionIndex.keys(), key = dimensionIndex.get ):
   dimensionCheck[ dimension ] = IntVar()
   Checkbutton( dimensionFrame, variable = dimensionCheck[ dimension ], fg = 'black' ).grid( row = pos, column = 1, padx = 10 )
   Label( dimensionFrame, text = dimensionTranslation[ dimension ] + ':' ).grid( row = pos, column = 2, pady = 10, sticky = E )
   dimensionEntry[ dimension ] = Label( dimensionFrame, width = 12, fg = 'black', bg = 'white', justify = CENTER, relief = SUNKEN )
   dimensionEntry[ dimension ].grid( row = pos, column = 3, padx = 10, sticky = W )
   dimensionScale[ dimension ] =  Scale( dimensionFrame, label = 'Escala', from_ = 10, to = 1, length = 30, width = 10, sliderlength = 5, troughcolor = 'white' )
   dimensionScale[ dimension ].grid( row = pos, column = 4, sticky = W )
   pos += 1

# Canvas para exibição do grafico
scroll = ttk.Scrollbar( window, orient = HORIZONTAL )
figure = Figure( dpi = 100, figsize = (5, 4) )
figure.subplots_adjust( left = 0.05, right = 0.9, top = 0.97, bottom = 0.05, hspace = 0.1 )
plotWidth = ( 400, 100 )
mplCanvas = FigureCanvas( figure, master = window )
mplCanvas.show()
dataCanvas = mplCanvas.get_tk_widget()

# Callback para deslocar o canvas da matplotlib com o slider (não ideal, procurando solução melhor) 
def move( mode, position, c = UNITS ):
   if float(position) < 0.0: position = 0.0
   if float(position) > 1.0: position = 1.0
   width = scroll.get()[1] - scroll.get()[0]
   space = 1.0 - width
   scroll.set( float(position) * space, float(position) * space + width )
scroll[ 'command' ] = move
dataCanvas.grid( row = 5, column = 2, rowspan = 2, sticky = (N, S, E, W) )
scroll.grid( row = 7, column = 2, pady = 5, sticky = (N, E, W) )

# Espaço para configurações personalizadas
###############################################################################################################################################################################

setControlValue( 1, parameterIndex[ 'Rigidez' ], 0.0 )
controlValue[ 1 ][ 'Rigidez'  ].set( getControlValue( 1, parameterIndex[ 'Rigidez'  ] ) )
setControlValue( 1, parameterIndex[ 'Amortecimento' ], 5.0 )
controlValue[ 1 ][ 'Amortecimento' ].set( getControlValue( 1, parameterIndex[ 'Amortecimento' ] ) )
		 
setControlValue( 2, parameterIndex[ 'Rigidez'  ], 15.0 )
controlValue[ 2 ][ 'Rigidez'  ].set( getControlValue( 2, parameterIndex[ 'Rigidez'  ] ) )
setControlValue( 2, parameterIndex[ 'Amortecimento' ], 20.0 )
controlValue[ 2 ][ 'Amortecimento' ].set( getControlValue( 2, parameterIndex[ 'Amortecimento' ] ) )

###############################################################################################################################################################################

# Regisra função de inicialização para ser executada em seguida
window.after( 100, init )
window.after( 1000, updateScreen )

# Inclui os elementos gráficos criados no loop de atualização da janela
window.mainloop()
