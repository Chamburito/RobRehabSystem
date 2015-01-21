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
serverPort = 11000
# Manipuladores de conexão com servidor ou cliente
infoConnectionId = None
dataConnectionId = None

# Método ativado no encerramento da janela principal
def close():
   print( 'Destroying Window' )
   if dataConnectionId is not None: sendMessage( dataConnectionId, 'Axis End' )   
   
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
axisNames = []

# Função de conexão com servidor ( Computador recebendo os dados )
def setupConnection():
   global playerNetworkId
   global axisNames
   global remoteServerEnabled
   
   maxIdLength = 10
   playerNetworkId = playerName.get() if len(playerName.get()) <= maxIdLength else playerName.get()[:maxIdLength]
   for axisId in range( nAxis ):
      axisNames[ playerNetworkId + ' ' + controlMode[ axisId ] ] = axisId
   
   if ( infoClientId is None ) or ( dataClientId is None ) or ( currentServerHost != serverHost.get() ):
      if infoClientId is not None: closeConnection( infoClientId )
      if dataClientId is not None: closeConnection( dataClientId )
      
      infoClientId = createConnection( serverHost.get(), str(serverPort), NetworkInterface.TCP )
      dataClientId = createConnection( serverHost.get(), str(serverPort), NetworkInterface.UDP )
      
      currentServerHost = serverHost.get()
      
   if infoClientId is not None:
      initLabel[ 'text' ] = 'COMUNICAÇÃO INICIALIZADA'
      ( host, port ) = getAddress( infoClientId )
      print( 'host: ' + str(host) + ' - port: ' + str(port) )
   if dataClientId is not None:
      ( host, port ) = getAddress( dataClientId )
      print( 'host: ' + str(host) + ' - port: ' + str(port) )
      
   window.after( 100, updateInfo )
   window.after( 100, sendData )


dimensionValue = {}
axisMotionTimes = {}
axisControlChecks = {}
axisControlValues = {}

def addAxis( axisName ):
   axisNames.append( axisName )
   dimensionValue[ axisName ] = {}
   for dimension in list(Dimension):
      dimensionValue[ axisName ][ dimension ] = []
      dimensionValue[ axisName ][ dimension ].append( 0 )
      
   axisMotionTimes[ axisName ] = []
   axisMotionTimes[ axisName ].append( execTime() * 1000 )
   
   axisControlChecks[ axisName ] = 0
   
   axisControlValues[ axisName ] = {}
   for parameterIndex in list(Parameter):
      axisControlValues[ axisName ][ parameterIndex ] = 0
   
   
# Função para receber valores de eixos do servidor central ou cliente conectado
def updateData():
   # Garantindo o preenchimento de todos os valores
   for axisName in axisNames:
      for dimension in list(Dimension):
         dimensionValue[ axisName ][ dimension ].append( dimensionValue[ axisName ][ dimension ][-1] )

   # Comunicação via socket UDP
   message = None
   if dataClientId is not None:
      message = receiveMessage( dataClientId )
      if message is not None:
         print( 'Received message: ' + message )
         dataList = message.split(':')
         if len(dataList) >= 4 and ( len(dataList) - 1 ) % 3 == 0:
            if dataList[0] == 'Axis Data':
               for axisDataOffset in range( 1, 3, len(clientData) - 2 ):
                  axisName = dataList[axisDataOffset]
                  
                  if axisName not in axisNames: addAxis( axisName )
                  
                  dimensionValue[ axisName ][ Dimension.Position ][-1] = int(dataList[ axisDataOffset + 1 ])
                  dimensionValue[ axisName ][ Dimension.Velocity ][-1] = int(dataList[ axisDataOffset + 2 ])
      else
         sendMessage( dataClientId, 'Axis Data Request' )
               
      window.after( 5, receiveData )

  
# Função de atualização de informações do cliente
def updateInfo():
   # Comunicação via socket TCP
   message = None   
   if infoConnectionId is not None:  
      message = receiveMessage( infoConnectionId )
      if message is not None: 
         print( 'Received Control Message: ' + message )
         infoList = message.split()
         if len(infoList) >= 4:
            axisName = infoList[1]
            valueIndex = int(infoList[2])
            if infoList[0] == 'Axis Status':
               statusValues[ axisName ][ valueIndex ] = int(infoList[3])
            elif infoList[0] == 'Axis Parameter':
               parameterValues[ axisName ][ valueIndex ] = int(infoList[3])
               
   window.after( 1000, updateInfo )


# Função de atualização da tela 
def updateScreen():
  
   for status in list(StatusWord):
      statusEntryValues[ status ].set( statusValues[ axisIdView.get() ][ status ] )
  
   time = len(dimensionValue[ axisIdView.get() ][ Dimension.Position ])
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
   sendMessage( infoConnectionId, 'Axis Control:' + str(axisIdView.get()) + ':' + str(ControlWord.FaultReset) + ':1' )
   sendMessage( infoConnectionId, 'Axis Control:' + str(axisIdView.get()) + ':' + str(ControlWord.FaultReset) + ':0' )

# Propriedades da Janela
window.title( 'RobRehabGui' )
window.resizable( width = False, height = False )
window.columnconfigure( 1, minsize = 400 )
window.columnconfigure( 2, minsize = 400 )
window.columnconfigure( 3, minsize = 400 )
      
# Frame de apresentação do programa
headerFrame = LabelFrame( window )
headerFrame.grid( row = 1, column = 1, columnspan = 3, pady = 5 )     
# Label do Título    
titleLabel = Label( headerFrame, width = 70, height = 5 )
titleLabel[ 'text' ] = 'CONTROLE DE IMPEDÂNCIA EXO-KANGUERA - EPOS CANOpen\nESCOLA DE ENGENHARIA DE SÃO CARLOS - EESC/USP\nLABORATÓRIO DE REABILITAÇÃO ROBÓTICA\n'
titleLabel[ 'font' ] = ( 'comic sans', 12, 'bold' )
titleLabel.grid( row = 1, column = 2 )
# Logos
robRehabLogo = PhotoImage( file='rehab_resized.gif' )
robRehabLabel = ttk.Label( headerFrame, image = robRehabLogo )
robRehabLabel.grid( row = 1, column = 1, padx = 10, pady = 10 )
eescLogo = PhotoImage( file='eesc_resized.gif' )
eescLabel = ttk.Label( headerFrame, image = eescLogo )
eescLabel.grid( row = 1, column = 3, padx = 10, pady = 5 )
      
# Label do status de inicialização      
initLabel = Label( window, height = 2, width = 60, relief = RIDGE )
initLabel['text'] = 'INICIALIZANDO COMUNICAÇÃO CANOpen COM AS EPOS'
initLabel.grid( row = 2, column = 1, columnspan = 3, pady = 5 )
      
# Configurações de conexão
serverFrame = LabelFrame( window, text = 'Configurações de Conexão' )
serverFrame.grid( row = 3, column = 1, columnspan = 2, pady = 5, ipady = 10 )

# Configuração de identificador de rede ( nome do usuário )
Label( serverFrame, text = 'Usuário:' ).pack( side = LEFT, padx = 10, anchor = E )
Entry( serverFrame, textvariable = playerName, width = 20, fg = 'black', bg = 'white', justify = CENTER ).pack( side = LEFT, padx = 10 )
# Configuração de host
Label( serverFrame, text = 'Endereço (Domínio ou IP):' ).pack( side = LEFT, padx = 10, anchor = E )
Entry( serverFrame, textvariable = serverHost, width = 20, fg = 'black', bg = 'white', justify = CENTER ).pack( side = LEFT, padx = 10 )
# Ativação de uso de servidor remoto ( dados dos eixos vão para servidor principal )
Label( serverFrame, text = 'Utilizar Servidor Remoto' ).pack( side = LEFT, padx = 10, anchor = E )
Checkbutton( serverFrame, variable = useRemoteServer, fg = 'black' ).pack( side = LEFT )
# Botão de ativação da conexão
ttk.Button( serverFrame, text = 'Conectar', command = setupConnection ).pack( side = LEFT, padx = 40 )

#Frames de configuração das controladoras dos eixos
axisFrame = LabelFrame( window, text = 'Seleção de Eixo' )
axisFrame.grid( row = 3, column = 3, pady = 10 )

axisViewName = StringVar()
axisSelector = ttk.Combobox( axisFrame, textvariable = axisViewName, justify = CENTER )
axisSelector.grid( row = 1, column = 1, columnspan = 2, padx = 10 )

Label( axisFrame, text = 'Controle' ).grid( row = 1, column = 3, padx = 10 )
controlEnabled = IntVar()
# Função de início/término do controle (Definida para cada Eixo)
def toggleControl(): sendMessage( infoConnectionId, 'Axis Control:{0:s}:Enable:{1:d}'.format( axisViewName.get(), controlEnabled.get() ) )   
Checkbutton( axisFrame, variable = controlEnabled[ axisId ], command = toggleControl, fg = 'black' ).grid( row = 1, column = 4, sticky = W )

pos = 0
parameterValues = {}
for parameterIndex in list(Parameter):
   Label( axisFrame, text = parameterName[ parameterIndex ] + ':', width = 10, justify = LEFT ).grid( row = 2, column = pos * 2 + 1, pady = 5 )

   parameterValues[ parameterIndex ] = DoubleVar()
   # Função de evento da caixa de texto
   def setParameter( event, parameterIndex = parameterIndex ):
      sendMessage( infoConnectionId, 'Axis Parameter:{0:s}:{1:d}:{2:.3g}'.format( axisIdView.get(), parameterIndex, parameterValues[ parameterIndex ].get() ) )
      axisControlValues[ axisViewName.get() ][ parameterIndex ] = parameterValues[ parameterIndex ].get()

def axisViewChange():
   if axisViewName.get() in axisNames:
      controlEnabled.set( axisControlValues[ axisViewName.get() ][ Parameter.Enabled ] )
      for parameterIndex in list(Parameter):
         parameterValues[ parameterIndex ].set( axisControlValues[ axisViewName.get() ][ parameterIndex ] )
axisSelector.bind( '<<ComboboxSelected>>', axisViewChange )

# controlCheck = {}
# controlValue = {}
# for axisId in range(nAxis):
   # axisFrame = LabelFrame( window, text = str(controlMode[ axisId ]) )
   # axisFrame.grid( row = 4, column = axisId + 1, pady = 10 )
   # Label( axisFrame, text = 'Visualização' ).grid( row = 1, column = 1, padx = 10 )
   # Radiobutton( axisFrame, variable = axisIdView, value = axisId, fg = 'black' ).grid( row = 1, column = 2, sticky = W )
   # Label( axisFrame, text = 'Controle' ).grid( row = 1, column = 3, padx = 10 )
   
   # controlCheck[ axisId ] = IntVar()
   # Função de início/término do controle (Definida para cada Eixo)
   # def toggleControl( axisId = axisId ):
      # setAxisEnabled( axisId, controlCheck[ axisId ].get() )
      # setControl( axisId, controlCheck[ axisId ].get(), axisId )
   
   # Checkbutton( axisFrame, variable = controlCheck[ axisId ], command = toggleControl, fg = 'black' ).grid( row = 1, column = 4, sticky = W )
   
   # pos = 0
   # controlValue[ axisId ] = {}
   # for parameter in sorted( parameterIndex.keys(), key = parameterIndex.get ):
      # Label( axisFrame, text = parameter + ':', width = pos * 6 + 6, justify = RIGHT ).grid( row = 2, column = pos * 2 + 1, pady = 5 )

      # controlValue[ axisId ][ parameter ] = DoubleVar()
      # controlValue[ axisId ][ parameter ].set( getControlValue( axisId, parameterIndex[ parameter ] ) )
      # Função de evento da caixa de texto
      # def setParameter( event, axisId = axisId, parameter = parameter ):
         # setControlValue( axisId, parameterIndex[ parameter ], controlValue[ axisId ][ parameter ].get() )
         # controlValue[ axisId ][ parameter ].set( getControlValue( axisId, parameterIndex[ parameter ] ) )
   
      # parameterEntry = Entry( axisFrame, textvariable = controlValue[ axisId ][ parameter ], width = 5, fg = 'black', bg = 'white', justify = CENTER )
      # parameterEntry.bind( ('<KeyPress-Return>', '<KeyRelease-Return>'), setParameter )
      # parameterEntry.bind( '<FocusOut>', setParameter )
      # parameterEntry.grid( row = 2, column = pos * 2 + 2, padx = 10, sticky = E )
      # pos += 1

# Botão de Reset de falhas
resetButton = ttk.Button( window, text = 'Reset de Falhas', command = resetFault )
resetButton.grid( row = 4, column = 1, sticky = S )

# Frame de exibição dos valores de StatusWord 
statusFrame = LabelFrame( window, text = 'LEITURA DE STATUS WORDS' )#, labelanchor = 'n' )
statusFrame.grid( row = 5, column = 1, rowspan = 2, pady = 30, ipadx = 10, ipady = 5, sticky = N )   
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
dimensionFrame.grid( row = 4, column = 3, rowspan = 3, pady = 5 ) 
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
dataCanvas.grid( row = 4, column = 2, rowspan = 2, sticky = (N, S, E, W) )
scroll.grid( row = 6, column = 2, pady = 5, sticky = (N, E, W) )

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
