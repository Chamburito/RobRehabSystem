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
axisDisplayName = StringVar()

# Lista de eixos visualizáveis
axesList = {}

# Função de conexão com servidor ( Computador recebendo os dados )
def setupConnection():
    
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
   
   
def addAxis( axisName ):
   print( 'New Axis created: ' + axisName )
   axesList[ axisName ] = Axis()
   axisSelector[ 'values' ] = list( axesList.keys() )
   if len(axisSelector[ 'values' ]) == 1: axisSelector.current( 0 )
   
# Função para receber valores de eixos do servidor central ou cliente conectado
def updateData():
   # Garantindo o preenchimento de todos os valores
   for axis in axexList.values():
      for dimension in list(Dimension):
         axis.dimensionValues[ dimension ].append( axis.dimensionValues[ dimension ][-1] )

   # Comunicação via socket UDP
   message = None
   if dataClientId is not None:
      message = receiveMessage( dataConnectionId )
      if message is not None:
         print( 'Received message: ' + message )
         dataList = message.split(':')
         if len(dataList) >= 4 and ( len(dataList) - 1 ) % 3 == 0:
            if dataList[0] == 'Axis Data':
               for axisDataOffset in range( 1, 3, len(clientData) - 2 ):
                  axisName = dataList[ axisDataOffset ]
                  
                  if axisName not in axesList: addAxis( axisName )
                  
                  axesList[ axisName ].dimensionValues[ Dimension.POSITION ][-1] = int(dataList[ axisDataOffset + 1 ])
                  axesList[ axisName ].dimensionValues[ Dimension.VELOCITY ][-1] = int(dataList[ axisDataOffset + 2 ])
      else:
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
            if axisName not in axesList: addAxis( axisName )
            valueIndex = int(infoList[2])
            if infoList[0] == 'Axis Status':
               axesList[ axisName ].statusValues[ valueIndex ] = int(infoList[3])
               if axisName == axisDisplayName.get():
                  statusEntryValues[ valueIndex ].set( axesList[ axisName ].statusValues[ valueIndex ] )
            elif infoList[0] == 'Axis Parameter':
               axesList[ axisName ].controlValues[ valueIndex ] = float(infoList[3])
               if axisName == axisDisplayName.get():
                  controlEntryValues[ valueIndex ].set( axesList[ axisName ].controlValues[ valueIndex ] )
            elif infoList[0] == 'Axis Control' and valueIndex == ControlWord.ENABLE_OPERATION:
               axesList[ axisName ].controlEnabled = int(infoList[3])
               if axisName == axisDisplayName.get():
                  controlEnabled.set( axesList[ axisName ].controlEnabled )
               
   window.after( 1000, updateInfo )


# Função de atualização da tela 
def updateScreen():
   global plotWidth
  
   nPlots = 0
   
   figure.clear()
  
   if len(axesList) > 0:
      currentAxis = axesList[ axisDisplayName.get() ]
      
      time = len(currentAxis.dimensionValues[ Dimension.Position ])
      
      # Plota valores de posição, velocidade, corrente e tensão no gráfico   
      begin = int(time * scroll.get()[0])
      end = int(time * scroll.get()[1])
      
      for check in dimensionChecks.values():
          if check.get() == 1:
            nPlots += 1
        
      plotId = 0
      for dimensionType in currentAxis.dimensionValues.keys():
          if dimensionChecks[ dimensionType ].get() == 1:
            plotId += 1
            plot = figure.add_subplot( nPlots, 1, plotId, anchor = 'E' )
            plot.set_ylabel( dimensionName[ dimensionType ], fontdict = { 'fontsize' : 8 } )
            plot.tick_params( 'y', labelsize = 8, labelright = 'on', labelleft = 'off' )
            plot.tick_params( 'x', labelsize = 0, labeltop = 'off', labelbottom = 'off' )
            #plot.autoscale( axis = 'y', tight = False )
            if time > plotWidth[0]:
                plot.set_xlim( left = begin, right = end )
            else:
                plot.set_xlim( left = 0, right = plotWidth[0] )
            interval = plotWidth[1] * dimensionScales[ dimensionType ].get()
            plot.set_ylim( top = interval / 2, bottom = -interval / 2 )
            plot.plot( currentAxis.dimensionValues[ dimensionType ], color = dimensionColor[ dimensionType ] )
            #plot.axhline( 0, 0, 1, color = 'black' )
            
          if len(currentAxis.dimensionValues[ dimensionType ]) > 0:
            dimensionEntries[ dimensionType ][ 'text' ] = '{:.3g} {}'.format( currentAxis.dimensionValues[ dimensionType ][-1], 
                                                                              dimensionUnit[ dimensionType ] )
          
      figure.axes[-1].tick_params( 'x', labelsize = 8, labeltop = 'off', labelbottom = 'on' )
      
      # Desloca slider quando gráfico cresce além do tamanho do canvas
      if time > plotWidth[0]:
          if scroll.get()[1] == 1.0:
            scroll.set( 1.0 - plotWidth[0] / time, 1.0 )
          else:
            scroll.set( scroll.get()[0] * (time - 1) / time, (scroll.get()[0] * (time - 1) + plotWidth[0]) / time )
   
   if len(axesList) == 0 or nPlots == 0:   
      plot = figure.add_subplot( 111 )
      plot.tick_params( labelsize = 8, labelright = 'on', labelleft = 'off' )
      plot.axis( [ 0, plotWidth[0], -plotWidth[1] / 2, plotWidth[1] / 2 ] )
      plot.axhline( 0, 0, 1, color = 'black' )
            
   mplCanvas.draw()
   mplCanvas.show()
   
   # Registra função de atualização para ser executada novamente
   window.after( 100, updateScreen )

# Função de Reset das falhas
def resetFault():
   sendMessage( infoConnectionId, 'Axis Control:' + str(axisDisplayName.get()) + ':' + str(ControlWord.FAULT_RESET) + ':1' )
   sendMessage( infoConnectionId, 'Axis Control:' + str(axisDisplayName.get()) + ':' + str(ControlWord.FAULT_RESET) + ':0' )

# Propriedades da Janela
window.title( 'RobRehabGui' )
window.resizable( width = False, height = False )
window.columnconfigure( 1, minsize = 400 )
window.columnconfigure( 2, minsize = 400 )
window.columnconfigure( 3, minsize = 400 )
      
# Frame de apresentação do programa
headerFrame = LabelFrame( window )
headerFrame.grid( row = 1, column = 1, columnspan = 3, pady = 10 )     
# Label do Título    
titleLabel = Label( headerFrame, width = 70, height = 5 )
titleLabel[ 'text' ] = ( 'CONTROLE DE IMPEDÂNCIA EXO-KANGUERA - EPOS CANOpen\n'
                         'ESCOLA DE ENGENHARIA DE SÃO CARLOS - EESC/USP\n'
                         'LABORATÓRIO DE REABILITAÇÃO ROBÓTICA' )
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
initLabel[ 'text' ] = 'INICIALIZE COMUNICAÇÃO COM AS EPOS'
initLabel.grid( row = 2, column = 1, columnspan = 3 )
      
# Frame auxiliar para melhor alinhamento
optionsFrame = Frame( window )
optionsFrame.grid( row = 3, column = 1, columnspan = 3, ipady = 10 )
      
# Configurações de conexão
serverFrame = LabelFrame( optionsFrame, text = 'Configurações de Conexão' )
serverFrame.pack( side = LEFT, padx = 20, pady = 10, ipady = 10 )

# Configuração de host
Label( serverFrame, text = 'Endereço (Domínio ou IP):' ).pack( side = LEFT, padx = 10, anchor = E )
Entry( serverFrame, textvariable = serverHost, width = 20, fg = 'black', bg = 'white', justify = CENTER ).pack( side = LEFT, padx = 10 )
# Botão de ativação da conexão
ttk.Button( serverFrame, text = 'Conectar', command = setupConnection ).pack( side = LEFT, padx = 20 )

#Frames de configuração das controladoras dos eixos
axisFrame = LabelFrame( optionsFrame, text = 'Seleção de Eixo' )
axisFrame.pack( side = LEFT, padx = 20, pady = 10 )

axisSelector = ttk.Combobox( axisFrame, textvariable = axisDisplayName, justify = CENTER )
axisSelector.grid( row = 1, column = 1, columnspan = 2, padx = 10 )

Label( axisFrame, text = 'Controle' ).grid( row = 1, column = 3, padx = 10 )
controlEnabled = IntVar()
# Função de início/término do controle (Definida para cada Eixo)
def toggleControl(): 
   sendMessage( infoConnectionId, 'Axis Control:{0:s}:{1:d}:{2:d}'.format( axisDisplayName.get(), 
                                                                           ControlWord.SWITCH_ON, controlEnabled.get() ) )
   sendMessage( infoConnectionId, 'Axis Control:{0:s}:{1:d}:{2:d}'.format( axisDisplayName.get(), 
                                                                           ControlWord.ENABLE_OPERATION, controlEnabled.get() ) )
   
Checkbutton( axisFrame, variable = controlEnabled, command = toggleControl, fg = 'black' ).grid( row = 1, column = 4, sticky = W )

pos = 0
controlEntryValues = {}
for parameterType in list(Parameter):
   Label( axisFrame, text = parameterName[ parameterType ] + ':', width = 15, justify = LEFT ).grid( row = 2, column = pos * 2 + 1, pady = 5 )

   controlEntryValues[ parameterType ] = DoubleVar()
   # Função de evento da caixa de texto
   def setParameter( event, parameterType = parameterType ):
      sendMessage( infoConnectionId, 'Axis Parameter:{0:s}:{1:d}:{2:.3g}'.format( axisDisplayName.get(), 
                                                                                  parameterType, controlEntryValues[ parameterType ].get() ) )
      axesList[ axisDisplayName.get() ].controlValues[ parameterType ] = controlEntryValues[ parameterType ].get()

   parameterEntry = Entry( axisFrame, textvariable = controlEntryValues[ parameterType ], width = 5, fg = 'black', bg = 'white', justify = CENTER )
   parameterEntry.bind( ('<KeyPress-Return>', '<KeyRelease-Return>'), setParameter )
   parameterEntry.bind( '<FocusOut>', setParameter )
   parameterEntry.grid( row = 2, column = pos * 2 + 2, padx = 10, sticky = E )
   pos += 1


def axisChange( event ):
   currentAxis = axesList[ axisDisplayName.get() ]
   for statusType in list(StatusWord):
      statusEntryValues[ statusType ].set( currentAxis.statusValues[ statusType ] )
   for parameterType in list(Parameter):
      controlEntryValues[ parameterType ].set( currentAxis.controlValues[ valueIndex ] )
   controlEnabled.set( currentAxis.controlEnabled )
      
axisSelector.bind( '<<ComboboxSelected>>', axisChange )

# Botão de Reset de falhas
resetButton = ttk.Button( window, text = 'Reset de Falhas', command = resetFault )
resetButton.grid( row = 4, column = 1, sticky = S )

# Frame de exibição dos valores de StatusWord 
statusFrame = LabelFrame( window, text = 'LEITURA DE STATUS WORDS' )
statusFrame.grid( row = 5, column = 1, rowspan = 2, pady = 30, ipadx = 10, ipady = 5, sticky = N )   
# Caixas para exibição dos valores de Status Word
pos = 1
statusEntries = {}
statusValues = {}
for statusName, statusType in StatusWord.__members__.items():
   Label( statusFrame, text = statusName.replace( '_', ' ' ) ).grid( row = pos, column = 1, padx = 20, pady = 5, sticky = E )
   statusValues[ statusType ] = IntVar()
   statusEntries[ statusType ] = Label( statusFrame, textvariable = statusValues[ statusType ], 
                                        width = 5, fg = 'black', bg = 'white', justify = CENTER, relief = SUNKEN )
   statusEntries[ statusType ].grid( row = pos, column = 2, padx = 10, sticky = W )
   pos += 1
   
# Frame de exibição dos valores medidos
dimensionFrame = LabelFrame( window, text = 'VALORES MEDIDOS' )
dimensionFrame.grid( row = 4, column = 3, rowspan = 3, pady = 5 ) 
# Caixas para exibição dos valores medidos
pos = 1
dimensionEntries = {}
dimensionScales = {}
dimensionChecks = {}
for dimensionType in list(Dimension):
   dimensionChecks[ dimensionType ] = IntVar()
   Checkbutton( dimensionFrame, variable = dimensionChecks[ dimensionType ], fg = 'black' ).grid( row = pos, column = 1, padx = 10 )
   Label( dimensionFrame, text = dimensionName[ dimensionType ] + ':' ).grid( row = pos, column = 2, pady = 10, sticky = E )
   dimensionEntries[ dimensionType ] = Label( dimensionFrame, width = 12, fg = 'black', bg = 'white', justify = CENTER, relief = SUNKEN )
   dimensionEntries[ dimensionType ].grid( row = pos, column = 3, padx = 10, sticky = W )
   dimensionScales[ dimensionType ] =  Scale( dimensionFrame, label = 'Escala', from_ = 10, to = 1, 
                                              length = 30, width = 10, sliderlength = 5, troughcolor = 'white' )
   dimensionScales[ dimensionType ].grid( row = pos, column = 4, sticky = W )
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

# Registra função para ser executada
window.after( 1000, updateScreen )

# Inclui os elementos gráficos criados no loop de atualização da janela
window.mainloop()
