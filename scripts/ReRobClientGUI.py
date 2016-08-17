#-*-coding:cp1252-*-

import sys
import math

from Tkinter import Tk as MainWindow
from Tkinter import *

from ReRobIPNetwork import *

SIGNAL_AMPLITUDE = 0.1;

UPDATE_PERIOD = 8.0;
UPDATE_FREQUENCY = 1 / UPDATE_PERIOD;
CONTROL_PASS_INTERVAL = 0.005
DISPLAY_POINTS_NUMBER = (int) ( UPDATE_PERIOD / CONTROL_PASS_INTERVAL );

PHASE_CYCLES_NUMBER = 5;

class SelectionList:
  values = []
  currentIndex = 0
  def GetValue( self, next = 0 ):
    if len( self.values ) == 0: return None
     
    self.currentIndex += next
    if self.currentIndex < 0: self.currentIndex = len( self.values ) - 1
    elif self.currentIndex >= len( self.values ): self.currentIndex = 0
      
    return self.values[ self.currentIndex ]

client = ClientConnection()
def Connect():
  client.Connect( currentServerAddress.get() )
  RefreshRobotsInfo()

robotSelection = SelectionList()
jointSelection = SelectionList()
def RefreshRobotsInfo():
  robotsInfo = client.RefreshInfo()
  robotSelection.values = robotsInfo[ 0 ]
  currentRobotID.set( robotSelection.GetValue() )
  jointSelection.values = robotsInfo[ 1 ]
  currentJointID.set( jointSelection.GetValue() )

def UpdateEvents():  
  window.after( 1000, UpdateEvents )

def UpdateData(): 
  client.SendAxisData( currentJointID.get(), 0.0, 0.0, stiffnessSlider.get() )
  measures = client.ReceiveAxisData( currentJointID.get() )
  for measureIndex in range( len( measureSliders ) ):
    measureSliders[ measureIndex ].set( measures[ measureIndex ] )

  window.after( 100, UpdateData )

# Criação da Janela principal
window = MainWindow()

# Variável de nome do usuario
userName = StringVar() 
userName.set( 'Player' )

# Variável de endereco para conexao
currentServerAddress = StringVar() 
currentServerAddress.set( DEFAULT_ADDRESS )

# Propriedades da Janela
window.title('RobRehabGUI')
window.resizable( width = False, height = False )
window.columnconfigure( 1, minsize = 300 )
window.columnconfigure( 2, minsize = 300 )

# Frame de apresentação do programa
headerFrame = LabelFrame( window )
headerFrame.grid( row = 1, column = 1, columnspan = 2, pady = 5 )     
# Label do Título    
titleLabel = Label( headerFrame, width = 50, height = 5 )
titleLabel['text'] = '''ROBOTIC REHABILITATION SYSTEM
                        \n\nCONTROL AND MONITORING GRAPHICAL INTERFACE'''
titleLabel['font'] = ('sans', 32, 'bold')
titleLabel.grid( row = 1, column = 2 )
# Logos
robRehabLogo = PhotoImage( file='rehab_resized.gif' )
robRehabLabel = Label( headerFrame, image = robRehabLogo )
robRehabLabel.grid( row = 1, column = 1, padx = 10, pady = 10 )
eescLogo = PhotoImage( file='eesc_resized.gif' )
eescLabel = Label( headerFrame, image = eescLogo )
eescLabel.grid( row = 1, column = 3, padx = 10, pady = 5 )

# Configurações de conexão
serverFrame = LabelFrame( window, text = 'Connection Configuration' )
serverFrame.grid( row = 2, column = 1, columnspan = 2, pady = 5, ipady = 10 )
# Configuração de identificador de rede ( nome do usuário )
Label( serverFrame, text = 'User:' ).pack( side = LEFT, padx = 10, anchor = E )
Entry( serverFrame, textvariable = userName, width = 20, fg = 'black', bg = 'white', justify = CENTER ).pack( side = LEFT, padx = 10 )
# Configuração de host
Label( serverFrame, text = 'Server Address:' ).pack( side = LEFT, padx = 10, anchor = E )
Entry( serverFrame, textvariable = currentServerAddress, width = 20, fg = 'black', bg = 'white', justify = CENTER ).pack( side = LEFT, padx = 10 )
# Botão de ativação da conexão
Button( serverFrame, text = 'Connect', command = Connect ).pack( side = LEFT, padx = 40 )

# Robot/Joint information and selection frame
selectionFrame = LabelFrame( window, text = 'Joint Selection' )
selectionFrame.grid( row = 3, column = 1, columnspan = 2, pady = 5, ipady = 10 )
# Robot/Joint information refresh button
Button( selectionFrame, text = 'Refresh Robots Info', command = RefreshRobotsInfo ).pack( side = LEFT, padx = 40 )
# Robot selector
Label( selectionFrame, text = 'Robot ID:' ).pack( side = LEFT, padx = 10, anchor = E )
currentRobotID = StringVar()
Button( selectionFrame, text = '<', command = lambda: currentRobotID.set( robotSelection.GetValue( -1 ) ) ).pack( side = LEFT, padx = 10 )
Entry( selectionFrame, textvariable = currentRobotID, width = 5, fg = 'black', bg = 'white', justify = CENTER ).pack( side = LEFT )
Button( selectionFrame, text = '>', command = lambda: currentRobotID.set( robotSelection.GetValue( +1 ) ) ).pack( side = LEFT, padx = 10 )
# Joint selector
Label( selectionFrame, text = 'Joint ID:' ).pack( side = LEFT, padx = 10, anchor = E )
currentJointID = StringVar()
Button( selectionFrame, text = '<', command = lambda: currentJointID.set( jointSelection.GetValue( -1 ) ) ).pack( side = LEFT, padx = 10 )
Entry( selectionFrame, textvariable = currentJointID, width = 5, fg = 'black', bg = 'white', justify = CENTER ).pack( side = LEFT )
Button( selectionFrame, text = '>', command = lambda: currentJointID.set( jointSelection.GetValue( +1 ) ) ).pack( side = LEFT, padx = 10 )

# Label do status de inicialização      
commandsFrame = LabelFrame( window, text = 'Commands', height = 2, width = 60, relief = RIDGE )
commandsFrame.grid( row = 4, column = 1, padx = 10, pady = 5, ipady = 10, sticky = E )
# Botoes de commandos
commands = [ ('Enable',(2,1)), ('Offset',(5,4)), ('Calibrate',(6,4)), ('Optimize',(7,4)) ]
for commandIndex in range( len( commands ) ):
  command = commands[ commandIndex ]
  Label( commandsFrame, text = command[ 0 ] ).grid( row = 1, column = commandIndex + 1 )
  commandButton = Button( commandsFrame, text = 'Off' )
  def SendCommand( button = commandButton, values = command[ 1 ] ):
    if button[ 'text' ] == 'Off' and client is not None:
      client.SendEvent( currentJointID.get(), values[ 0 ] )
      button[ 'text' ] = 'On'
    elif client is not None:
      client.SendEvent( currentJointID.get(), values[ 1 ] )
      button[ 'text' ] = 'Off'
      
  commandButton[ 'command' ] = SendCommand
  commandButton.grid( row = 2, column = commandIndex + 1, padx = 20 )

#Frame de selecao de rigidez
stiffnessFrame = LabelFrame( window, text = 'Robot Stiffness', height = 2, width = 60, relief = RIDGE )
stiffnessFrame.grid( row = 5, column = 1, padx = 10, pady = 5, ipady = 10, sticky = E )
#Slider de rigidez
stiffnessSlider = Scale( stiffnessFrame, from_ = 0, to = 100, tickinterval = 10, orient = HORIZONTAL, width = 20, length = 300 )
stiffnessSlider.pack( side = LEFT, padx = 10, anchor = E )

#Measures display frame
measuresFrame = LabelFrame( window, text = 'Joint Measures', height = 2, width = 60, relief = RIDGE )
measuresFrame.grid( row = 4, column = 2, rowspan = 2, padx = 10, pady = 5, ipady = 10, sticky = W )
#Measure display labels and sliders
measureNames = [ 'Position', 'Velocity', 'Acceleration', 'Force', 'Stiffness', 'Damping' ]
measureSliders = []
for measureIndex in range( len( measureNames ) ):
  Label( measuresFrame, text = measureNames[ measureIndex ] ).grid( row = 1, column = measureIndex + 1 )
  measureSliders.append( Scale( measuresFrame, from_ = -100, to = 100, orient = VERTICAL, width = 20, length = 150 ) )
  measureSliders[ -1 ].grid( row = 2, column = measureIndex + 1, padx = 10, sticky = E )


# Registra funcoes a serem executadas seguida
window.after( 1000, UpdateEvents )
window.after( 100, UpdateData )

# Inclui os elementos gráficos criados no loop de atualização da janela
window.mainloop() 
