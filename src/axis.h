/* 
 * File:   AXIS.h
 * Author: GUILHERME FERNANDES
 *
 * ESTA FUNÇÃO REALIZA O CONTROLE DE UM EIXO DA EPOS
 * 
 * Created on 26 de Janeiro de 2012, 19:34
 */

#ifndef AXIS_H
#define	AXIS_H

#include "epos_network.h"

#ifdef WIN32
  #include "timing_windows.h"
#else
  #include "timing_unix.h"
#endif

#include <stdio.h>
#include <string.h>

enum OperationMode { HOMMING_MODE = 0x06, PROFILE_VELOCITY_MODE = 0x03, PROFILE_POSITION_MODE = 0x01,
	POSITION_MODE = 0xFF, VELOCITY_MODE = 0xFE, CURRENT_MODE = 0xFD, MASTER_ENCODER_MODE = 0xFB, STEP_MODE = 0xFA };

enum StatusWord { READY_2_SWITCH_ON = 1, SWITCHED_ON = 2, OPERATION_ENABLED = 4, FAULT = 8, VOLTAGE_ENABLED = 16,
	QUICK_STOPPED = 32, SWITCH_ON_DISABLE = 64, REMOTE_NMT = 512, TARGET_REACHED = 1024, SETPOINT_ACK = 4096 };

enum ControlWord { SWITCH_ON = 1, ENABLE_VOLTAGE = 2, QUICK_STOP = 4, ENABLE_OPERATION = 8, NEW_SETPOINT = 16,
	CHANGE_IMMEDIATEDLY = 32, ABS_REL = 64, FAULT_RESET = 128, HALT = 256 };

enum Dimension { POSITION, POSITION_SETPOINT, VELOCITY, VELOCITY_SETPOINT, CURRENT, TENSION, ANGLE, ANGLE_SETPOINT, FORCE, N_DIMS }; 

enum Parameter { KP, KD, N_PARAMS }; 

enum FrameType { SDO_TX, SDO_RX, PDO01_RX, PDO01_TX, PDO02_RX, PDO02_TX, N_FRAMES };
static const char* frame_names[ N_FRAMES ] = { "SDO_TX_0", "SDO_RX_0", "PDO01_RX_0", "PDO01_TX_0", "PDO02_RX_0", "PDO02_TX_0" };

typedef struct _Axis
{
  CAN_Frame* frame_list[ N_FRAMES ];
  double dimension_values[ N_DIMS ];
  double control_values[ N_PARAMS ];
  short unsigned int statusWord, controlWord;
  short int output;
  bool active;
}
Axis;

// Created axes count
static size_t n_axes = 0;

void axis_enable( Axis* );
void axis_disable( Axis* );
void axis_destroy( Axis* );
double axis_get_dimension_value( Dimension );
void axis_set_dimension_value( Dimension, double );   
double axis_get_control_value( Axis*, Parameter );
void axis_set_control_value( Axis*, Parameter, double );
void axis_set_output( Axis*, int );
bool axis_get_status_word( Axis*, StatusWord );
void axis_set_control_word( Axis*, ControlWord, bool );
void axis_read_values( Axis* );
void axis_write_values( Axis* );
void axis_set_operation_mode( Axis* OperationMode );
void axis_set_digital_output( Axis*, bool );

// Create CAN controlled DC motor handle
Axis* axis_create() 
{
  Axis* axis = (Axis*) malloc( sizeof(Axis) );

  for( int i = 0; i < N_DIMS; i++ )
    axis->dimension_values[ i ] = 0.0;
  for( int i = 0; i < N_PARAMS; i++ )
    axis->control_values[ i ] = 10.0;

  char network_address[20];
	
  for( int frame_id = 0; frame_id < N_FRAMES; frame_id += 2 )
  {
    sprintf( network_address, "%s%u", frame_names[ frame_id ], n_axes + 1 );
    printf( "axis_init: creating frame %s\n", network_address );
    axis->frame_list[ frame_id ] = epos_network_init_frame( FRAME_OUT, "CAN2", network_address );

    if( axis->frame_list[ frame_id ] == NULL ) axis_destroy( axis );
		    
    sprintf( network_address, "%s%u", frame_names[ frame_id + 1 ], n_axes + 1 );
    printf( "axis_init: creating frame %s\n", network_address );
    axis->frame_list[ frame_id + 1 ] = epos_network_init_frame( FRAME_IN, "CAN1", network_address );

    if( axis->frame_list[ frame_id + 1 ] == NULL ) axis_destroy( axis );
  }

  if( axis != NULL )
  {
    axis_disable( axis );
    n_axes++;
  }

  return axis;
}

void axis_enable( Axis* axis )
{
  axis_set_control_word( axis, SWITCH_ON, false );
  axis_set_control_word( axis, ENABLE_VOLTAGE, true );
  axis_set_control_word( axis, QUICK_STOP, true );
  axis_set_control_word( axis, ENABLE_OPERATION, false );
  axis_write_values( axis );
        
  delay( 500 );
        
  axis_set_control_word( axis, SWITCH_ON, true );
  axis_set_control_word( axis, ENABLE_VOLTAGE, true );
  axis_set_control_word( axis, QUICK_STOP, true );
  axis_set_control_word( axis, ENABLE_OPERATION, false );
  axis_write_values( axis );

  delay( 500 );
        
  axis_set_control_word( axis, SWITCH_ON, true );
  axis_set_control_word( axis, ENABLE_VOLTAGE, true );
  axis_set_control_word( axis, QUICK_STOP, true );
  axis_set_control_word( axis, ENABLE_OPERATION, true );
  axis_write_values( axis );

  axis->active = true;
}

void axis_disable( Axis* axis )
{
  axis_set_control_word( axis, SWITCH_ON, true );
  axis_set_control_word( axis, ENABLE_VOLTAGE, true );
  axis_set_control_word( axis, QUICK_STOP, true );
  axis_set_control_word( axis, ENABLE_OPERATION, false );
  axis_write_values( axis );
        
  delay( 500 );
        
  axis_set_control_word( axis, SWITCH_ON, false );
  axis_set_control_word( axis, ENABLE_VOLTAGE, true );
  axis_set_control_word( axis, QUICK_STOP, true );
  axis_set_control_word( axis, ENABLE_OPERATION, false );
  axis_write_values( axis );

  axis->active = false;
}

void axis_destroy( Axis* axis )
{

  free( axis );
  axis = NULL;
}

class Axisx
{
  private:

  //FRAMES DA REDE CAN
  CANFrame SDO_TX, SDO_RX, PDO01_RX, PDO01_TX, PDO02_RX, PDO02_TX;
  double dimensionValue[ N_DIMS ];
  double controlValue[ N_PARAMS ];
  short unsigned int statusWord, controlWord;
  short int output;

  vector<double> setpoints;

  public:

  bool active;

  void Enable()
    {  
      PDOsetControlWord( SWITCH_ON, false );
      PDOsetControlWord( ENABLE_VOLTAGE, true );
      PDOsetControlWord( QUICK_STOP, true );
      PDOsetControlWord( ENABLE_OPERATION, false );
      WritePDO01();
        
      delay( 500 );
        
      PDOsetControlWord( SWITCH_ON, true );
      PDOsetControlWord( ENABLE_VOLTAGE, true );
      PDOsetControlWord( QUICK_STOP, true );
      PDOsetControlWord( ENABLE_OPERATION, false );
      WritePDO01();

      delay( 500 );
        
      PDOsetControlWord( SWITCH_ON, true );
      PDOsetControlWord( ENABLE_VOLTAGE, true );
      PDOsetControlWord( QUICK_STOP, true );
      PDOsetControlWord( ENABLE_OPERATION, true );
      WritePDO01();

      active = true;
    }

    void Disable()
    {          
      PDOsetControlWord( SWITCH_ON, true );
      PDOsetControlWord( ENABLE_VOLTAGE, true );
      PDOsetControlWord( QUICK_STOP, true );
      PDOsetControlWord( ENABLE_OPERATION, false );
      WritePDO01();
        
      delay( 500 );
        
      PDOsetControlWord( SWITCH_ON, false );
      PDOsetControlWord( ENABLE_VOLTAGE, true );
      PDOsetControlWord( QUICK_STOP, true );
      PDOsetControlWord( ENABLE_OPERATION, false );
      WritePDO01();

      active = false;
    }

    double PDOgetValue( Dimension index )
    {
      if( index >= N_DIMS )
      {
        fprintf( stderr, "PDOgetValue: invalid value index: %d\n", index );
        return 0;
      }
      
      return dimensionValue[ index ];
    }

    void PDOsetValue( Dimension index, double value )
    {
      if( index >= N_DIMS )
      {
        fprintf( stderr, "PDOsetValue: invalid value index: %d\n", index );
        return;
      }
      
      dimensionValue[ index ] = value;
	
      return;
    }
      
    double GetControlValue( Parameter index )
    {
      if( index >= N_PARAMS )
      {
        fprintf( stderr, "GetParameterValue: invalid value index: %d\n", index );
        return 0;
      }
      
      return controlValue[ index ];
    }

    void SetControlValue( Parameter index, double value )
    {
      if( index >= N_PARAMS )
      {
        fprintf( stderr, "SetParameterValue: invalid value index: %d\n", index );
        return;
      }
      
      controlValue[ index ] = value;
	
      return;
    }

    void PDOsetOutput( int state )
    {
      if (state > 0) output = 0x2000;
      else output = 0x0000;
    }
	    
    bool PDOgetStatusWord( StatusWord index )
    {
      if( (statusWord & index) > 0 ) return true;
      else return false;
    }

    void PDOsetControlWord( ControlWord index, bool state )
    {
      if( state ) controlWord = controlWord | index;
      else controlWord = controlWord & (~index);
    }


    Axisx(){}

    //Construtor da classe
    Axisx( string selectedDatabase, string selectedCluster, string netAddress ) 
    {
      //Inicializa variaveis
      for( int i = 0; i < N_DIMS; i++ )
        dimensionValue[ i ] = 0.0;
      for( int i = 0; i < N_PARAMS; i++ )
        controlValue[ i ] = 10.0;

      //CRIA E INICIALIZA OS CAN FRAMES PARA O EIXO
	
      string temp;
	
      temp = "SDO_TX_0";
      temp += netAddress;
      SDO_TX.Init( nxMode_FrameOutSinglePoint, "CAN2", selectedDatabase, selectedCluster, temp ); //11
		    
      temp = "SDO_RX_0";
      temp += netAddress;
      SDO_RX.Init( nxMode_FrameInSinglePoint, "CAN1", selectedDatabase, selectedCluster, temp ); //8

      temp = "PDO01_TX_0";
      temp += netAddress;
      PDO01_TX.Init( nxMode_FrameOutSinglePoint, "CAN2", selectedDatabase, selectedCluster, temp ); //11
	    
      temp = "PDO01_RX_0";
      temp += netAddress;
      PDO01_RX.Init( nxMode_FrameInSinglePoint, "CAN1", selectedDatabase, selectedCluster, temp ); //8
		    
      temp = "PDO02_TX_0";
      temp += netAddress;
      PDO02_TX.Init( nxMode_FrameOutSinglePoint, "CAN2", selectedDatabase, selectedCluster, temp ); //11

      temp = "PDO02_RX_0";
      temp += netAddress;
      PDO02_RX.Init( nxMode_FrameInSinglePoint, "CAN1", selectedDatabase, selectedCluster, temp ); //8

      Disable();
    }
    
    virtual ~Axis()
    {
      Disable();
      setpoints.clear();
    }
	    
	void ReadSetpoints( string fileName )
	{
      fstream file;
      file.open( fileName.c_str() );

      setpoints.clear();

      double setpoint;
      while( file >> setpoint )
        setpoints.push_back( setpoint );

      //setpoints.push_back( 0.0 );

      file.close();
	}

	double GetSetpoint()
	{
      static int index;

      return setpoints[ (index++) % (int) setpoints.size() ];
	}

    //Realiza a leitura do PDO01 - Posição e Corrente
    void ReadPDO01()
    {    
      static u8 readPayload[8];
      PDO01_RX.Read( readPayload );
	    
      //ATUALIZA O VALOR DE POSIÇÃO
      dimensionValue[ POSITION ] = readPayload[3] * 0x1000000 + readPayload[2] * 0x10000 + readPayload[1] * 0x100 + readPayload[0];
	    
      //ATUALIZA O VALOR DE CORRENTE
      dimensionValue[ CURRENT ] = readPayload[5] * 0x100 + readPayload[4];

      //ATUALIZA A STATUS WORD
      statusWord = readPayload[7] * 0x100 + readPayload[6];
    }

    //Realiza a leitura do PDO02 - Velocidade
    void ReadPDO02()
    {
      static u8 readPayload[8];
      PDO02_RX.Read( readPayload );
	    
      //ATUALIZA O VALOR DE POSIÇÃO
      dimensionValue[ VELOCITY ] = readPayload[3] * 0x1000000 + readPayload[2] * 0x10000 + readPayload[1] * 0x100 + readPayload[0];
	
      //ATUALIZA O VALOR DA ANNALÓGICA 01
      dimensionValue[ TENSION ] = readPayload[5] * 0x100 + readPayload[4];
    }

    //Método de escrita do PDO01 - Setpoint de posição e corrente
    void WritePDO01()
    {	    
      //MONTA O PAYLOAD DE WRITE
      static u8 writePayload[8];
      writePayload[0] = ( (int) dimensionValue[ POSITION_SETPOINT ] & 0x000000ff );
      writePayload[1] = ( (int) dimensionValue[ POSITION_SETPOINT ] & 0x0000ff00 ) / 0x100;
      writePayload[2] = ( (int) dimensionValue[ POSITION_SETPOINT ] & 0x00ff0000 ) / 0x10000;
      writePayload[3] = ( (int) dimensionValue[ POSITION_SETPOINT ] & 0xff000000 ) / 0x1000000;
      writePayload[4] = ( 0 & 0x000000ff );
      writePayload[5] = ( 0 & 0x0000ff00 ) / 0x100; 
      writePayload[6] = ( controlWord & 0x000000ff );
      writePayload[7] = ( controlWord & 0x0000ff00 ) / 0x100; 

      //ENVIA COMANDO DE START PDOS PARA A REDE
      PDO01_TX.Write( writePayload );
    }

    //Método de escrita do PDO02 - Setpoint de velocidade
    void WritePDO02()
    {	    
      //MONTA O PAYLOAD DE WRITE
      static u8 writePayload[8];
      writePayload[0] = ( (int) dimensionValue[ VELOCITY_SETPOINT ] & 0x000000ff );
      writePayload[1] = ( (int) dimensionValue[ VELOCITY_SETPOINT ] & 0x0000ff00 ) / 0x100;
      writePayload[2] = ( (int) dimensionValue[ VELOCITY_SETPOINT ] & 0x00ff0000 ) / 0x10000;
      writePayload[3] = ( (int) dimensionValue[ VELOCITY_SETPOINT ] & 0xff000000 ) / 0x1000000;
      writePayload[4] = ( output & 0x000000ff );
      writePayload[5] = ( output & 0x0000ff00 ) / 0x100; 
      writePayload[6] = ( 0 & 0x000000ff );
      writePayload[7] = ( 0 & 0x0000ff00 ) / 0x100; 

      //ENVIA COMANDO DE START PDOS PARA A REDE
      PDO02_TX.Write( writePayload );
    }

    //MÉTODO DE REQUISIÇÂO DE LEITURA DE SDO
    void RequireReadSDO( int index, int subIndex )
    {
      //MONTA O PAYLOAD DE WRITE requisição de leitura
      static u8 writePayload[8];
      writePayload[0] = 0x40; 
      writePayload[1] = ( index & 0x000000ff );
      writePayload[2] = ( index & 0x0000ff00 ) / 0xff;
      writePayload[3] = subIndex;
      writePayload[4] = 0;
      writePayload[5] = 0;
      writePayload[6] = 0;
      writePayload[7] = 0;

      //ENVIA COMANDO DE START PDOS PARA A REDE
      SDO_TX.Write( writePayload );	
    }

    //MÉTODO DE LEITURA DE SDO
    int ReadSDO()
    {
      //MONTA O PAYLOAD DE WRITE requisição de leitura
      static u8 readPayload[8];
	
      SDO_RX.Read( readPayload );

      int result = readPayload[7] * 0x1000000 + readPayload[6] * 0x10000 + readPayload[5] * 0x100 + readPayload[4];

      return result; 
    }

    //MÉTODO DE ESCRITA DE UM SDO
    void WriteSDO( int index, u8 subIndex, short int value )
    {
	  //MONTA O PAYLOAD DE WRITE
      static u8 writePayload[8];
      writePayload[0] = 0x22; 
      writePayload[1] = ( index & 0x000000ff );
      writePayload[2] = ( index & 0x0000ff00 ) / 0x100;
      writePayload[3] = subIndex;
      writePayload[4] = ( value & 0x000000ff );
      writePayload[5] = ( value & 0x0000ff00 ) / 0x100;
      writePayload[6] = ( value & 0x00ff0000 ) / 0x10000;
      writePayload[7] = ( value & 0xff000000 ) / 0x1000000;

      //ENVIA COMANDO DE START PDOS PARA A REDE
      SDO_TX.Write( writePayload );		
    }


    void VCS_SetOperationMode( OperationMode mode )
    {	
      //Escreve o modo de operação desejado
      WriteSDO( 0x6060, 0x00, mode );
      return;  
    }


    void SetDigitalOutput()
    {	
      //Escreve o modo de operação desejado
      WriteSDO( 0x6060, 0x00, 0xff );
      return;  
    }

    void ResetDigitalOutput()
    {	
      //Escreve o modo de operação desejado
      WriteSDO( 0x6060, 0x00, 0x00 );
      return;  
    }

};


#endif	/* AXIS_H */

