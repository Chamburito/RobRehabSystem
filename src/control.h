#ifndef CONTROL_H
#define CONTROL_H

#include "axis.h"
#include "epos_network.h"

#ifdef WIN32
  #include "threads_windows.h"
#else
  #include "threads_unix.h"
#endif

#include <cmath>
#include <iostream>

using namespace std;
  
enum Joint { HIPS = 2, KNEE = 1, ANKLE = 0 };  // Índices dos algoritmos de controle

/////////////////////////////////////////////////////////////////////////////////
/////                       DISPOSITIVOS DE CONTROLE                        /////
/////////////////////////////////////////////////////////////////////////////////

//ENDEREÇAMENTO DA BASE DE DADOS CAN
const char* CAN_DATABASE = "database";
const char* CAN_CLUSTER = "NETCAN";
    
const char* NET_ID_SERVO_01 = "1";
const char* NET_ID_SERVO_02 = "2";
const char* NET_ID_SERVO_03 = "3";

//INICIALIZAÇÃO DA REDE CAN
EPOSNetwork eposNetwork( CAN_DATABASE, CAN_CLUSTER );

//INICIALIZAÇÃO DAS EPOS
const int N_EPOS = 3;
Axis EPOS[ N_EPOS ] = { Axis( CAN_DATABASE, CAN_CLUSTER, NET_ID_SERVO_01 ),
                      Axis( CAN_DATABASE, CAN_CLUSTER, NET_ID_SERVO_02 ),
                      Axis( CAN_DATABASE, CAN_CLUSTER, NET_ID_SERVO_03 ) };


/////////////////////////////////////////////////////////////////////////////////
////////// CONSTANTES GERAIS ////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

const double PI = 3.141592;
const double Ts = 0.005;				  // tempo de amostragem


/////////////////////////////////////////////////////////////////////////////////
////////// CONSTANTES QUADRIL ///////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

const double p1 = 0.0168;
const double p2 = 2.5749;

const double Ks_1 = 78.9; // N/m
const double passo_AES1 = 3e-3;

const double a_q = 0.063, b_q = 0.060, c_q = 0.284, d_q = 0.117, l_zero_q = 0.348;

const double h12_q = c_q * c_q + d_q * d_q;  //onde h12_q=h1_q^2
const double h22_q = a_q * a_q + b_q * b_q;  //onde h22_q=h2_q^2
const double h1_q = sqrt( h12_q );
const double h2_q = sqrt( h22_q );

const double alpha_q = atan( a_q / b_q );  //ângulo entre a_quadril e h2
const double beta_q = atan( d_q / c_q );  //ângulo entre c_quadril e h1

/////////////////////////////////////////////////////////////////////////////////
////////// GANHOS PADRÂO QUADRIL ////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

double Kq = 10.0;
double Bq = 10.0;

/////////////////////////////////////////////////////////////////////////////////
////////// FUNÇÂO QUADRIL ///////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void* hipsControl( void* args )
{
  Axis* epos = (Axis*) args;

  double refTension = epos->PDOgetValue( TENSION ); // mV
  double refPosition = ( p1 * refTension + p2 ); // mm

  // Amostragem dos últimos valores do sinal de tensão para uso de valor médio
  static double analogSamples[6];
  
  int execTime, elapsedTime;

  epos->VCS_SetOperationMode( VELOCITY_MODE );
  
  //epos->SetControlValue( KP, Kq );
  //epos->SetControlValue( KD, Bq );

  static double thetad_q; // ?

  //epos->ReadSetpoints( "theta_q.txt" );

  while( epos->active/*epos->PDOgetStatusWord( SWITCHED_ON )*/ )
  {
    execTime = get_exec_time_milisseconds();
    
    analogSamples[5] = analogSamples[4];
    analogSamples[4] = analogSamples[3];
    analogSamples[3] = analogSamples[2];
    analogSamples[2] = analogSamples[1];
    analogSamples[1] = analogSamples[0];
    analogSamples[0] = epos->PDOgetValue( TENSION ); // mV
  
    double tension = ( analogSamples[0] + analogSamples[1] + analogSamples[2] + analogSamples[3]+ analogSamples[4] + analogSamples[5] ) / 6; // mV

    double position = ( p1 * tension + p2 );	// mm

    double Fl = -Ks_1 * ( refPosition - position ); // N

    //thetad_q = epos->GetSetpoint();
    thetad_q = epos->PDOgetValue( POSITION_SETPOINT );
    cout << "Angle setpoint: " << thetad_q << endl;

    //-------- Mecanismo de 4 barras do quadril [metros] --------

    int encoder_AES1 = epos->PDOgetValue( POSITION );
    double l_efetuador = ( encoder_AES1 / 4096 ) * passo_AES1 + ( refPosition - position ) / 1000; // m

    double l_real_q = l_zero_q + l_efetuador; // m

    double cos_eta_q = ( l_real_q * l_real_q + h2_q * h2_q - h1_q * h1_q ) / ( 2 * l_real_q * h2_q );
    double sin_eta_q = sqrt( (double)( 1 - cos_eta_q * cos_eta_q ) );
    
    double gama_q = acos( ( h1_q * h1_q + h2_q * h2_q - l_real_q * l_real_q ) / ( 2 * h1_q * h2_q ) );

    double theta_q = alpha_q + beta_q + gama_q - PI;

    double dthetadl = sqrt( h12_q + h22_q - 2 * h1_q * h2_q * cos(gama_q) ) / ( h1_q * h2_q * sin(gama_q) );

    Kq = epos->GetControlValue( KP );
    Bq = epos->GetControlValue( KD );

    if( (int)Kq == 0 ) Kq = 1.0;
    if( (int)Bq == 0 ) Bq = 1.0;
  
    double K_AES1 = Kq * dthetadl * ( 1 / ( h2_q * sin_eta_q ) );
    double B_AES1 = Bq * dthetadl * ( 1 / ( h2_q * sin_eta_q ) );

    double Fv_q = ( Kq * thetad_q ) / ( h2_q * sin_eta_q );

    //-------- Lei de Controle --------

    double Vel_d = ( Fv_q - K_AES1 * ( ( encoder_AES1 / 4096 ) * passo_AES1 ) + ( ( K_AES1 - Ks_1 ) / Ks_1 ) * Fl ) / B_AES1;

    int Vel_d_rpm = ( Vel_d / passo_AES1 ) * 60 / ( 2 * PI ); // rpm
    
    //cout << "angle: " << theta_q << "  vel ref: " << Vel_d_rpm << "  force: " << Fl << endl;  

    epos->PDOsetValue( ANGLE, 180 * theta_q / PI );
    epos->PDOsetValue( ANGLE_SETPOINT, 180 * thetad_q / PI );
    epos->PDOsetValue( VELOCITY_SETPOINT, Vel_d_rpm );
    epos->PDOsetValue( FORCE, Fl );
    epos->WritePDO02();
    
    elapsedTime = get_exec_time_milisseconds() - execTime;
    cout << "elapsed time: " << elapsedTime << endl;
    if( elapsedTime < (int) ( 1000 * Ts ) ) delay( 1000 * Ts - elapsedTime );
    cout << "elapsed time: " << get_exec_time_milisseconds() - execTime << endl;
    cout << endl;
  }

  epos->PDOsetValue( VELOCITY_SETPOINT, 0 );
  epos->WritePDO02();

  exit_thread( 0 );
  return NULL;
}


/////////////////////////////////////////////////////////////////////////////////
////////// CONSTANTES JOELHO ////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

const int N = 150;						            // Redução da junta
const int ENCODER_IN_RES = 4096;			    // Resolução do encoder do motor
const int ENCODER_OUT_RES = 2000;		      // Resolução do encoder de saída
const int Ks = 104;						            // Constante elástica

const double a2 = -0.9428;
const double a3 = 0.3333;
const double b1 = 0.0976;
const double b2 = 0.1953;
const double b3 = 0.0976;

const double c2 = -1.889;
const double c3 = 0.8949;
const double d1 = 0.0015;
const double d2 = 0.0029;
const double d3 = 0.0015;

const double kp = 370;				    // Ganho proporcional
const double ki = 3.5;					// Ganho integrativo
const double kd = 0.0;					// Ganho derivativo

/////////////////////////////////////////////////////////////////////////////////
////////// GANHOS JOELHO ////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

double Kv = 0.0;
double Bv = 5.0;

/////////////////////////////////////////////////////////////////////////////////
////////// FUNÇÃO JOELHO ////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void* kneeControl( void* args )
{
  Axis* epos = (Axis*) args;
  
  int execTime, elapsedTime;

  epos->VCS_SetOperationMode( VELOCITY_MODE );

  //epos->SetControlValue( KP, Kv );
  //epos->SetControlValue( KD, Bv );

  static double omega_l[3];
  static double omega_lf[3];
  static double theta_l[2];
  static double torque_l[3];
  static double torque_lf[3];
  static double control[2];
  static double error[3];

  static double theta_ld, omega_ld; // ?

  //epos->ReadSetpoints( "theta_j.txt" );

  while( epos->active/*epos->PDOgetStatusWord( SWITCHED_ON )*/ )
  {
    execTime = get_exec_time_milisseconds();

    double Im = epos->PDOgetValue( CURRENT );
    //double theta_c = ( ( epos->PDOgetValue( POSITION ) - epos->PDOgetValue( POSITION_SETPOINT ) ) * 2 * PI ) / ( ENCODER_IN_RES * N );
    //double theta_m = ( ( epos->PDOgetValue( POSITION ) - epos->PDOgetValue( POSITION_SETPOINT ) ) * 2 * PI ) / ENCODER_IN_RES;
    double theta_c = ( ( epos->PDOgetValue( POSITION ) ) * 2 * PI ) / ( ENCODER_IN_RES * N );
    double theta_m = ( ( epos->PDOgetValue( POSITION ) ) * 2 * PI ) / ENCODER_IN_RES;
        
    double omega_m = epos->PDOgetValue( VELOCITY );

    //theta_ld = epos->GetSetpoint();
    //theta_ld = epos->PDOgetValue( POSITION_SETPOINT );
    //omega_ld = epos->PDOgetValue( VELOCITY_SETPOINT );

    //---------------Controle de Impedância--------------------//
      
    Kv = epos->GetControlValue( KP );
    Bv = epos->GetControlValue( KD );

    cout << "Kv: " << Kv << " - Bv: " << Bv << endl;
	  
    omega_l[0] = ( theta_l[0] - theta_l[1] ) / Ts;
    omega_lf[0] = -c2 * omega_lf[1] - c3 * omega_lf[2] + d1 * omega_l[0] + d2 * omega_l[1] + d3 * omega_l[2];

    //theta_l[0] = ( ( -EPOS[0].PDOgetValue( POSITION ) - EPOS[0].PDOgetValue( POSITION_SETPOINT ) ) * 2 * PI ) / ENCODER_OUT_RES;
    theta_l[0] = ( ( -EPOS[0].PDOgetValue( POSITION ) ) * 2 * PI ) / ENCODER_OUT_RES;

    double torque_d = -Kv * ( theta_l[0] - theta_ld ) - Bv * ( omega_lf[0] - omega_ld );

    epos->PDOsetValue( ANGLE, (int) ( 1000 * theta_l[0] ) );
    //cout << "epos 2: theta_l: " << theta_l[0] << endl;

    torque_l[0] = Ks * ( theta_c - theta_l[0] );
    epos->PDOsetValue( FORCE, (int) torque_l[0] );

    torque_lf[0] = -a2 * torque_lf[1] - a3 * torque_lf[2] + b1 * torque_l[0] + b2 * torque_l[1] + b3 * torque_l[2];

    error[0] = ( torque_d - torque_lf[0] );

    control[0] = control[1] + kp * ( error[0] - error[1] ) + ki * Ts * error[0] + ( kd / Ts ) * ( error[0] - 2 * error[1] + error[2] );
    //controle = (0.9872*controle2_ant + 0.009265*controle2_ant2 + 333.3*erro_ant - 332*erro_ant2);

    error[2] = error[1];
    error[1] = error[0];

    theta_l[1] = theta_l[0];

    control[1] = control[0];

    omega_l[2] = omega_l[1];
    omega_l[1] = omega_l[0];

    omega_lf[2] = omega_lf[1];
    omega_lf[1] = omega_lf[0];

    torque_l[2] = torque_l[1];
    torque_l[1] = torque_l[0];

    torque_lf[2] = torque_lf[1];
    torque_lf[1] = torque_lf[0];

    epos->PDOsetValue( ANGLE, 180 * theta_l[0] / PI );
    epos->PDOsetValue( ANGLE_SETPOINT, 180 * theta_ld / PI );
    epos->PDOsetValue( VELOCITY_SETPOINT, (int) control[0] );
    epos->WritePDO02();

    elapsedTime = get_exec_time_milisseconds() - execTime;
    cout << "controle: " << control[0] << endl;
    cout << "elapsed time: " << elapsedTime << endl;
    //delay( 5 - elapsedTime );
    if( elapsedTime < (int) ( 1000 * Ts ) ) delay( 1000 * Ts - elapsedTime );
    cout << "elapsed time + sleep: " << get_exec_time_milisseconds() - execTime << endl;
    cout << endl;
  }
  
  epos->PDOsetValue( VELOCITY_SETPOINT, 0 );
  epos->WritePDO02();

  exit_thread( 0 );
  return NULL;
}

void* ankleControl( void* args )
{
  Axis* epos = (Axis*) args;
  
  int execTime, elapsedTime;

  int velocitySetpoint;

  while( epos->active/*epos->PDOgetStatusWord( SWITCHED_ON )*/ )
  {
     execTime = get_exec_time_milisseconds();

	 epos->VCS_SetOperationMode( VELOCITY_MODE );

	 if( epos->PDOgetValue( POSITION_SETPOINT ) > 2000 ) epos->PDOsetValue( POSITION_SETPOINT, 2000 );
	 else if( epos->PDOgetValue( POSITION_SETPOINT ) < -2000 ) epos->PDOsetValue( POSITION_SETPOINT, -2000 );

	 velocitySetpoint = -( epos->PDOgetValue( POSITION ) - epos->PDOgetValue( POSITION_SETPOINT ) ) / 20;

	 epos->PDOsetValue( VELOCITY_SETPOINT, velocitySetpoint );
     epos->WritePDO02();

	 elapsedTime = get_exec_time_milisseconds() - execTime;
     //cout << "ankleControl: elapsed time: " << elapsedTime << endl;
     if( elapsedTime < (int) ( 1000 * Ts ) ) delay( 1000 * Ts - elapsedTime );
     //cout << "ankleControl: elapsed time + sleep: " << get_exec_time_milisseconds() - execTime << endl;
     //cout << endl;
  }

  epos->PDOsetValue( VELOCITY_SETPOINT, 0 );
  epos->WritePDO02();

  exit_thread( 0 );
  return NULL;
}


typedef void* (*async_function)( void* ); // Use "async_function" type as "void (*)( void* )" type

async_function functions[] = { ankleControl, kneeControl, hipsControl };

async_function controlFunction( Joint index )
{
  if( index >= 0 && index < 3 )
    return functions[ index ];
  else
    return NULL;
}

#endif /* CONTROL_H */ 
