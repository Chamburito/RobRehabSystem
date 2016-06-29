#include "robot_control/interface.h"

#include <math.h>

#include "matrices.h"
#include "debug/data_logging.h"

#define DOFS_NUMBER 1

const char* DOF_NAMES[ DOFS_NUMBER ] = { "angle" };

typedef struct _ControlData
{
  double positionsMaxList[ DOFS_NUMBER ];
  Matrix statesProbability;
  Matrix Aux_rec, Aux;
  Matrix W, Z, V, U;
  Matrix K_rec, L_rec, P_rec;
  double proportionalGain, derivativeGain;
  int logID;
}
ControlData;

DECLARE_MODULE_INTERFACE( ROBOT_CONTROL_INTERFACE ) 


Controller InitController( const char* data )
{
  ControlData* newController = (ControlData*) malloc( sizeof(ControlData) );
  
  newController->positionsMaxList[ 0 ] = 0.0;
  
  double stateProbabilityData[ 3 ][ 3 ] = { { 0.9317, 0.0595, 0.0088 }, { 0.0175, 0.9708, 0.0117 }, { 0.0088, 0.04, 0.9512 } };
  newController->statesProbability = Matrices.Create( (double*) stateProbabilityData, 3, 3 );
  
  newController->Aux_rec = Matrices.CreateSquare( 2, MATRIX_ZERO );
  
  double W_data[ 12 ][ 12 ] = { { 1.0,   0,   0,   0,    0,    0,      0,      0, 1.0,   0,   0,    0 }, 
                                {   0, 1.0,   0,   0,    0,    0,      0,      0,   0, 1.0,   0,    0 },
                                {   0,   0, 0.1,   0,    0,    0,      0,      0,   0,   0, 1.0,    0 },
                                {   0,   0,   0, 0.1,    0,    0,      0,      0,   0,   0,   0,  1.0 },
                                {   0,   0,   0,   0, 10.0,    0,      0,      0,   0,   0,   0,    0 },
                                {   0,   0,   0,   0,    0, 10.0,      0,      0,   0,   0,   0,    0 },
                                {   0,   0,   0,   0,    0,    0, 10e-10,      0, 1.0,   0,   0,    0 },
                                {   0,   0,   0,   0,    0,    0,      0, 10e-10,   0, 1.0,   0, -1.0 },
                                { 1.0,   0,   0,   0,    0,    0,    1.0,      0,   0,   0,   0,    0 },
                                {   0, 1.0,   0,   0,    0,    0,      0,    1.0,   0,   0,   0,    0 },
                                {   0,   0, 1.0,   0,    0,    0,      0,      0,   0,   0,   0,    0 }, 
                                {   0,   0,   0, 1.0,    0,    0,      0,   -1.0,   0,   0,   0,    0 } };
  newController->W = Matrices.Create( (double*) W_data, 12, 12 );
  
  Matrices.Print( newController->W );
  
  newController->Z = Matrices.Create( NULL, 12, 2 );
  Matrices.SetElement( newController->Z, 8, 0, 1 );
  Matrices.SetElement( newController->Z, 9, 1, 1 );
  newController->V = Matrices.Create( NULL, 12, 2 );
  Matrices.SetElement( newController->V, 10, 0, 1 );
  Matrices.SetElement( newController->V, 11, 1, 1 );
  newController->U = Matrices.Create( NULL, 12, 2 );
  Matrices.SetElement( newController->U, 4, 0, -1 );
  Matrices.SetElement( newController->U, 5, 1, -1 );
  Matrices.SetElement( newController->U, 6, 1, 1 );
  Matrices.SetElement( newController->U, 7, 1, 1 );
  
  newController->K_rec = Matrices.CreateSquare( 2, MATRIX_ZERO );
  newController->L_rec = Matrices.CreateSquare( 2, MATRIX_ZERO );
  newController->P_rec = Matrices.CreateSquare( 2, MATRIX_ZERO );
  
  newController->Aux = Matrices.Create( NULL, 12, 2 );
  
  newController->logID = DataLogging.InitLog( "test", 1, 1000 );
  DataLogging.SetDataPrecision( newController->logID, 0 );
  
  return (Controller) newController;
}

void EndController( Controller ref_controller )
{
  if( ref_controller == NULL ) return;
  
  ControlData* controller = (ControlData*) ref_controller;
  
  Matrices.Discard( controller->statesProbability );
  
  Matrices.Discard( controller->Aux_rec );
  Matrices.Discard( controller->Aux );
  Matrices.Discard( controller->W );
  Matrices.Discard( controller->Z );
  Matrices.Discard( controller->V );
  Matrices.Discard( controller->U );
  Matrices.Discard( controller->K_rec );
  Matrices.Discard( controller->L_rec );
  Matrices.Discard( controller->P_rec );
  
  DataLogging.EndLog( controller->logID );
  
  free( controller );
}

size_t GetJointsNumber( Controller ref_controller )
{
  return DOFS_NUMBER;
}

char** GetJointNamesList( Controller ref_controller )
{
  return (char**) DOF_NAMES;
}

size_t GetAxesNumber( Controller ref_controller )
{
  return DOFS_NUMBER;
}

char** GetAxisNamesList( Controller ref_controller )
{
  return (char**) DOF_NAMES;
}

void SetControlState( Controller ref_controller, enum ControlState controlState )
{
  fprintf( stderr, "Setting robot control state: %x\n", controlState );
}

void RunControlStep( Controller ref_controller, double** jointMeasuresTable, double** axisMeasuresTable, double** jointSetpointsTable, double** axisSetpointsTable )
{
  if( ref_controller == NULL ) return;
  
  ControlData* controller = (ControlData*) ref_controller;
  
  if( fabs( jointMeasuresTable[ 0 ][ CONTROL_POSITION ] ) > controller->positionsMaxList[ 0 ] ) 
    controller->positionsMaxList[ 0 ] = fabs( jointMeasuresTable[ 0 ][ CONTROL_POSITION ] );
  
  if( fabs( controller->positionsMaxList[ 0 ] ) < 0.0001 ) controller->positionsMaxList[ 0 ] = 0.0001;
  
  double jointPosition = jointMeasuresTable[ 0 ][ CONTROL_POSITION ] / controller->positionsMaxList[ 0 ];
  size_t jointSector = 0;
  double jointSectorLength = 2.0 / 3;
  
  if( jointPosition <= -1.0 + jointSectorLength ) jointSector = 0;
  else if( jointPosition > -1.0 + jointSectorLength && jointPosition < 1.0 - jointSectorLength ) jointSector = 1;
  else if( jointPosition >= 1.0 - jointSectorLength ) jointSector = 2;
  
  fprintf( stderr, "pos: %.5f, max: %.5f, norm: %.5f, sector: %lu\r", jointMeasuresTable[ 0 ][ CONTROL_POSITION ], controller->positionsMaxList[ 0 ], jointPosition, jointSector );
  //DataLogging.RegisterValues( controller->logID, 1, jointSector );
  
  double positionError = jointSetpointsTable[ 0 ][ CONTROL_POSITION ] - jointMeasuresTable[ 0 ][ CONTROL_POSITION ];
  double velocityError = jointSetpointsTable[ 0 ][ CONTROL_VELOCITY ] - jointMeasuresTable[ 0 ][ CONTROL_VELOCITY ];
  
  if( fabs( positionError ) < 0.01 ) positionError = 0.01; 
  
  double Dr = 1.7863 * positionError;
  double Cr = 0.1981 * velocityError;
  
  Matrices.SetElement( controller->W, 0, 0, Matrices.GetElement( controller->Aux_rec, 0, 0 ) );
  Matrices.SetElement( controller->W, 1, 1, Matrices.GetElement( controller->Aux_rec, 1, 1 ) );
  Matrices.SetElement( controller->W, 7, 11, -1.0/Dr );
  Matrices.SetElement( controller->W, 11, 7, -1.0/Dr );
  
  Matrices.SetElement( controller->U, 7, 1, Cr/Dr );
  
  // Aux = inv(W) * U
  Matrices.Inverse( controller->W, controller->Aux );
  Matrices.Dot( controller->Aux, MATRIX_KEEP, controller->U, MATRIX_KEEP, controller->Aux );
  
  Matrices.Dot( controller->V, MATRIX_TRANSPOSE, controller->Aux, MATRIX_KEEP, controller->K_rec ); // K_rec(:,:) = V' * Aux
  Matrices.Dot( controller->Z, MATRIX_TRANSPOSE, controller->Aux, MATRIX_KEEP, controller->L_rec ); // L_rec(:,:) = Z' * Aux
  Matrices.Dot( controller->U, MATRIX_TRANSPOSE, controller->Aux, MATRIX_KEEP, controller->P_rec ); // P_rec(:,:) = U' * Aux
  
  double probability = Matrices.GetElement( controller->statesProbability, jointSector, jointSector );
  
  Matrices.Sum( controller->Aux_rec, 1.0, controller->P_rec, probability, controller->Aux_rec ); // AUX_i+1 = AUX_i + p_ij * P_rec(:,:);
  
  double proportionalGain = Matrices.GetElement( controller->K_rec, 1, 1 );
  
  double result = 1.7863 * ( jointMeasuresTable[ 0 ][ CONTROL_ACCELERATION ] + proportionalGain * positionError ) + 0.1981 * jointMeasuresTable[ 0 ][ CONTROL_VELOCITY ] + 0.1111;
  
  jointSetpointsTable[ 0 ][ CONTROL_FORCE ] = result;
  
  if( jointSetpointsTable[ 0 ][ CONTROL_STIFFNESS ] == 0.0 ) jointSetpointsTable[ 0 ][ CONTROL_FORCE ] = 0.0;
}
