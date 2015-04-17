#ifndef EMG_JOINT_CONTROL_H
#define EMG_JOINT_CONTROL_H

#include "async_debug.h"

#include "emg_processing.h"

typedef struct _EMGJointControl
{
  unsigned int jointID;
  MuscleProperties* agonistMusclesList;
  size_t agonistMusclesNumber;
  MuscleProperties* antagonistMusclesList;
  size_t antagonistMusclesNumber;
}
EMGJointControl;

static EMGJointControl* emgJointControlsList = NULL;

const double coeffs_forceActiveRF[ ] = { 344.3169, -6.4302, 0.1123, 0.0016, 0.0 };
const double coeffs_forcePassiveRF[ ] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
const double coeffs_momentArmRF[ ] = { 0.0479, -0.0006, 0.0, 0.0, 0.0 };
const double coeffs_normLengthRF[ ] = { 0.2200, -0.0061, 0.0, 0.0, 0.0 };

const double coeffs_forceActiveBF[ ] = { 503.2041, -3.6544, 0.0714, 0.0011, 0.0 };
const double coeffs_forcePassiveBF[ ] = { 572.0215, 14.477, 0.1017, -0.0001, 0.0};
const double coeffs_momentArmBF[ ] = { -0.0275, 0.0005, 0.0, 0.0, 0.0 };
const double coeffs_normLengthBF[ ] = { 1.5362, 0.0034, 0.0, 0.0, 0.0 };

static double forceActiveRF, forcePassiveRF, momentArmRF, normLengthRF;
static double forceActiveBF, forcePassiveBF, momentArmBF, normLengthBF;

const double phiZeroRF = 0.08726646;
const double phiZeroBF = 0.0;

static double phiRF;
static double phiBF;

const double alphaRF = 1.0;
const double alphaBF = 1.0;

static double forceRF, torqueRF, forceBF, torqueBF, torqueMuscle, stiffnessMuscle;

double EMGJointControl_GetTorque( double activationMeasure_1, double activationMeasure_2, double jointAngle )
{
  //Rectus Femoris 
  forceActiveRF = 0.0;
  for (int i=0; i < COEFFS_NUMBER; i++)
    forceActiveRF += coeffs_forceActiveRF[i] * pow( jointAngle, i );
  
  forcePassiveRF = 0.0;
  for (int i=0; i < COEFFS_NUMBER; i++)
    forcePassiveRF += coeffs_forcePassiveRF[i] * pow( jointAngle, i );
  
  momentArmRF = 0.0;
  for (int i=0; i < COEFFS_NUMBER; i++)
    momentArmRF += coeffs_momentArmRF[i] * pow( jointAngle, i );
  
  normLengthRF = 0.0;
  for (int i=0; i < COEFFS_NUMBER; i++)
    normLengthRF += coeffs_normLengthRF[i] * pow( jointAngle, i );
  
  phiRF = asin(sin(phiZeroRF)/normLengthRF);
  
  forceRF = alphaRF * ( forceActiveRF * activationMeasure_1 + forcePassiveRF ) * cos( phiRF );
  torqueRF = forceRF*momentArmRF;
  
  
  //Biceps Femoris
  forceActiveBF = 0.0;
  for (int i=0; i < COEFFS_NUMBER; i++)
    forceActiveBF += coeffs_forceActiveBF[i] * pow( jointAngle, i );
  
  forcePassiveBF = 0.0;
  for (int i=0; i < COEFFS_NUMBER; i++)
    forcePassiveBF += coeffs_forcePassiveBF[i] * pow( jointAngle, i );
  
  momentArmBF = 0.0;
  for (int i=0; i < COEFFS_NUMBER; i++)
    momentArmBF += coeffs_momentArmBF[i] * pow( jointAngle, i );
  
  normLengthBF = 0.0;
  for (int i=0; i < COEFFS_NUMBER; i++)
    normLengthBF += coeffs_normLengthBF[i] * pow( jointAngle, i );
  
  phiBF = asin(sin(phiZeroBF)/normLengthBF); 
  
  forceBF = alphaBF * ( forceActiveBF * activationMeasure_2 + forcePassiveBF ) * cos( phiBF );
  torqueBF = forceBF * momentArmBF;
  
  
  //Torque Total
  torqueMuscle = torqueRF - torqueBF;
  
  return torqueMuscle;
}

double EMGJointControl_GetStiffness( double activationMeasure_1, double activationMeasure_2, double jointAngle )
{
  const double beta = 0.03;
  
  //Rectus Femoris 
  forceActiveRF = 0.0;
  for (int i=0; i < COEFFS_NUMBER; i++)
    forceActiveRF += coeffs_forceActiveRF[i] * pow( jointAngle, i );
  
  forcePassiveRF = 0.0;
  for (int i=0; i < COEFFS_NUMBER; i++)
    forcePassiveRF += coeffs_forcePassiveRF[i] * pow( jointAngle, i );
  
  momentArmRF = 0.0;
  for (int i=0; i < COEFFS_NUMBER; i++)
    momentArmRF += coeffs_momentArmRF[i] * pow( jointAngle, i );
  
  normLengthRF = 0.0;
  for (int i=0; i < COEFFS_NUMBER; i++)
    normLengthRF += coeffs_normLengthRF[i] * pow( jointAngle, i );
  
  phiRF = asin(sin(phiZeroRF)/normLengthRF);
  
  forceRF = alphaRF * ( forceActiveRF * activationMeasure_1 + forcePassiveRF ) * cos( phiRF );
  torqueRF = forceRF * momentArmRF;
  
  
  //Biceps Femoris
  forceActiveBF = 0.0;
  for (int i=0; i < COEFFS_NUMBER; i++)
    forceActiveBF += coeffs_forceActiveBF[i] * pow( jointAngle, i );
  
  forcePassiveBF = 0.0;
  for (int i=0; i < COEFFS_NUMBER; i++)
    forcePassiveBF += coeffs_forcePassiveBF[i] * pow( jointAngle, i );
  
  momentArmBF = 0.0;
  for (int i=0; i < COEFFS_NUMBER; i++)
    momentArmBF += coeffs_momentArmBF[i] * pow( jointAngle, i );
  
  normLengthBF = 0.0;
  for (int i=0; i < COEFFS_NUMBER; i++)
    normLengthBF += coeffs_normLengthBF[i] * pow( jointAngle, i );
  
  phiBF = asin(sin(phiZeroBF)/normLengthBF); 
  
  forceBF = alphaBF * ( forceActiveBF * activationMeasure_2 + forcePassiveBF ) * cos( phiBF );
  torqueBF = forceBF * momentArmBF;
  
  //Stiffness Total
  stiffnessMuscle = fabs( beta * ( torqueRF /*+ torqueBF*/ ) );
  
  if( stiffnessMuscle > 30.0 ) stiffnessMuscle = 30.0;
  
  DEBUG_PRINT( "theta: %.3f - stiffness: %.3f", jointAngle, stiffnessMuscle );
  
  return stiffnessMuscle;
}

#endif /* EMG_JOINT_CONTROL_H */
