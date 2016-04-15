#include <OpenSim/OpenSim.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>
#include <OpenSim/Simulation/MarkersReference.h>
#include <OpenSim/Simulation/CoordinateReference.h>
#include <OpenSim/Simulation/InverseKinematicsSolver.h>

#include "robot_control/interface.h"

class MarkersReferenceStream : public OpenSim::MarkersReference
{
public:
  MarkersReferenceStream() : OpenSim::MarkersReference()
  {
    std::cout << "marker reference stream constructor" << std::endl;
  }

  ~MarkersReferenceStream()
  {
    std::cout << "MarkersReference: marker reference stream destructor" << std::endl;
    markerNames.clear();
    markerValues.clear();
    markerWeights.clear();
  }

  //--------------------------------------------------------------------------
  // Reference Interface
  //--------------------------------------------------------------------------
  int getNumRefs() const override { return markerNames.size(); }
  /** get the time range for which the MarkersReference values are valid,	based on the loaded marker data.*/
  SimTK::Vec2 getValidTimeRange() const override { return SimTK::Vec2( 0.0, 1.0 ); }
  /** get the names of the markers serving as references */
  const SimTK::Array_<std::string>& getNames() const override { return markerNames; }
  /** get the value of the MarkersReference */
  void getValues( const SimTK::State &s, SimTK::Array_<SimTK::Vec3> &values ) const override { values = markerValues; }
  /** get the speed value of the MarkersReference */
  //virtual void getSpeedValues(const SimTK::State &s, SimTK::Array_<SimTK::Vec3> &speedValues) const;
  /** get the acceleration value of the MarkersReference */
  //virtual void getAccelerationValues(const SimTK::State &s, SimTK::Array_<SimTK::Vec3> &accValues) const;
  /** get the weighting (importance) of meeting this MarkersReference in the same order as names*/
  void getWeights( const SimTK::State &s, SimTK::Array_<double> &weights ) const override { weights = markerWeights; }
  
  // Custom Methods
  void setReference( std::string name, double weight ) 
  { 
    markerNames.push_back( name );
    markerValues.push_back( SimTK::Vec3( 0 ) );
    markerWeights.push_back( weight );
    _markerWeightSet.adoptAndAppend( new OpenSim::MarkerWeight( name, weight ) );
    std::cout << "MarkersReference: references list " << markerNames << std::endl;
  }
  
  void setValue( int index, SimTK::Vec3& value ) { markerValues[ index ] = value; }
  
private:
  SimTK::Array_<std::string> markerNames;
  SimTK::Array_<SimTK::Vec3> markerValues;
  SimTK::Array_<double> markerWeights;
};

typedef struct _ControllerData
{
  OpenSim::Model* osimModel;
  SimTK::State state;
  SimTK::Integrator* integrator;
  OpenSim::Manager* manager;
  OpenSim::InverseKinematicsSolver* ikSolver;
  OpenSim::CoordinateSet coordinatesList; 
  SimTK::Array_<OpenSim::CoordinateReference> coordinateReferences;
  SimTK::Array_<char*> jointNames;
  OpenSim::MarkerSet markers;
  MarkersReferenceStream markersReference;
  SimTK::Array_<size_t> markerReferenceAxes;
  SimTK::Array_<char*> axisNames;
}
ControllerData;

typedef void* Controller;


DEFINE_INTERFACE( ROBOT_CONTROL_INTERFACE ) 


Controller InitController( const char* data )
{
  ControllerData* newModel = new ControllerData;
  
  try 
  {
    // Create an OpenSim model from XML (.osim) file
    std::cout << "OSim: trying to load model file " << data << std::endl; newModel->osimModel = new OpenSim::Model( std::string( data ) + ".xml" );
    
    OpenSim::JointSet& jointSet = newModel->osimModel->updJointSet();
    std::cout << "OSim: found " << jointSet.getSize() << " joints" << std::endl;
    for( int jointIndex = 0; jointIndex < jointSet.getSize(); jointIndex++ )
    {
      OpenSim::CoordinateSet& jointCoordinateSet = jointSet[ jointIndex ].upd_CoordinateSet();
      std::cout << "OSim: found " << jointCoordinateSet.getSize() << " coordinates in joint " << jointSet[ jointIndex ].getName() << std::endl;
      for( int coordinateIndex = 0; coordinateIndex < jointCoordinateSet.getSize(); coordinateIndex++ )
      {
        newModel->coordinatesList.adoptAndAppend( &(jointCoordinateSet[ coordinateIndex ]) );
        newModel->jointNames.push_back( (char*) jointCoordinateSet[ coordinateIndex ].getName().c_str() );
      }
    }
    
    OpenSim::MarkerSet& markerSet = newModel->osimModel->updMarkerSet();
    std::cout << "OSim: found " << markerSet.getSize() << " markers" << std::endl;
    for( int markerIndex = 0; markerIndex < markerSet.getSize(); markerIndex++ )
    {
      std::string markerName = markerSet[ markerIndex ].getName();
      const char* REFERENCE_AXES_NAMES[ 3 ] = { "_ref_X", "_ref_Y", "_ref_Z" };
      for( size_t referenceAxisIndex = 0; referenceAxisIndex < 3; referenceAxisIndex++ )
      {
        if( markerName.rfind( REFERENCE_AXES_NAMES[ referenceAxisIndex ] ) != std::string::npos )
        {
          std::cout << "OSim: found reference marker " << markerName << std::endl;
          newModel->markers.adoptAndAppend( &(markerSet[ markerIndex ]) );
          newModel->markersReference.setReference( markerName, 1.0 );
          newModel->markerReferenceAxes.push_back( referenceAxisIndex );
          newModel->axisNames.push_back( (char*) markerSet[ markerIndex ].getName().c_str() );
          break;
        }
      }
    }
    
    // Initialize the system (make copy)
    std::cout << "OSim: initialize state" << std::endl; newModel->state = SimTK::State( newModel->osimModel->initSystem() );
    
    newModel->ikSolver = new OpenSim::InverseKinematicsSolver( *(newModel->osimModel), newModel->markersReference, newModel->coordinateReferences );
    newModel->ikSolver->setAccuracy( 1.0e-4 );
    newModel->state.updTime() = 0.0;
    newModel->ikSolver->assemble( newModel->state ); std::cout << "OSim: IK solver created" << std::endl;
    
    // Create the integrator and manager for the simulation.
    newModel->integrator = new SimTK::RungeKuttaMersonIntegrator( newModel->osimModel->getMultibodySystem() );
    newModel->integrator->setAccuracy( 1.0e-4 ); std::cout << "OSim: integrator created" << std::endl;
    newModel->manager = new OpenSim::Manager( *(newModel->osimModel), *(newModel->integrator) ); 
    
    newModel->manager->setInitialTime( 0.0 );
    newModel->manager->setFinalTime( CONTROL_PASS_INTERVAL ); std::cout << "OSim: integration manager created" << std::endl;
  }
  catch( OpenSim::Exception ex )
  {
    std::cout << ex.getMessage() << std::endl;
    EndController( (Controller) newModel );
    return NULL;
  }
  catch( std::exception ex )
  {
    std::cout << ex.what() << std::endl;
    EndController( (Controller) newModel );
    return NULL;
  }
  catch( ... )
  {
    std::cout << "UNRECOGNIZED EXCEPTION" << std::endl;
    EndController( (Controller) newModel );
    return NULL;
  }
  
  std::cout << "OpenSim model loaded successfully ! (" << newModel->osimModel->getNumCoordinates() << " coordinates)" << std::endl;
  
  return (Controller) newModel;
}

void EndController( Controller controller )
{
  if( controller == NULL ) return;
  
  ControllerData* model = (ControllerData*) controller;
  
  delete model->integrator;
  delete model->manager;
  delete model->ikSolver;
  delete model->osimModel;
  
  model->markers.clearAndDestroy();
  model->coordinatesList.clearAndDestroy();
  model->coordinateReferences.clear();  
  
  delete model;
}

size_t GetJointsNumber( Controller controller )
{
  if( controller == NULL ) return 0;
  
  ControllerData* model = (ControllerData*) controller;
  
  return (size_t) model->coordinatesList.getSize();
}

char** GetJointNamesList( Controller controller )
{
  if( controller == NULL ) return NULL;
  
  ControllerData* model = (ControllerData*) controller;
  
  return model->jointNames.data();
}

size_t GetAxesNumber( Controller controller )
{
  if( controller == NULL ) return 0;
  
  ControllerData* model = (ControllerData*) controller;
  
  return (size_t) model->markers.getSize();
}

char** GetAxisNamesList( Controller controller )
{
  if( controller == NULL ) return NULL;
  
  ControllerData* model = (ControllerData*) controller;
  
  return model->axisNames.data();
}

void RunControlStep( Controller controller, double** jointMeasuresTable, double** axisMeasuresTable, double** jointSetpointsTable, double** axisSetpointsTable )
{
  if( controller == NULL ) return;
  
  ControllerData* model = (ControllerData*) controller;
  
  for( int jointIndex = 0; jointIndex < model->coordinatesList.getSize(); jointIndex++ )
  {
    model->coordinatesList[ jointIndex ].setValue( model->state, jointMeasuresTable[ jointIndex ][ CONTROL_POSITION ] );
    model->coordinatesList[ jointIndex ].setSpeedValue( model->state, jointMeasuresTable[ jointIndex ][ CONTROL_VELOCITY ] );
  }
  
  model->manager->integrate( model->state );
  
  /*for( int jointIndex = 0; jointIndex < model->coordinatesList.getSize(); jointIndex++ )
  {
    jointMeasuresTable[ jointIndex ][ CONTROL_POSITION ] = model->coordinatesList[ jointIndex ].getValue( model->state );
    jointMeasuresTable[ jointIndex ][ CONTROL_VELOCITY ] = model->coordinatesList[ jointIndex ].getSpeedValue( model->state );
    jointMeasuresTable[ jointIndex ][ CONTROL_ACCELERATION ] = model->coordinatesList[ jointIndex ].getAccelerationValue( model->state );
  }*/
  
  SimTK::Vec3 markerPosition, markerVelocity, markerAcceleration;
  model->osimModel->getMultibodySystem().realize( model->state, SimTK::Stage::Position );
  model->osimModel->getMultibodySystem().realize( model->state, SimTK::Stage::Velocity );
  model->osimModel->getMultibodySystem().realize( model->state, SimTK::Stage::Acceleration );
  for( int markerIndex = 0; markerIndex < model->markers.getSize(); markerIndex++ )
  {
    model->osimModel->getSimbodyEngine().getPosition( model->state, model->markers[ markerIndex ].getBody(), model->markers[ markerIndex ].getOffset(), markerPosition );
    model->osimModel->getSimbodyEngine().getVelocity( model->state, model->markers[ markerIndex ].getBody(), model->markers[ markerIndex ].getOffset(), markerVelocity );
    model->osimModel->getSimbodyEngine().getAcceleration( model->state, model->markers[ markerIndex ].getBody(), model->markers[ markerIndex ].getOffset(), markerAcceleration );
    
    axisMeasuresTable[ markerIndex ][ CONTROL_POSITION ] = markerPosition[ model->markerReferenceAxes[ markerIndex ] ];
    axisMeasuresTable[ markerIndex ][ CONTROL_VELOCITY ] = markerVelocity[ model->markerReferenceAxes[ markerIndex ] ];
    axisMeasuresTable[ markerIndex ][ CONTROL_ACCELERATION ] = markerAcceleration[ model->markerReferenceAxes[ markerIndex ] ];
    //axisMeasuresTable[ 0 ][ CONTROL_FORCE ] = jointMeasuresTable[ 0 ][ CONTROL_FORCE ];
    
    markerPosition[ model->markerReferenceAxes[ markerIndex ] ] = axisSetpointsTable[ markerIndex ][ CONTROL_POSITION ];
    model->markersReference.setValue( markerIndex, markerPosition );
  }
  
  model->ikSolver->track( model->state );  
  
  for( int jointIndex = 0; jointIndex < model->coordinatesList.getSize(); jointIndex++ )
  {
    jointSetpointsTable[ 0 ][ CONTROL_POSITION ] = model->coordinatesList[ jointIndex ].getValue( model->state );
    jointSetpointsTable[ 0 ][ CONTROL_VELOCITY ] = model->coordinatesList[ jointIndex ].getSpeedValue( model->state );
    jointSetpointsTable[ 0 ][ CONTROL_ACCELERATION ] = model->coordinatesList[ jointIndex ].getAccelerationValue( model->state );
    //jointSetpointsTable[ 0 ][ CONTROL_FORCE ] = axisSetpointsTable[ 0 ][ CONTROL_FORCE ];
  }
  
  std::cout << "marker position: " << markerPosition << std::endl;
  std::cout << "joint positions: " << model->coordinatesList[0].getValue(model->state) << ", " << model->coordinatesList[1].getValue(model->state) << std::endl;
}
