#include <OpenSim/OpenSim.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>
#include <OpenSim/Simulation/MarkersReference.h>
#include <OpenSim/Simulation/CoordinateReference.h>
#include <OpenSim/Simulation/InverseKinematicsSolver.h>

enum { ROBOT_POSITION, ROBOT_VELOCITY, ROBOT_ACCELERATION, ROBOT_FORCE = ROBOT_ACCELERATION, ROBOT_MECHANICS_VARS_NUMBER };

const size_t DOFS_NUMBER = 1;

typedef void* MechanicalModel;

extern "C" __declspec(dllexport) MechanicalModel __cdecl InitModel(const char*);
extern "C" __declspec(dllexport) void __cdecl EndModel(MechanicalModel);
extern "C" __declspec(dllexport) size_t __cdecl GetDoFsNumber(MechanicalModel);
extern "C" __declspec(dllexport) void __cdecl SolveForwardKinematics(MechanicalModel, double**, double**);
extern "C" __declspec(dllexport) void __cdecl SolveInverseKinematics(MechanicalModel, double**, double**);
extern "C" __declspec(dllexport) void __cdecl SolveForwardDynamics(MechanicalModel, double**, double**);
extern "C" __declspec(dllexport) void __cdecl SolveInverseDynamics(MechanicalModel, double**, double**);
extern "C" __declspec(dllexport) void __cdecl GetJointControlActions(MechanicalModel, double**, double**);

typedef struct _ModelData
{
	OpenSim::Model osimModel;
	OpenSim::CoordinateSet coordinatesList;
	OpenSim::InverseKinematicsSolver* ikSolver;
	SimTK::State fkState, ikState;
	SimTK::Integrator* integrator;
	OpenSim::Manager* manager;
}
ModelData;

MechanicalModel InitModel( const char* data )
{
	ModelData* newModel = new ModelData;

	try 
	{
		// Create an OpenSim model and set its name
		newModel->osimModel.setName( "Knee" );

		// Get a reference to the model's femur body
		OpenSim::Body& femur = newModel->osimModel.getGroundBody();

		// Add display geometry to the femur to visualize in the GUI
		femur.addDisplayGeometry( "anchor1.vtp" );

		// Specify properties of a 20 kg, 0.1 m^3 tibia body
		double tibiaMass = 20.0, tibiaWidth = 0.1, tibiaLength = 0.4;
		SimTK::Vec3 tibiaMassCenter( 0 );
		SimTK::Inertia tibiaInertia = tibiaMass * SimTK::Inertia::brick( tibiaLength, tibiaWidth, tibiaWidth );

		// Create a new tibia body with specified properties
		OpenSim::Body* tibia = new OpenSim::Body( "tibia", tibiaMass, tibiaMassCenter, tibiaInertia );

		// Add display geometry to the tibia to visualize in the GUI
		tibia->addDisplayGeometry( "anchor1.vtp" );

		// Create a new free joint with 6 degrees-of-freedom (coordinates) between the tibia and femur bodies
		SimTK::Vec3 locationInParent( tibiaLength / 2, tibiaWidth / 2, 0 ), orientationInParent( 0 ), locationInBody( -tibiaLength / 2, tibiaWidth / 2, 0 ), orientationInBody( 0 );
		OpenSim::PinJoint* knee = new OpenSim::PinJoint( "knee", femur, locationInParent, orientationInParent, *tibia, locationInBody, orientationInBody );

		//// Add the tibia body to the model
		newModel->osimModel.addBody( tibia );

		//// Define the acceleration of gravity
		newModel->osimModel.setGravity( SimTK::Vec3( 0, -9.80665, 0 ) );

		newModel->osimModel.setUseVisualizer( true );

		newModel->coordinatesList.insert( 0, &(knee->upd_CoordinateSet()[ 0 ]) );

		//// Initialize the system
		SimTK::State& localState = newModel->osimModel.initSystem();
		newModel->fkState = SimTK::State( localState );
		newModel->ikState = SimTK::State( localState );

		OpenSim::MarkersReference markersReference;
		SimTK::Array_<OpenSim::CoordinateReference> coordinateReferences;
		newModel->ikSolver = new OpenSim::InverseKinematicsSolver( newModel->osimModel, markersReference, coordinateReferences );

		// Create the integrator and manager for the simulation.
		newModel->integrator = new SimTK::RungeKuttaMersonIntegrator( newModel->osimModel.getMultibodySystem() );
		newModel->integrator->setAccuracy( 1.0e-4 );
		newModel->manager = new OpenSim::Manager( newModel->osimModel, *(newModel->integrator) );

		// Define the initial and final simulation times
		double initialTime = 0.0;
		double finalTime = 1.0;
		// Integrate from initial time to final time
		newModel->manager->setInitialTime( initialTime );
		newModel->manager->setFinalTime( finalTime );
	}
	catch( OpenSim::Exception ex )
	{
		std::cout << ex.getMessage() << std::endl;
		EndModel( (MechanicalModel) newModel );
		return NULL;
	}
	catch( std::exception ex )
	{
		std::cout << ex.what() << std::endl;
		EndModel( (MechanicalModel) newModel );
		return NULL;
	}
	catch( ... )
	{
		std::cout << "UNRECOGNIZED EXCEPTION" << std::endl;
		EndModel( (MechanicalModel) newModel );
		return NULL;
	}

	std::cout << "OpenSim model loaded successfully." << std::endl;

	return (MechanicalModel) newModel;
}

void EndModel( MechanicalModel robot )
{
	ModelData* model = (ModelData*) robot;

	delete model->integrator;
	delete model->manager;

	delete model;
}

size_t GetDoFsNumber( MechanicalModel robot )
{
	return DOFS_NUMBER;
}

void SolveForwardKinematics( MechanicalModel robot, double** jointValuesTable, double** axisValuesTable )
{
	ModelData* model = (ModelData*) robot;

	model->coordinatesList[ 0 ].setValue( model->fkState, jointValuesTable[ 0 ][ ROBOT_POSITION ] );

	model->manager->integrate( model->fkState);
	std::cout << "\nvelocity: " << model->coordinatesList[ 0 ].getSpeedValue( model->fkState) << std::endl;

	axisValuesTable[ 0 ][ ROBOT_POSITION ] = jointValuesTable[ 0 ][ ROBOT_POSITION ];
	axisValuesTable[ 0 ][ ROBOT_VELOCITY ] = jointValuesTable[ 0 ][ ROBOT_VELOCITY ];
	axisValuesTable[ 0 ][ ROBOT_ACCELERATION ] = jointValuesTable[ 0 ][ ROBOT_ACCELERATION ];
}

void SolveInverseKinematics( MechanicalModel robot, double** axisValuesTable, double** jointValuesTable )
{
	jointValuesTable[ 0 ][ ROBOT_POSITION ] = axisValuesTable[ 0 ][ ROBOT_POSITION ];
	jointValuesTable[ 0 ][ ROBOT_VELOCITY ] = axisValuesTable[ 0 ][ ROBOT_VELOCITY ];
	jointValuesTable[ 0 ][ ROBOT_ACCELERATION ] = axisValuesTable[ 0 ][ ROBOT_ACCELERATION ];
}

void SolveForwardDynamics( MechanicalModel robot, double** jointValuesTable, double** axisValuesTable )
{
	axisValuesTable[ 0 ][ ROBOT_FORCE ] = jointValuesTable[ 0 ][ ROBOT_FORCE ];
}

void SolveInverseDynamics( MechanicalModel robotD, double** axisValuesTable, double** jointValuesTable )
{
	jointValuesTable[ 0 ][ ROBOT_FORCE ] = axisValuesTable[ 0 ][ ROBOT_FORCE ];
}

void GetJointControlActions( MechanicalModel robot, double** jointMeasuresTable, double** jointSetpointsTable )
{
	return;
}