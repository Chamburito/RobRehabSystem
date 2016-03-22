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
		newModel->osimModel.setName( "Leg" );

		// Get a reference to the model's femur body
		OpenSim::Body& ground = newModel->osimModel.getGroundBody();

		// Specify properties of a 20 kg, 0.1 m^3 tibia body
		double mass = 20.0, width = 0.1, length = 0.4;
		SimTK::Vec3 massCenter( 0 );
		SimTK::Inertia inertia = mass * SimTK::Inertia::brick( length, width, width );

		// Create a new tibia body with specified properties
		OpenSim::Body* femur = new OpenSim::Body( "femur", mass, massCenter, inertia );

		// Add display geometry to the femur to visualize in the GUI
		femur->addDisplayGeometry( "anchor1.vtp" );

		// Create a new tibia body with specified properties
		OpenSim::Body* tibia = new OpenSim::Body( "tibia", mass, massCenter, inertia );

		// Add display geometry to the tibia to visualize in the GUI
		tibia->addDisplayGeometry( "anchor1.vtp" );

		// Create a new tibia body with specified properties
		//OpenSim::Body* foot = new OpenSim::Body( "foot", mass, massCenter, inertia );

		// Add display geometry to the tibia to visualize in the GUI
		//foot->addDisplayGeometry( "anchor1.vtp" );

		// Create a new free joint with 6 degrees-of-freedom (coordinates) between the tibia and femur bodies
		OpenSim::FreeJoint* measure = new OpenSim::FreeJoint( "measure", ground, SimTK::Vec3( 0 ), SimTK::Vec3( 0 ), *femur, SimTK::Vec3( 0 ), SimTK::Vec3( 0 ) );

		OpenSim::PinJoint* hip = new OpenSim::PinJoint( "hip", ground, SimTK::Vec3( 0 ), SimTK::Vec3( 0 ), *femur, SimTK::Vec3( 0 ), SimTK::Vec3( 0 ) );

		OpenSim::PinJoint* knee = new OpenSim::PinJoint( "knee", *femur, SimTK::Vec3( length / 2, width / 2, 0 ), SimTK::Vec3( 0 ), *tibia, SimTK::Vec3( -length / 2, width / 2, 0 ), SimTK::Vec3( 0 ) );

		//OpenSim::PinJoint* ankle = new OpenSim::PinJoint( "ankle", *tibia, SimTK::Vec3( length / 2, width / 2, 0 ), SimTK::Vec3( 0 ), *foot, SimTK::Vec3( -length / 2, width / 2, 0 ), SimTK::Vec3( 0 ) );

		//// Add the bodies to the model
		newModel->osimModel.addBody( femur );
		newModel->osimModel.addBody( tibia );
		//newModel->osimModel.addBody( foot );

		//// Define the acceleration of gravity
		newModel->osimModel.setGravity( SimTK::Vec3( 0, -9.80665, 0 ) );

		newModel->osimModel.setUseVisualizer( true );

		newModel->coordinatesList.insert( 0, &(hip->upd_CoordinateSet()[ 0 ]) );
		newModel->coordinatesList.insert( 1, &(knee->upd_CoordinateSet()[ 0 ]) );
		//newModel->coordinatesList.insert( 2, &(newModel->osimModel.updCoordinateSet()[ 1 ]) );

		//// Initialize the system
		SimTK::State& localState = newModel->osimModel.initSystem();
		newModel->fkState = SimTK::State( localState );
		newModel->ikState = SimTK::State( localState );

		OpenSim::MarkersReference markersReference;
		SimTK::Array_<OpenSim::CoordinateReference> coordinateReferences;
		//newModel->ikSolver = new OpenSim::InverseKinematicsSolver( newModel->osimModel, markersReference, coordinateReferences );

		// Create the integrator and manager for the simulation.
		newModel->integrator = new SimTK::RungeKuttaMersonIntegrator( newModel->osimModel.getMultibodySystem() );
		newModel->integrator->setAccuracy( 1.0e-4 );
		newModel->manager = new OpenSim::Manager( newModel->osimModel, *(newModel->integrator) );

		// Define the initial and final simulation times
		double initialTime = 0.0;
		double finalTime = 0.005;
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

	std::cout << "OpenSim model loaded successfully ! (" << newModel->osimModel.getNumCoordinates() << " coordinates)" << std::endl;

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

	model->coordinatesList[ 0 ].setValue( model->fkState, 0.0 );
	model->coordinatesList[ 1 ].setValue( model->fkState, jointValuesTable[ 0 ][ ROBOT_POSITION ] );

	model->manager->integrate( model->fkState );
	std::cout << "axis: " << model->coordinatesList[ 0 ].getValue( model->fkState ) << std::endl; 
	std::cout << "knee: " << model->coordinatesList[ 1 ].getValue( model->fkState ) << std::endl;
	//std::cout << "ankle: " << model->coordinatesList[ 2 ].getValue( model->fkState ) << std::endl;

	jointValuesTable[ 0 ][ ROBOT_POSITION ] = model->coordinatesList[ 1 ].getValue( model->fkState );

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