#include <OpenSim/OpenSim.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>
#include <OpenSim/Simulation/MarkersReference.h>
#include <OpenSim/Simulation/CoordinateReference.h>
#include <OpenSim/Simulation/InverseKinematicsSolver.h>

enum { ROBOT_POSITION, ROBOT_VELOCITY, ROBOT_ACCELERATION, ROBOT_FORCE, ROBOT_MECHANICS_VARS_NUMBER };

const size_t DOFS_NUMBER = 1;

class MarkersReferenceStream : public OpenSim::MarkersReference
{
public:
	MarkersReferenceStream() : OpenSim::MarkersReference()
	{
		std::cout << "marker reference stream constructor" << std::endl;
		markerNames.push_back("Marker");
		markerValues.push_back(SimTK::Vec3(0));
		markerWeights.push_back(0.7);
		_markerWeightSet.adoptAndAppend(new OpenSim::MarkerWeight("Marker", 1.0));
		std::cout << "created reference with " << markerNames.size() << " markers: " << markerNames << std::endl;
	}

	~MarkersReferenceStream()
	{
		std::cout << "marker reference stream destructor" << std::endl;
		markerNames.clear();
		markerValues.clear();
		markerWeights.clear();
	}

	//--------------------------------------------------------------------------
	// Reference Interface
	//--------------------------------------------------------------------------
	int getNumRefs() const override { /*std::cout << "marker reference get num refs: " << markerNames.size() << std::endl;*/ return markerNames.size(); }
	/** get the time range for which the MarkersReference values are valid,	based on the loaded marker data.*/
	SimTK::Vec2 getValidTimeRange() const override { /*std::cout << "marker reference get time range: " << SimTK::Vec2(0.0, 1.0) << std::endl;*/ return SimTK::Vec2(0.0, 1.0); }
	/** get the names of the markers serving as references */
	const SimTK::Array_<std::string>& getNames() const override { /*std::cout << "marker reference get names: " << markerNames << std::endl;*/ return markerNames; }
	/** get the value of the MarkersReference */
	void getValues(const SimTK::State &s, SimTK::Array_<SimTK::Vec3> &values) const override { /*std::cout << "marker reference get values: " << markerValues << std::endl;*/ values = markerValues; }
	/** get the speed value of the MarkersReference */
	//virtual void getSpeedValues(const SimTK::State &s, SimTK::Array_<SimTK::Vec3> &speedValues) const;
	/** get the acceleration value of the MarkersReference */
	//virtual void getAccelerationValues(const SimTK::State &s, SimTK::Array_<SimTK::Vec3> &accValues) const;
	/** get the weighting (importance) of meeting this MarkersReference in the same order as names*/
	void getWeights(const SimTK::State &s, SimTK::Array_<double> &weights) const override { /*std::cout << "marker reference get weights: " << markerWeights << std::endl;*/ weights = markerWeights; }

	// Custom Method
	void setValue(int index, SimTK::Vec3& value) { /*std::cout << "set marker reference " << markerNames[ index ] << std::endl;*/  markerValues[index] = value; }

private:
	SimTK::Array_<std::string> markerNames;
	SimTK::Array_<SimTK::Vec3> markerValues;
	SimTK::Array_<double> markerWeights;
};

typedef struct _ControllerData
{
	OpenSim::Model osimModel;
	OpenSim::CoordinateSet coordinatesList;
	OpenSim::InverseKinematicsSolver* ikSolver;
	SimTK::State fkState, ikState;
	SimTK::Integrator* integrator;
	OpenSim::Manager* manager;
	OpenSim::MarkerSet* markers;
	MarkersReferenceStream* markersReference;
	SimTK::Array_<OpenSim::CoordinateReference>* coordinateReferences;
}
ControllerData;

typedef void* Controller;

extern "C" __declspec(dllexport) Controller __cdecl InitController(const char*);
extern "C" __declspec(dllexport) void __cdecl EndController(Controller);
extern "C" __declspec(dllexport) size_t __cdecl GetDoFsNumber(Controller);
extern "C" __declspec(dllexport) const char** __cdecl GetJointNamesList(Controller);
extern "C" __declspec(dllexport) const char** __cdecl GetAxisNamesList(Controller);
extern "C" __declspec(dllexport) void __cdecl RunControlStep(Controller, double**, double**, double**, double**);


Controller InitController( const char* data )
{
	ControllerData* newModel = new ControllerData;

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
		//OpenSim::FreeJoint* measure = new OpenSim::FreeJoint( "measure", ground, SimTK::Vec3( 0 ), SimTK::Vec3( 0 ), *tibia, SimTK::Vec3( length / 2, width / 2, 0 ), SimTK::Vec3( 0 ) );
		//std::cout << "created free joint with " << measure->upd_CoordinateSet().getSize() << " DoFs" << std::endl;

		OpenSim::PinJoint* hip = new OpenSim::PinJoint( "hip", ground, SimTK::Vec3( 0 ), SimTK::Vec3( 0 ), *femur, SimTK::Vec3( 0 ), SimTK::Vec3( 0 ) );

		OpenSim::PinJoint* knee = new OpenSim::PinJoint( "knee", *femur, SimTK::Vec3( length / 2, width / 2, 0 ), SimTK::Vec3( 0 ), *tibia, SimTK::Vec3( -length / 2, width / 2, 0 ), SimTK::Vec3( 0 ) );

		//OpenSim::PinJoint* ankle = new OpenSim::PinJoint( "ankle", *tibia, SimTK::Vec3( length / 2, width / 2, 0 ), SimTK::Vec3( 0 ), *foot, SimTK::Vec3( -length / 2, width / 2, 0 ), SimTK::Vec3( 0 ) );

		//// Add the bodies to the model
		newModel->osimModel.addBody( femur );
		newModel->osimModel.addBody( tibia );
		//newModel->osimModel.addBody( foot );

		newModel->markers = new OpenSim::MarkerSet();
		newModel->markers->addMarker( "Marker", new double[3]{ length / 2, width / 2, 0 }, *tibia );
		newModel->osimModel.updateMarkerSet( *(newModel->markers) );

		//// Define the acceleration of gravity
		newModel->osimModel.setGravity( SimTK::Vec3( 0, -9.80665, 0 ) );

		newModel->osimModel.setUseVisualizer( true );

		newModel->coordinatesList.adoptAndAppend( &(hip->upd_CoordinateSet()[ 0 ]) );
		newModel->coordinatesList.adoptAndAppend( &(knee->upd_CoordinateSet()[ 0 ]) );

		//newModel->coordinatesList.adoptAndAppend( &(measure->upd_CoordinateSet()[ 3 ]) );
		//newModel->coordinatesList.adoptAndAppend( &(measure->upd_CoordinateSet()[ 4 ]) );

		//std::cout << "marker position: " << newModel->coordinatesList[2].getValue(newModel->fkState) << ", " << newModel->coordinatesList[3].getValue(newModel->fkState) << std::endl;

		//// Initialize the system
		SimTK::State& localState = newModel->osimModel.initSystem();
		newModel->fkState = SimTK::State( localState );
		newModel->ikState = SimTK::State( localState );

		newModel->markersReference = new MarkersReferenceStream();
		newModel->coordinateReferences = new SimTK::Array_<OpenSim::CoordinateReference>();
		newModel->ikSolver = new OpenSim::InverseKinematicsSolver( newModel->osimModel, *(newModel->markersReference), *(newModel->coordinateReferences) );
		newModel->ikSolver->setAccuracy(1.0e-4);
		newModel->ikState.updTime() = 0.0;
		newModel->ikSolver->assemble( newModel->ikState );

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

	std::cout << "OpenSim model loaded successfully ! (" << newModel->osimModel.getNumCoordinates() << " coordinates)" << std::endl;

	return (Controller) newModel;
}

void EndController( Controller robot )
{
	ControllerData* model = (ControllerData*) robot;

	delete model->integrator;
	delete model->manager;

	delete model;
}

size_t GetDoFsNumber( Controller robot )
{
	return DOFS_NUMBER;
}

const char* JOINT_NAMES_LIST[ DOFS_NUMBER ] = { "knee" };
const char** GetJointNamesList( Controller controller )
{
	return (const char**) JOINT_NAMES_LIST;
}

const char* AXIS_NAMES_LIST[ DOFS_NUMBER ] = { "theta" };
const char** GetAxisNamesList( Controller controller )
{
	return (const char**) AXIS_NAMES_LIST;
}

void RunControlStep( Controller robot, double** jointMeasuresTable, double** axisMeasuresTable, double** jointSetpointsTable, double** axisSetpointsTable )
{
	ControllerData* model = (ControllerData*) robot;

	//model->coordinatesList[ 0 ].setValue( model->fkState, 0.0 );
	//model->coordinatesList[ 1 ].setValue( model->fkState, jointMeasuresTable[ 0 ][ ROBOT_POSITION ] );

	model->manager->integrate( model->fkState );

	//jointMeasuresTable[ 0 ][ ROBOT_POSITION ] = model->coordinatesList[ 1 ].getValue( model->fkState );

	axisMeasuresTable[ 0 ][ ROBOT_POSITION ] = jointMeasuresTable[ 0 ][ ROBOT_POSITION ];
	axisMeasuresTable[ 0 ][ ROBOT_VELOCITY ] = jointMeasuresTable[ 0 ][ ROBOT_VELOCITY ];
	axisMeasuresTable[ 0 ][ ROBOT_ACCELERATION ] = jointMeasuresTable[ 0 ][ ROBOT_ACCELERATION ];
	axisMeasuresTable[ 0 ][ ROBOT_FORCE ] = jointMeasuresTable[ 0 ][ ROBOT_FORCE ];

	//model->markers->get( "Marker" ).

	//model->markersReference->setValue(0, SimTK::Vec3(axisSetpointsTable[0][ROBOT_POSITION], axisSetpointsTable[0][ROBOT_POSITION], 0.0) );
	//model->ikSolver->track( model->ikState );

	model->coordinatesList[ 0 ].setValue( model->ikState, model->coordinatesList[0].getValue(model->fkState));
	model->coordinatesList[ 1 ].setValue( model->ikState, model->coordinatesList[1].getValue(model->fkState));

	OpenSim::MarkerSet& markers = model->osimModel.updMarkerSet();
	OpenSim::Marker& marker = markers.get("Marker");
	SimTK::Vec3 markerPosition;
	model->osimModel.getMultibodySystem().realize(model->fkState, SimTK::Stage::Position);
	model->osimModel.getSimbodyEngine().getPosition(model->fkState, marker.getBody(), marker.getOffset(), markerPosition);

	std::cout << "marker position: " << markerPosition << std::endl;
	std::cout << "joint positions: " << model->coordinatesList[0].getValue(model->ikState) << ", " << model->coordinatesList[1].getValue(model->ikState) << std::endl;

	//model->coordinatesList[ 0 ].setValue( model->fkState, model->coordinatesList[ 0 ].getValue( model->ikState ) );
	//model->coordinatesList[ 1 ].setValue( model->fkState, model->coordinatesList[ 1 ].getValue( model->ikState ) );

	//std::cout << "marker position: " << model->coordinatesList[2].getValue(model->fkState) << ", " << model->coordinatesList[3].getValue(model->fkState) << std::endl;
	//std::cout << std::endl;

	//model->manager->integrate( model->fkState );

	jointSetpointsTable[ 0 ][ ROBOT_POSITION ] = axisSetpointsTable[ 0 ][ ROBOT_POSITION ];
	jointSetpointsTable[ 0 ][ ROBOT_VELOCITY ] = axisSetpointsTable[ 0 ][ ROBOT_VELOCITY ];
	jointSetpointsTable[ 0 ][ ROBOT_ACCELERATION ] = axisSetpointsTable[ 0 ][ ROBOT_ACCELERATION ];
	jointSetpointsTable[ 0 ][ ROBOT_FORCE ] = axisSetpointsTable[ 0 ][ ROBOT_FORCE ];
}