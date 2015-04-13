#pragma once

#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include "../utilities/assimp_mesh_loader.hpp"
#include "heightfield.h"

#define CUBE_HALF_EXTENTS 1

class BulletRayCastVehicle {
public:
	typedef std::vector< std::pair<double, double> > WorkspaceBounds;
	typedef std::vector< std::pair<double, double> > StateVarRanges;

	typedef std::vector<double> StateVars;

	class State {
	public:
		State() {}

		State(const State &s) : stateVars(s.stateVars.begin(), s.stateVars.end()) {}

		State(const StateVars &vars) : stateVars(vars.begin(), vars.begin()+3) {}

		const StateVars& getStateVars() const { return stateVars; }

		bool equals(const State& s) const { return false; }

#ifdef WITHGRAPHICS
		void draw(const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {}
#endif

	private:
		StateVars stateVars;
	};

	class Edge {
	public:
		Edge(const State& s) {}

		/* needed for being inserted into NN datastructure */
		const StateVars& getStateVars() const { return end.getStateVars(); }
		int getPointIndex() const { return treeIndex; }
		void setPointIndex(int ptInd) { treeIndex = ptInd; }

#ifdef WITHGRAPHICS
		void draw(const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {}
#endif

		const State start, end;
		double cost;
	
	private:
		int treeIndex;
	};

	BulletRayCastVehicle(const InstanceFileMap &args) : m_defaultContactProcessingThreshold(BT_LARGE_FLOAT),
		suspensionRestLength(0.6), wheelDirectionCS0(0,-1,0), wheelAxleCS(-1,0,0), bounds(3) {

		btCollisionShape *groundShape = new btBoxShape(btVector3(50,3,50));
		m_collisionShapes.push_back(groundShape);
		m_collisionConfiguration = new btDefaultCollisionConfiguration();
		m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
		btVector3 worldMin(-1000,-1000,-1000);
		btVector3 worldMax(1000,1000,1000);

		bounds[0].first = -1000;
		bounds[0].second = 1000;
		bounds[1].first = -1000;
		bounds[1].second = 1000;
		bounds[2].first = -1000;
		bounds[2].second = 1000;

		m_overlappingPairCache = new btAxisSweep3(worldMin,worldMax);
		m_constraintSolver = new btSequentialImpulseConstraintSolver();
		m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_overlappingPairCache,m_constraintSolver,m_collisionConfiguration);

		btTransform tr;
		tr.setIdentity();

		int width=128;
		int length=128;

		char *heightfieldData = MyHeightfield;

		//btScalar maxHeight = 20000.f;//exposes a bug
		btScalar maxHeight = 100;

		bool useFloatDatam=false;
		bool flipQuadEdges=false;

		btHeightfieldTerrainShape *heightFieldShape = new btHeightfieldTerrainShape(width,length,heightfieldData,maxHeight,upIndex,useFloatDatam,flipQuadEdges);;
		btVector3 mmin,mmax;
		heightFieldShape->getAabb(btTransform::getIdentity(),mmin,mmax);

		groundShape = heightFieldShape;

		heightFieldShape->setUseDiamondSubdivision(true);

		btVector3 localScaling(100,1,100);
		localScaling[upIndex]=1.f;
		groundShape->setLocalScaling(localScaling);

		//tr.setOrigin(btVector3(0,9940,0));
		tr.setOrigin(btVector3(0,49.4,0));

		m_collisionShapes.push_back(groundShape);

		//create ground object
		btRigidBody *ground = localCreateRigidBody(0,tr,groundShape);
		ground->setContactProcessingThreshold(m_defaultContactProcessingThreshold);
		m_dynamicsWorld->addRigidBody(ground);


		tr.setOrigin(btVector3(0,0,0));//-64.5f,0));

		btCollisionShape *chassisShape = new btBoxShape(btVector3(1.f,0.5f,2.f));
		m_collisionShapes.push_back(chassisShape);

		btCompoundShape *compound = new btCompoundShape();
		m_collisionShapes.push_back(compound);
		btTransform localTrans;
		localTrans.setIdentity();
		//localTrans effectively shifts the center of mass with respect to the chassis
		localTrans.setOrigin(btVector3(0,1,0));

		compound->addChildShape(localTrans,chassisShape);

		tr.setOrigin(btVector3(0,0.f,0));

		m_carChassis = localCreateRigidBody(800,tr,compound);//chassisShape);
		//m_carChassis->setDamping(0.2,0.2);

		m_wheelShape = new btCylinderShapeX(btVector3(wheelWidth,wheelRadius,wheelRadius));

		gVehicleSteering = 0.f;
		m_carChassis->setCenterOfMassTransform(btTransform::getIdentity());
		m_carChassis->setLinearVelocity(btVector3(0,0,0));
		m_carChassis->setAngularVelocity(btVector3(0,0,0));
		m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(m_carChassis->getBroadphaseHandle(),m_dynamicsWorld->getDispatcher());

		/// create vehicle
		{

			m_vehicleRayCaster = new btDefaultVehicleRaycaster(m_dynamicsWorld);
			m_vehicle = new btRaycastVehicle(m_tuning,m_carChassis,m_vehicleRayCaster);

			///never deactivate the vehicle
			m_carChassis->setActivationState(DISABLE_DEACTIVATION);

			m_dynamicsWorld->addVehicle(m_vehicle);

			float connectionHeight = 1.2f;


			bool isFrontWheel=true;

			//choose coordinate system
			m_vehicle->setCoordinateSystem(rightIndex,upIndex,forwardIndex);

			btVector3 connectionPointCS0(CUBE_HALF_EXTENTS-(0.3*wheelWidth),connectionHeight,2*CUBE_HALF_EXTENTS-wheelRadius);

			m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);

			connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),connectionHeight,2*CUBE_HALF_EXTENTS-wheelRadius);

			m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);

			connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),connectionHeight,-2*CUBE_HALF_EXTENTS+wheelRadius);

			isFrontWheel = false;
			m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);

			connectionPointCS0 = btVector3(CUBE_HALF_EXTENTS-(0.3*wheelWidth),connectionHeight,-2*CUBE_HALF_EXTENTS+wheelRadius);

			m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);

			for(int i=0; i<m_vehicle->getNumWheels(); i++) {
				btWheelInfo &wheel = m_vehicle->getWheelInfo(i);
				wheel.m_suspensionStiffness = suspensionStiffness;
				wheel.m_wheelsDampingRelaxation = suspensionDamping;
				wheel.m_wheelsDampingCompression = suspensionCompression;
				wheel.m_frictionSlip = wheelFriction;
				wheel.m_rollInfluence = rollInfluence;
			}
		}
	}

	~BulletRayCastVehicle() {

	}

	//from the perspective of the environment
	const WorkspaceBounds& getBounds() const {
		return bounds;
	}

	bool safeEdge(const BulletRayCastVehicle &agent, const Edge &edge, double dt) const {
		return true;
	}

	bool safePoses(const BulletRayCastVehicle &agent, const std::vector<fcl::Transform3f> &poses) const {
		return true;;
	}


	//from the perspective of the agent
	StateVarRanges getStateVarRanges(const WorkspaceBounds& bounds) const {

		//possible extend to include additional state variables besides x,y,z
		return bounds;
	}

	State buildState(const StateVars& vars) const {
		return State(vars);
	}

	Edge steer(const State &start, const State &goal, double dt) const {
		Edge e(start);
		return e;
	}

	Edge randomSteer(const State &start, double dt) const {
		Edge e(start);
		return e;
	}

	bool isGoal(const State& state, const State& goal) const {
		return false;
	}

#ifdef WITHGRAPHICS
	void draw() const {}

	void drawSolution(const std::vector<const Edge*> &solution, double dt = std::numeric_limits<double>::infinity()) const {}

	void animateSolution(const std::vector<const Edge*> &solution, unsigned int poseNumber) const {}
#endif

private:
	btRigidBody *localCreateRigidBody(float mass, const btTransform &startTransform, btCollisionShape *shape) const {
		btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if(isDynamic)
			shape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

		btDefaultMotionState *myMotionState = new btDefaultMotionState(startTransform);

		btRigidBody::btRigidBodyConstructionInfo cInfo(mass,myMotionState,shape,localInertia);

		btRigidBody *body = new btRigidBody(cInfo);

		return body;
	}

	void step(btDynamicsWorld *m_dynamicsWorld, btRaycastVehicle *m_vehicle, double gEngineForce,
	          double gBreakingForce, double gVehicleSteering, double dt) {
		int wheelIndex = 2;
		m_vehicle->applyEngineForce(gEngineForce,wheelIndex);
		m_vehicle->setBrake(gBreakingForce,wheelIndex);
		wheelIndex = 3;
		m_vehicle->applyEngineForce(gEngineForce,wheelIndex);
		m_vehicle->setBrake(gBreakingForce,wheelIndex);


		wheelIndex = 0;
		m_vehicle->setSteeringValue(gVehicleSteering,wheelIndex);
		wheelIndex = 1;
		m_vehicle->setSteeringValue(gVehicleSteering,wheelIndex);

		m_dynamicsWorld->stepSimulation(dt);
	}

	btDynamicsWorld *m_dynamicsWorld;
	btAlignedObjectArray<btCollisionShape *> m_collisionShapes;
	btDefaultCollisionConfiguration *m_collisionConfiguration;
	btCollisionDispatcher *m_dispatcher;
	btBroadphaseInterface *m_overlappingPairCache;
	btConstraintSolver *m_constraintSolver;
	btScalar m_defaultContactProcessingThreshold;
	btRigidBody *m_carChassis;


	btRaycastVehicle::btVehicleTuning m_tuning;
	btVehicleRaycaster *m_vehicleRayCaster;
	btRaycastVehicle *m_vehicle;
	btCollisionShape *m_wheelShape;

	float   gEngineForce = 0.f;
	float   gBreakingForce = 0.f;

	float   maxEngineForce = 1000.f;//this should be engine/velocity dependent
	float   maxBreakingForce = 100.f;

	float   gVehicleSteering = 0.f;
	float   steeringIncrement = 0.04f;
	float   steeringClamp = 0.3f;
	float   wheelRadius = 0.5f;
	float   wheelWidth = 0.4f;
	float   wheelFriction = 1000;//BT_LARGE_FLOAT;
	float   suspensionStiffness = 20.f;
	float   suspensionDamping = 2.3f;
	float   suspensionCompression = 4.4f;
	float   rollInfluence = 0.1f;//1.0f;

	btScalar suspensionRestLength;

	//y-axis is up
	int rightIndex = 0;
	int upIndex = 1;
	int forwardIndex = 2;
	btVector3 wheelDirectionCS0;
	btVector3 wheelAxleCS;

	WorkspaceBounds bounds;
};