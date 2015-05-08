#pragma once

#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include "../utilities/assimp_mesh_loader.hpp"
#include "heightfield.h"

#ifdef WITHGRAPHICS
#include "GLDebugDrawer.hpp"
#endif

#define CUBE_HALF_EXTENTS 1
#define STATE_VAR_COUNT 40

class BulletRayCastVehicle {
public:
	typedef std::vector< std::pair<double, double> > WorkspaceBounds;
	typedef std::vector< std::pair<double, double> > StateVarRanges;

	typedef std::vector<double> StateVars;

	class State {
	public:
		State() : stateVars(STATE_VAR_COUNT, 0) {}

		State(const State &s) : stateVars(s.stateVars.begin(), s.stateVars.end()) {}

		State(const StateVars &vars) : stateVars(vars.begin(), vars.end()) {
			if(stateVars.size() < STATE_VAR_COUNT) {
				stateVars.resize(STATE_VAR_COUNT, 0);
			}
		}

		const StateVars &getStateVars() const {
			return stateVars;
		}

		bool equals(const State &s) const {
			for(unsigned int i = 0; i < stateVars.size(); ++i) {
				if(fabs(stateVars[i] - s.stateVars[i]) > 0.000001) {
					return false;
				}
			}
			return true;
		}

		void print() const {
			for(auto v : stateVars) {
				fprintf(stderr, "%g ", v);
			}
			fprintf(stderr, "\n");
		}

#ifdef WITHGRAPHICS
		void draw(const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {}
#endif

	private:
		StateVars stateVars;
	};

	class Edge {
	public:
		Edge(const State &s) : start(s), end(s), cost(0), gas(0), brake(0), steer(0), duration(0) {
			populateTreeStateVars();
		}
		Edge(const State &start, const State &end, double cost, double gas, double brake, double steer, double duration) : start(start),
			end(end), cost(cost), treeIndex(0), gas(gas), brake(brake), steer(steer), duration(duration) {
			populateTreeStateVars();
		}
		Edge(const Edge &e) : start(e.start), end(e.end), cost(e.cost), treeIndex(e.treeIndex), gas(e.gas), brake(e.brake),
			steer(e.steer), duration(e.duration) {
			populateTreeStateVars();
		}
		/* needed for being inserted into NN datastructure */
		const StateVars &getTreeStateVars() const {
			return treeStateVars;
		}
		int getPointIndex() const {
			return treeIndex;
		}
		void setPointIndex(int ptInd) {
			treeIndex = ptInd;
		}

#ifdef WITHGRAPHICS
		void draw(const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {}
#endif

		const State start, end;
		double cost;

	private:
		void populateTreeStateVars() {
			const StateVars& stateVars = end.getStateVars();
			treeStateVars.insert(treeStateVars.end(), stateVars.begin(), stateVars.begin() + 14);
		}

		int treeIndex;
		double gas, brake, steer, duration;
		StateVars treeStateVars;
	};

	BulletRayCastVehicle(const InstanceFileMap &args) : m_defaultContactProcessingThreshold(BT_LARGE_FLOAT),
		suspensionRestLength(0.6), bounds(14) {

		boost::char_separator<char> sep(" ");
		boost::tokenizer< boost::char_separator<char> > tokens(args.value("Goal Thresholds"), sep);
		for(auto token : tokens) {
			goalThresholds.push_back(std::stod(token));
		}

		btCollisionShape *groundShape = new btBoxShape(btVector3(50,50,50));
		m_collisionShapes.push_back(groundShape);
		m_collisionConfiguration = new btDefaultCollisionConfiguration();
		m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
		btVector3 worldMin(-1000,-1000,-1000);
		btVector3 worldMax(1000,1000,1000);

		m_overlappingPairCache = new btAxisSweep3(worldMin,worldMax);
		m_constraintSolver = new btSequentialImpulseConstraintSolver();
		m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_overlappingPairCache,m_constraintSolver,m_collisionConfiguration);

		btTransform tr;
		tr.setIdentity();

		// int width=128;
		// int length=128;
		// char *heightfieldData = MyHeightfield;

		int width=10;
		int length=10;

		//btScalar maxHeight = 20000.f;//exposes a bug
		btScalar maxHeight = 10;

		smallHeightField.resize(width*length, 0);

		for(unsigned int i = 0; i < smallHeightField.size(); ++i) {
			smallHeightField[i] = ((double)rand() / (double)RAND_MAX) * (double) maxHeight;
		}

		for(unsigned int i = 40; i < 60; ++i) {
			smallHeightField[i] = 0;
		}

		void *heightfieldData = smallHeightField.data();

		bool useFloatDatam=true;
		bool flipQuadEdges=false;

		btHeightfieldTerrainShape *heightFieldShape = new btHeightfieldTerrainShape(width,length,heightfieldData,maxHeight,upIndex,useFloatDatam,flipQuadEdges);;
		btVector3 mmin,mmax;
		heightFieldShape->getAabb(btTransform::getIdentity(),mmin,mmax);

		groundShape = heightFieldShape;

		heightFieldShape->setUseDiamondSubdivision(true);

		btVector3 localScaling(10,1,10);
		//localScaling[upIndex]=1.f;
		groundShape->setLocalScaling(localScaling);

		tr.setOrigin(btVector3(0,5,0));

		m_collisionShapes.push_back(groundShape);

		// get the bounding box of the ground terrain
		btTransform t;
		t.setIdentity();
		btVector3 min, max;
		groundShape->getAabb(t, min, max);

		for(unsigned int i = 0; i < 3; ++i) {
			bounds[i].first = min[i];
			bounds[i].second = max[i];
		}

		for(unsigned int i = 3; i < 14; ++i) {
			bounds[i].first = -10;
			bounds[i].second = 10;
		}


		btVector3 localInertia(0,0,0);

		btDefaultMotionState *groundMotionState = new btDefaultMotionState(tr);

		btRigidBody::btRigidBodyConstructionInfo groundInfo(0,groundMotionState,groundShape,localInertia);

		btRigidBody *ground  = new btRigidBody(groundInfo);

		ground->setContactProcessingThreshold(m_defaultContactProcessingThreshold);
		m_dynamicsWorld->addRigidBody(ground);

		tr.setOrigin(btVector3(0,0,0));//-64.5f,0));

		btCollisionShape *chassisShape = new btBoxShape(btVector3(1,0.5,2));
		m_collisionShapes.push_back(chassisShape);

		btCompoundShape *compound = new btCompoundShape();
		m_collisionShapes.push_back(compound);
		btTransform localTrans;
		localTrans.setIdentity();
		//localTrans effectively shifts the center of mass with respect to the chassis
		localTrans.setOrigin(btVector3(0,1,0));

		compound->addChildShape(localTrans,chassisShape);

		tr.setOrigin(btVector3(0,0,0));

		chassisShape->calculateLocalInertia(800,localInertia);

		btRigidBody::btRigidBodyConstructionInfo chassisInfo(800,new btDefaultMotionState(tr),compound,localInertia);

		m_carChassis = new btRigidBody(chassisInfo);

		//m_carChassis->setDamping(0.2,0.2);

		m_wheelShape = new btCylinderShapeX(btVector3(wheelWidth,wheelRadius,wheelRadius));

		m_carChassis->setCenterOfMassTransform(btTransform::getIdentity());
		m_carChassis->setLinearVelocity(btVector3(0,0,0));
		m_carChassis->setAngularVelocity(btVector3(0,0,0));
		m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(m_carChassis->getBroadphaseHandle(),m_dynamicsWorld->getDispatcher());

		/// create vehicle
		m_vehicleRayCaster = new btDefaultVehicleRaycaster(m_dynamicsWorld);
		m_vehicle = new btRaycastVehicle(m_tuning,m_carChassis,m_vehicleRayCaster);

		///never deactivate the vehicle
		m_carChassis->setActivationState(DISABLE_DEACTIVATION);

		m_dynamicsWorld->addAction(m_vehicle);
		//m_dynamicsWorld->addVehicle(m_vehicle);
		m_dynamicsWorld->addRigidBody(m_carChassis);

		float connectionHeight = 1.2f;

		bool isFrontWheel=true;

		//choose coordinate system
		m_vehicle->setCoordinateSystem(rightIndex,upIndex,forwardIndex);

		btVector3 wheelDirectionCS0(0,-1,0);
		btVector3 wheelAxleCS(-1,0,0);

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

#ifdef WITHGRAPHICS
		debugDrawer = new GLDebugDrawer();
		m_dynamicsWorld->setDebugDrawer(debugDrawer);
#endif
	}

	~BulletRayCastVehicle() {
		//cleanup in the reverse order of creation/initialization

		//remove the rigidbodies from the dynamics world and delete them
		int i;
		for(i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ; i--) {
			btCollisionObject *obj = m_dynamicsWorld->getCollisionObjectArray()[i];
			btRigidBody *body = btRigidBody::upcast(obj);
			if(body && body->getMotionState()) {
				delete body->getMotionState();
			}
			m_dynamicsWorld->removeCollisionObject(obj);
			delete obj;
		}

		//delete collision shapes
		for(int j=0; j<m_collisionShapes.size(); j++) {
			btCollisionShape *shape = m_collisionShapes[j];
			delete shape;
		}

		//delete dynamics world
		delete m_dynamicsWorld;

		delete m_vehicleRayCaster;

		delete m_vehicle;

		delete m_wheelShape;

		//delete solver
		delete m_constraintSolver;

		//delete broadphase
		delete m_overlappingPairCache;

		//delete dispatcher
		delete m_dispatcher;

		delete m_collisionConfiguration;
	}

	unsigned int getTreeStateSize() const {
		return 14;
	}

	State getCurrentState() const {
		return State(packCurrentVehicle());
	}

	//from the perspective of the environment
	const WorkspaceBounds &getBounds() const {
		return bounds;
	}

	bool safeEdge(const BulletRayCastVehicle &agent, const Edge &edge, double dt) const {
		//all edges are safe in this domain
		return true;
	}

	bool safePoses(const BulletRayCastVehicle &agent, const std::vector<fcl::Transform3f> &poses) const {
		//all poses are safe in this domain
		return true;;
	}


	//from the perspective of the agent
	StateVarRanges getStateVarRanges(const WorkspaceBounds &bounds) const {

		//possibly extend to include additional state variables besides x,y,z
		return bounds;
	}

	State buildState(const StateVars &vars) const {
		return State(vars);
	}

	Edge steer(const State &start, const State &goal, double dt) const {
		return randomSteer(start, dt);
	}

	Edge randomSteer(const State &start, double dt) const {
		unpackCurrentVehicle(start.getStateVars());

		double gas = maxEngineForce;
		double brake = 0;
		double steer = start.getStateVars()[13];

		if(distribution(generator) < 0.95) {
			gas = distribution(generator) * maxEngineForce;
		} else {
			brake = distribution(generator) * maxBreakingForce;
		}

		steer += (2 * distribution(generator) * steeringIncrement - steeringIncrement);
		steer += (10 * distribution(generator) * steeringIncrement);

		if(steer < -steeringClamp) {
			steer = -steeringClamp;
		} else if(steer > steeringClamp) {
			steer = steeringClamp;
		}

		step(gas, brake, steer, dt);

		State newState(packCurrentVehicle());
// auto vars = start.getStateVars();
// fprintf(stderr, "%g , %g, %g -> ", vars[0], vars[1], vars[2]);
// vars = newState.getStateVars();
// fprintf(stderr, "%g , %g, %g\n", vars[0], vars[1], vars[2]);

		return Edge(start, newState, dt, gas, brake, steer, dt);
	}

	bool isGoal(const State &state, const State &goal) const {
		const StateVars &s = state.getStateVars();
		const StateVars &g = goal.getStateVars();

		return fabs(s[0] - g[0]) < goalThresholds[0] &&
		       fabs(s[2] - g[2]) < goalThresholds[2];
	}

#ifdef WITHGRAPHICS
	void draw() const {
		btVector3 wheelColor(1,0,0);

		for(unsigned int i=0; i<m_vehicle->getNumWheels(); i++) {
			btTransform tr = m_vehicle->getWheelInfo(i).m_worldTransform;

			btQuaternion q = tr.getRotation();

			q *= btQuaternion(0.5,0.5,-0.5,0.5);

			tr.setRotation(q);

			debugDrawer->drawCylinder(wheelRadius, wheelWidth / 2, upIndex, tr, wheelColor);
		}

		m_dynamicsWorld->debugDrawWorld();

		//randomSteer(State(packCurrentVehicle()), 0.1);
	}

	void drawSolution(const std::vector<const Edge *> &solution, double dt = std::numeric_limits<double>::infinity()) const {

	}

	void animateSolution(const std::vector<const Edge *> &solution, unsigned int poseNumber) const {
		unsigned int edgeNumber = poseNumber / 2;
		unsigned int endpoint = poseNumber % 2;
		const Edge *edge = solution[edgeNumber];
		if(endpoint == 0)
			unpackCurrentVehicle(edge->start.getStateVars());
		else
			unpackCurrentVehicle(edge->end.getStateVars());

		draw();
	}
#endif

// private:
	std::vector<double> packCurrentVehicle() const {
		std::vector<double> state(STATE_VAR_COUNT);

		int i = 0;
		btTransform transform = m_carChassis->getCenterOfMassTransform();

		btVector3 translation = transform.getOrigin();
		state[i++] = translation.x();
		state[i++] = translation.y();
		state[i++] = translation.z();

		btQuaternion rotation = transform.getRotation();
		state[i++] = rotation.x();
		state[i++] = rotation.y();
		state[i++] = rotation.z();
		state[i++] = rotation.w();

		// m_carChassis->getMotionState()->getWorldTransform(transform);
		// translation = transform.getOrigin();
		// state[i++] = translation.x();
		// state[i++] = translation.y();
		// state[i++] = translation.z();

		// rotation = transform.getRotation();
		// state[i++] = rotation.x();
		// state[i++] = rotation.y();
		// state[i++] = rotation.z();
		// state[i++] = rotation.w();

		btVector3 linearVelocity = m_carChassis->getLinearVelocity();
		state[i++] = linearVelocity.x();
		state[i++] = linearVelocity.y();
		state[i++] = linearVelocity.z();

		btVector3 angularVelocity = m_carChassis->getAngularVelocity();
		state[i++] = angularVelocity.x();
		state[i++] = angularVelocity.y();
		state[i++] = angularVelocity.z();


		for(unsigned int j = 0; j < 4; ++j) {
			const btWheelInfo &info = m_vehicle->getWheelInfo(i);
			state[i++] = info.m_steering;
			state[i++] = info.m_suspensionRelativeVelocity;
			state[i++] = info.m_wheelsSuspensionForce;
			state[i++] = info.m_rotation;
		}

		return state;
	}

	void unpackCurrentVehicle(const StateVars &vars) const {
		int i = 0;
		//Update the vehicle to match the state
		btVector3 COMtranslation(vars[i++], vars[i++], vars[i++]);
		btQuaternion COMrotation(vars[i++], vars[i++], vars[i++], vars[i++]);
		btTransform COMtransform(COMrotation, COMtranslation);
		
		m_carChassis->setCenterOfMassTransform(COMtransform);

		// btVector3 Wtranslation(vars[i++], vars[i++], vars[i++]);
		// btQuaternion Wrotation(vars[i++], vars[i++], vars[i++], vars[i++]);
		// btTransform Wtransform(Wrotation, Wtranslation);

		m_carChassis->getMotionState()->setWorldTransform(COMtransform);

		btVector3 linearVelocity(vars[i++], vars[i++], vars[i++]);
		btVector3 angularVelocity(vars[i++], vars[i++], vars[i++]);

		m_carChassis->setLinearVelocity(linearVelocity);
		m_carChassis->setAngularVelocity(angularVelocity);

		m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(m_carChassis->getBroadphaseHandle(),m_dynamicsWorld->getDispatcher());

		for(unsigned int j = 0; j < 4; ++j) {
			btWheelInfo &info = m_vehicle->getWheelInfo(j);
			info.m_steering = vars[i++];
			info.m_suspensionRelativeVelocity = vars[i++];
			info.m_wheelsSuspensionForce = vars[i++];
			info.m_rotation = vars[i++];
			m_vehicle->updateWheelTransform(j, true);
		}
	}

	void step(double gEngineForce, double gBreakingForce, double gVehicleSteering, double dt) const {
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

	float maxEngineForce = 1000.f;//this should be engine/velocity dependent
	float maxBreakingForce = 100.f;

	float steeringIncrement = 0.04f;
	float steeringClamp = 0.3f;
	float wheelRadius = 0.5f;
	float wheelWidth = 0.4f;
	float wheelFriction = 1000;//BT_LARGE_FLOAT;
	float suspensionStiffness = 20.f;
	float suspensionDamping = 2.3f;
	float suspensionCompression = 4.4f;
	float rollInfluence = 0.1f;//1.0f;

	btScalar suspensionRestLength;

	//y-axis is up
	int rightIndex = 0;
	int upIndex = 1;
	int forwardIndex = 2;

	WorkspaceBounds bounds;
	std::vector<float> smallHeightField;
	std::vector<double> goalThresholds;
	mutable std::uniform_real_distribution<double> distribution;
	mutable std::default_random_engine generator;

#ifdef WITHGRAPHICS
	GLDebugDrawer *debugDrawer;
#endif
};