#pragma once

#include <bullet/btBulletDynamicsCommon.h>

class BulletInterface {
public:
	BulletInterface() {
		broadphase = new btDbvtBroadphase();

		// Set up the collision configuration and dispatcher
		collisionConfiguration = new btDefaultCollisionConfiguration();
		dispatcher = new btCollisionDispatcher(collisionConfiguration);

		// The actual physics solver
		solver = new btSequentialImpulseConstraintSolver();

		// The world.
		dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
		dynamicsWorld->setGravity(btVector3(0, -10, 0));

		groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 1);
		groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
		btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
		groundRigidBody = new btRigidBody(groundRigidBodyCI);
		dynamicsWorld->addRigidBody(groundRigidBody);
	}

	void addEnvironment() {
		btConvexTriangleMeshShape mesh(new btTriangleMesh());
	}

	void addAgent() {
		btConvexTriangleMeshShape mesh(new btTriangleMesh());
	}

	~BulletInterface() {
		dynamicsWorld->removeRigidBody(groundRigidBody);
		delete groundMotionState;
		delete groundRigidBody;
		delete groundShape;
		delete dynamicsWorld;
		delete solver;
		delete dispatcher;
		delete collisionConfiguration;
		delete broadphase;
	}

private:
	btBroadphaseInterface* broadphase;
	btDefaultCollisionConfiguration* collisionConfiguration;
	btCollisionDispatcher* dispatcher;
	btSequentialImpulseConstraintSolver* solver;
	btDiscreteDynamicsWorld* dynamicsWorld;
	btCollisionShape* groundShape;
	btDefaultMotionState* groundMotionState;
	btRigidBody* groundRigidBody;
};