#pragma once

#include <bullet/btBulletDynamicsCommon.h>
#include "../utilities/assimp_mesh_loader.hpp"

class BulletInterface {
public:
	BulletInterface(const InstanceFileMap& args) {
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
		

		std::string agentMeshFile = args.value("Agent Mesh");
		AssimpMeshLoader agentMeshLoader(agentMeshFile.c_str());
		std::vector<btTriangleMesh*> agentMeshes;
		agentMeshLoader.get(agentMeshes);

		
		//btTriangleMesh *environmentMesh = new btTriangleMesh();
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