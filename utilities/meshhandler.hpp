#include <fcl/BVH/BVH_model.h>
#include <fcl/broadphase/broadphase.h>

#include "fcl_helpers.hpp"
#include "assimp_mesh_loader.hpp"

typedef fcl::BVHModel<fcl::OBBRSS> Model;

class StaticEnvironmentMeshHandler {
public:
	StaticEnvironmentMeshHandler(const std::string &filename, const std::string &pose) {
		AssimpMeshLoader meshLoader(filename.c_str());

		if(meshLoader.error) {
			exit(0);
		}

		meshLoader.get(vertices, triangles);

		fcl::Transform3f tf = fcl_helpers::parseTransform(pose);

		for(unsigned int i = 0; i < vertices.size(); i++) {
			const std::vector<fcl::Vec3f> &verts = vertices[i];
			const std::vector<fcl::Triangle> &tris = triangles[i];

			if(verts.size() == 0 || tris.size() == 0) continue;

			auto model = boost::shared_ptr<Model>(new Model());
			model->beginModel();
			model->addSubModel(verts, tris);
			model->endModel();

			worldObjects.push_back(new fcl::CollisionObject(model, tf));
		}

		worldCollisionModel = new fcl::DynamicAABBTreeCollisionManager();
		worldCollisionModel->registerObjects(worldObjects);
		worldCollisionModel->setup();
	}

	~StaticEnvironmentMeshHandler() {
		for(unsigned int i = 0; i < worldObjects.size(); ++i)
			delete worldObjects[i];
		delete worldCollisionModel;
	}

	const fcl::BroadPhaseCollisionManager* getCollisionChecker() const {
		return worldCollisionModel;
	}

#ifdef WITHGRAPHICS
	void draw() const {
		const OpenGLWrapper& opengl = OpenGLWrapper::getOpenGLWrapper();

		for(unsigned int i = 0; i < triangles.size(); ++i) {
			const std::vector<fcl::Vec3f>& verts = vertices[i];
			const std::vector<fcl::Triangle>& tris = triangles[i];
			std::vector<double> pts(12, 1.0);
			for(auto tri : tris) {
				
				pts[0] = verts[tri[0]][0];
				pts[1] = verts[tri[0]][1];
				pts[2] = verts[tri[0]][2];
				
				pts[4] = verts[tri[1]][0];
				pts[5] = verts[tri[1]][1];
				pts[6] = verts[tri[1]][2];

				pts[8] = verts[tri[2]][0];
				pts[9] = verts[tri[2]][1];
				pts[10] = verts[tri[2]][2];

				opengl.drawTriangles(pts);
				//opengl.drawPoints(pts);
			}
		}
	}
#endif

private:
	fcl::BroadPhaseCollisionManager *worldCollisionModel;
	std::vector<fcl::CollisionObject *> worldObjects;
	std::vector< std::vector<fcl::Vec3f> > vertices;
	std::vector< std::vector<fcl::Triangle> > triangles;
};

class SimpleAgentMeshHandler {
public:
	SimpleAgentMeshHandler(const std::string &filename) {
		AssimpMeshLoader meshLoader(filename.c_str());

		if(meshLoader.error) {
			exit(0);
		}

		meshLoader.get(vertices, triangles);

		fcl::Transform3f tf;
		for(unsigned int i = 0; i < vertices.size(); i++) {
			const std::vector<fcl::Vec3f> &verts = vertices[i];
			const std::vector<fcl::Triangle> &tris = triangles[i];

			if(verts.size() == 0 || tris.size() == 0) continue;

			agentModel = boost::shared_ptr<Model>(new Model());
			agentModel->beginModel();
			agentModel->addSubModel(verts, tris);
			agentModel->endModel();
		}
	}

	fcl::CollisionObject* getMeshPose(const fcl::Transform3f &tf) const {
		return new fcl::CollisionObject(agentModel, tf);
	}

#ifdef WITHGRAPHICS
	void draw() const {
		const OpenGLWrapper& opengl = OpenGLWrapper::getOpenGLWrapper();

		for(unsigned int i = 0; i < triangles.size(); ++i) {
			const std::vector<fcl::Vec3f>& verts = vertices[i];
			const std::vector<fcl::Triangle>& tris = triangles[i];
			std::vector<double> pts(12, 1.0);
			for(auto tri : tris) {
				
				pts[0] = verts[tri[0]][0];
				pts[1] = verts[tri[0]][1];
				pts[2] = verts[tri[0]][2];
				
				pts[4] = verts[tri[1]][0];
				pts[5] = verts[tri[1]][1];
				pts[6] = verts[tri[1]][2];

				pts[8] = verts[tri[2]][0];
				pts[9] = verts[tri[2]][1];
				pts[10] = verts[tri[2]][2];

				opengl.drawTriangles(pts);
				//opengl.drawPoints(pts);
			}
		}
	}
#endif

private:
	boost::shared_ptr<Model> agentModel;
	std::vector< std::vector<fcl::Vec3f> > vertices;
	std::vector< std::vector<fcl::Triangle> > triangles;
};

class MeshHandler {
public:
	static bool isInCollision(const StaticEnvironmentMeshHandler &environment,
								const SimpleAgentMeshHandler &agent,
								const std::vector<fcl::Transform3f> &poses,
								bool debug = false) {

		std::vector<fcl::CollisionObject*> agentPoses;
		for(const fcl::Transform3f &pose : poses) {
			fcl::CollisionObject *agentPose = agent.getMeshPose(pose);
			agentPoses.push_back(agentPose);
		}

		fcl_helpers::CollisionData collisionData;

		fcl::BroadPhaseCollisionManager *agentCollisionModel = new fcl::DynamicAABBTreeCollisionManager();
		agentCollisionModel->registerObjects(agentPoses);
		agentCollisionModel->setup();

		auto worldCollisionModel = environment.getCollisionChecker();
		worldCollisionModel->collide(agentCollisionModel, &collisionData, fcl_helpers::defaultCollisionFunction);

		if(debug) {
			for(unsigned int c = 0; c < collisionData.result.numContacts(); ++c) {
				auto vec = collisionData.result.getContact(c).pos;
				fprintf(stderr, "%g %g %g\n", vec[0], vec[1], vec[2]);
			}
		}

		for(unsigned int i = 0; i < agentPoses.size(); ++i) {
			delete agentPoses[i];
		}

		return collisionData.result.numContacts() > 0;
	}
};