#pragma once

#include <fcl/BVH/BVH_model.h>

#ifdef __APPLE__
//Wow.
#undef nil
#endif
#include <fcl/broadphase/broadphase.h>

#include "fcl_helpers.hpp"
#include "assimp_mesh_loader.hpp"

typedef fcl::BVHModel<fcl::OBBRSS> Model;

class StaticEnvironmentMeshHandler {
public:
	StaticEnvironmentMeshHandler(const std::string &filename, const std::string &pose) : transform(16, 0) {
		AssimpMeshLoader meshLoader(filename.c_str());

		if(meshLoader.error) {
			exit(0);
		}

		meshLoader.get(vertices, triangles, normals);

		fcl::Transform3f tf = fcl_helpers::parseTransform(pose);

		for(unsigned int i = 0; i < 16; ++i) {
			if((i%4) == (i/4)) transform[i] = 1;
		}

		for(unsigned int i = 0; i < vertices.size(); i++) {
			std::vector<fcl::Vec3f> &verts = vertices[i];
			for(unsigned int j = 0; j < verts.size(); ++j) {
				auto v = tf.transform(verts[j]);
				for(unsigned int k = 0; k < 3; ++k) {
					verts[j][k] = v[k];
				}
			}

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

	const fcl::BroadPhaseCollisionManager *getCollisionChecker() const {
		return worldCollisionModel;
	}

#ifdef WITHGRAPHICS
	void draw(const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {
		const OpenGLWrapper &opengl = OpenGLWrapper::getOpenGLWrapper();

		for(unsigned int i = 0; i < triangles.size(); ++i) {
			const std::vector<fcl::Vec3f> &verts = vertices[i];
			const std::vector<fcl::Triangle> &tris = triangles[i];
			std::vector<double> pts(84, 1.0);

			auto c = color.getColor();
			for(auto tri : tris) {
				unsigned int cur = 0;
				for(unsigned int i = 0; i < 3; ++i) {
					for(unsigned int j = 0; j < 3; ++j) {
						pts[cur++] = verts[tri[i]][j];
					}
					cur++; // add one extra for the 4th vector component
					for(unsigned int j = 0; j < 3; ++j) {
						pts[cur++] = normals[tri[i]][j];
					}
					cur++; // add one extra for the 4th vector component
					for(unsigned int j = 0; j < 4; ++j) {
						pts[cur++] = c[j];
					}
					for(unsigned int j = 0; j < 16; ++j) {
						pts[cur++] = transform[j];
					}
				}

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
	std::vector< std::vector<double> > normals;
	std::vector<double> transform;
};

class SimpleAgentMeshHandler {
public:
	SimpleAgentMeshHandler() {}

	SimpleAgentMeshHandler(const std::string &filename) {
		AssimpMeshLoader meshLoader(filename.c_str());

		if(meshLoader.error) {
			exit(0);
		}

		meshLoader.get(vertices, triangles, normals);

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

	virtual fcl::CollisionObject *getMeshPose(const fcl::Transform3f &tf) const {
		return new fcl::CollisionObject(agentModel, tf);
	}

#ifdef WITHGRAPHICS
	virtual void draw(const OpenGLWrapper::Color &color = OpenGLWrapper::Color(),
	                  const std::vector<double> &transform = OpenGLWrapper::getOpenGLWrapper().getIdentity()) const {
		const OpenGLWrapper &opengl = OpenGLWrapper::getOpenGLWrapper();

		for(unsigned int i = 0; i < triangles.size(); ++i) {
			const std::vector<fcl::Vec3f> &verts = vertices[i];
			const std::vector<fcl::Triangle> &tris = triangles[i];
			std::vector<double> pts(84, 1.0);
			auto c = color.getColor();
			for(auto tri : tris) {
				unsigned int cur = 0;
				for(unsigned int i = 0; i < 3; ++i) {
					for(unsigned int j = 0; j < 3; ++j) {
						pts[cur++] = verts[tri[i]][j];
					}
					cur++; // add one extra for the 4th vector component
					for(unsigned int j = 0; j < 3; ++j) {
						pts[cur++] = normals[tri[i]][j];
					}
					cur++; // add one extra for the 4th vector component
					for(unsigned int j = 0; j < 4; ++j) {
						pts[cur++] = c[j];
					}
					for(unsigned int j = 0; j < 16; ++j) {
						pts[cur++] = transform[j];
					}
				}

				opengl.drawTriangles(pts);
				// opengl.drawPoints(pts);
			}
		}
	}
#endif

protected:
	void multiply(const std::vector<double> &m1, const std::vector<double> &m2, std::vector<double> &out) const {
		std::vector<double> temp(16);
		for(unsigned int row = 0; row < 4; ++row) {
			for(unsigned int col = 0; col < 4; ++col) {
				double sum = 0;
				for(unsigned int i = 0; i < 4; i++) {
					double elem1 = m1[row * 4 + i];
					double elem2 = m2[col + 4 * i];
					sum += elem1 * elem2;
				}
				temp[row * 4 + col] = sum;
			}
		}
		for(unsigned int i = 0; i < 16; i++) out[i] = temp[i];
	}


	boost::shared_ptr<Model> agentModel;
	std::vector< std::vector<fcl::Vec3f> > vertices;
	std::vector< std::vector<fcl::Triangle> > triangles;
	std::vector< std::vector<double> > normals;
};

class CylinderHandler : public SimpleAgentMeshHandler {
public:
	CylinderHandler(double radius, double length) {
		cylinder = boost::shared_ptr<Cylinder>(new fcl::Cylinder(radius, length));

		std::vector<double> upNormal(3), downNormal(3);
		upNormal[2] = 1;
		downNormal[2] = -1;

		vertices.emplace_back();
		std::vector<fcl::Vec3f> &verts = vertices.back();

		triangles.emplace_back();
		std::vector<fcl::Triangle> &tris = triangles.back();

		double topZ = length / 2;
		double bottomZ = -length / 2;

		verts.emplace_back(0, 0, topZ);
		normals.push_back(upNormal);
		verts.emplace_back(0, 0, bottomZ);
		normals.push_back(downNormal);

		unsigned int circlePoints = 90;
		double x = radius;
		double y = 0;

		verts.emplace_back(x, y, topZ);
		normals.push_back(upNormal);
		verts.emplace_back(x, y, bottomZ);
		normals.push_back(downNormal);

		for(unsigned int i = 1; i < circlePoints+1; ++i) {
			double rad = (double)(i % circlePoints) / (double)circlePoints * 2 * M_PI;
			x = cos(rad) * radius;
			y = sin(rad) * radius;

			verts.emplace_back(x, y, topZ);
			normals.push_back(upNormal);
			verts.emplace_back(x, y, bottomZ);
			normals.push_back(downNormal);

			tris.emplace_back(0, verts.size()-4, verts.size()-2);
			tris.emplace_back(1, verts.size()-3, verts.size()-1);

			tris.emplace_back(verts.size()-4, verts.size()-2, verts.size()-3);
			tris.emplace_back(verts.size()-3, verts.size()-1, verts.size()-4);
		}

	}

	virtual fcl::CollisionObject *getMeshPose(const fcl::Transform3f &tf) const {
		return new fcl::CollisionObject(cylinder, tf);
	}

private:
	boost::shared_ptr<fcl::Cylinder> cylinder;
};

class ConeHandler : public SimpleAgentMeshHandler {
public:
	ConeHandler(double radius, double length) {
		cone = boost::shared_ptr<fcl::Cone>(new fcl::Cone(radius, length));

		std::vector<double> upNormal(3), downNormal(3);
		upNormal[2] = 1;
		downNormal[2] = -1;

		vertices.emplace_back();
		std::vector<fcl::Vec3f> &verts = vertices.back();

		triangles.emplace_back();
		std::vector<fcl::Triangle> &tris = triangles.back();

		double topZ = length / 2;
		double bottomZ = -length / 2;

		verts.emplace_back(0, 0, topZ);
		normals.push_back(upNormal);
		verts.emplace_back(0, 0, bottomZ);
		normals.push_back(downNormal);

		unsigned int circlePoints = 90;
		double x0 = radius;
		double y0 = 0;
		unsigned int vertNum = 2;
		for(unsigned int i = 1; i < circlePoints+1; ++i) {
			double rad = (double)(i % circlePoints) / (double)circlePoints * 2 * M_PI;
			double x1 = cos(rad) * radius;
			double y1 = sin(rad) * radius;

			verts.emplace_back(x0, y0, bottomZ);
			verts.emplace_back(x1, y1, bottomZ);
			tris.emplace_back(1, vertNum, vertNum+1);
			tris.emplace_back(0, vertNum, vertNum+1);
			normals.push_back(downNormal);
			normals.push_back(downNormal);
			vertNum += 2;
		}
	}

	virtual fcl::CollisionObject *getMeshPose(const fcl::Transform3f &tf) const {
		return new fcl::CollisionObject(cone, tf);
	}

private:
	boost::shared_ptr<fcl::Cone> cone;
};

class CapsuleHandler : public SimpleAgentMeshHandler {
public:
	CapsuleHandler(double radius, double length) {
		capsule = boost::shared_ptr<fcl::Capsule>(new fcl::Capsule(radius, length));
	}

	virtual fcl::CollisionObject *getMeshPose(const fcl::Transform3f &tf) const {
		return new fcl::CollisionObject(capsule, tf);
	}

private:
	boost::shared_ptr<fcl::Capsule> capsule;
};

class BoxHandler : public SimpleAgentMeshHandler {
public:
	BoxHandler(double x, double y, double z) {
		box = boost::shared_ptr<fcl::Box>(new fcl::Box(x, y, z));
	}

	virtual fcl::CollisionObject *getMeshPose(const fcl::Transform3f &tf) const {
		return new fcl::CollisionObject(box, tf);
	}

private:
	boost::shared_ptr<fcl::Box> box;
};

class MeshHandler {
public:
	static bool isInCollision(const StaticEnvironmentMeshHandler &environment,
	                          const std::vector<const SimpleAgentMeshHandler *> &agent,
	                          const std::vector<std::vector<fcl::Transform3f> > &poses,
	                          bool checkSelfCollision = false,
	                          bool debug = false) {

		std::vector<fcl::CollisionObject *> agentPoses;
		for(unsigned int i = 0; i < poses.size(); ++i) {
			const std::vector<fcl::Transform3f> &pose = poses[i];

			std::vector<fcl::CollisionObject *> selfCollisionObjects;

			for(unsigned int j = 0; j < pose.size(); ++j) {
				fcl::CollisionObject *agentPose = agent[j]->getMeshPose(pose[j]);
				agentPoses.push_back(agentPose);
				if(checkSelfCollision) {
					selfCollisionObjects.push_back(agentPose);
				}
			}

			if(checkSelfCollision) {
				fcl::BroadPhaseCollisionManager *selfCollisionManager = new fcl::DynamicAABBTreeCollisionManager();
				selfCollisionManager->registerObjects(selfCollisionObjects);
				selfCollisionManager->setup();

				fcl_helpers::CollisionData selfCollisionData;
				selfCollisionManager->collide(&selfCollisionData, fcl_helpers::defaultCollisionFunction);

				if(selfCollisionData.result.numContacts() > 0) {
					for(unsigned int i = 0; i < agentPoses.size(); ++i) {
						delete agentPoses[i];
					}
					return true;
				}
			}
		}

		fcl_helpers::CollisionData collisionData;

		fcl::BroadPhaseCollisionManager *agentCollisionModel = new fcl::DynamicAABBTreeCollisionManager();
		agentCollisionModel->registerObjects(agentPoses);
		agentCollisionModel->setup();

		auto worldCollisionModel = environment.getCollisionChecker();
		worldCollisionModel->collide(agentCollisionModel, &collisionData, fcl_helpers::defaultCollisionFunction);

		if(debug) {
			fprintf(stderr, "checking %lu meshes\n", agentPoses.size());

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