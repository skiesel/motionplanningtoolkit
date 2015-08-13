#pragma once

#include <vector>

#include "../utilities/meshhandler.hpp"
#include "../utilities/instancefilemap.hpp"

template<class Agent>
class Map3D {
public:
	typedef std::vector< std::pair<double, double> > WorkspaceBounds;

	typedef typename Agent::Edge Edge;
	typedef typename Agent::State State;

	Map3D(const InstanceFileMap &args, const fcl::Transform3f &transform = fcl::Transform3f()) :
		mesh(args.value("Environment Mesh"), args.value("Environment Location")), bounds(3) {
		boost::char_separator<char> sep(" ");
		boost::tokenizer< boost::char_separator<char> > tokens(args.value("Environment Bounding Box"), sep);
		auto token = tokens.begin();

		bounds[0].first = stod(*token);
		bounds[0].second = stod(*++token);
		bounds[1].first = stod(*++token);
		bounds[1].second = stod(*++token);
		bounds[2].first = stod(*++token);
		bounds[2].second = stod(*++token);
	}

	const WorkspaceBounds& getBounds() const {
		return bounds;
	}

	bool safeEdge(const Agent &agent, const Edge &edge, double dt, bool checkSelfCollision = false) const {
		auto agentMeshes = agent.getMeshes();
		auto agentPoses = agent.getPoses(edge, dt);

		for(const auto &poses : agentPoses) {
			for(const auto &pose : poses) {
				const auto &position = pose.getTranslation();
				for(unsigned int i = 0; i < 3; ++i) {
					if(position[i] < bounds[i].first || position[i] > bounds[i].second) {
						return false;
					}
				}
			}
		}

		return !MeshHandler::isInCollision(mesh, agentMeshes, agentPoses, checkSelfCollision);;
	}

	bool safePoses(const Agent &agent, const std::vector<fcl::Transform3f> &poses, const State &state=State()) const {
		auto agentMeshes = agent.getMeshes();

		std::vector<std::vector<fcl::Transform3f>> wrapper(poses.size());
		for(unsigned int i = 0; i < poses.size(); ++i) {
			wrapper[i].emplace_back(poses[i]);
		}

		bool safe = !MeshHandler::isInCollision(mesh, agentMeshes, wrapper);

		if(!safe) {
			fprintf(stderr, "collision\n");
		}

		return safe;
	}

	bool safePose(const Agent &agent, const fcl::Transform3f &pose, const State &state=State()) const {
		auto agentMeshes = agent.getMeshes();
		std::vector<std::vector<fcl::Transform3f>> poseWrapper(1);
		poseWrapper.back().push_back(pose);
		return !MeshHandler::isInCollision(mesh, agentMeshes, poseWrapper);
	}

#ifdef WITHGRAPHICS
	void draw() const {
		drawBoundingBox();
		mesh.draw();
	}

	void drawBoundingBox() const {
		OpenGLWrapper::getOpenGLWrapper().drawLine(bounds[0].first, bounds[1].first, bounds[2].first, bounds[0].second, bounds[1].first, bounds[2].first, OpenGLWrapper::Color::Red());
		OpenGLWrapper::getOpenGLWrapper().drawLine(bounds[0].first, bounds[1].first, bounds[2].first, bounds[0].first, bounds[1].second, bounds[2].first, OpenGLWrapper::Color::Red());
		OpenGLWrapper::getOpenGLWrapper().drawLine(bounds[0].second, bounds[1].first, bounds[2].first, bounds[0].second, bounds[1].second, bounds[2].first, OpenGLWrapper::Color::Red());
		OpenGLWrapper::getOpenGLWrapper().drawLine(bounds[0].first, bounds[1].second, bounds[2].first, bounds[0].second, bounds[1].second, bounds[2].first, OpenGLWrapper::Color::Red());

		OpenGLWrapper::getOpenGLWrapper().drawLine(bounds[0].first, bounds[1].first, bounds[2].second, bounds[0].second, bounds[1].first, bounds[2].second, OpenGLWrapper::Color::Red());
		OpenGLWrapper::getOpenGLWrapper().drawLine(bounds[0].first, bounds[1].first, bounds[2].second, bounds[0].first, bounds[1].second, bounds[2].second, OpenGLWrapper::Color::Red());
		OpenGLWrapper::getOpenGLWrapper().drawLine(bounds[0].second, bounds[1].first, bounds[2].second, bounds[0].second, bounds[1].second, bounds[2].second, OpenGLWrapper::Color::Red());
		OpenGLWrapper::getOpenGLWrapper().drawLine(bounds[0].first, bounds[1].second, bounds[2].second, bounds[0].second, bounds[1].second, bounds[2].second, OpenGLWrapper::Color::Red());

		OpenGLWrapper::getOpenGLWrapper().drawLine(bounds[0].first, bounds[1].first, bounds[2].first, bounds[0].first, bounds[1].first, bounds[2].second, OpenGLWrapper::Color::Red());
		OpenGLWrapper::getOpenGLWrapper().drawLine(bounds[0].first, bounds[1].second, bounds[2].first, bounds[0].first, bounds[1].second, bounds[2].second, OpenGLWrapper::Color::Red());
		OpenGLWrapper::getOpenGLWrapper().drawLine(bounds[0].second, bounds[1].first, bounds[2].first, bounds[0].second, bounds[1].first, bounds[2].second, OpenGLWrapper::Color::Red());
		OpenGLWrapper::getOpenGLWrapper().drawLine(bounds[0].second, bounds[1].second, bounds[2].first, bounds[0].second, bounds[1].second, bounds[2].second, OpenGLWrapper::Color::Red());
	}
#endif

private:

	StaticEnvironmentMeshHandler mesh;
	WorkspaceBounds bounds;
};