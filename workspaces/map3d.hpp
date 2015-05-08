#pragma once

#include <vector>

#include "../utilities/meshhandler.hpp"
#include "../utilities/instancefilemap.hpp"

template<class Agent>
class Map3D {
public:
	typedef std::vector< std::pair<double, double> > WorkspaceBounds;

	typedef typename Agent::Edge Edge;

	Map3D(const InstanceFileMap &args) :
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
		return !MeshHandler::isInCollision(mesh, agentMeshes, agentPoses, checkSelfCollision);
	}

	bool safePoses(const Agent &agent, const std::vector<fcl::Transform3f> &poses) const {
		auto agentMeshes = agent.getMeshes();
		return !MeshHandler::isInCollision(mesh, agentMeshes, poses);
	}

#ifdef WITHGRAPHICS
	void draw() { mesh.draw(); }
#endif

private:
	StaticEnvironmentMeshHandler mesh;
	WorkspaceBounds bounds;
};