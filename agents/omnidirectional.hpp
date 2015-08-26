#pragma once

#include <cmath>
#include <stdlib.h>

#include "abstracttransformstate.hpp"
#include "../utilities/fcl_helpers.hpp"
#include "../utilities/openglwrapper.hpp"
#include "../utilities/instancefilemap.hpp"

class Omnidirectional {
public:
	typedef std::vector< std::pair<double, double> > WorkspaceBounds;
	typedef std::vector< std::pair<double, double> > StateVarRanges;

	typedef std::vector<double> StateVars;

	class State {
	public:
		State() : stateVars(3) {
			stateVars[0] = 0;
			stateVars[1] = 0;
			stateVars[2] = 0;
		}
		State(double x, double y, double z = 0) : stateVars(3) {
			stateVars[0] = x;
			stateVars[1] = y;
			stateVars[2] = z;
		}
		State(const State &s) : stateVars(s.stateVars.begin(), s.stateVars.end()) {}

		State(const StateVars &vars) : stateVars(vars.begin(), vars.begin()+3) {}

		const bool equals(const State &s) const {
			return fabs(stateVars[0] - s.stateVars[0]) <= 0.000001 &&
			       fabs(stateVars[1] - s.stateVars[1]) <= 0.000001 &&
			       fabs(stateVars[2] - s.stateVars[2]) <= 0.000001;
		}

		double x() const {
			return stateVars[0];
		}
		double y() const {
			return stateVars[1];
		}
		double z() const {
			return stateVars[2];
		}
		const StateVars &getStateVars() const {
			return stateVars;
		}

		void print() const {
			for(auto v : stateVars) {
				fprintf(stderr, "%g ", v);
			}
			fprintf(stderr, "\n");
		}

#ifdef WITHGRAPHICS
		void draw(const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {
			const auto &identity = OpenGLWrapper::getOpenGLWrapper().getIdentity();

			std::vector<double> pt(stateVars.begin(), stateVars.end());
			pt.push_back(1);
			pt.push_back(0);
			pt.push_back(0);
			pt.push_back(1);
			pt.push_back(1);
			pt.insert(pt.end(), color.getColor().begin(), color.getColor().end());
			pt.insert(pt.end(), identity.begin(), identity.end());

			OpenGLWrapper::getOpenGLWrapper().drawPoints(pt);
		}
#endif

	private:
		StateVars stateVars;
	};

	class Edge {
	public:
		Edge(const State &start) : start(start), end(start), cost(0), treeIndex(0) {}
		Edge(const State &start, const State &end, double cost) : start(start), end(end), cost(cost), treeIndex(0) {}
		Edge(const Edge &e) : start(e.start), end(e.end), cost(e.cost), treeIndex(e.treeIndex) {}

		/* needed for being inserted into NN datastructure */
		const StateVars &getTreeStateVars() const {
			return end.getStateVars();
		}
		int getPointIndex() const {
			return treeIndex;
		}
		void setPointIndex(int ptInd) {
			treeIndex = ptInd;
		}

#ifdef WITHGRAPHICS
		void draw(const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {
			const auto &identity = OpenGLWrapper::getOpenGLWrapper().getIdentity();
			std::vector<double> line(start.getStateVars().begin(), start.getStateVars().end());
			line.push_back(1);
			line.push_back(0);
			line.push_back(0);
			line.push_back(1);
			line.push_back(1);
			line.insert(line.end(), color.getColor().begin(), color.getColor().end());
			line.insert(line.end(), identity.begin(), identity.end());

			line.insert(line.end(), end.getStateVars().begin(), end.getStateVars().end());
			line.push_back(1);
			line.push_back(0);
			line.push_back(0);
			line.push_back(1);
			line.push_back(1);
			line.insert(line.end(), color.getColor().begin(), color.getColor().end());
			line.insert(line.end(), identity.begin(), identity.end());

			OpenGLWrapper::getOpenGLWrapper().drawLines(line);
		}
#endif

		const State start, end;
		double cost;
		int treeIndex;
	};

	Omnidirectional(const InstanceFileMap &args) :
		mesh(args.value("Agent Mesh")) {

		boost::char_separator<char> sep(" ");
		boost::tokenizer< boost::char_separator<char> > tokens(args.value("Goal Thresholds"), sep);
		for(auto token : tokens) {
			goalThresholds.push_back(std::stod(token));
		}
	}

	unsigned int getTreeStateSize() const {
		return 3;
	}

	StateVarRanges getStateVarRanges(const WorkspaceBounds &bounds) const {
		return bounds;
	}

	State buildState(const StateVars &stateVars) const {
		return State(stateVars);
	}

	bool isGoal(const State &state, const State &goal) const {
		return fabs(state.x() - goal.x()) < goalThresholds[0] &&
		       fabs(state.y() - goal.y()) < goalThresholds[1] &&
		       fabs(state.z() - goal.z()) < goalThresholds[2];
	}

	Edge steer(const State &start, const State &goal, double dt) const {
		double startX = start.x();
		double startY = start.y();
		double startZ = start.z();

		double dx = goal.x() - startX;
		double dy = goal.y() - startY;
		double dz = goal.z() - startZ;

		double dist = sqrt(dx*dx + dy*dy + dz*dz);
		double fraction = dt / dist;
		if(fraction > 1) fraction = 1;


		State state(startX + dx * fraction,
		            startY + dy * fraction,
		            startZ + dz * fraction);

		return Edge(start, state, dt);
	}

	Edge randomSteer(const State &start, double dt) const {
		double startX = start.x();
		double startY = start.y();
		double startZ = start.z();

		double randX = (double)rand() / (double)RAND_MAX;
		double randY = (double)rand() / (double)RAND_MAX;
		double randZ = (double)rand() / (double)RAND_MAX;

		double dist = sqrt(randX*randX + randY*randY + randZ*randZ);

		State state(startX + randX / dist,
		            startY + randY / dist,
		            startZ + randZ / dist);

		return Edge(start, state, dt);
	}

	std::vector<const SimpleAgentMeshHandler *> getMeshes() const {
		std::vector<const SimpleAgentMeshHandler *> meshes(1, &mesh);
		return meshes;
	}

	std::vector<State> getRepresentiveStatesForLocation(const std::vector<double> &loc) const {
		std::vector<State> states;

		// fcl::Vec3f pose(loc[0], loc[1], loc[2]);

		// retPoses.emplace_back();
		// retPoses.back().push_back(fcl::Transform3f(pose));

		return states;
	}

	std::vector<std::vector<fcl::Transform3f> > getPoses(const Edge &edge, double dt) const {
		std::vector<std::vector<fcl::Transform3f> > retPoses;

		double startX = edge.start.x();
		double startY = edge.start.y();
		double startZ = edge.start.z();

		double endX = edge.end.x();
		double endY = edge.end.y();
		double endZ = edge.end.z();

		double dx = endX - startX;
		double dy = endY - startY;
		double dz = endZ - startZ;

		double dist = sqrt(dx*dx + dy*dy + dz*dz);
		unsigned int iterations = dist / dt;

		if(iterations < 1) {
			fcl::Vec3f start(startX, startY, startZ);
			fcl::Vec3f end(endX, endY, endZ);
			retPoses.emplace_back();
			retPoses.back().push_back(fcl::Transform3f(start));
			retPoses.emplace_back();
			retPoses.back().push_back(fcl::Transform3f(end));
		} else {
			double step = dt / dist;

			for(unsigned int i = 0; i < iterations; ++i) {
				double stepSize = step * (double)i;

				fcl::Vec3f translation(startX + stepSize * dx,
				                       startY + stepSize * dy,
				                       startZ + stepSize * dz);
				retPoses.emplace_back();
				retPoses.back().push_back(fcl::Transform3f(translation));
			}
			if((double)iterations * dt < dist) {
				fcl::Vec3f end(endX, endY, endZ);
				retPoses.emplace_back();
				retPoses.back().push_back(fcl::Transform3f(end));
			}
		}

		return retPoses;
	}

#ifdef WITHGRAPHICS
	void draw() const {
		mesh.draw();
	}

	void drawSolution(const std::vector<const Edge *> &solution, double dt = std::numeric_limits<double>::infinity()) const {
		for(const Edge *edge : solution) {
			std::vector< std::vector<fcl::Transform3f> > poses = getPoses(*edge, dt);
			auto transform = OpenGLWrapper::getOpenGLWrapper().getIdentity();
			for(auto pose : poses) {
				const Vec3f &translation = pose[0].getTranslation();
				transform[12] = translation[0];
				transform[13] = translation[1];
				transform[14] = translation[2];

				mesh.draw(OpenGLWrapper::Color(), transform);
			}
		}
	}

	void animateSolution(const std::vector<const Edge *> &solution, unsigned int poseNumber) const {
		auto transform = OpenGLWrapper::getOpenGLWrapper().getIdentity();
		unsigned int edgeNumber = poseNumber / 2;
		unsigned int endpoint = poseNumber % 2;
		const Edge *edge = solution[edgeNumber];
		std::vector< std::vector<fcl::Transform3f> > poses = getPoses(*edge, std::numeric_limits<double>::infinity());
		const Vec3f &translation = poses[endpoint][0].getTranslation();
		transform[12] = translation[0];
		transform[13] = translation[1];
		transform[14] = translation[2];

		mesh.draw(OpenGLWrapper::Color(), transform);
	}
#endif

private:
	SimpleAgentMeshHandler mesh;
	std::vector<double> goalThresholds;
};