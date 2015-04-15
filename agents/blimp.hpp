#pragma once

#include <cmath>
#include <stdlib.h>
#include <random>

#include "../utilities/fcl_helpers.hpp"
#include "../utilities/openglwrapper.hpp"
#include "../utilities/instancefilemap.hpp"

class Blimp {

	enum LABELS {
		X = 0,
		Y = 1,
		Z = 2,
		THETA = 3,
		V = 4,
		PSI = 5,
		VZ = 6,
	};

public:
	typedef std::vector< std::pair<double, double> > WorkspaceBounds;
	typedef std::vector< std::pair<double, double> > StateVarRanges;

	typedef std::vector<double> StateVars;

	class State {
	public:
		State() {}
		
		State(const State &s) : stateVars(s.stateVars.begin(), s.stateVars.end()) {}

		State(const StateVars &vars) : stateVars(vars.begin(), vars.begin()) {}

		const bool equals(const State &s) const { return false; }

		const StateVars& getStateVars() const { return stateVars; }

#ifdef WITHGRAPHICS
		void draw(const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {}
#endif

	private:
		StateVars stateVars;
	};

	class Edge {
	public:
		Edge(const State &start) : start(start), end(start), cost(0), treeIndex(0) {}
		Edge(const State &start, const State &end, double cost) : start(start), end(end), cost(cost), treeIndex(0) {}
		Edge(const Edge& e) : start(e.start), end(e.end), cost(e.cost), treeIndex(e.treeIndex) {}

		/* needed for being inserted into NN datastructure */
		const StateVars& getStateVars() const { return end.getStateVars(); }
		int getPointIndex() const { return treeIndex; }
		void setPointIndex(int ptInd) { treeIndex = ptInd; }

#ifdef WITHGRAPHICS
		void draw(const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {}
#endif

		const State start, end;
		double cost;
		int treeIndex;
	};

	Blimp(const InstanceFileMap &args) : mesh(args.value("Agent Mesh")), linearAccelerations(-1, 1), zLinearAccelerations(-1, 1), angularAccelerations(-0.1745, 0.1745) {
		blimpLength = stod(args.value("Blimp Length"));
	}

	StateVarRanges getStateVarRanges(const WorkspaceBounds& bounds) const {
		return bounds;
	}

	State buildState(const StateVars& stateVars) const {
		return State(stateVars);
	}

	bool isGoal(const State &state, const State &goal) const {
		return false;
	}

	Edge steer(const State &start, const State &goal, double dt) const {
		return randomSteer(start, dt);
	}

	Edge randomSteer(const State &start, double dt) const {
		double a = linearAccelerations(generator);
		double w = angularAccelerations(generator);
		double z = zLinearAccelerations(generator);

		const StateVars& vars = start.getStateVars();
		StateVars newState(7);

		newState[X] = vars[X] + cos(vars[THETA]) * vars[V] * dt;
		newState[Y] = vars[Y] + sin(vars[THETA]) * vars[V] * dt;
		newState[THETA] = vars[THETA] + vars[V] * tan(vars[PSI]) / blimpLength;

		newState[Z] = vars[Z] + vars[VZ] * dt;

		newState[V] = vars[V] + a * dt;
		newState[PSI] = vars[PSI] + w * dt;
		newState[VZ] = vars[VZ] + z * dt;

		return Edge(start, State(newState), dt);
	}

	const SimpleAgentMeshHandler& getMesh() const {
		return mesh;
	}

	std::vector<fcl::Transform3f> getRepresentivePosesForLocation(const std::vector<double> &loc) const {
		std::vector<fcl::Transform3f> poses;
		return poses;
	}

	std::vector<fcl::Transform3f> getPoses(const Edge &edge, double dt) const {
		std::vector<fcl::Transform3f> poses;
		return poses;
	}

#ifdef WITHGRAPHICS
	void draw() const {}

	void drawSolution(const std::vector<const Edge*> &solution, double dt = std::numeric_limits<double>::infinity()) const {}

	void animateSolution(const std::vector<const Edge*> &solution, unsigned int poseNumber) const {}
#endif

private:
	SimpleAgentMeshHandler mesh;
	double blimpLength;
	mutable std::uniform_real_distribution<double> linearAccelerations, zLinearAccelerations, angularAccelerations;
	mutable std::default_random_engine generator;
};