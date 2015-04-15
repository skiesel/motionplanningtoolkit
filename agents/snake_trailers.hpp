#pragma once

#include <cmath>
#include <stdlib.h>
#include <random>

#include "../utilities/fcl_helpers.hpp"
#include "../utilities/openglwrapper.hpp"
#include "../utilities/instancefilemap.hpp"

class SnakeTrailers {

	enum LABELS {
		X = 0,
		Y = 1,
		V = 2,
		PSI = 3,
		THETA = 4,
	};

public:
	typedef std::vector< std::pair<double, double> > WorkspaceBounds;
	typedef std::vector< std::pair<double, double> > StateVarRanges;

	typedef std::vector<double> StateVars;

	class State {
	public:
		State() {}
		
		State(const State &s) : stateVars(s.stateVars.begin(), s.stateVars.end()) {}

		State(const StateVars &vars) : stateVars(vars.begin(), vars.begin() + 5 + trailerCount) {}

		const bool equals(const State &s) const { return false; }

		const StateVars& getStateVars() const { return stateVars; }

#ifdef WITHGRAPHICS
		void draw(const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {}
#endif

		static unsigned int trailerCount;

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

	SnakeTrailers(const InstanceFileMap &args) : mesh(""), linearAccelerations(-1, 1), angularAccelerations(-0.1745, 0.1745) {
		trailerCount = State::trailerCount = stoi(args.value("Trailer Count"));
		trailerLength = stod(args.value("Trailer Length"));
		hitchLength = stod(args.value("Hitch Length"));
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

		const StateVars& vars = start.getStateVars();
		StateVars newState(5 + trailerCount);

		newState[X] = vars[X] + cos(vars[THETA]) * vars[V] * dt;
		newState[Y] = vars[Y] + sin(vars[THETA]) * vars[V] * dt;
		newState[THETA] = vars[THETA] + vars[V] * tan(vars[PSI]) / trailerLength;
		newState[V] = vars[V] + a * dt;
		newState[PSI] = vars[PSI] + w * dt;

		double coeff = vars[V] / (trailerLength + hitchLength);
		double prev = vars[THETA];

		for(unsigned int i = 1; i < trailerCount+1; ++i) {
			newState[THETA + i] = vars[THETA + i] + coeff * sin(prev - vars[THETA + i]);
			coeff *= cos(prev - vars[THETA + i]);
			prev = vars[THETA + i];
		}

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

	unsigned int trailerCount;
	double trailerLength, hitchLength;
	mutable std::uniform_real_distribution<double> linearAccelerations, angularAccelerations;
	mutable std::default_random_engine generator;
};