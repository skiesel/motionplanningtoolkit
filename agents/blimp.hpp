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
		State() : stateVars(7) {}
		
		State(const State &s) : stateVars(s.stateVars.begin(), s.stateVars.end()) {}

		State(const StateVars &vars) : stateVars(vars.begin(), vars.end()) {}

		State& operator=(const State &s) {
			stateVars.resize(7);
			for(unsigned int i = 0; i < s.stateVars.size(); ++i)
				stateVars[i] = s.stateVars[i];
			return *this;
		}

		const bool equals(const State &s) const {
			for(unsigned int i = 0; i < 3; ++i) {
				if(fabs(stateVars[0] - s.stateVars[0]) > 0.000001) return false;
			}
			return true;
		}

		const StateVars& getStateVars() const { return stateVars; }

		void print() const {
			for(auto v : stateVars) {
				fprintf(stderr, "%g ", v);
			}
			fprintf(stderr, "\n");
		}

#ifdef WITHGRAPHICS
		void draw(const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {
			const auto &identity = OpenGLWrapper::getOpenGLWrapper().getIdentity();

			std::vector<double> pt(stateVars.begin(), stateVars.begin() + 3);
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
		Edge(const State &start) : start(start), end(start), cost(0), dt(0), a(0), w(0), z(0), treeIndex(0) {}
		Edge(const State &start, const State &end, double cost, double a, double w, double z) : start(start),
			end(end), cost(cost), dt(cost), a(a), w(w), z(z), treeIndex(0) {}
		Edge(const Edge& e) : start(e.start), end(e.end), cost(e.cost), dt(e.dt), a(e.a), w(e.w), z(e.z), treeIndex(e.treeIndex) {}

		/* needed for being inserted into NN datastructure */
		const StateVars& getTreeStateVars() const { return end.getStateVars(); }
		int getPointIndex() const { return treeIndex; }
		void setPointIndex(int ptInd) { treeIndex = ptInd; }

#ifdef WITHGRAPHICS
		void draw(const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {
			const auto &identity = OpenGLWrapper::getOpenGLWrapper().getIdentity();
			std::vector<double> line(start.getStateVars().begin(), start.getStateVars().begin() + 3);
			line.push_back(1);
			line.push_back(0);
			line.push_back(0);
			line.push_back(1);
			line.push_back(1);
			line.insert(line.end(), color.getColor().begin(), color.getColor().end());
			line.insert(line.end(), identity.begin(), identity.end());

			line.insert(line.end(), end.getStateVars().begin(), end.getStateVars().begin() + 3);
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
		double cost, dt, a, w, z;
		int treeIndex;
	};

	Blimp(const InstanceFileMap &args) : mesh(args.value("Agent Mesh")), linearAccelerations(-1, 1), zLinearAccelerations(-1, 1), angularAccelerations(-0.1745, 0.1745) {
		blimpLength = stod(args.value("Blimp Length"));
		
		minimumVelocity = stod(args.value("Minimum Velocity"));
		maximumVelocity = stod(args.value("Maximum Velocity"));
		
		minimumTurning = stod(args.value("Minimum Turning"));
		maximumTurning = stod(args.value("Maximum Turning"));

		minimumVelocityZ = stod(args.value("Minimum Velocity Z"));
		maximumVelocityZ = stod(args.value("Maximum Velocity Z"));

		boost::char_separator<char> sep(" ");
		boost::tokenizer< boost::char_separator<char> > tokens(args.value("Goal Thresholds"), sep);
		for(auto token : tokens) {
			goalThresholds.push_back(std::stod(token));
		}

#ifdef WITHGRAPHICS
		OpenGLWrapper::setExternalKeyboardCallback([&](int key){
			this->keyboard(key);
		});
#endif
	}

	unsigned int getTreeStateSize() const {
		return 7;
	}

	StateVarRanges getStateVarRanges(const WorkspaceBounds& b) const {
		StateVarRanges bounds(b.begin(), b.end());
		bounds.emplace_back(0, 2*M_PI);
		bounds.emplace_back(minimumVelocity, maximumVelocity);
		bounds.emplace_back(minimumTurning, maximumTurning);
		bounds.emplace_back(minimumVelocityZ, maximumVelocityZ);

		return bounds;
	}

	State buildState(const StateVars& stateVars) const {
		return State(stateVars);
	}

	bool isGoal(const State &state, const State &goal) const {
		const StateVars &s = state.getStateVars();
		const StateVars &g = goal.getStateVars();

		return fabs(s[X] - g[X]) < goalThresholds[X] &&
		       fabs(s[Y] - g[Y]) < goalThresholds[Y] &&
		       fabs(s[Z] - g[Z]) < goalThresholds[Z];
	}

	Edge steer(const State &start, const State &goal, double dt) const {
		return randomSteer(start, dt);
	}

	Edge randomSteer(const State &start, double dt) const {
		double a = linearAccelerations(generator);
		double w = angularAccelerations(generator);
		double z = zLinearAccelerations(generator);

		State end = doStep(start, a, w, z, dt);

		return Edge(start, end, dt, a, w, z);
	}

	std::vector<const SimpleAgentMeshHandler*> getMeshes() const {
		std::vector<const SimpleAgentMeshHandler*> meshes(1, &mesh);
		return meshes;
	}

	std::vector<fcl::Transform3f> getRepresentivePosesForLocation(const std::vector<double> &loc) const {
		std::vector<fcl::Transform3f> poses;

		fcl::Vec3f pose(loc[0], loc[1], loc[2]);

		unsigned int rotations = 4;
		double increment = M_PI / ((double)rotations * 2.);
		fcl::Matrix3f rotation;
		rotation.setIdentity();

		for(unsigned int i = 0; i < rotations; ++i) {
			double cosTheta = cos((double)i * increment);
			double sinTheta = sin((double)i * increment);

			rotation(0,0) = cosTheta;
			rotation(1,0) = -sinTheta;
			rotation(0,1) = sinTheta;
			rotation(1,1) = cosTheta;

			poses.emplace_back(rotation, pose);
		}

		return poses;
	}

	std::vector<std::vector<fcl::Transform3f> > getPoses(const Edge &edge, double dt) const {
		std::vector<std::vector<fcl::Transform3f> > retPoses(1);
		std::vector<fcl::Transform3f>& poses = retPoses[0];
		return retPoses;
	}

#ifdef WITHGRAPHICS
	void keyboard(int key) {
		double a = 0, w = 0, z = 0, dt = 0.1;

		switch(key) {
			case 'Q':
				a = 1;
				break;
			case 'W':
				w = 1;
				break;
			case 'E':
				z = 1;
				break;
		}

		const StateVars& vars = state.getStateVars();
		StateVars newState(7);

		newState[X] = vars[X];
		newState[Y] = vars[Y];
		newState[THETA] = vars[THETA];

		newState[Z] = vars[Z];

		newState[V] = vars[V] + a * dt;
		newState[PSI] = vars[PSI] + w * dt;
		newState[VZ] = vars[VZ] + z * dt;

		state = State(newState);
	}

	void draw() const {
		double dt = 0.1;

		state = doStep(state, 0, 0, 0, dt);

		auto transform = stateToOpenGLTransform(state);

		mesh.draw(color, transform);
	}

	void drawSolution(const std::vector<const Edge*> &solution, double dt = std::numeric_limits<double>::infinity()) const {
		for(const Edge* edge : solution) {
			unsigned int steps = std::isinf(dt) ? 1 : edge->dt / dt;
		
			State state = edge->start;

			for(unsigned int step = 0; step < steps; ++step) {
				auto transform = stateToOpenGLTransform(state);
				mesh.draw(color, transform);
				state = doStep(state, edge->a, edge->w, edge->z, dt);
			}
		}
	}

	void animateSolution(const std::vector<const Edge*> &solution, unsigned int poseNumber) const {
		unsigned int edgeNumber = poseNumber / 2;
		unsigned int endpoint = poseNumber % 2;
		const Edge *edge = solution[edgeNumber];

		auto transform = stateToOpenGLTransform(endpoint == 0 ? edge->start : edge->end);
		mesh.draw(color, transform);
	}

#endif

private:
	State doStep(const State& s, double a, double w, double z, double dt) const {
		const StateVars& vars = s.getStateVars();
		StateVars newState(7);

		newState[X] = vars[X] + cos(vars[THETA]) * vars[V] * dt;
		newState[Y] = vars[Y] + sin(vars[THETA]) * vars[V] * dt;
		newState[THETA] = normalizeTheta(vars[THETA] + vars[V] * tan(vars[PSI]) / blimpLength);

		newState[Z] = vars[Z] + vars[VZ] * dt;

		newState[V] = vars[V] + a * dt;
		newState[PSI] = vars[PSI] + w * dt;
		newState[VZ] = vars[VZ] + z * dt;

		if(newState[V] > maximumVelocity) { newState[V] = maximumVelocity; }
		else if(newState[V] < minimumVelocity ) { newState[V] = minimumVelocity; }

		if(newState[PSI] > maximumTurning) { newState[PSI] = maximumTurning; }
		else if(newState[PSI] < minimumTurning) { newState[PSI] = minimumTurning; }

		if(newState[VZ] > maximumVelocityZ) { newState[VZ] = maximumVelocityZ; }
		else if(newState[VZ] < minimumVelocityZ ) { newState[VZ] = minimumVelocityZ; }

		return State(newState);
	}

	std::vector<double> stateToOpenGLTransform(const State& s) const {
		std::vector<double> transform = OpenGLWrapper::getOpenGLWrapper().getIdentity();

		const StateVars &vars = s.getStateVars();

		double sinVal = sin(vars[THETA]);
		double cosVal = cos(vars[THETA]);

		transform[0] = cosVal;
		transform[2] = -sinVal;
		transform[8] = sinVal;
		transform[10] = cosVal;

		transform[12] = vars[X];
		transform[13] = -vars[Z];
		transform[14] = vars[Y];

		return transform;
	}

	fcl::Transform3f stateToFCLTransform(const State& s) const {
		const StateVars &vars = s.getStateVars();

		fcl::Vec3f pose(vars[X], vars[Y], vars[Z]);
		
		fcl::Matrix3f rotation;
		rotation.setIdentity();

		double sinVal = sin(vars[THETA]);
		double cosVal = cos(vars[THETA]);

		rotation(0,0) = cosVal;
		rotation(1,0) = -sinVal;
		rotation(0,1) = sinVal;
		rotation(1,1) = cosVal;

		return fcl::Transform3f(rotation, pose);
	}

	double normalizeTheta(double t) const {
		return (t - 2 * M_PI * std::floor((t + M_PI) / (2 * M_PI)));
	}

	SimpleAgentMeshHandler mesh;
	double blimpLength, minimumVelocity, maximumVelocity, minimumTurning, maximumTurning, minimumVelocityZ, maximumVelocityZ;
	mutable std::uniform_real_distribution<double> linearAccelerations, zLinearAccelerations, angularAccelerations;
	mutable std::default_random_engine generator;

	std::vector<double> goalThresholds;

#ifdef WITHGRAPHICS
	const OpenGLWrapper::Color color;
	mutable State state;
#endif
};