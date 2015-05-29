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
		State() : stateVars(5 + trailerCount) {}
		
		State(const State &s) : stateVars(s.stateVars.begin(), s.stateVars.end()) {}

		State(const StateVars &vars) : stateVars(vars.begin(), vars.end()) {
			stateVars.resize(5 + trailerCount);
		}

		State& operator=(const State &s) {
			stateVars.resize(5 + trailerCount);
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

			std::vector<double> pt(stateVars.begin(), stateVars.begin() + 2);
			pt.push_back(0);
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

		static unsigned int trailerCount;

	private:
		StateVars stateVars;
	};

	class Edge {
	public:
		Edge(const State &start) : start(start), end(start), cost(0), dt(0), a(0), w(0), treeIndex(0) {
			populateTreeStateVars();
		}

		Edge(const State &start, const State &end, double cost, double a, double w) : start(start), end(end),
			cost(cost), dt(cost), a(a), w(w), treeIndex(0) {
			populateTreeStateVars();
		}

		Edge(const Edge& e) : start(e.start), end(e.end), cost(e.cost), dt(e.dt), a(e.a), w(e.w), treeIndex(e.treeIndex) {
			populateTreeStateVars();
		}

		/* needed for being inserted into NN datastructure */
		const StateVars& getTreeStateVars() const { return treeStateVars; }
		int getPointIndex() const { return treeIndex; }
		void setPointIndex(int ptInd) { treeIndex = ptInd; }

#ifdef WITHGRAPHICS
		void draw(const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {
			const auto &identity = OpenGLWrapper::getOpenGLWrapper().getIdentity();
			std::vector<double> line(start.getStateVars().begin(), start.getStateVars().begin() + 2);
			line.push_back(0);
			line.push_back(1);
			line.push_back(0);
			line.push_back(0);
			line.push_back(1);
			line.push_back(1);
			line.insert(line.end(), color.getColor().begin(), color.getColor().end());
			line.insert(line.end(), identity.begin(), identity.end());

			line.insert(line.end(), end.getStateVars().begin(), end.getStateVars().begin() + 2);
			line.push_back(0);
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
		double cost, dt, a, w;
		int treeIndex;
		StateVars treeStateVars;

		private:
			void populateTreeStateVars() {
			const StateVars& stateVars = end.getStateVars();
			treeStateVars.insert(treeStateVars.end(), stateVars.begin(), stateVars.end());
		}
	};

	SnakeTrailers(const InstanceFileMap &args) : mesh(args.value("Agent Mesh")), linearAccelerations(-0.1, 1), angularAccelerations(-M_PI / 18., M_PI / 18.) {

		trailerCount = State::trailerCount = stoi(args.value("Trailer Count"));
		trailerLength = stod(args.value("Trailer Length"));
		hitchLength = stod(args.value("Hitch Length"));
		
		minimumVelocity = stod(args.value("Minimum Velocity"));
		maximumVelocity = stod(args.value("Maximum Velocity"));
		
		minimumTurning = stod(args.value("Minimum Turning"));
		maximumTurning = stod(args.value("Maximum Turning"));

		boost::char_separator<char> sep(" ");
		boost::tokenizer< boost::char_separator<char> > tokens(args.value("Goal Thresholds"), sep);
		for(auto token : tokens) {
			goalThresholds.push_back(std::stod(token));
		}

#ifdef WITHGRAPHICS
		//make sure that the state gets populated AFTER trailercount is set
		state = State();

		OpenGLWrapper::setExternalKeyboardCallback([&](int key){
			this->keyboard(key);
		});
#endif
	}

	StateVarRanges getStateVarRanges(const WorkspaceBounds& b) const {
		StateVarRanges bounds(b.begin(), b.begin() + 2);
		bounds.emplace_back(minimumVelocity, maximumVelocity);
		bounds.emplace_back(minimumTurning, maximumTurning);
		for(unsigned int i = 0; i < trailerCount+1; ++i) {
			bounds.emplace_back(0, 2*M_PI);
		}

		return bounds;
	}

	unsigned int getTreeStateSize() const {
		return 5 + trailerCount;
	}

	State buildState(const StateVars& stateVars) const {
		return State(stateVars);
	}

	bool isGoal(const State &state, const State &goal) const {
		const StateVars &s = state.getStateVars();
		const StateVars &g = goal.getStateVars();

		return fabs(s[X] - g[X]) < goalThresholds[X] &&
		       fabs(s[Y] - g[Y]) < goalThresholds[Y];
	}

	Edge steer(const State &start, const State &goal, double dt) const {
		double a = linearAccelerations(generator);
		double w = angularAccelerations(generator);

		State end = doStep(start, a, w, dt);

		return Edge(start, end, dt, a, w);
	}

	Edge randomSteer(const State &start, double dt) const {
		double a = linearAccelerations(generator);
		double w = angularAccelerations(generator);

		State end = doStep(start, a, w, dt);

		return Edge(start, end, dt, a, w);
	}

	const std::vector<const SimpleAgentMeshHandler*> getMeshes() const {
		std::vector<const SimpleAgentMeshHandler*> meshes(trailerCount + 1, &mesh);
		return meshes;
	}

	std::vector< std::vector<fcl::Transform3f> > getRepresentivePosesForLocation(const std::vector<double> &loc) const {
		std::vector<std::vector<fcl::Transform3f> > retPoses;

		fcl::Vec3f pose(loc[0], loc[1], loc[2]);

		unsigned int rotations = 4;
		double increment = M_PI / ((double)rotations * 2.);
		fcl::Matrix3f rotation;
		rotation.setIdentity();

		for(unsigned int i = 0; i < rotations; ++i) {
			double cosVal = cos((double)i * increment);
			double sinVal = sin((double)i * increment);

			rotation(0,0) = cosVal;
			rotation(1,0) = -sinVal;
			rotation(0,1) = sinVal;
			rotation(1,1) = cosVal;

			retPoses.emplace_back();
			retPoses.back().emplace_back(rotation, pose);
		}

		return retPoses;
	}

	std::vector< std::vector<fcl::Transform3f> > getPoses(const Edge &edge, double dt) const {
		std::vector< std::vector<fcl::Transform3f> > poses;

		unsigned int steps = edge.dt / dt;
		if(steps == 0) {
			steps = 1;
		}
		
		State state = edge.start;

		for(unsigned int step = 0; step < steps; ++step) {
			const std::vector<fcl::Transform3f> transforms = stateToFCLTransforms(state);

			poses.emplace_back();
			for(const fcl::Transform3f &transform : transforms) {
				poses.back().push_back(transform);
			}

			state = doStep(state, edge.a, edge.w, dt);
		}

		return poses;
	}

#ifdef WITHGRAPHICS
	void keyboard(int key) {
		double a = 0, w = 0, dt = 0.1;

		switch(key) {
			case 'Q': a = 1; break;
			case 'W': w = 0.1; break;
			case 'E': a = -1; break;
			case 'R': w = -0.1; break;
		}

		const StateVars& vars = state.getStateVars();
		StateVars newState(5 + trailerCount);

		newState[X] = vars[X];
		newState[Y] = vars[Y];
		newState[THETA] = vars[THETA];

		newState[V] = vars[V] + a * dt;
		newState[PSI] = vars[PSI] + w * dt;

		for(unsigned int i = 1; i < trailerCount+1; ++i) {
			newState[THETA + i] = vars[THETA + i];
		}

		state = State(newState);
	}

	void draw() const {
		double a = 0, w = 0, dt = 0.1;

		state = doStep(state, a, w, dt);
		const StateVars &vars = state.getStateVars();
		auto transforms = stateToOpenGLTransforms(state);

		for(const std::vector<double> &transform : transforms) {
			mesh.draw(color, transform);
		}
	}

	void drawSolution(const std::vector<const Edge*> &solution, double dt = std::numeric_limits<double>::infinity()) const {
		for(const Edge* edge : solution) {
			unsigned int steps = std::isinf(dt) ? 1 : edge->dt / dt;
		
			State state = edge->start;

			for(unsigned int step = 0; step < steps; ++step) {

				auto transforms = stateToOpenGLTransforms(state);
				for(const std::vector<double> &transform : transforms) {
					mesh.draw(color, transform);
				}

				state = doStep(state, edge->a, edge->w, dt);
			}
		}
	}

	void animateSolution(const std::vector<const Edge*> &solution, unsigned int poseNumber) const {
		unsigned int edgeNumber = poseNumber / 2;
		unsigned int endpoint = poseNumber % 2;
		const Edge *edge = solution[edgeNumber];

		auto transforms = stateToOpenGLTransforms(endpoint == 0 ? edge->start : edge->end);
		for(const std::vector<double> &transform : transforms) {
			mesh.draw(color, transform);
		}
	}
#endif

private:
	State doStep(const State& s, double a, double w, double dt) const {
		const StateVars& vars = s.getStateVars();
		StateVars newState(5 + trailerCount);

		newState[X] = vars[X] + cos(vars[THETA]) * vars[V] * dt;
		newState[Y] = vars[Y] + sin(vars[THETA]) * vars[V] * dt;
		newState[THETA] = normalizeTheta(vars[THETA] + vars[V] * tan(vars[PSI]) / trailerLength * dt);
		newState[V] = vars[V] + a * dt;
		newState[PSI] = vars[PSI] + w * dt;


		if(newState[V] > maximumVelocity) { newState[V] = maximumVelocity; }
		else if(newState[V] < minimumVelocity ) { newState[V] = minimumVelocity; }

		if(newState[PSI] > maximumTurning) { newState[PSI] = maximumTurning; }
		else if(newState[PSI] < minimumTurning) { newState[PSI] = minimumTurning; }


		double coeff = vars[V] / (trailerLength + hitchLength);
		double prev = vars[THETA];

		for(unsigned int i = 1; i < trailerCount+1; ++i) {
			newState[THETA + i] = normalizeTheta(vars[THETA + i] + coeff * sin(prev - vars[THETA + i]) * dt);
			coeff *= cos(prev - vars[THETA + i]);
			prev = vars[THETA + i];
		}

		return State(newState);
	}

	std::vector< std::vector<double> > stateToOpenGLTransforms(const State& s) const {
		std::vector< std::vector<double> > transforms;
		transforms.push_back(OpenGLWrapper::getOpenGLWrapper().getIdentity());

		const StateVars &vars = s.getStateVars();

		double sinVal = sin(vars[THETA]);
		double cosVal = cos(vars[THETA]);

		transforms.back()[0] = cosVal;
		transforms.back()[1] = sinVal;
		transforms.back()[4] = -sinVal;
		transforms.back()[5] = cosVal;

		transforms.back()[12] = vars[X];
		transforms.back()[13] = vars[Y];

		for(unsigned int i = 1; i < trailerCount + 1; ++i) {
			
			transforms.push_back(OpenGLWrapper::getOpenGLWrapper().getIdentity());

			std::vector<double> &previous = transforms[transforms.size() - 2];

			transforms.back()[12] = -(trailerLength + hitchLength);

			double t = vars[THETA + i] - vars[THETA + i - 1];

			double sinVal = sin(t);
			double cosVal = cos(t);

			transforms.back()[0] = cosVal;
			transforms.back()[1] = sinVal;
			transforms.back()[4] = -sinVal;
			transforms.back()[5] = cosVal;

			multiply(transforms.back(), previous, transforms.back());
		}
		return transforms;
	}

	std::vector<fcl::Transform3f> stateToFCLTransforms(const State& s) const {
		std::vector<fcl::Transform3f> transforms;

		const StateVars &vars = s.getStateVars();

		fcl::Vec3f pose;
		pose[0] = vars[X];
		pose[1] = vars[Y];
		pose[2] = 0;

		fcl::Matrix3f rotation;
		rotation.setIdentity();

		double sinVal = sin(vars[THETA]);
		double cosVal = cos(vars[THETA]);

		rotation(0,0) = cosVal;
		rotation(1,0) = -sinVal;
		rotation(0,1) = sinVal;
		rotation(1,1) = cosVal;

		transforms.emplace_back(rotation, pose);

		for(unsigned int i = 1; i < trailerCount + 1; ++i) {
			
			std::vector<double> transform2 = OpenGLWrapper::getOpenGLWrapper().getIdentity();

			pose[0] = -(trailerLength + hitchLength);

			double t = vars[THETA + i] - vars[THETA + i - 1];

			sinVal = sin(t);
			cosVal = cos(t);

			fcl::Matrix3f rotation2;
			rotation2.setIdentity();

			rotation(0,0) = cosVal;
			rotation(1,0) = -sinVal;
			rotation(0,1) = sinVal;
			rotation(1,1) = cosVal;

			rotation = rotation * rotation2;

			transforms.emplace_back(rotation, pose);
		}

		return transforms;
	}

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

	double normalizeTheta(double t) const {
		return (t - 2 * M_PI * std::floor((t + M_PI) / (2 * M_PI)));
	}

	SimpleAgentMeshHandler mesh;

	unsigned int trailerCount;
	double trailerLength, hitchLength, minimumVelocity, maximumVelocity, minimumTurning, maximumTurning;
	mutable std::uniform_real_distribution<double> linearAccelerations, angularAccelerations;
	mutable std::default_random_engine generator;

	std::vector<double> goalThresholds;

#ifdef WITHGRAPHICS
	const OpenGLWrapper::Color color;
	mutable State state;
#endif
};

unsigned int SnakeTrailers::State::trailerCount = 0;