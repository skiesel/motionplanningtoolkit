#pragma once

#include <cmath>
#include <stdlib.h>
#include <random>
#include "abstracttransformstate.hpp"
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
	typedef AbstractTransformState AbstractState;

	typedef std::vector< std::pair<double, double> > WorkspaceBounds;
	typedef std::vector< std::pair<double, double> > StateVarRanges;

	typedef std::vector<double> StateVars;
	typedef std::vector<double> Control;

	class State {
	public:
		State() : stateVars(7) {}

		State(double x, double y, double z, double theta) : stateVars(7) {
			stateVars[X] = x;
			stateVars[Y] = y;
			stateVars[Z] = z;
			stateVars[THETA] = theta;
		}

		State(const State &s) : stateVars(s.stateVars.begin(), s.stateVars.end()) {}

		State(const StateVars &vars) : stateVars(vars.begin(), vars.end()) {}

		State& operator=(const State &s) {
			stateVars.resize(7);
			for(unsigned int i = 0; i < s.stateVars.size(); ++i)
				stateVars[i] = s.stateVars[i];
			return *this;
		}

		const bool equals(const State &s) const {
			for(unsigned int i = 0; i < 7; ++i) {
				if(fabs(stateVars[i] - s.stateVars[i]) > 0.000001) return false;
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

		AbstractState toAbstractState() const {
			fprintf(stderr, "toAbstractState not implemented\n");
			exit(0);
			return AbstractState();
		}

#ifdef WITHGRAPHICS
		std::vector<double> toOpenGLTransform() const {
			std::vector<double> transform = OpenGLWrapper::getOpenGLWrapper().getIdentity();

			double sinVal = sin(stateVars[THETA]);
			double cosVal = cos(stateVars[THETA]);

			transform[0] = cosVal;
			transform[1] = -sinVal;
			transform[4] = sinVal;
			transform[5] = cosVal;

			transform[12] = stateVars[X];
			transform[13] = stateVars[Y];
			transform[14] = stateVars[Z];

			return transform;
		}
#endif

		fcl::Transform3f toFCLTransform() const {
			fcl::Vec3f pose(stateVars[X], stateVars[Y], stateVars[Z]);

			fcl::Matrix3f rotation;
			rotation.setIdentity();

			double sinVal = sin(stateVars[THETA]);
			double cosVal = cos(stateVars[THETA]);

			rotation(0,0) = cosVal;
			rotation(1,0) = -sinVal;
			rotation(0,1) = sinVal;
			rotation(1,1) = cosVal;

			return fcl::Transform3f(rotation, pose);
		}

		fcl::Transform3f getTransform() const {
			return toFCLTransform();
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

	// private:
		StateVars stateVars;
	};

	class Edge {
	public:
		Edge(const State &start) : start(start), end(start), cost(0), dt(0), a(0), w(0), z(0), treeIndex(0) {
			buildTreeVars();
		}
		Edge(const State &start, const State &end, double cost, double a, double w, double z) : start(start),
			end(end), cost(cost), dt(cost), a(a), w(w), z(z), treeIndex(0) {
				buildTreeVars();
			}
		Edge(const Edge& e) : start(e.start), end(e.end), cost(e.cost), dt(e.dt), a(e.a), w(e.w), z(e.z), treeIndex(e.treeIndex) {
			buildTreeVars();
		}

		Edge& operator=(const Edge &e) {
			this->start = e.start;
			this->end = e.end;
			this->cost = e.cost;
			this->dt = e.dt;
			this->a = e.a;
			this->w = e.w;
			this->z = e.z;
			this->parent = e.parent;
			return *this;
		}

		void buildTreeVars() {
			const auto& vars = end.getStateVars();
			treeVars.resize(vars.size());
			for(unsigned int i = 0; i < vars.size(); ++i) {
				treeVars[i] = (Blimp::NormalizeStateVars[i].first + vars[i]) * Blimp::NormalizeStateVars[i].second;				
			}
		}

		/* needed for being inserted into NN datastructure */
		const StateVars& getTreeStateVars() const { return treeVars; }
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

		Edge *parent;
		State start, end;
		double cost, dt, a, w, z;
		int treeIndex;
		StateVars treeVars;
	};

	Blimp(const InstanceFileMap &args) : mesh(args.value("Agent Mesh")) {
		blimpLength = stod(args.value("Blimp Length"));

		minimumVelocity = stod(args.value("Minimum Velocity"));
		maximumVelocity = stod(args.value("Maximum Velocity"));

		minimumTurning = stod(args.value("Minimum Turning"));
		maximumTurning = stod(args.value("Maximum Turning"));

		minimumVelocityZ = stod(args.value("Minimum Velocity Z"));
		maximumVelocityZ = stod(args.value("Maximum Velocity Z"));

		auto environmentBoundingBox = args.doubleList("Environment Bounding Box");

		for(unsigned int i = 0; i < environmentBoundingBox.size(); i+=2) {
			double term1 = -environmentBoundingBox[i];
			double term2 = 1. / (environmentBoundingBox[i+1] - environmentBoundingBox[i]);
			NormalizeStateVars.emplace_back(term1, term2);
		}
		
		NormalizeStateVars.emplace_back(M_PI / 2., 1. / (2.*M_PI)); //theta
		NormalizeStateVars.emplace_back(-minimumVelocity, 1. / (maximumVelocity - minimumVelocity)); //v
		NormalizeStateVars.emplace_back(-minimumTurning, 1. / (maximumTurning - minimumTurning)); //psi
		NormalizeStateVars.emplace_back(-minimumVelocityZ, 1. / (maximumVelocityZ - minimumVelocityZ)); //vz

		integrationStepSize = stod(args.value("Integration Step Size"));

		linearAccelerations = std::uniform_real_distribution<double>(stod(args.value("Minimum Forward Acceleration")), stod(args.value("Maximum Forward Acceleration")));
		zLinearAccelerations = std::uniform_real_distribution<double>(stod(args.value("Minimum Z Acceleration")), stod(args.value("Maximum Z Acceleration")));
		angularAccelerations = std::uniform_real_distribution<double>(stod(args.value("Minimum Angular Acceleration")), stod(args.value("Maximum Angular Acceleration")));

		controlBounds.emplace_back(stod(args.value("Minimum Forward Acceleration")), stod(args.value("Maximum Forward Acceleration")));
		controlBounds.emplace_back(stod(args.value("Minimum Z Acceleration")), stod(args.value("Maximum Z Acceleration")));
		controlBounds.emplace_back(stod(args.value("Minimum Angular Acceleration")), stod(args.value("Maximum Angular Acceleration")));

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

	Control controlFromVector(const std::vector<double> &controls) const {
		return controls;
	}

	const std::vector< std::pair<double, double> >& getControlBounds() const {
		return controlBounds;
	}

	State getRandomStateNear(const AbstractState &a, const State &s, double radius) const {
		fprintf(stderr, "getRandomStateNear no implemented\n");
		exit(0);
		return State();
	}

	// State transformToState(const State &s, const fcl::Transform3f &transform) const {
	// 	fcl::Quaternion3f orientation = transform.getQuatRotation();
	// 	fcl::Vec3f axis;
	// 	double theta;
	// 	orientation.toAxisAngle(axis, theta);
	// 	theta = (theta - 2 * M_PI * std::floor((theta + M_PI) / (2 * M_PI)));

	// 	fcl::Vec3f position = transform.getTranslation();

	// 	return State(position[0], position[1], position[2], theta);
	// }

	bool isGoal(const State &state, const State &goal) const {
		const StateVars &s = state.getStateVars();
		const StateVars &g = goal.getStateVars();

		return fabs(s[X] - g[X]) < goalThresholds[X] &&
		       fabs(s[Y] - g[Y]) < goalThresholds[Y] &&
		       fabs(s[Z] - g[Z]) < goalThresholds[Z];
	}

	Edge steerWithControl(const State &start, const Edge &getControlsFromThisEdge, double dt) const {
		double a = getControlsFromThisEdge.a;
		double w = getControlsFromThisEdge.w;
		double z = getControlsFromThisEdge.z;

		State end = doSteps(start, a, w, z, dt);

		return Edge(start, end, dt, a, w, z);
	}

	Edge steerWithControl(const State &start, const std::vector<double> controls, double dt) const {
		/* Be careful about the order these are being passed in */

		double a = controls[0];
		double z = controls[1];
		double w = controls[2];

		State end = doSteps(start, a, w, z, dt);

		return Edge(start, end, dt, a, w, z);
	}

	Edge steer(const State &start, const State &goal, double dt) const {
		return randomSteer(start, dt);
	}

	Edge randomSteer(const State &start, double dt) const {
		double a = linearAccelerations(GlobalRandomGenerator);
		double w = angularAccelerations(GlobalRandomGenerator);
		double z = zLinearAccelerations(GlobalRandomGenerator);

		State end = doSteps(start, a, w, z, dt);

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
		std::vector<std::vector<fcl::Transform3f> > retPoses;

		unsigned int steps = std::isinf(dt) ? 0 : edge.dt / dt;

		State state = edge.start;

		retPoses.emplace_back();
		retPoses.back().push_back(state.toFCLTransform());

		for(unsigned int step = 0; step < steps; ++step) {
			state = doSteps(edge.start, edge.a, edge.w, edge.z, dt * (double)step);

			retPoses.emplace_back();

			// drawMesh(state);

			retPoses.back().push_back(state.toFCLTransform());
		}

		retPoses.emplace_back();
		retPoses.back().push_back(edge.end.toFCLTransform());

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

		state = doSteps(state, 0, 0, 0, dt, true);

		auto transform = state.toOpenGLTransform();

		mesh.draw(color, transform);
	}

	void drawMesh(const State &s) const {
		auto transform = s.toOpenGLTransform();
		mesh.draw(color, transform);
	}


	void drawMesh(const fcl::Transform3f &transform) const {
		drawMesh(transform, color);

	}
	void drawMesh(const fcl::Transform3f &transform, const OpenGLWrapper::Color& color) const {
		std::vector<double> glTransform = OpenGLWrapper::getOpenGLWrapper().getIdentity();

		const fcl::Vec3f &translation = transform.getTranslation();
		const fcl::Matrix3f &orientation = transform.getRotation();

		glTransform[0] = orientation(0,0);
		glTransform[1] = orientation(0,1);
		glTransform[2] = orientation(0,2);
		glTransform[4] = orientation(1,0);
		glTransform[5] = orientation(1,1);
		glTransform[6] = orientation(1,2);
		glTransform[8] = orientation(2,0);
		glTransform[9] = orientation(2,1);
		glTransform[10] = orientation(2,2);


		glTransform[12] = translation[0];
		glTransform[13] = translation[1];
		glTransform[14] = translation[2];

		mesh.draw(color, glTransform);
	}

	void drawSolution(const std::vector<const Edge*> &solution, double dt = std::numeric_limits<double>::infinity()) const {
		for(const Edge* edge : solution) {
			unsigned int steps = std::isinf(dt) ? 1 : edge->dt / dt;

			State state = edge->start;

			edge->draw(OpenGLWrapper::Color::Green());

			for(unsigned int step = 0; step < steps; ++step) {
				auto transform = state.toOpenGLTransform();
				mesh.draw(color, transform);
				state.draw(OpenGLWrapper::Color::Green());

				state = doSteps(state, edge->a, edge->w, edge->z, dt, true);
			}
		}
	}

	void animateSolution(const std::vector<const Edge*> &solution, unsigned int poseNumber) const {
		unsigned int edgeNumber = poseNumber / 2;
		unsigned int endpoint = poseNumber % 2;
		const Edge *edge = solution[edgeNumber];

		auto transform = (endpoint == 0 ? edge->start : edge->end).toOpenGLTransform();
		mesh.draw(color, transform);
	}

#endif

// private:

	State doSteps(const State& s, double a, double w, double z, double dt, bool ignoreIntegrationStepSize = false) const {
		const StateVars& vars = s.getStateVars();
		StateVars newState = vars;

		unsigned int steps = ignoreIntegrationStepSize ? 0 : dt / integrationStepSize;

		double leftOver = ignoreIntegrationStepSize ? dt : dt - ((double)steps * integrationStepSize);

		assert(leftOver >= 0);

		double stepSize = integrationStepSize;

		for(unsigned int i = 0; i < steps+1; ++i) {

			if(i == steps) {
				stepSize = leftOver;
			}

			newState[X] = newState[X] + cos(newState[THETA]) * newState[V] * stepSize;
			newState[Y] = newState[Y] + sin(newState[THETA]) * newState[V] * stepSize;
			newState[THETA] = normalizeTheta(newState[THETA] + newState[V] * tan(newState[PSI]) / blimpLength);

			newState[Z] = newState[Z] + newState[VZ] * dt;

			newState[V] = newState[V] + a * stepSize;
			newState[PSI] = newState[PSI] + w * stepSize;
			newState[VZ] = newState[VZ] + z * stepSize;

			if(newState[V] > maximumVelocity) { newState[V] = maximumVelocity; }
			else if(newState[V] < minimumVelocity ) { newState[V] = minimumVelocity; }

			if(newState[PSI] > maximumTurning) { newState[PSI] = maximumTurning; }
			else if(newState[PSI] < minimumTurning) { newState[PSI] = minimumTurning; }

			if(newState[VZ] > maximumVelocityZ) { newState[VZ] = maximumVelocityZ; }
			else if(newState[VZ] < minimumVelocityZ ) { newState[VZ] = minimumVelocityZ; }
		}

		return State(newState);
	}

	double normalizeTheta(double t) const {
		return (t - 2 * M_PI * std::floor((t + M_PI) / (2 * M_PI)));
	}

	SimpleAgentMeshHandler mesh;
	double blimpLength, minimumVelocity, maximumVelocity, minimumTurning, maximumTurning, minimumVelocityZ, maximumVelocityZ, integrationStepSize;
	mutable std::uniform_real_distribution<double> linearAccelerations, zLinearAccelerations, angularAccelerations;

	std::vector< std::pair<double, double> > controlBounds;

	std::vector<double> goalThresholds;

#ifdef WITHGRAPHICS
	const OpenGLWrapper::Color color;
	mutable State state;
#endif
	static std::vector<std::pair<double, double>> NormalizeStateVars;
};

std::vector<std::pair<double, double>> Blimp::NormalizeStateVars;
