#pragma once

#include <cmath>
#include <stdlib.h>
#include <random>

#include "abstracttransformstate.hpp"
#include "../utilities/fcl_helpers.hpp"
#include "../utilities/openglwrapper.hpp"
#include "../utilities/instancefilemap.hpp"

class Geometric {
public:
	typedef AbstractTransformState AbstractState;
	
	typedef std::vector< std::pair<double, double> > WorkspaceBounds;
	typedef std::vector< std::pair<double, double> > StateVarRanges;

	typedef std::vector<double> StateVars;
	typedef std::vector<double> Control;

	class State {
	public:
		State() : stateVars(7) {
			stateVars[3] = 1;
		}

		State(const State &s) : stateVars(s.stateVars.begin(), s.stateVars.end()) {}

		State(const StateVars &vars) : stateVars(vars.begin(), vars.end()) {}

		State& operator=(const State &s) {
			stateVars.resize(7);
			stateVars[3] = 1;
			for(unsigned int i = 0; i < s.stateVars.size(); ++i)
				stateVars[i] = s.stateVars[i];
			return *this;
		}

		const StateVars& getStateVars() const { return stateVars; }

		const bool equals(const State &s) const {
			for(unsigned int i = 0; i < 7; ++i) {
				if(fabs(stateVars[i] - s.stateVars[i]) > 0.000001) return false;
			}
			return true;
		}

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
			std::vector<double> translation = OpenGLWrapper::getOpenGLWrapper().getIdentity();
			translation[12] = stateVars[0];
			translation[13] = stateVars[1];
			translation[14] = stateVars[2];

			// fcl::Quaternion3f quaternion(stateVars[3], stateVars[4], stateVars[5], stateVars[6]);
			// fcl::Matrix3f rotationMatrix;
			// quaternion.toRotation(rotationMatrix);

			// rotationMatrix = rotationMatrix.transpose();

			// std::vector<double> rotation = OpenGLWrapper::getOpenGLWrapper().getIdentity();
			// for(unsigned int i = 0; i < 9; i++) {
			// 	rotation[i] = rotationMatrix(i % 3, i / 3);
			// }

			// std::vector<double> transform(16);
			// math::multiply(rotation, translation, transform);

			// return transform;
			return translation;
		}
#endif

		fcl::Transform3f toFCLTransform() const {
			fcl::Vec3f pose(stateVars[0], stateVars[1], stateVars[2]);
			fcl::Quaternion3f quaternion(stateVars[3], stateVars[4], stateVars[5], stateVars[6]);
			quaternion = math::normalize(quaternion);
			return fcl::Transform3f(quaternion, pose);
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
		Edge(const State &start) : start(start), end(start), cost(0), treeIndex(0) {
			buildTreeVars();
		}
		Edge(const State &start, const State &end, const fcl::Vec3f &translation, const fcl::Quaternion3f &rotation,
			double dt) : start(start), end(end), translation(translation), rotation(rotation), dt(dt), cost(dt), treeIndex(0) {
				buildTreeVars();
			}
		Edge(const Edge& e) : start(e.start), end(e.end), translation(e.translation), rotation(e.rotation),
			dt(e.dt), cost(e.cost), treeIndex(e.treeIndex) {
			buildTreeVars();
		}

		Edge& operator=(const Edge &e) {
			this->start = e.start;
			this->end = e.end;
			this->dt = e.dt;
			this->cost = e.cost;
			this->translation = e.translation;
			this->rotation = e.rotation;
			this->parent = e.parent;
			return *this;
		}

		void buildTreeVars() {
			const auto& vars = end.getStateVars();
			treeVars.resize(vars.size());
			for(unsigned int i = 0; i < vars.size(); ++i) {
				treeVars[i] = (Geometric::NormalizeStateVars[i].first + vars[i]) * Geometric::NormalizeStateVars[i].second;				
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
		fcl::Vec3f translation;
		fcl::Quaternion3f rotation;
		double dt, cost;
		int treeIndex;
		StateVars treeVars;
	};

	Geometric(const InstanceFileMap &args) : mesh(args.value("Agent Mesh")), color(OpenGLWrapper::Color::Red()) {

		auto environmentBoundingBox = args.doubleList("Environment Bounding Box");

		for(unsigned int i = 0; i < environmentBoundingBox.size(); i+=2) {
			double term1 = -environmentBoundingBox[i];
			double term2 = 1. / (environmentBoundingBox[i+1] - environmentBoundingBox[i]);
			NormalizeStateVars.emplace_back(term1, term2);
		}

		NormalizeStateVars.emplace_back(0, 1);
		NormalizeStateVars.emplace_back(0, 1);
		NormalizeStateVars.emplace_back(0, 1);
		NormalizeStateVars.emplace_back(0, 1);

		integrationStepSize = stod(args.value("Integration Step Size"));

		boost::char_separator<char> sep(" ");
		boost::tokenizer< boost::char_separator<char> > tokens(args.value("Goal Thresholds"), sep);
		for(auto token : tokens) {
			goalThresholds.push_back(std::stod(token));
		}

#ifdef WITHGRAPHICS
		OpenGLWrapper::setExternalKeyboardCallback([&](int key){});
#endif
	}

	unsigned int getTreeStateSize() const {
		return 7;
	}

	StateVarRanges getStateVarRanges(const WorkspaceBounds& b) const {
		StateVarRanges bounds(b.begin(), b.end());
		bounds.emplace_back(-1, 1);
		bounds.emplace_back(-1, 1);
		bounds.emplace_back(-1, 1);
		bounds.emplace_back(-1, 1);
		return bounds;
	}

	State buildState(const StateVars& stateVars) const {
		return State(stateVars);
	}

	Control controlFromVector(const std::vector<double> &controls) const {
		return controls;
	}

	const std::vector< std::pair<double, double> >& getControlBounds() const {
		fprintf(stderr, "Geometric::getControlBounds not implemented");
		exit(0);
		return controlBounds;
	}

	State getRandomStateNear(const AbstractState &a, const State &s, double radius) const {
		fprintf(stderr, "getRandomStateNear no implemented\n");
		exit(0);
		return State();
	}

	// State transformToState(const State &s, const fcl::Transform3f &transform) const {
	// 	fcl::Vec3f position = transform.getTranslation();
	// 	fcl::Quaternion3f orientation = transform.getQuatRotation();
	// 	StateVars vars(7);
	// 	vars[0] = position[0];
	// 	vars[1] = position[1];
	// 	vars[2] = position[2];
	// 	vars[3] = orientation.getW();
	// 	vars[4] = orientation.getX();
	// 	vars[5] = orientation.getY();
	// 	vars[6] = orientation.getZ();

	// 	return State(vars);
	// }

	bool isGoal(const State &state, const State &goal) const {
		const StateVars &s = state.getStateVars();
		const StateVars &g = goal.getStateVars();

		return fabs(s[0] - g[0]) < goalThresholds[0] &&
		       fabs(s[1] - g[1]) < goalThresholds[1] &&
		       fabs(s[2] - g[2]) < goalThresholds[2];
	}

	Edge steerWithControl(const State &start, const Edge &getControlsFromThisEdge, double dt) const {
		State end = doSteps(start, getControlsFromThisEdge.translation, getControlsFromThisEdge.rotation, dt);

		return Edge(start, end, getControlsFromThisEdge.translation, getControlsFromThisEdge.rotation, dt);
	}

	Edge steerWithControl(const State &start, const std::vector<double> controls, double dt) const {
		/* Be careful about the order these are being passed in */
		fcl::Vec3f translation(controls[0], controls[1], controls[2]);
		fcl::Quaternion3f rotation(controls[3], controls[4], controls[5], controls[6]);
		rotation = math::normalize(rotation);

		State end = doSteps(start, translation, rotation, dt);

		return Edge(start, end, translation, rotation, dt);
	}

	Edge steer(const State &start, const State &goal, double dt) const {
		fprintf(stderr, "Geometric::steer not implemented\n");
		exit(0);
		return Edge(start);
	}

	Edge randomSteer(const State &start, double dt) const {
		fcl::Vec3f translation;// = math::randomPointInSphere();
		translation[3] = -0.01;
		fcl::Quaternion3f rotation;// = math::getRandomUnitQuaternion();
		
		State end = doSteps(start, translation, rotation, dt);

		return Edge(start, end, translation, rotation, dt);
	}

	std::vector<const SimpleAgentMeshHandler*> getMeshes() const {
		std::vector<const SimpleAgentMeshHandler*> meshes(1, &mesh);
		return meshes;
	}

	std::vector<fcl::Transform3f> getRepresentivePosesForLocation(const std::vector<double> &loc) const {
		std::vector<fcl::Transform3f> poses;

		fprintf(stderr, "Geometric::getRepresentivePosesForLocation not implemented\n");
		exit(0);

		return poses;
	}

	std::vector<std::vector<fcl::Transform3f> > getPoses(const Edge &edge, double dt) const {
		std::vector<std::vector<fcl::Transform3f> > retPoses;

		unsigned int steps = std::isinf(dt) ? 0 : edge.dt / dt;

		State state = edge.start;

		retPoses.emplace_back();
		retPoses.back().push_back(state.toFCLTransform());

		for(unsigned int step = 0; step < steps; ++step) {
			state = doSteps(edge.start, edge.translation, edge.rotation, dt * (double)step);
			retPoses.emplace_back();
			retPoses.back().push_back(state.toFCLTransform());
		}

		retPoses.emplace_back();
		retPoses.back().push_back(edge.end.toFCLTransform());

		return retPoses;
	}

#ifdef WITHGRAPHICS
	void draw() const {
		double dt = 0.1;

		state = doSteps(state, fcl::Vec3f(), fcl::Quaternion3f(), dt);

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

				state = doSteps(state, edge->translation, edge->rotation, dt);
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

	State doSteps(const State& s, const fcl::Vec3f &translation, const fcl::Quaternion3f &rotation, double dt) const {
		const StateVars& vars = s.getStateVars();
		StateVars newState = vars;

		newState[0] += translation[0] * dt;
		newState[1] += translation[1] * dt;
		newState[2] += translation[2] * dt;

		fcl::Quaternion3f baseQuaternion(newState[3], newState[4], newState[5], newState[6]);
		fcl::Quaternion3f rot = rotation;

		if(dt > 1) {
			int steps = dt;
			dt = dt - steps;

			for(unsigned int i = 0; i < steps; ++i) {
				baseQuaternion = rot * baseQuaternion;
				baseQuaternion = math::normalize(baseQuaternion);
			}
		}

		rot = math::slerp(fcl::Quaternion3f(), rot, dt);
		baseQuaternion = rot * baseQuaternion;
		baseQuaternion = math::normalize(baseQuaternion);

		newState[3] = baseQuaternion.getW();
		newState[4] = baseQuaternion.getX();
		newState[5] = baseQuaternion.getY();
		newState[6] = baseQuaternion.getZ();

		return State(newState);
	}

	SimpleAgentMeshHandler mesh;
	double integrationStepSize;

	std::vector< std::pair<double, double> > controlBounds;

	std::vector<double> goalThresholds;

#ifdef WITHGRAPHICS
	const OpenGLWrapper::Color color;
	mutable State state;
#endif
	static std::vector<std::pair<double, double>> NormalizeStateVars;
};

std::vector<std::pair<double, double>> Geometric::NormalizeStateVars;
