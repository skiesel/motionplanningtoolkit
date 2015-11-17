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
	typedef AbstractXYThetaTransformState AbstractState;
	typedef std::vector<AbstractState> AbstractEdge;

	typedef flann::L2<double> DistanceEvaluator;
	typedef flann::L2<double> AbstractDistanceEvaluator;

	typedef std::vector< std::pair<double, double> > WorkspaceBounds;
	typedef std::vector< std::pair<double, double> > StateVarRanges;

	typedef std::vector<double> StateVars;
	typedef std::vector<double> Control;

	class State {
	public:
		State() : stateVars(5 + trailerCount) {}

		State(double x, double y, double theta) : stateVars(5 + trailerCount) {
			stateVars[X] = x;
			stateVars[Y] = y;
			stateVars[THETA] = theta;
			for(unsigned int i = PSI; i < stateVars.size(); ++i) {
				stateVars[i] = 0;
			}
		}

		State(const State &s) : stateVars(s.stateVars.begin(), s.stateVars.end()) {}

		State(const StateVars &vars) : stateVars(vars.begin(), vars.end()) {
			stateVars.resize(5 + trailerCount);
			for(unsigned int i = vars.size(); i < stateVars.size(); ++i) {
				stateVars[i] = 0;
			}
		}

		State(const AbstractState &s) : stateVars(s.treeStateVars.begin(), s.treeStateVars.end()) {
			stateVars.resize(5 + trailerCount);
			for(unsigned int i = s.treeStateVars.size(); i < stateVars.size(); ++i) {
				stateVars[i] = 0;
			}
		}

		State &operator=(const State &s) {
			stateVars.resize(5 + trailerCount);
			for(unsigned int i = 0; i < s.stateVars.size(); ++i) {
				stateVars[i] = s.stateVars[i];
			}
			for(unsigned int i = s.stateVars.size(); i < stateVars.size(); ++i) {
				stateVars[i] = 0;
			}
			return *this;
		}

		const bool equals(const State &s) const {
			for(unsigned int i = 0; i < 5 + trailerCount; ++i) {
				if(fabs(stateVars[i] - s.stateVars[i]) > 0.000001) return false;
			}
			return true;
		}

		std::vector<fcl::Transform3f> getMeshPoses() const {
			return toFCLTransforms();
		}

		std::vector<fcl::Transform3f> toFCLTransforms() const {
			std::vector<fcl::Transform3f> transforms;

			fcl::Vec3f axis(0,1,0);

			fcl::Quaternion3f quaternion;
			quaternion.fromAxisAngle(axis, M_PI / 2);
			fcl::Transform3f baseRotation(quaternion);

			//change to z axis
			axis[0] = 0;
			axis[1] = 0;
			axis[2] = 1;
			quaternion.fromAxisAngle(axis, stateVars[THETA]);

			fcl::Vec3f pose;
			pose[0] = stateVars[X];
			pose[1] = stateVars[Y];
			pose[2] = 0;

			fcl::Transform3f transform = fcl::Transform3f(pose) * fcl::Transform3f(quaternion);

			transforms.emplace_back(transform);

			pose[0] = -(trailerLength + hitchLength);
			pose[1] = 0;

			fcl::Transform3f trailerTranslation(pose);

			for(unsigned int i = 1; i < trailerCount + 1; ++i) {
				double t = stateVars[THETA + i] - stateVars[THETA + i - 1];

				quaternion.fromAxisAngle(axis, t);
				fcl::Transform3f trailerRotation(quaternion);

				transform *= trailerTranslation * trailerRotation;

				transforms.emplace_back(transform);
			}

			for(unsigned int i = 0; i < transforms.size(); i++) {
				 transforms[i] *= baseRotation;
			}

			return transforms;
		}

#ifdef WITHGRAPHICS
		std::vector< std::vector<double> > toOpenGLTransforms() const {
			std::vector< std::vector<double> > transforms;

			//We want the base cylinder and cone rotated correctly to start with
			std::vector<double> baseRotation = OpenGLWrapper::getOpenGLWrapper().getIdentity();

			double sinVal = sin(M_PI / 2);
			double cosVal = cos(M_PI / 2);

			baseRotation[0] = cosVal;
			baseRotation[2] = sinVal;
			baseRotation[8] = -sinVal;
			baseRotation[10] = cosVal;

			auto leadTranslate = OpenGLWrapper::getOpenGLWrapper().getIdentity();

			leadTranslate[3] = stateVars[X];
			leadTranslate[7] = stateVars[Y];

			auto leadRotate = OpenGLWrapper::getOpenGLWrapper().getIdentity();

			sinVal = sin(stateVars[THETA]);
			cosVal = cos(stateVars[THETA]);

			leadRotate[0] = cosVal;
			leadRotate[1] = -sinVal;
			leadRotate[4] = sinVal;
			leadRotate[5] = cosVal;

			auto transform = OpenGLWrapper::getOpenGLWrapper().getIdentity();

			math::multiply(transform, leadTranslate, transform);
			math::multiply(transform, leadRotate, transform);

			transforms.push_back(transform);

			auto trailerTranslate = OpenGLWrapper::getOpenGLWrapper().getIdentity();
			trailerTranslate[3] = -(trailerLength + hitchLength);

			for(unsigned int i = 1; i < trailerCount + 1; ++i) {

				auto trailerRotate = OpenGLWrapper::getOpenGLWrapper().getIdentity();

				double t = stateVars[THETA + i] - stateVars[THETA + i - 1];

				sinVal = sin(t);
				cosVal = cos(t);

				trailerRotate[0] = cosVal;
				trailerRotate[1] = -sinVal;
				trailerRotate[4] = sinVal;
				trailerRotate[5] = cosVal;

				math::multiply(transform, trailerTranslate, transform);
				math::multiply(transform, trailerRotate, transform);
				
				transforms.push_back(transform);
			}

			for(unsigned int i = 0; i < transforms.size(); i++) {
				math::multiply(transforms[i], baseRotation, transforms[i]);
				transforms[i] = math::transpose(transforms[i]);
			}

			return transforms;
		}
#endif

		fcl::Transform3f toFCLTransform() const {
			return toFCLTransforms()[0];
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
		static double trailerLength;
		static double hitchLength;

		// private:
		StateVars stateVars;
	};

	class Edge {
	public:
		Edge(const State &start) : start(start), end(start), cost(0), dt(0), a(0), w(0), g(0), treeIndex(0) {
			populateTreeStateVars();
		}

		Edge(const State &start, const State &end, double cost, double a, double w) : start(start), end(end),
			cost(cost), dt(cost), a(a), w(w), g(0), treeIndex(0) {
			populateTreeStateVars();
		}

		Edge(const Edge &e) : start(e.start), end(e.end), cost(e.cost), dt(e.dt), a(e.a), w(e.w), g(e.g), treeIndex(e.treeIndex) {
			populateTreeStateVars();
		}

		Edge &operator=(const Edge &e) {
			start = e.start;
			end = e.end;
			cost = e.cost;
			dt = e.dt;
			a = e.a;
			w = e.w;
			g = e.g;
			treeIndex = e.treeIndex;
			parent = e.parent;
			return *this;
		}

		double gCost() const {
			return g;
		}

		void updateParent(Edge* p) {
			parent = p;
			g = p->gCost() + cost;
		}

		/* needed for being inserted into NN datastructure */
		const StateVars &getTreeStateVars() const {
			return treeStateVars;
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
		State start, end;
		double cost, dt, a, w, g;
		int treeIndex;
		StateVars treeStateVars;
		Edge *parent;

	private:
		void populateTreeStateVars() {
			const auto &vars = end.getStateVars();
			treeStateVars.resize(vars.size());
			for(unsigned int i = 0; i < vars.size(); ++i) {
				treeStateVars[i] = (-SnakeTrailers::NormalizeStateVars[i].first + vars[i]) * SnakeTrailers::NormalizeStateVars[i].second;
			}
		}
	};

	SnakeTrailers(const InstanceFileMap &args) {

		trailerCount = State::trailerCount = stoi(args.value("Trailer Count"));
		trailerWidth = args.doubleVal("Trailer Width");
		trailerLength = State::trailerLength = args.doubleVal("Trailer Length");
		hitchLength = State::hitchLength = args.doubleVal("Hitch Length");

		minimumVelocity = args.doubleVal("Minimum Velocity");
		maximumVelocity = args.doubleVal("Maximum Velocity");

		minimumTurning = args.doubleVal("Minimum Turning");
		maximumTurning = args.doubleVal("Maximum Turning");

		linearAccelerations = std::uniform_real_distribution<double>(args.doubleVal("Minimum Velocity"), args.doubleVal("Maximum Velocity"));
		angularAccelerations = std::uniform_real_distribution<double>(args.doubleVal("Minimum Angular Acceleration"), args.doubleVal("Maximum Angular Acceleration"));

		controlBounds.emplace_back(args.doubleVal("Minimum Velocity"), args.doubleVal("Maximum Velocity"));
		controlBounds.emplace_back(args.doubleVal("Minimum Angular Acceleration"), args.doubleVal("Maximum Angular Acceleration"));

		integrationStepSize = args.doubleVal("Integration Step Size");

		auto environmentBoundingBox = args.doubleList("Environment Bounding Box");

		for(unsigned int i = 0; i < environmentBoundingBox.size() - 2; i+=2) {
			double term1 = environmentBoundingBox[i];
			double term2 = 1. / (environmentBoundingBox[i+1] - environmentBoundingBox[i]);
			NormalizeStateVars.emplace_back(term1, term2);
		}

		NormalizeStateVars.emplace_back(minimumVelocity, 1. / (maximumVelocity - minimumVelocity)); //v
		NormalizeStateVars.emplace_back(minimumTurning, 1. / (maximumTurning - minimumTurning)); //psi
		NormalizeStateVars.emplace_back(-M_PI, 1. / (2.*M_PI)); //theta
		for(unsigned int i = 0; i < trailerCount; ++i) {
			NormalizeStateVars.emplace_back(NormalizeStateVars.back());
		}

		boost::char_separator<char> sep(" ");
		boost::tokenizer< boost::char_separator<char> > tokens(args.value("Goal Thresholds"), sep);
		for(auto token : tokens) {
			goalThresholds.push_back(std::stod(token));
		}

		// meshes.push_back(new ConeHandler(trailerWidth, trailerLength));
		meshes.push_back(new CylinderHandler(trailerWidth, trailerLength));
		if(trailerCount > 0) {
			meshes.push_back(new CylinderHandler(trailerWidth, trailerLength));
			for(unsigned int i = 0; i < trailerCount-1; ++i) {
				meshes.push_back(meshes[1]);
			}
		}

#ifdef WITHGRAPHICS
		//make sure that the state gets populated AFTER trailercount is set
		state = State();
		OpenGLWrapper::setExternalKeyboardCallback([&](int key) {
			this->keyboard(key);
		});
#endif
	}

	StateVarRanges getStateVarRanges(const WorkspaceBounds &b) const {
		StateVarRanges bounds(b.begin(), b.begin() + 2);
		bounds.emplace_back(minimumVelocity, maximumVelocity);
		bounds.emplace_back(minimumTurning, maximumTurning);
		for(unsigned int i = 0; i < trailerCount+1; ++i) {
			bounds.emplace_back(-M_PI, M_PI);
		}

		return bounds;
	}

	StateVarRanges getAbstractStateVarRanges(const WorkspaceBounds &b) const {
		StateVarRanges bounds(b.begin(), b.begin() + 2);
		bounds.emplace_back(-M_PI, M_PI);
		return bounds;
	}

	unsigned int getTreeStateSize() const {
		return 5 + trailerCount;
	}

	unsigned int getTreeAbstractStateSize() const {
		return 3;
	}

	DistanceEvaluator getDistanceEvaluator() const {
		return DistanceEvaluator();
	}

	AbstractDistanceEvaluator getAbstractDistanceEvaluator() const {
		return DistanceEvaluator();
	}

	Control controlFromVector(const std::vector<double> &controls) const {
		return controls;
	}

	const std::vector< std::pair<double, double> > &getControlBounds() const {
		return controlBounds;
	}

	State buildState(const StateVars &stateVars) const {
		return State(stateVars);
	}

	State getRandomStateNearAbstractState(const AbstractState &s, double radius) const {
		State concrete(s);

		concrete.stateVars[V] = minimumVelocity + zeroToOne(GlobalRandomGenerator) * (maximumVelocity - minimumVelocity);
		concrete.stateVars[PSI] = minimumTurning + zeroToOne(GlobalRandomGenerator) * (maximumTurning - minimumTurning);
		concrete.stateVars[THETA] = zeroToOne(GlobalRandomGenerator) * 2 * M_PI - M_PI;

		double offset = 0.25 / 2 * M_PI;
		for(unsigned int i = 1; i < trailerCount+1; ++i) {
			concrete.stateVars[THETA + i] = zeroToOne(GlobalRandomGenerator) * 0.25 * M_PI - offset;
		}

		Edge e = randomSteer(concrete, radius);
		return e.end;
	}

	State getRandomStateNearState(const State &s, double radius) const {
		Edge e = randomSteer(s, radius);
		return e.end;
	}

	bool isGoal(const State &state, const State &goal) const {
		const StateVars &s = state.getStateVars();
		const StateVars &g = goal.getStateVars();

		return fabs(s[X] - g[X]) < goalThresholds[X] &&
		       fabs(s[Y] - g[Y]) < goalThresholds[Y];
	}

	Edge steerWithControl(const State &start, const Edge &getControlsFromThisEdge, double dt) const {
		double a = getControlsFromThisEdge.a;
		double w = getControlsFromThisEdge.w;

		State end = doSteps(start, a, w, dt);

		return Edge(start, end, dt, a, w);
	}

	Edge steerWithControl(const State &start, const std::vector<double> controls, double dt) const {
		/* Be careful about the order these are being passed in */

		double a = controls[0];
		double w = controls[1];

		State end = doSteps(start, a, w, dt);

		return Edge(start, end, dt, a, w);
	}

	Edge steer(const State &start, const State &goal, double dt) const {
		double a = linearAccelerations(GlobalRandomGenerator);
		double w = angularAccelerations(GlobalRandomGenerator);

		State end = doSteps(start, a, w, dt);

		return Edge(start, end, dt, a, w);
	}

	Edge randomSteer(const State &start, double dt) const {
		double a = linearAccelerations(GlobalRandomGenerator);
		double w = angularAccelerations(GlobalRandomGenerator);

		State end = doSteps(start, a, w, dt);

		return Edge(start, end, dt, a, w);
	}

	Edge constructEdge(const State &start, const State &end) const {
		fprintf(stderr, "SnakeTrailers::constructEdge not implemented\n");
		exit(1);
	}

	const std::vector<const SimpleAgentMeshHandler *> getMeshes() const {
		return meshes;
	}

	std::vector<std::vector<fcl::Transform3f> > getMeshPoses(const Edge &edge, double dt) const {
		std::vector<std::vector<fcl::Transform3f> > retPoses;

		unsigned int steps = std::isinf(dt) ? 0 : edge.dt / dt;
		
		State state = edge.start;

		retPoses.emplace_back(state.toFCLTransforms());

		for(unsigned int step = 0; step < steps; ++step) {
			state = doSteps(edge.start, edge.a, edge.w, dt * (double)step);

			retPoses.emplace_back(state.toFCLTransforms());

			// drawMesh(state);
		}

		retPoses.emplace_back(state.toFCLTransforms());

		return retPoses;
	}

	inline bool areAbstractEdgesSymmetric() const {
		return true;
	}

	std::vector<const SimpleAgentMeshHandler *> getAbstractMeshes() const {
		std::vector<const SimpleAgentMeshHandler*> mesh = {meshes[0]};
		return mesh;
	}

	AbstractState toAbstractState(const State& state) const {
		AbstractState s;
		s.treeStateVars.push_back(state.stateVars[X]);
		s.treeStateVars.push_back(state.stateVars[Y]);
		s.treeStateVars.push_back(state.stateVars[THETA]);

		s.transform = state.toFCLTransform();

		return s;
	}

	std::vector<std::vector<fcl::Transform3f> > getAbstractMeshPoses(const AbstractEdge &edge, double dt) const {
		auto ps = math::interpolate(edge[0].transform, edge[1].transform, dt);
		std::vector<std::vector<fcl::Transform3f> > poses;

		for(const auto &p : ps) {
			poses.emplace_back(1, p);
		}
		return poses;
	}

	std::vector<fcl::Transform3f> getAbstractMeshPoses(const AbstractState &state) const {
		std::vector<fcl::Transform3f> poses = { state.transform };
		return poses;
	}

	AbstractEdge generateAbstractEdge(const AbstractState &s1, const AbstractState &s2) const {
		AbstractEdge edge = {s1, s2};
		return edge;
	}

	std::vector<State> getRepresentiveStatesForLocation(const std::vector<double> &loc) const {
		std::vector<State> states;

		// fcl::Vec3f pose(loc[0], loc[1], loc[2]);

		// unsigned int rotations = 4;
		// double increment = M_PI / ((double)rotations * 2.);
		// fcl::Matrix3f rotation;
		// rotation.setIdentity();

		// for(unsigned int i = 0; i < rotations; ++i) {
		// 	double cosVal = cos((double)i * increment);
		// 	double sinVal = sin((double)i * increment);

		// 	rotation(0,0) = cosVal;
		// 	rotation(1,0) = -sinVal;
		// 	rotation(0,1) = sinVal;
		// 	rotation(1,1) = cosVal;

		// 	retPoses.emplace_back();
		// 	retPoses.back().emplace_back(rotation, pose);
		// }

		return states;
	}

	std::vector< std::vector<fcl::Transform3f> > getPoses(const Edge &edge, double dt) const {
		std::vector< std::vector<fcl::Transform3f> > poses;

		unsigned int steps = edge.dt / dt;
		if(steps == 0) {
			steps = 1;
		}

		State state = edge.start;

		for(unsigned int step = 0; step < steps; ++step) {
			const std::vector<fcl::Transform3f> transforms = state.toFCLTransforms();

			poses.emplace_back();
			for(const fcl::Transform3f &transform : transforms) {
				poses.back().push_back(transform);
			}

			state = doSteps(state, edge.a, edge.w, dt);
		}

		const std::vector<fcl::Transform3f> transforms = edge.end.toFCLTransforms();

		poses.emplace_back();
		for(const fcl::Transform3f &transform : transforms) {
			poses.back().push_back(transform);
		}

		return poses;
	}

#ifdef WITHGRAPHICS
	void keyboard(int key) {}

	void draw() const {
		double a = 0, w = 0, dt = 0.1;

		state = doSteps(state, a, w, dt, true);

		drawMesh(state);
	}

	void drawMesh(const State &s) const {
		auto transforms = s.toOpenGLTransforms();

		for(unsigned int i = 0; i < meshes.size(); ++i) {
			meshes[i]->draw(OpenGLWrapper::Color::Blue(), transforms[i]);
		}
	}

	void drawSolution(const std::vector<const Edge *> &solution, double dt = std::numeric_limits<double>::infinity()) const {
		for(const Edge *edge : solution) {
			unsigned int steps = std::isinf(dt) ? 1 : edge->dt / dt;

			State state = edge->start;

			for(unsigned int step = 0; step < steps; ++step) {

				auto transforms = state.toOpenGLTransforms();
				for(unsigned int i = 0; i < meshes.size(); ++i) {
					meshes[i]->draw(color, transforms[i]);
				}

				state = doSteps(state, edge->a, edge->w, dt);
			}
		}
	}

	void animateSolution(const std::vector<const Edge *> &solution, unsigned int poseNumber) const {
		unsigned int edgeNumber = poseNumber / 2;
		unsigned int endpoint = poseNumber % 2;
		const Edge *edge = solution[edgeNumber];

		auto transforms = (endpoint == 0 ? edge->start : edge->end).toOpenGLTransforms();
		for(unsigned int i = 0; i < meshes.size(); ++i) {
			meshes[i]->draw(color, transforms[i]);
		}
	}
#endif

// private:

	// State doSteps(const State &s, double a, double w, double dt, bool ignoreIntegrationStepSize=false) const {
	// 	StateVars newState = s.getStateVars();

	// 	unsigned int steps = ignoreIntegrationStepSize ? 0 : dt / integrationStepSize;

	// 	double leftOver = ignoreIntegrationStepSize ? dt : dt - ((double)steps * integrationStepSize);

	// 	assert(leftOver >= 0);

	// 	double stepSize = integrationStepSize;

	// 	for(unsigned int i = 0; i < steps+1; ++i) {

	// 		if(i == steps) {
	// 			stepSize = leftOver;
	// 		}

	// 		newState[X] = newState[X] + cos(newState[THETA]) * newState[V] * stepSize;
	// 		newState[Y] = newState[Y] + sin(newState[THETA]) * newState[V] * stepSize;
	// 		newState[THETA] = normalizeTheta(newState[THETA] + newState[V] * tan(newState[PSI]) / trailerLength * stepSize);
	// 		newState[V] = newState[V] + a * stepSize;
	// 		newState[PSI] = newState[PSI] + w * stepSize;


	// 		if(newState[V] > maximumVelocity) {
	// 			newState[V] = maximumVelocity;
	// 		} else if(newState[V] < minimumVelocity) {
	// 			newState[V] = minimumVelocity;
	// 		}

	// 		if(newState[PSI] > maximumTurning) {
	// 			newState[PSI] = maximumTurning;
	// 		} else if(newState[PSI] < minimumTurning) {
	// 			newState[PSI] = minimumTurning;
	// 		}

	// 		double coeff = newState[V] / (trailerLength + hitchLength);
	// 		double prev = newState[THETA];

	// 		StateVars tempVars = newState;
	// 		for(unsigned int i = 1; i < trailerCount+1; ++i) {
	// 			newState[THETA + i] = normalizeTheta(tempVars[THETA + i] + coeff * sin(prev - tempVars[THETA + i]) * stepSize);
	// 			coeff *= cos(prev - tempVars[THETA + i]);
	// 			prev = tempVars[THETA + i];
	// 		}

	// 		newState = tempVars;
	// 	}

	// 	return State(newState);
	// }


	State doSteps(const State &s, double a, double w, double dt, bool ignoreIntegrationStepSize=false) const {
		const StateVars &vars = s.getStateVars();
		StateVars newState(5 + trailerCount);

		newState[X] = vars[X] + cos(vars[THETA]) * vars[V] * dt;
		newState[Y] = vars[Y] + sin(vars[THETA]) * vars[V] * dt;
		newState[THETA] = normalizeTheta(vars[THETA] + vars[V] * tan(vars[PSI]) / trailerLength * dt);
		newState[V] = vars[V] + a * dt;
		newState[PSI] = vars[PSI] + w * dt;


		if(newState[V] > maximumVelocity) {
			newState[V] = maximumVelocity;
		} else if(newState[V] < minimumVelocity) {
			newState[V] = minimumVelocity;
		}

		if(newState[PSI] > maximumTurning) {
			newState[PSI] = maximumTurning;
		} else if(newState[PSI] < minimumTurning) {
			newState[PSI] = minimumTurning;
		}


		double coeff = vars[V] / (trailerLength + hitchLength);
		double prev = vars[THETA];

		for(unsigned int i = 1; i < trailerCount+1; ++i) {
			newState[THETA + i] = normalizeTheta(vars[THETA + i] + coeff * sin(prev - vars[THETA + i]) * dt);
			coeff *= cos(prev - vars[THETA + i]);
			prev = vars[THETA + i];
		}

		return State(newState);
	}

	double normalizeTheta(double t) const {
		return (t - 2 * M_PI * std::floor((t + M_PI) / (2 * M_PI)));
	}

	std::vector<const SimpleAgentMeshHandler *> meshes;

	unsigned int trailerCount;
	double trailerLength, trailerWidth, hitchLength, minimumVelocity, maximumVelocity, minimumTurning, maximumTurning, integrationStepSize;
	mutable std::uniform_real_distribution<double> linearAccelerations, angularAccelerations, zeroToOne;

	std::vector< std::pair<double, double> > controlBounds;

	std::vector<double> goalThresholds;

#ifdef WITHGRAPHICS
	const OpenGLWrapper::Color color;
	mutable State state;
#endif

	static std::vector<std::pair<double, double>> NormalizeStateVars;
};

std::vector<std::pair<double, double>> SnakeTrailers::NormalizeStateVars;

unsigned int SnakeTrailers::State::trailerCount = 0;
double SnakeTrailers::State::trailerLength = 0;
double SnakeTrailers::State::hitchLength = 0;


