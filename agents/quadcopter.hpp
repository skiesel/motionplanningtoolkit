#pragma once

#include <cmath>
#include <stdlib.h>
#include <random>
#include "../utilities/fcl_helpers.hpp"
#include "../utilities/openglwrapper.hpp"
#include "../utilities/instancefilemap.hpp"

class Quadcopter {
	enum LABELS {
		X = 0,
		Y = 1,
		Z = 2,
		DX = 3,
		DY = 4,
		DZ = 5,
		ROLL = 6,
		PITCH = 7,
		YAW = 8,
		DROLL = 9,
		DPITCH = 10,
		DYAW = 11,
	};

public:
	typedef AbstractFullTransformState AbstractState;
	typedef std::vector<AbstractState> AbstractEdge;

	typedef std::vector< std::pair<double, double> > WorkspaceBounds;
	typedef std::vector< std::pair<double, double> > StateVarRanges;

	typedef std::vector<double> StateVars;
	typedef std::vector<double> Control;

	typedef flann::L2<double> DistanceEvaluator;
	typedef flann::L2<double> AbstractDistanceEvaluator;

	class State {
	public:
		State() : stateVars(12) {}

		State(double x, double y, double z) : stateVars(12, 0) {
			stateVars[X] = x;
			stateVars[Y] = y;
			stateVars[Z] = z;
		}

		State(const State &s) : stateVars(s.stateVars.begin(), s.stateVars.end()) {}

		State(const StateVars &vars) : stateVars(vars.begin(), vars.end()) {}

		State(const AbstractState &s) : stateVars(s.treeStateVars.begin(), s.treeStateVars.end()) {
			stateVars.resize(12);
		}

		State &operator=(const State &s) {
			stateVars.resize(12);
			for(unsigned int i = 0; i < s.stateVars.size(); ++i)
				stateVars[i] = s.stateVars[i];
			return *this;
		}

		const bool equals(const State &s) const {
			for(unsigned int i = 0; i < 12; ++i) {
				if(fabs(stateVars[i] - s.stateVars[i]) > 0.000001) return false;
			}
			return true;
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

		fcl::Transform3f toFCLTransform() const {
			fcl::Vec3f pose(stateVars[X], stateVars[Y], stateVars[Z]);

			fcl::Matrix3f rotation;
			rotation.setIdentity();

			double sinRoll = sin(stateVars[ROLL]);
			double sinPitch = sin(stateVars[PITCH]);
			double sinYaw = sin(stateVars[YAW]);

			double cosRoll = cos(stateVars[ROLL]);
			double cosPitch = cos(stateVars[PITCH]);
			double cosYaw = cos(stateVars[YAW]);

			rotation(0,0) = cosPitch * cosYaw;
			rotation(1,0) = -cosRoll * sinYaw + sinRoll * sinPitch * cosYaw;
			rotation(2,0) = sinRoll * sinYaw + cosRoll * sinPitch * cosYaw;
			
			rotation(0,1) = cosPitch * sinYaw;
			rotation(1,1) = cosRoll * cosYaw + sinRoll * sinPitch * sinYaw;
			rotation(2,1) = -sinRoll * cosYaw + cosRoll * sinPitch * sinYaw;
			
			rotation(0,2) = -sinPitch;
			rotation(1,2) = sinRoll * cosPitch;
			rotation(2,2) = cosRoll * cosPitch;

			return fcl::Transform3f(rotation, pose);
		}

		std::vector<fcl::Transform3f> getMeshPoses() const {
			return std::vector<fcl::Transform3f>(1, toFCLTransform());
		}

#ifdef WITHGRAPHICS
		std::vector<double> toOpenGLTransform() const {
			std::vector<double> transform = OpenGLWrapper::getOpenGLWrapper().getIdentity();

			// double sinVal = sin(stateVars[THETA]);
			// double cosVal = cos(stateVars[THETA]);

			// transform[0] = cosVal;
			// transform[1] = -sinVal;
			// transform[4] = sinVal;
			// transform[5] = cosVal;

			// transform[12] = stateVars[X];
			// transform[13] = stateVars[Y];
			// transform[14] = stateVars[Z];

			return transform;
		}

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
		Edge(const State &start) : parent(NULL), start(start), end(start), cost(0), dt(0), p0(0), p1(0), p2(0), p3(0), g(0), treeIndex(0) {
			buildTreeVars();
		}
		Edge(const State &start, const State &end, double cost, double p0, double p1, double p2, double p3) : parent(NULL), start(start),
			end(end), cost(cost), dt(cost), p0(p0), p1(p1), p2(p2), p3(p3), g(0), treeIndex(0) {
			buildTreeVars();
		}
		Edge(const Edge &e) : parent(e.parent), start(e.start), end(e.end), cost(e.cost), dt(e.dt), p0(e.p0), p1(e.p1), p2(e.p2), p3(e.p3), g(e.g), treeIndex(e.treeIndex) {
			buildTreeVars();
		}

		Edge &operator=(const Edge &e) {
			this->start = e.start;
			this->end = e.end;
			this->cost = e.cost;
			this->g = e.g;
			this->dt = e.dt;
			this->p0 = e.p0;
			this->p1 = e.p1;
			this->p2 = e.p2;
			this->p3 = e.p3;
			this->parent = e.parent;
			return *this;
		}

		void buildTreeVars() {
			const auto &vars = end.getStateVars();
			treeVars = vars;
			// treeVars.resize(vars.size());
			// for(unsigned int i = 0; i < vars.size(); ++i) {
			// 	treeVars[i] = (Blimp::NormalizeStateVars[i].first + vars[i]) * Blimp::NormalizeStateVars[i].second;
			// }
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
			return treeVars;
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
		double cost, dt, p0, p1, p2, p3, g;
		int treeIndex;
		StateVars treeVars;
	};

	Quadcopter(const InstanceFileMap &args) : mesh(args.doubleVal("Quadcopter Radius") * 2, args.doubleVal("Quadcopter Radius") * 2, args.doubleVal("Quadcopter Height")) {
		quadRadius = args.doubleVal("Quadcopter Radius");
		quadInnerRadius = args.doubleVal("Quadcopter Inner Radius");
		quadMass = args.doubleVal("Quadcopter Mass");
		quadThrust = args.doubleVal("Quadcopter Thrust");
		quadDrag = args.doubleVal("Quadcopter Drag");

		quadIxx = quadIyy = 2 * quadMass * quadInnerRadius * quadInnerRadius / 5 + 2 * quadMass * quadRadius * quadRadius;
		quadIzz = 2 * quadMass * quadInnerRadius * quadInnerRadius / 5 + 4 * quadMass * quadRadius * quadRadius;

		minimumVelocity = args.doubleVal("Minimum Velocity");
		maximumVelocity = args.doubleVal("Maximum Velocity");

		minimumAngularVelocity = args.doubleVal("Minimum Angular Velocity");
		maximumAngularVelocity = args.doubleVal("Maximum Angular Velocity");

		auto environmentBoundingBox = args.doubleList("Environment Bounding Box");

		integrationStepSize = args.doubleVal("Integration Step Size");

		propVelocity = std::uniform_real_distribution<double>(args.doubleVal("Minimum Prop Velocity"), args.doubleVal("Maximum Prop Velocity"));

		for(unsigned int i = 0; i < 4; ++i)
			controlBounds.emplace_back(args.doubleVal("Minimum Prop Velocity"), args.doubleVal("Maximum Prop Velocity"));

		boost::char_separator<char> sep(" ");
		boost::tokenizer< boost::char_separator<char> > tokens(args.value("Goal Thresholds"), sep);
		for(auto token : tokens) {
			goalThresholds.push_back(std::stod(token));
		}
	}

	DistanceEvaluator getDistanceEvaluator() const {
		return DistanceEvaluator();
	}

	AbstractDistanceEvaluator getAbstractDistanceEvaluator() const {
		return DistanceEvaluator();
	}

	unsigned int getTreeStateSize() const {
		return 12;
	}

	unsigned int getTreeAbstractStateSize() const {
		return 3;
	}

	StateVarRanges getStateVarRanges(const WorkspaceBounds &b) const {
		StateVarRanges bounds(b.begin(), b.end()); //XYZ
		bounds.emplace_back(minimumVelocity, maximumVelocity); //DX
		bounds.emplace_back(minimumVelocity, maximumVelocity); //DY
		bounds.emplace_back(minimumVelocity, maximumVelocity); //DZ
		bounds.emplace_back(-M_PI, M_PI); //ROLL
		bounds.emplace_back(-M_PI, M_PI); //PITCH
		bounds.emplace_back(-M_PI, M_PI); //YAW
		bounds.emplace_back(minimumAngularVelocity, maximumAngularVelocity); //DROLL
		bounds.emplace_back(minimumAngularVelocity, maximumAngularVelocity); //DPITCH
		bounds.emplace_back(minimumAngularVelocity, maximumAngularVelocity); //DYAW

		return bounds;
	}

	StateVarRanges getAbstractStateVarRanges(const WorkspaceBounds &b) const {
		StateVarRanges bounds(b.begin(), b.end()); //XYZ
		bounds.emplace_back(-M_PI, M_PI); //ROLL
		bounds.emplace_back(-M_PI, M_PI); //PITCH
		bounds.emplace_back(-M_PI, M_PI); //YAW
		return bounds;
	}

	State buildState(const StateVars &stateVars) const {
		return State(stateVars);
	}

	Control controlFromVector(const std::vector<double> &controls) const {
		return controls;
	}

	const std::vector< std::pair<double, double> > &getControlBounds() const {
		return controlBounds;
	}

	State getRandomStateNearAbstractState(const AbstractState &s, double radius) const {
		State concrete(s);

		concrete.stateVars[DX] = zeroToOne(GlobalRandomGenerator) * 2 - 1;
		concrete.stateVars[DY] = zeroToOne(GlobalRandomGenerator) * 2 - 1;
		concrete.stateVars[DZ] = zeroToOne(GlobalRandomGenerator) * 2 - 1;

		auto quat = math::getRandomUnitQuaternion();
		fcl::Vec3f axes;
		quat.toAxes(&axes);

		concrete.stateVars[ROLL] = axes[0];
		concrete.stateVars[PITCH] = axes[1];
		concrete.stateVars[YAW] = axes[2];

		double angularRange = (maximumAngularVelocity - minimumAngularVelocity);

		concrete.stateVars[DROLL] = (zeroToOne(GlobalRandomGenerator) * angularRange) + minimumAngularVelocity;
		concrete.stateVars[DPITCH] = (zeroToOne(GlobalRandomGenerator) * angularRange) + minimumAngularVelocity;
		concrete.stateVars[DYAW] = (zeroToOne(GlobalRandomGenerator) * angularRange) + minimumAngularVelocity;

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
		       fabs(s[Y] - g[Y]) < goalThresholds[Y] &&
		       fabs(s[Z] - g[Z]) < goalThresholds[Z];
	}

	Edge steerWithControl(const State &start, const Edge &getControlsFromThisEdge, double dt) const {
		double p0 = getControlsFromThisEdge.p0;
		double p1 = getControlsFromThisEdge.p1;
		double p2 = getControlsFromThisEdge.p2;
		double p3 = getControlsFromThisEdge.p3;

		State end = doSteps(start, p0, p1, p2, p3, dt);

		return Edge(start, end, dt, p0, p1, p2, p3);
	}

	Edge steerWithControl(const State &start, const std::vector<double> controls, double dt) const {
		/* Be careful about the order these are being passed in */

		double p0 = controls[0];
		double p1 = controls[1];
		double p2 = controls[2];
		double p3 = controls[3];

		State end = doSteps(start, p0, p1, p2, p3, dt);

		return Edge(start, end, dt, p0, p1, p2, p3);
	}

	double steerDistancePeek(const State &a, const State &b) const {
		const StateVars &av = a.getStateVars();
		const StateVars &bv = b.getStateVars();

		double sum = 0;
		for(unsigned int i = 0; i < 3; ++i) {
			sum += fabs(av[i] - bv[i]);
		}
		return sum;
	}

	Edge steer(const State &start, const State &goal, double dt) const {
		Edge bestEdge(start);
		double bestDistance = std::numeric_limits<double>::infinity();
		for(unsigned int i = 0; i < 10; ++i) {
			Edge e = randomSteer(start, dt);
			double d = steerDistancePeek(e.end, goal);
			if(d < bestDistance) {
				bestDistance = d;
				bestEdge = e;
			}
		}

		return bestEdge;
	}

	Edge randomSteer(const State &start, double dt) const {
		double p0 = propVelocity(GlobalRandomGenerator);
		double p1 = propVelocity(GlobalRandomGenerator);
		double p2 = propVelocity(GlobalRandomGenerator);
		double p3 = propVelocity(GlobalRandomGenerator);

		State end = doSteps(start, p0, p1, p2, p3, dt);

		return Edge(start, end, dt, p0, p1, p2, p3);
	}

	Edge constructEdge(const State &start, const State &end) const {
		fprintf(stderr, "Quadcopter::constructEdge not implemented\n");
		exit(1);
	}

	std::vector<const SimpleAgentMeshHandler *> getMeshes() const {
		std::vector<const SimpleAgentMeshHandler *> meshes(1, &mesh);
		return meshes;
	}

	std::vector<const SimpleAgentMeshHandler *> getAbstractMeshes() const {
		return getMeshes();
	}

	AbstractState toAbstractState(const State& state) const {
		AbstractState s;
		s.treeStateVars.push_back(state.stateVars[X]);
		s.treeStateVars.push_back(state.stateVars[Y]);
		s.treeStateVars.push_back(state.stateVars[Z]);
		s.transform = state.toFCLTransform();
		return s;
	}

	std::vector<State> getRepresentiveStatesForLocation(const std::vector<double> &loc) const {
		std::vector<State> states;

		fprintf(stderr, "Quadcopter::getRepresentiveStatesForLocation not implemented\n");
		exit(1);

		// fcl::Vec3f pose(loc[0], loc[1], loc[2]);

		// unsigned int rotations = 4;
		// double increment = M_PI / ((double)rotations * 2.);
		// fcl::Matrix3f rotation;
		// rotation.setIdentity();

		// for(unsigned int i = 0; i < rotations; ++i) {
		// 	double cosTheta = cos((double)i * increment);
		// 	double sinTheta = sin((double)i * increment);

		// 	rotation(0,0) = cosTheta;
		// 	rotation(1,0) = -sinTheta;
		// 	rotation(0,1) = sinTheta;
		// 	rotation(1,1) = cosTheta;

		// 	poses.emplace_back(rotation, pose);
		// }

		return states;
	}

	std::vector<std::vector<fcl::Transform3f> > getMeshPoses(const Edge &edge, double dt) const {
		std::vector<std::vector<fcl::Transform3f> > retPoses;

		unsigned int steps = std::isinf(dt) ? 0 : edge.dt / dt;
		
		State state = edge.start;

		retPoses.emplace_back();
		retPoses.back().push_back(state.toFCLTransform());

		for(unsigned int step = 0; step < steps; ++step) {
			state = doSteps(edge.start, edge.p0, edge.p1, edge.p2, edge.p3, dt * (double)step);

			retPoses.emplace_back();

			// drawMesh(state);

			retPoses.back().push_back(state.toFCLTransform());
		}

		retPoses.emplace_back();
		retPoses.back().push_back(edge.end.toFCLTransform());

		return retPoses;
	}

	inline bool areAbstractEdgesSymmetric() const {
		return true;
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

#ifdef WITHGRAPHICS
	void drawMesh(const State &s) const {
		auto transform = s.toOpenGLTransform();
		mesh.draw(color, transform);
	}

	void drawMesh(const fcl::Transform3f &transform) const {
		drawMesh(transform, color);

	}
	void drawMesh(const fcl::Transform3f &transform, const OpenGLWrapper::Color &color) const {
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

	void drawSolution(const std::vector<const Edge *> &solution, double dt = std::numeric_limits<double>::infinity()) const {
		for(const Edge *edge : solution) {
			unsigned int steps = std::isinf(dt) ? 1 : edge->dt / dt;

			State state = edge->start;

			edge->draw(OpenGLWrapper::Color::Green());

			for(unsigned int step = 0; step < steps; ++step) {
				auto transform = state.toOpenGLTransform();
				mesh.draw(color, transform);
				state.draw(OpenGLWrapper::Color::Green());

				state = doSteps(state, edge->p0, edge->p1, edge->p2, edge->p3, dt, true);
			}
		}
	}

	void animateSolution(const std::vector<const Edge *> &solution, unsigned int poseNumber) const {
		unsigned int edgeNumber = poseNumber / 2;
		unsigned int endpoint = poseNumber % 2;
		const Edge *edge = solution[edgeNumber];

		auto transform = (endpoint == 0 ? edge->start : edge->end).toOpenGLTransform();
		mesh.draw(color, transform);
	}

#endif

// private:

	State doSteps(const State &s, double p0, double p1, double p2, double p3, double dt, bool ignoreIntegrationStepSize = false) const {
		const StateVars &vars = s.getStateVars();
		StateVars newState = vars;

		unsigned int steps = ignoreIntegrationStepSize ? 0 : dt / integrationStepSize;

		double leftOver = ignoreIntegrationStepSize ? dt : dt - ((double)steps * integrationStepSize);

		assert(leftOver >= 0);

		double stepSize = integrationStepSize;

		double U1 = quadThrust * (p0 * p0 + p1 * p1 + p2 * p2 + p3 * p3);
		double U2 = quadThrust * quadRadius * (-p1 * p1 + p3 * p3);
		double U3 = quadThrust * quadRadius * (-p0 * p0 + p2 * p2);
		double U4 = quadDrag * (-p0 * p0 + p1 * p1 - p2 * p2 + p3 * p3);

		for(unsigned int i = 0; i < steps+1; ++i) {

			if(i == steps) {
				stepSize = leftOver;
			}

			double ddx = (sin(newState[YAW]) * sin(newState[ROLL]) + cos(newState[YAW]) * sin(newState[PITCH]) * cos(newState[ROLL])) * U1 / quadMass;
			double ddy = (-cos(newState[YAW]) * sin(newState[ROLL]) + sin(newState[YAW]) * sin(newState[PITCH]) * cos(newState[ROLL])) * U1 / quadMass;
			double ddz = -9.81 + (cos(newState[PITCH]) * cos(newState[ROLL])) * U1 / quadMass;

			newState[DX] += ddx * stepSize;
			newState[DY] += ddy * stepSize;
			newState[DZ] += ddz * stepSize;

			newState[X] += newState[DX] * stepSize;
			newState[Y] += newState[DY] * stepSize;
			newState[Z] += newState[DZ] * stepSize;

			double ddroll = (quadIyy - quadIzz) / quadIxx * newState[DPITCH] * newState[DYAW] - U2 / quadIxx;
			double ddpitch = (quadIzz - quadIxx) / quadIyy * newState[DROLL] * newState[DYAW] - U3 / quadIyy;
			double ddyaw = (quadIxx - quadIyy) / quadIzz * newState[DROLL] * newState[DPITCH] - U4 / quadIzz;

			newState[DROLL] += ddroll * stepSize;
			newState[DPITCH] += ddpitch * stepSize;
			newState[DYAW] += ddyaw * stepSize;

			newState[ROLL] = normalizeTheta(newState[ROLL] + newState[DROLL] * stepSize);
			newState[PITCH] = normalizeTheta(newState[PITCH] + newState[DPITCH] * stepSize);
			newState[YAW] = normalizeTheta(newState[YAW] + newState[DYAW] * stepSize);

			for(unsigned int i = DX; i <= DZ; i++) {
				if(newState[i] > maximumVelocity) {
					newState[i] = maximumVelocity;
				} else if(newState[i] < minimumVelocity) {
					newState[i] = minimumVelocity;
				}
			}

			for(unsigned int i = DROLL; i <= DYAW; i++) {
				if(newState[i] > maximumAngularVelocity) {
					newState[i] = maximumAngularVelocity;
				} else if(newState[i] < minimumAngularVelocity) {
					newState[i] = minimumAngularVelocity;
				}
			}
		}

		return State(newState);
	}

	double normalizeTheta(double t) const {
		return (t - 2 * M_PI * std::floor((t + M_PI) / (2 * M_PI)));
	}

	BoxHandler mesh;
	double quadRadius, quadInnerRadius, quadMass, quadIxx, quadIyy, quadIzz, quadThrust, quadDrag;
	double minimumVelocity, maximumVelocity, minimumAngularVelocity, maximumAngularVelocity;
	double integrationStepSize;
	mutable std::uniform_real_distribution<double> propVelocity, zeroToOne;

	std::vector< std::pair<double, double> > controlBounds;

	std::vector<double> goalThresholds;

#ifdef WITHGRAPHICS
	const OpenGLWrapper::Color color;
	mutable State state;
#endif
};