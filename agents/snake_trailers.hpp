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
	typedef std::vector<double> Control;

	class State {
	public:
		State() : stateVars(5 + trailerCount) {}

		State(double x, double y, double theta) : stateVars(5 + trailerCount) {
			stateVars[X] = x;
			stateVars[Y] = y;
			stateVars[THETA] = theta;
		}

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
			for(unsigned int i = 0; i < 5 + trailerCount; ++i) {
				if(fabs(stateVars[i] - s.stateVars[i]) > 0.000001) return false;
			}
			return true;
		}

		std::vector<fcl::Transform3f> toFCLTransforms() const {
			std::vector<fcl::Transform3f> transforms;

			fcl::Vec3f axis(0,1,0);

			fcl::Quaternion3f quaternion;
			quaternion.fromAxisAngle(axis, M_PI / 2);
			fcl::Transform3f baseTransform(quaternion);

			//change to z axis
			axis[1] = 0;
			axis[2] = 1;

			quaternion.fromAxisAngle(axis, stateVars[THETA]);
			fcl::Transform3f rotation(quaternion);

			fcl::Vec3f pose;
			pose[0] = stateVars[X];
			pose[1] = stateVars[Y];
			pose[2] = 0;
			fcl::Transform3f translation(pose);

			baseTransform = translation * rotation * baseTransform;

			transforms.emplace_back(baseTransform);

			const auto &pos = transforms.back().getTranslation();

			pose[0] = -(trailerLength + hitchLength);
			pose[1] = 0;

			translation = fcl::Transform3f(pose);

			for(unsigned int i = 1; i < trailerCount + 1; ++i) {
				double t = stateVars[THETA + i] - stateVars[THETA + i - 1];

				quaternion.fromAxisAngle(axis, t);
				rotation = fcl::Transform3f(quaternion);

				baseTransform = translation * rotation * baseTransform;

				transforms.emplace_back(baseTransform);
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
			baseRotation[2] = -sinVal;
			baseRotation[8] = sinVal;
			baseRotation[10] = cosVal;

			auto translate = OpenGLWrapper::getOpenGLWrapper().getIdentity();

			translate[12] = stateVars[X];
			translate[13] = stateVars[Y];

			transforms.push_back(OpenGLWrapper::getOpenGLWrapper().getIdentity());

			math::multiply(baseRotation, translate, transforms.back());

			auto rotate = OpenGLWrapper::getOpenGLWrapper().getIdentity();

			sinVal = sin(stateVars[THETA]);
			cosVal = cos(stateVars[THETA]);

			rotate[0] = cosVal;
			rotate[1] = -sinVal;
			rotate[4] = sinVal;
			rotate[5] = cosVal;

			math::multiply(transforms.back(), rotate, transforms.back());

			translate[13] = 0;
			translate[12] = -(trailerLength + hitchLength);

			for(unsigned int i = 1; i < trailerCount + 1; ++i) {
				transforms.push_back(OpenGLWrapper::getOpenGLWrapper().getIdentity());

				std::vector<double> &previous = transforms[transforms.size() - 2];

				math::multiply(previous, translate, transforms.back());

				double t = stateVars[THETA + i] - stateVars[THETA + i - 1];

				sinVal = sin(t);
				cosVal = cos(t);

				rotate[0] = cosVal;
				rotate[1] = -sinVal;
				rotate[4] = sinVal;
				rotate[5] = cosVal;

				math::multiply(transforms.back(), rotate, transforms.back());
			}
			return transforms;
		}
#endif
		fcl::Transform3f getTransform() const {
			return toFCLTransforms()[0];
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
		static double trailerLength;
		static double hitchLength;

	// private:
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

		Edge& operator=(const Edge& e) {
			start = e.start;
			end = e.end;
			cost = e.cost;
			dt = e.dt;
			a = e.a;
			w = e.w;
			treeIndex = e.treeIndex;
			parent = e.parent;
			return *this;
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
		State start, end;
		double cost, dt, a, w;
		int treeIndex;
		StateVars treeStateVars;
		Edge* parent;

		private:
			void populateTreeStateVars() {
			const auto& vars = end.getStateVars();
			treeStateVars.resize(vars.size());
			for(unsigned int i = 0; i < vars.size(); ++i) {
				treeStateVars[i] = (SnakeTrailers::NormalizeStateVars[i].first + vars[i]) * SnakeTrailers::NormalizeStateVars[i].second;				
			}
		}
	};

	SnakeTrailers(const InstanceFileMap &args) {

		trailerCount = State::trailerCount = stoi(args.value("Trailer Count"));
		trailerWidth = stod(args.value("Trailer Width"));
		trailerLength = State::trailerLength = stod(args.value("Trailer Length"));
		hitchLength = State::hitchLength = stod(args.value("Hitch Length"));

		minimumVelocity = stod(args.value("Minimum Velocity"));
		maximumVelocity = stod(args.value("Maximum Velocity"));

		minimumTurning = stod(args.value("Minimum Turning"));
		maximumTurning = stod(args.value("Maximum Turning"));

		linearAccelerations = std::uniform_real_distribution<double>(stod(args.value("Minimum Velocity")), stod(args.value("Maximum Velocity")));
		angularAccelerations = std::uniform_real_distribution<double>(stod(args.value("Minimum Angular Acceleration")), stod(args.value("Maximum Angular Acceleration")));

		controlBounds.emplace_back(stod(args.value("Minimum Velocity")), stod(args.value("Maximum Velocity")));
		controlBounds.emplace_back(stod(args.value("Minimum Angular Acceleration")), stod(args.value("Maximum Angular Acceleration")));


		auto environmentBoundingBox = args.doubleList("Environment Bounding Box");

		fprintf(stderr, "...ignoring z component in environment bounding box when normalizing tree vars\n");

		for(unsigned int i = 0; i < environmentBoundingBox.size() - 2; i+=2) {
			double term1 = -environmentBoundingBox[i];
			double term2 = 1. / (environmentBoundingBox[i+1] - environmentBoundingBox[i]);
			NormalizeStateVars.emplace_back(term1, term2);
		}
		
		NormalizeStateVars.emplace_back(-minimumVelocity, 1. / (maximumVelocity - minimumVelocity)); //v
		NormalizeStateVars.emplace_back(-minimumTurning, 1. / (maximumTurning - minimumTurning)); //psi
		NormalizeStateVars.emplace_back(M_PI / 2., 1. / (2.*M_PI)); //theta
		for(unsigned int i = 0; i < trailerCount; ++i) {
			NormalizeStateVars.emplace_back(NormalizeStateVars.back());
		}


		boost::char_separator<char> sep(" ");
		boost::tokenizer< boost::char_separator<char> > tokens(args.value("Goal Thresholds"), sep);
		for(auto token : tokens) {
			goalThresholds.push_back(std::stod(token));
		}

		meshes.push_back(new ConeHandler(trailerWidth, trailerLength));
		if(trailerCount > 0) {
			meshes.push_back(new CylinderHandler(trailerWidth, trailerLength));
			for(unsigned int i = 0; i < trailerCount-1; ++i) {
				meshes.push_back(meshes[1]);
			}
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

	Control controlFromVector(const std::vector<double> &controls) const {
		return controls;
	}

	const std::vector< std::pair<double, double> >& getControlBounds() const {
		return controlBounds;
	}

	State buildState(const StateVars& stateVars) const {
		return State(stateVars);
	}

	State transformToState(const State &s, const fcl::Transform3f &transform) const {
		fcl::Quaternion3f orientation = transform.getQuatRotation();
		fcl::Vec3f axis;
		double theta;
		orientation.toAxisAngle(axis, theta);
		theta = (theta - 2 * M_PI * std::floor((theta + M_PI) / (2 * M_PI)));

		fcl::Vec3f position = transform.getTranslation();

		return State(position[0], position[1], theta);
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

		State end = doStep(start, a, w, dt);

		return Edge(start, end, dt, a, w);
	}

	Edge steerWithControl(const State &start, const std::vector<double> controls, double dt) const {
		/* Be careful about the order these are being passed in */

		double a = controls[0];
		double w = controls[1];

		State end = doStep(start, a, w, dt);

		return Edge(start, end, dt, a, w);
	}

	Edge steer(const State &start, const State &goal, double dt) const {
		double a = linearAccelerations(GlobalRandomGenerator);
		double w = angularAccelerations(GlobalRandomGenerator);

		State end = doStep(start, a, w, dt);

		return Edge(start, end, dt, a, w);
	}

	Edge randomSteer(const State &start, double dt) const {
		double a = linearAccelerations(GlobalRandomGenerator);
		double w = angularAccelerations(GlobalRandomGenerator);

		State end = doStep(start, a, w, dt);

		return Edge(start, end, dt, a, w);
	}

	const std::vector<const SimpleAgentMeshHandler*> getMeshes() const {
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
			const std::vector<fcl::Transform3f> transforms = state.toFCLTransforms();

			poses.emplace_back();
			for(const fcl::Transform3f &transform : transforms) {
				poses.back().push_back(transform);
			}

			state = doStep(state, edge.a, edge.w, dt);
		}

		const std::vector<fcl::Transform3f> transforms = edge.end.toFCLTransforms();

		poses.emplace_back();
		for(const fcl::Transform3f &transform : transforms) {
			poses.back().push_back(transform);
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
		double a = 0, w = 0, dt = 0.01;

		for(unsigned int i = 0; i < 10; ++i)
			state = doStep(state, a, w, dt);

		auto transforms = state.toOpenGLTransforms();

		for(unsigned int i = 0; i < meshes.size(); ++i) {
			meshes[i]->draw(color, transforms[i]);
		}
	}

	void drawMesh(const State &s) const {
		auto transforms = s.toOpenGLTransforms();

		for(unsigned int i = 0; i < meshes.size(); ++i) {
			meshes[i]->draw(color, transforms[i]);
		}
	}

	void drawSolution(const std::vector<const Edge*> &solution, double dt = std::numeric_limits<double>::infinity()) const {
		for(const Edge* edge : solution) {
			unsigned int steps = std::isinf(dt) ? 1 : edge->dt / dt;

			State state = edge->start;

			for(unsigned int step = 0; step < steps; ++step) {

				auto transforms = state.toOpenGLTransforms();
				for(unsigned int i = 0; i < meshes.size(); ++i) {
					meshes[i]->draw(color, transforms[i]);
				}

				state = doStep(state, edge->a, edge->w, dt);
			}
		}
	}

	void animateSolution(const std::vector<const Edge*> &solution, unsigned int poseNumber) const {
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

	double normalizeTheta(double t) const {
		return (t - 2 * M_PI * std::floor((t + M_PI) / (2 * M_PI)));
	}

	std::vector<const SimpleAgentMeshHandler*> meshes;

	unsigned int trailerCount;
	double trailerLength, trailerWidth, hitchLength, minimumVelocity, maximumVelocity, minimumTurning, maximumTurning;
	mutable std::uniform_real_distribution<double> linearAccelerations, angularAccelerations;

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


