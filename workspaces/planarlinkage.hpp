#pragma once

#include "../utilities/instancefilemap.hpp"
#include <vector>
#include <math.h>
#include <random>

namespace Planar {
	class PlanarVector {
	public:
		PlanarVector(double f = 0.0f) : x(f),
										y(f) {
		}

		PlanarVector(double x, double y) : x(x),
										   y(y) {
		}

		PlanarVector(const PlanarVector &) = default;

		PlanarVector(PlanarVector &&) = default;

		PlanarVector &operator=(const PlanarVector &) = default;

		PlanarVector &operator=(PlanarVector &&) = default;

		bool operator==(PlanarVector &rhs) {
			return x == rhs.x && y == rhs.y;
		}

		double x, y;
	};

/**
 * From: http://paulbourke.net/geometry/pointlineplane/
 * Based on the implementation of Damian Coventry
 */
	class LineSegment {
	public:
		PlanarVector begin;
		PlanarVector end;

		LineSegment() {
		}

		LineSegment(const PlanarVector &begin, const PlanarVector &end) : begin(begin),
																		  end(end) {
		}

		enum class IntersectResult {
			PARALLEL, COINCIDENT, NOT_INTERESECTING, INTERESECTING, OVERLAPPING
		};

		IntersectResult intersect(const LineSegment &segment, PlanarVector &intersection) {
			double denom = ((segment.end.y - segment.begin.y) * (end.x - begin.x)) -
						   ((segment.end.x - segment.begin.x) * (end.y - begin.y));

			double numerA = ((segment.end.x - segment.begin.x) * (begin.y - segment.begin.y)) -
							((segment.end.y - segment.begin.y) * (begin.x - segment.begin.x));

			double numberB = ((end.x - begin.x) * (begin.y - segment.begin.y)) -
							 ((end.y - begin.y) * (begin.x - segment.begin.x));

			if (denom == 0.0f) {
				if (numerA == 0.0f && numberB == 0.0f) {
					// The lines are coincident. Check if the segments are overlapping or not.
					// It is sufficient to check only one dimension, since the points are on the same line.

					double localMin = std::min(begin.x, end.x);
					double localMax = std::max(begin.x, end.x);

					double segmentMin = std::min(segment.begin.x, segment.end.x);
					double segmentMax = std::max(segment.begin.x, segment.end.x);

					return (localMax <= segmentMin || segmentMax <= localMin) ? IntersectResult::PARALLEL
																			  : IntersectResult::OVERLAPPING;
//					return IntersectResult::COINCIDENT;
				}

				return IntersectResult::PARALLEL;
			}

			double ua = numerA / denom;
			double ub = numberB / denom;

			if (ua >= 0.0f && ua <= 1.0f && ub >= 0.0f && ub <= 1.0f) {
				// Get the intersection point.
				intersection.x = begin.x + ua * (end.x - begin.x);
				intersection.y = begin.y + ua * (end.y - begin.y);

				return IntersectResult::INTERESECTING;
			}

			return IntersectResult::NOT_INTERESECTING;
		}

#ifdef WITHGRAPHICS

		void draw(const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {
			OpenGLWrapper::getOpenGLWrapper()
					.drawLine(begin.x, begin.y, 0, end.x, end.y, 0, OpenGLWrapper::Color::Blue());
		}

#endif
	};

/**
 * @return True if the segments are coincident or intersecting.
 */
	bool checkCollision(LineSegment segment1, LineSegment segment2) {
		PlanarVector intersection;

		LineSegment::IntersectResult intersectResult = segment1.intersect(segment2, intersection);

		// Filter out joint collision for neighbor segments.
		if (segment1.end == segment2.begin || segment2.end == segment1.begin) {
			return intersectResult == LineSegment::IntersectResult::COINCIDENT;
		}

		return intersectResult == LineSegment::IntersectResult::COINCIDENT ||
			   intersectResult == LineSegment::IntersectResult::INTERESECTING;
	}
}

class Link {
public:
	Link()
			: segment(),
			  angle(0) {
	}

	Planar::PlanarVector updateLineSegment(const Planar::PlanarVector startPoint, const double absAngle) {

		Planar::PlanarVector endPoint;

		endPoint.x = startPoint.x - std::cos(absAngle);
		endPoint.y = startPoint.y + std::sin(absAngle);

		segment.begin = startPoint;
		segment.end = endPoint;

		return endPoint;
	}

	double getAngle() const {
		return angle;
	}

	void setAngle(const double angle) {
		this->angle = angle;
	}

	void addAngle(const double angle) {
		this->angle += angle;
	}

	Planar::LineSegment getSegment() const {
		return segment;
	}

#ifdef WITHGRAPHICS

	void draw(const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {
		segment.draw(color);
	}

#endif

private:
	double angle;
	Planar::LineSegment segment;
};

class PlanarLinkage {
public:
	typedef std::vector<double> StateVars;
	typedef std::vector<double> Control;

	typedef std::vector<std::pair<double, double> > WorkspaceBounds;
	typedef std::vector<std::pair<double, double> > StateVarRanges;

	class AbstractState;
	class State;
	class Edge;

	typedef std::vector<AbstractState> AbstractEdge;

	class AbstractState {
	public:
		AbstractState(StateVars stateVars) : stateVars(std::move(stateVars)) {
			stateVars.resize(getTreeAbstractStateSize());
		}

		const StateVars &getTreeStateVars() const {
			return stateVars;
		}

		static AbstractState getRandomAbstractState(const std::vector<std::pair<double, double>> &bounds) {
			std::vector<double> stateVars;
			for (int i = 0; i < getTreeAbstractStateSize(); ++i) {
				std::uniform_real_distribution<double> distribution(bounds[i].first, bounds[i].second);
				stateVars.push_back(distribution(GlobalRandomGenerator));
			}

			return AbstractState(std::move(stateVars));
		}

		static double evaluateDistance(const AbstractState &lhs, const AbstractState &rhs) {
			return euclideanDistance(lhs.getTreeStateVars(), rhs.getTreeStateVars());
		}

	private:
		StateVars stateVars;
	};

	class State {
	public:
		State(const int numberOfLinks) : treeStateVars(numberOfLinks),
										 links(numberOfLinks) {
			setAngles(treeStateVars);
		}

		State(const State &) = default;

		State(State &&) = default;

		State &operator=(const State &) = default;

		State &operator=(State &&) = default;

		explicit State(StateVars vars) : treeStateVars(std::move(vars)),
										 links(treeStateVars.size()) {
			setAngles(treeStateVars);
		}

		const StateVars &getStateVars() const {
			return treeStateVars;
		}

		bool equals(const State &s) const {
			for (unsigned int i = 0; i < getStateVars().size(); ++i) {
				if (fabs(getStateVars()[i] - s.getStateVars()[i]) > 0.000001) {
					return false;
				}
			}
			return true;
		}

		void print() const {
			const auto &stateVars = getStateVars();
			for (auto v : stateVars) {
				fprintf(stderr, "%g ", v);
			}
			fprintf(stderr, "\n");
		}

		void move(Control control) {
			const int numberOfLinks = links.size();
			assert(control.size() == numberOfLinks);

			Planar::PlanarVector currentEnd;
			double absAngle = 0;
			for (int i = 0; i < numberOfLinks; ++i) {
				double angle = control[i];
				links[i].addAngle(angle);

				absAngle += links[i].getAngle();
				currentEnd = links[i].updateLineSegment(currentEnd, absAngle);
			}
		}

		void setAngles(std::vector<double> angles) {
			const int numberOfLinks = links.size();
			assert(angles.size() == numberOfLinks);

			Planar::PlanarVector currentEnd;
			double absAngle = 0;
			for (int i = 0; i < numberOfLinks; ++i) {
				double angle = angles[i];
				links[i].setAngle(angle);

				absAngle += links[i].getAngle();
				currentEnd = links[i].updateLineSegment(currentEnd, absAngle);
			}
		}

		bool checkCollision(Link link1, Link link2) const {
			return Planar::checkCollision(link1.getSegment(), link2.getSegment());
		}

		bool hasCollision() const {
			const int size = links.size();

			for (int i = 0; i < size; ++i) {
				for (int j = i + 1; j < size; ++j) {
					if (checkCollision(links[i], links[j])) {
						return true;
					}
				}
			}

			return false;
		}

//		StateVars getEndEffectorLocation() const {
//			const Link &lastLink = links.back();
//			const Planar::PlanarVector &endEffector = lastLink.getSegment().end;
//
//			StateVars endEffectorLocation = {endEffector.x, endEffector.y};
//			return endEffectorLocation;
//		}

#ifdef WITHGRAPHICS

		void draw(const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {
			for (Link link : links) {
				link.draw(color);
			}
		}

		void draw2DAbstractEdge(const AbstractState &state, const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {

		}

#endif

		static std::vector<State> interpolate(const State &a, const State &b, const double dt) {
			const int dim = a.getStateVars().size();
			std::vector<State> intermediateStates;

			BOOST_ASSERT(dim == b.getStateVars().size());
			BOOST_ASSERT(dim > 0);

			// Find largest difference
			double maxDifference = 0;
//			int maxDifferenceIndex = 0;
			for (int i = 0; i < dim; ++i) {
				double difference = std::abs(a.getStateVars()[i] - b.getStateVars()[i]);
				if (difference > maxDifference) {
					maxDifference = difference;
//					maxDifferenceIndex = i;
				}
			}

			const int numberOfSteps = maxDifference / dt;

			// If there are no intermediate states
			if (numberOfSteps == 0) {
				return intermediateStates;
			}

			// Calculate step sizes
			std::vector<double> stepSizes(dim);
			for (int i = 0; i < dim; ++i) {
				double difference = b.getStateVars()[i] - a.getStateVars()[i];
				stepSizes[i] = difference / numberOfSteps;
			}

			// Generate intermediate states;
			for (int i = 0; i < numberOfSteps; ++i) {

				StateVars intermediateStateVars(dim);

				for (int j = 0; j < dim; ++j) {
					intermediateStateVars[j] = a.getStateVars()[j] + i * stepSizes[j];
				}

				intermediateStates.emplace_back(std::move(intermediateStateVars));
			}

			return intermediateStates;
		}

		static double evaluateDistance(const State &lhs, const State &rhs) {
			const auto &rhsStateVars = rhs.getStateVars();
			const auto &lhsStateVars = lhs.getStateVars();

			return PlanarLinkage::euclideanDistance(rhsStateVars, lhsStateVars);
		}

		StateVars treeStateVars;
	private:
		std::vector<Link> links;
	};

	class Edge {
	public:
		Edge(const State &s) : start(s),
							   end(s),
							   duration(0),
							   g(0),
							   parent(NULL) {
		}

		Edge(const State start, const State end, double cost, const std::vector<double> control, double duration)
				: start(std::move(start)),
				  end(std::move(end)),
				  cost(cost),
				  g(0),
				  treeIndex(0),
				  duration(duration),
				  control(std::move(control)),
				  parent(NULL) {
		}

		Edge(const Edge &) = default;

		Edge(Edge &&) = default;

		Edge &operator=(const Edge &) = default;

		Edge &operator=(Edge &&) = default;

		double gCost() const {
			return g;
		}

		void updateParent(Edge *p) {
			parent = p;
			g = p->gCost() + cost;
		}

		void print() {
			start.print();
			end.print();
		}

#ifdef WITHGRAPHICS

		void draw(const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {
		}

#endif

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

		Control getControl() const {
			return control;
		}

		State start, end;
		double cost, duration, g;
		int treeIndex;
		Edge *parent;
		Control control;
	};

	PlanarLinkage(const InstanceFileMap &args) : numberOfLinks(args.integerVal("Dimensions")),
												 workspaceBounds(numberOfLinks) {

		for (int i = 0; i < workspaceBounds.size(); ++i) {
			workspaceBounds[i].first = -M_PI;
			workspaceBounds[i].second = M_PI;
		}

		const auto &stateVarDomains = getStateVarRanges(workspaceBounds);
		for (const auto &range : stateVarDomains) {
			distributions.emplace_back(range.first, range.second);
		}

		auto tokens = args.doubleList("Goal Thresholds");
		for (const auto &token : tokens) {
			goalThresholds.push_back(token);
		}
	}

	unsigned int getTreeStateSize() const {
		return numberOfLinks;
	}

	static unsigned int getTreeAbstractStateSize() {
		//this should at some point not be a hardcoded value but reference
		//a reasonable place where it is actually defined....
		return 2;
	}

	StateVarRanges getStateVarRanges(const WorkspaceBounds &bounds) const {
		return bounds;
	}

	bool isGoal(const State &state, const State &goal) const {
		const auto &targetStateVars = state.getStateVars();
		const auto &goalStateVars = goal.getStateVars();
		const int numberOfLinks = targetStateVars.size();

		assert(numberOfLinks == goalStateVars.size());

		double difference = 0;

		for (int i = 0; i < numberOfLinks; ++i) {
			difference = targetStateVars[i] - goalStateVars[i];

			if (std::abs(difference) > goalThresholds[i]) {
				return false;
			}
		}
		return true;
	}

	Edge randomSteer(const State &start, double dt) const {
		const int dimensions = start.getStateVars().size();
		StateVars stateVars(dimensions);

		for (int i = 0; i < dimensions; ++i) {
			double v = distributions[i](GlobalRandomGenerator);
			stateVars[i] = start.getStateVars()[i] + v;
		}

		return steer(start, State(std::move(stateVars)), dt);
	}

	Edge steer(const State &start, const State &goal, double dt) const {
		const double dist = State::evaluateDistance(start, goal);
		const double ratio = dt / dist;
		const double scale = std::min(ratio, 1.0);

		const auto &startVars = start.getStateVars();
		const auto &goalVars = goal.getStateVars();
		const int dimensions = startVars.size();

		BOOST_ASSERT(goalVars.size() == dimensions);

		StateVars stateVars(dimensions);
		Control controls(dimensions);

		for (int i = 0; i < dimensions; ++i) {
			double diff = goalVars[i] - startVars[i];
			controls[i] = diff * scale;
			stateVars[i] = startVars[i] + controls[i];
		}

		return Edge(start, buildState(std::move(stateVars)), dist * scale, controls, dist * scale);
	}

	Edge steerWithControl(const State &start, const Edge &getControlFromThisEdge, double dt) const {
		return steerWithControl(start, getControlFromThisEdge.getControl(), dt);
	}

	Edge steerWithControl(const State &start, const Control controls, double dt) const {
		const int numberOfLinks = start.getStateVars().size();
		StateVars stateVars(numberOfLinks);

		BOOST_ASSERT(numberOfLinks == controls.size());

		double max = 0;

		for (int i = 0; i < numberOfLinks; ++i) {
			double v = controls[i];
			max = std::max(max, v);
			stateVars[i] = start.getStateVars()[i] + v;
		}

		return steer(start, State(std::move(stateVars)), dt);
	}

	Edge constructEdge(const State &start, const State &end) const {
		fprintf(stderr, "PlanarLinkage::constructEdge not implemented\n");
		exit(1);
	}

	State buildState(const StateVars &stateVars) const {
		return State(stateVars);
	}

#ifdef WITHGRAPHICS

	void drawMesh() {
	}

	void drawMesh(const State &s) const {
	}

	void drawMesh(const fcl::Transform3f &transform, const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {
	}

	void drawSolution(const fcl::Transform3f &transform, const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {
	}

	void drawSolution(const std::vector<const Edge *> &solution, double dt = std::numeric_limits<
			double>::infinity()) const {
	}

#endif

	WorkspaceBounds getControlBounds() const {
		return workspaceBounds;
	}

	Control controlFromVector(const Control &controls) const {
		return controls;
	}

	/*** Workspace interface ***/
	WorkspaceBounds getBounds() const {
		return workspaceBounds;
	}

	State getRandomStateNearAbstractState(const AbstractState &state, double radius) const {
		StateVars sourceStateVars = state.getTreeStateVars();
		BOOST_ASSERT_MSG(sourceStateVars.size() == getTreeAbstractStateSize(),
						 "Invalid abstract state.");

		sourceStateVars.resize(numberOfLinks);

		// Extend the abstract state with random coordinates
		for (int i = getTreeAbstractStateSize(); i < numberOfLinks; ++i) {
			sourceStateVars[i]  = distributions[i](GlobalRandomGenerator);
		}

		Edge edge = randomSteer(buildState(std::move(sourceStateVars)), radius);
		return edge.end;
	}

	AbstractState toAbstractState(const State &state) const {
		StateVars stateVars = state.getStateVars();
		stateVars.resize(getTreeAbstractStateSize());

		return AbstractState(stateVars);
	}

	State generateRandomState() const {
		StateVars stateVars(numberOfLinks);

		for (int i = 0; i < numberOfLinks; ++i) {
			stateVars[i] = distributions[i](GlobalRandomGenerator);
		}

		return State(std::move(stateVars));
	}

	bool safeEdge(const PlanarLinkage &agent, const Edge &edge, double dt, bool checkSelfCollision = false) const {
		auto intermediateStates = PlanarLinkage::State::interpolate(edge.start, edge.end, dt);
		intermediateStates.push_back(edge.end);
		intermediateStates.push_back(edge.start);

		return safeStates(intermediateStates);
	}

	inline bool areAbstractEdgesSymmetric() const {
		return true;
	}

	AbstractEdge generateAbstractEdge(const AbstractState &s1, const AbstractState &s2) const {
		std::vector<AbstractState> edge = {s1, s2};
		return edge;
	}

	bool safeAbstractEdge(const PlanarLinkage &agent, const AbstractEdge &edge, double dt) const {
		return true;
//		auto intermediateStates = PlanarLinkage::State::interpolate(edge[0], edge[1], dt);
//		intermediateStates.push_back(edge[0]);
//		intermediateStates.push_back(edge[1]);
//
//		return safeStates(agent, intermediateStates);
	}

	bool safeStates(const PlanarLinkage &agent, const std::vector<State> &states) const {
		return safeStates(states);
	}

	bool safeStates(const std::vector<State> &states) const {
		for (const auto &state : states) {
			if (state.hasCollision()) return false;
		}
		return true;
	}

	bool safeState(const PlanarLinkage &pl, const State &state) const {
		return !state.hasCollision();
	}

	bool safeAbstractState(const PlanarLinkage &pl, const AbstractState &state) const {
		return true;
//		return !state.hasCollision();
	}

	bool safeState(const State &state) const {
		return !state.hasCollision();
	}

	State getRandomAbstractState(const std::vector<std::pair<double, double>> &bounds) {
		StateVars stateVars;

		for (std::pair<double, double> lowerUpper : bounds) {
			std::uniform_real_distribution<double> distribution(lowerUpper.first, lowerUpper.second);
			stateVars.emplace_back(distribution(GlobalRandomGenerator));
		}
		BOOST_ASSERT_MSG(getTreeAbstractStateSize() == stateVars.size(), "Invalid abstract state size");

		return State(stateVars);
	}

	static double euclideanDistance(const StateVars &lhsStateVars, const StateVars &rhsStateVars) {
		BOOST_ASSERT_MSG(rhsStateVars.size() == lhsStateVars.size(),
						 "Cannot evaluate the distance of two state vector with different dimensionality.");

		const int dimensions = rhsStateVars.size();

		double linearDistance = 0;
		double euclideanSum = 0;

		for (int i = 0; i < dimensions; ++i) {
			double diff = std::abs(rhsStateVars[i] - lhsStateVars[i]);
			linearDistance += diff;
			euclideanSum += diff * diff;
		}

		return std::sqrt(euclideanSum);
	}

#ifdef WITHGRAPHICS

	void draw() const {
	}

#endif

private:
	const int numberOfLinks;
	std::vector<double> goalThresholds;
	WorkspaceBounds workspaceBounds;

	mutable std::vector<std::uniform_real_distribution<double> > distributions;
};



