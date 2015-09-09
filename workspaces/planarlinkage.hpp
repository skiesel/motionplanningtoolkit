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

	Planar::LineSegment getSegment() {
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

	class State;

	class Edge;

	typedef State AbstractState;
	typedef std::vector<AbstractState> AbstractEdge;

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

		const StateVars &getTreeStateVars() const {
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

#ifdef WITHGRAPHICS

		void draw(const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {
			for (Link link : links) {
				link.draw(color);
			}
		}

#endif

		std::vector<State> getTransforms() const {
			return std::vector<State> {*this};
		}

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

		static State getRandomAbstractState(const std::vector<std::pair<double, double>> &bounds) {
			std::vector<double> stateVars;
			for (std::pair<double, double> lowerUpper : bounds) {
				std::uniform_real_distribution<double> distribution(lowerUpper.first, lowerUpper.second);
				stateVars.push_back(distribution(GlobalRandomGenerator));
			}

			return State(stateVars);
		}

		static double evaluateDistance(const State &lhs, const State &rhs) {
			const auto &rhsStateVars = rhs.getStateVars();
			const auto &lhsStateVars = lhs.getStateVars();

			BOOST_ASSERT_MSG(rhsStateVars.size() == lhsStateVars.size(),
							 "Cannot evaluate the distance of two states with different dimensionality.");

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

		boost::char_separator<char> sep(" ");
		boost::tokenizer<boost::char_separator<char> > tokens(args.value("Goal Thresholds"), sep);
		for (const auto &token : tokens) {
			goalThresholds.push_back(std::stod(token));
		}
	}

	unsigned int getTreeStateSize() const {
		return numberOfLinks;
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
		Control controls(dimensions);
		StateVars stateVars(dimensions);

		double max = 0;

		for (int i = 0; i < dimensions; ++i) {
			auto &distribution = distributions[i];
			double v = distribution(GlobalRandomGenerator) * dt;

			max = std::max(max, v);
			controls[i] = v;
			stateVars[i] = start.getStateVars()[i] + v;
		}

		auto newState = buildState(stateVars);
		return Edge(start, newState, State::evaluateDistance(start, newState), controls, max);
	}

	Edge steer(const State &start, const State &goal, double dt) const {
		return randomSteer(start, dt);
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

		auto newState = buildState(stateVars);
		return Edge(start, newState, State::evaluateDistance(start, newState), controls, max);
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
		const auto &sourceStateVars = state.getStateVars();
		const int dimensions = sourceStateVars.size();
		Control controls(dimensions);
		StateVars stateVars(dimensions);

		for (int i = 0; i < dimensions; ++i) {
			auto &distribution = distributions[i];
			double v = distribution(GlobalRandomGenerator);
			controls[i] = v;
		}

		State tempState(controls);
		const double distance = State::evaluateDistance(state, tempState);
		const double ratio = distance < radius ? 1 : radius / distance; // Not uniform!

		for (int i = 0; i < dimensions; ++i) {
			stateVars[i] = sourceStateVars[i] + controls[i] * ratio;
		}

		return State(stateVars);
	}

	AbstractState toAbstractState(const State &s) const {
		return s;
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
//		fprintf(stderr, "PlanarLinkage::generateAbstractEdge not implemented\n");
//		exit(1);

		std::vector <AbstractState> edge = AbstractState::interpolate(s1, s2, 0.1);
		// TODO Check whether the edge should include the end points or not
		// TODO Add dt as parameter

		return edge;
	}

	bool safeAbstractEdge(const PlanarLinkage &agent, const AbstractEdge &edge, double dt) const {
		return safeStates(edge);
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
		return !state.hasCollision();
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

		return State(stateVars);
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



