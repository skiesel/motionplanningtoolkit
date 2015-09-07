#pragma once


#include <vector>

#include "../utilities/instancefilemap.hpp"

template<typename Agent>
class Kink {
public:
	typedef std::vector<std::pair<double, double> > WorkspaceBounds;

	typedef typename Agent::Edge Edge;
	typedef typename Agent::State State;
	typedef typename Agent::AbstractState AbstractState;
	typedef typename Agent::AbstractEdge AbstractEdge;

	Kink(const InstanceFileMap &args) : dimensions(args.integerVal("Dimensions")),
										width(args.doubleVal("Width")),
										workspaceBounds(dimensions) {

		BOOST_ASSERT_MSG(dimensions > 1, "Number of dimensions must be more than 1.");

		for (int i = 0; i < workspaceBounds.size(); ++i) {
			workspaceBounds[i].first = 0;
			workspaceBounds[i].second = 1;
		}
	}

	const WorkspaceBounds &getBounds() const {
		return workspaceBounds;
	}

	bool safeEdge(const Agent &agent, const Edge &edge, double dt, bool checkSelfCollision = false) const {
		auto intermediateStates = State::interpolate(edge.start, edge.end, dt);
		intermediateStates.push_back(edge.end);
		intermediateStates.push_back(edge.start);

		return safeStates(agent, intermediateStates);
	}

	bool safeAbstractEdge(const Agent &agent, const AbstractEdge &edge, double dt, bool checkSelfCollision = false) const {
		return safeAbstractStates(agent, edge);
	}

	bool safeStates(const Agent &agent, const std::vector<State> &states) const {
		for ( const auto &state : states) {
			if (!safeAbstractState(agent, state)) {
				return false;
			}
		}

		return true;
	}

	bool safeAbstractStates(const Agent &agent, const std::vector<AbstractState> &states) const {
		for (const auto &state : states) {
			if (!safeAbstractState(agent, state)) {
				return false;
			}
		}

		return true;
	}

	bool safeState(const Agent &agent, const State &state) const {
		return safeAbstractState(agent, state);
	}

	bool safeAbstractState(const Agent &agent, const AbstractState &state) const {
		const auto &stateVars = state.getTreeStateVars();
		BOOST_ASSERT_MSG(stateVars.size() > 1, "Number of dimensions must be more than 1.");

		const double x = stateVars[0];
		const double y = stateVars[1];
		if (!inBounds(stateVars) || !safeXYDimensions(x, y)) {
			return false;
		}

		const double halfWidth = width / 2;
		for (int i = 2; i < stateVars.size(); ++i) {
			if (std::abs(0.50 - stateVars[i]) > halfWidth) {
				return false;
			}
		}

		return true;
	}

	bool inBounds(typename Agent::StateVars stateVars) const {
		for (auto var : stateVars) {
			if (var < 0 || var > 1) {
				return false;
			}
		}
		return true;
	}

	bool safeXYDimensions(const double x, const double y) const {
		const double halfWidth = width / 2;

		// Check collision against left and right obstacles
		if (x < 0.25 - halfWidth || x > 0.75 + halfWidth) {
			return false;
		}

		// Right vertical path
		if (std::abs(0.75 - x) < halfWidth) {
			// first section from the bottom || second section
			return y < 0.25 + halfWidth || (y > 0.5 - halfWidth && y < 0.75 + halfWidth);
		}

		// Left vertical path
		if (std::abs(0.25 - x) < halfWidth) {
			// first section from the top || second section
			return y > 0.75 + halfWidth || (y < 0.5 + halfWidth && y > 0.25 - halfWidth);
		}

		// x is on the middle path check whether y is on one of the three horizontal paths
		return std::abs(0.25 - y) < halfWidth || std::abs(0.50 - y) < halfWidth || std::abs(0.75 - y) < halfWidth;
	}

#ifdef WITHGRAPHICS

	void draw() const {
	}

	void drawBoundingBox() const {
	}

#endif

private:
	const int dimensions;
	const double width;
	WorkspaceBounds workspaceBounds;
};