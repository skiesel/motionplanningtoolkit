#pragma once

#include <vector>

#include "../utilities/instancefilemap.hpp"

template<typename Agent>
class NarrowPassage {
public:
	typedef std::vector<std::pair<double, double> > WorkspaceBounds;

	typedef typename Agent::Edge Edge;
	typedef typename Agent::State State;
	typedef typename Agent::AbstractState AbstractState;
	typedef typename Agent::AbstractEdge AbstractEdge;

	NarrowPassage(const InstanceFileMap &args, const bool scaleSpace) : dimensions(args.integerVal("Dimensions")),
																		   width(args.doubleVal("Width")),
																		   thickness(args.doubleVal("Thickness")),
																		   workspaceBounds(dimensions),
																		   scaleObstacles(scaleSpace) {

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
		for (auto state : states) {
			if (!safeAbstractState(agent, state)) {
				return false;
			}
		}

		return true;
	}

	bool safeAbstractStates(const Agent &agent, const std::vector<AbstractState> &states) const {
		for (auto state : states) {
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
		auto stateVars = state.getTreeStateVars();
		BOOST_ASSERT_MSG(stateVars.size() > 1, "Number of dimensions must be more than 1.");

		if (!inBounds(stateVars)) {
			return false;
		}

		return scaleObstacles ? isSafeInHalfSpace(stateVars) : isSafeInFullSpace(stateVars);
	}

	bool inBounds(const typename Agent::StateVars &stateVars) const {
		for (auto var : stateVars) {
			if (var < 0 || var > 1) {
				return false;
			}
		}

		return true;
	}

	bool isSafeInFullSpace(const typename Agent::StateVars &stateVars) const {
		const double x = stateVars[0];
		const double y = stateVars[1];

		const double halfWidth = width / 2;
		const double halfThickness = thickness / 2;

		// Check: y on belt, else safe
		if (std::abs(0.50 - y) > halfThickness) {
			return true;
		}

		// Check: x in hole
		if (std::abs(0.50 - x) > halfWidth) {
			return false;
		}

		// Check: every other dimension in hole
		for (int i = 2; i < stateVars.size(); ++i) {
			if (std::abs(0.50 - stateVars[i]) > halfWidth) {
				return false;
			}
		}

		return true;
	}

	bool isSafeInHalfSpace(const typename Agent::StateVars &stateVars) const {
		const double x = stateVars[0];
		const double y = stateVars[1];

		const double halfWidth = width / 2;
		const double halfThickness = thickness / 2;

		// Check: y on belt, else safe
		if (std::abs(0.50 - y) > halfThickness) {
			return true;
		}

		// Check: x in hole
		if (std::abs(0.25 - x) > halfWidth && x < 0.50) {
			return false;
		}

		// Check: every other dimension in hole
		for (int i = 2; i < stateVars.size(); ++i) {
			if (std::abs(0.50 - stateVars[i]) > halfWidth) {
				return false;
			}
		}

		return true;
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
	const double thickness;
	WorkspaceBounds workspaceBounds;
	const bool scaleObstacles;
};
