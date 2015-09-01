#pragma once

#pragma once

#include <vector>

#include "../utilities/meshhandler.hpp"
#include "../utilities/instancefilemap.hpp"

class Kink {
public:
	typedef std::vector<std::pair<double, double> > WorkspaceBounds;

	typedef typename Agent::Edge Edge;
	typedef typename Agent::State State;
	typedef typename Agent::AbstractState AbstractState;
	typedef typename Agent::AbstractEdge AbstractEdge;

	Kink(const InstanceFileMap &args) : dimensions(args.integerVal("Dimensions")),
										 bounds(dimensions) {
	}

	const WorkspaceBounds &getBounds() const {
		return bounds;
	}

	bool safeEdge(const Agent &agent, const Edge &edge, double dt, bool checkSelfCollision = false) const {
	}

	bool safeAbstractEdge(const Agent &agent, const AbstractEdge &edge, double dt, bool checkSelfCollision = false) const {
	}

	bool safeStates(const Agent &agent, const std::vector<State> &states) const {
	}

	bool safeAbstractStates(const Agent &agent, const std::vector<AbstractState> &states) const {
	}

	bool safeState(const Agent &agent, const State &state) const {
	}

	bool safeAbstractState(const Agent &agent, const AbstractState &state) const {
	}

#ifdef WITHGRAPHICS

	void draw() const {
	}

	void drawBoundingBox() const {
	}

#endif

private:
	const int dimensions;
	const WorkspaceBounds bounds;
};