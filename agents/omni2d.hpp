#pragma once

class Omni2D {
public:
	typedef std::vector< std::pair<double, double> > WorkspaceBounds;
	typedef std::vector< std::pair<double, double> > StateVarRanges;

	typedef std::vector<double> StateVars;

	class State;
	typedef State AbstractState;
	class Edge;
	typedef Edge AbstractEdge;

	class State {
	public:
		State() : stateVars(2, 0) {}
		State(double x, double y) : stateVars(2) {
			stateVars[0] = x;
			stateVars[1] = y;
		}
		State(const State &s) : stateVars(s.stateVars.begin(), s.stateVars.end()) {}

		State(const StateVars &vars) : stateVars(vars.begin(), vars.begin()+2) {}

		const bool equals(const State &s) const {
			return fabs(stateVars[0] - s.stateVars[0]) <= 0.000001 &&
			       fabs(stateVars[1] - s.stateVars[1]) <= 0.000001;
		}

		const StateVars &getStateVars() const { return stateVars; }

		void print() const {
			for(auto v : stateVars) {
				fprintf(stderr, "%g ", v);
			}
			fprintf(stderr, "\n");
		}

#ifdef WITHGRAPHICS
		void draw(const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {
			const auto &identity = OpenGLWrapper::getOpenGLWrapper().getIdentity();

			std::vector<double> pt(stateVars.begin(), stateVars.end());
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

	private:
		StateVars stateVars;
	};

	class Edge {
	public:
		Edge(const State &start) : start(start), end(start), cost(0), treeIndex(0) {}
		Edge(const State &start, const State &end, double cost) : start(start), end(end), cost(cost), treeIndex(0) {}
		Edge(const Edge &e) : start(e.start), end(e.end), cost(e.cost), treeIndex(e.treeIndex) {}

		const StateVars &getTreeStateVars() const { return end.getStateVars(); }
		int getPointIndex() const { return treeIndex; }
		void setPointIndex(int ptInd) { treeIndex = ptInd; }

#ifdef WITHGRAPHICS
		void draw(const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {
			const auto &identity = OpenGLWrapper::getOpenGLWrapper().getIdentity();
			std::vector<double> line(start.getStateVars().begin(), start.getStateVars().end());
			line.push_back(0);
			line.push_back(1);
			line.push_back(0);
			line.push_back(0);
			line.push_back(1);
			line.push_back(1);
			line.insert(line.end(), color.getColor().begin(), color.getColor().end());
			line.insert(line.end(), identity.begin(), identity.end());

			line.insert(line.end(), end.getStateVars().begin(), end.getStateVars().end());
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

		const State start, end;
		double cost;
		int treeIndex;
	};

	Omni2D(const InstanceFileMap &args) {}

	unsigned int getTreeStateSize() const {
		return 2;
	}

	StateVarRanges getStateVarRanges(const WorkspaceBounds &bounds) const {
		return bounds;
	}

	State buildState(const StateVars &stateVars) const {
		return State(stateVars);
	}

	bool isGoal(const State &state, const State &goal) const {
		const StateVars &stateVars = state.getStateVars();
		const StateVars &goalVars = goal.getStateVars();

		return fabs(stateVars[0] - goalVars[0]) < goalThresholds[0] &&
		       fabs(stateVars[1] - goalVars[1]) < goalThresholds[1];
	}

	Edge steer(const State &start, const State &goal, double dt) const {
		const StateVars &startStateVars = start.getStateVars();
		const StateVars &goalStateVars = goal.getStateVars();

		double startX = startStateVars[0];
		double startY = startStateVars[1];

		double dx = goalStateVars[0] - startX;
		double dy = goalStateVars[1] - startY;

		double dist = sqrt(dx*dx + dy*dy);
		double fraction = dt / dist;
		if(fraction > 1) fraction = 1;


		State state(startX + dx * fraction,
		            startY + dy * fraction);

		return Edge(start, state, dt);
	}

	Edge randomSteer(const State &start, double dt) const {
		const StateVars &startStateVars = start.getStateVars();

		double startX = startStateVars[0];
		double startY = startStateVars[1];

		double randX = (double)rand() / (double)RAND_MAX;
		double randY = (double)rand() / (double)RAND_MAX;

		double dist = sqrt(randX*randX + randY*randY);

		State state(startX + randX / dist,
		            startY + randY / dist);

		return Edge(start, state, dt);
	}

#ifdef WITHGRAPHICS
	void draw() const {
		
	}
#endif

private:
	std::vector<double> goalThresholds;
};