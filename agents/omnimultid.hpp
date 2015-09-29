#pragma once

class OmniMultiD {
public:
	typedef std::vector<double> StateVars;
	typedef std::vector<double> Control;

	typedef std::vector <std::pair<double, double>> WorkspaceBounds;
	typedef std::vector <std::pair<double, double>> ControlBounds;
	typedef std::vector <std::pair<double, double>> StateVarRanges;

	class State;

	class Edge;

	typedef State AbstractState;
	typedef std::vector <AbstractState> AbstractEdge;

	typedef flann::L2<double> DistanceEvaluator;
	typedef flann::L2<double> AbstractDistanceEvaluator;

	class State {
	public:
		State(const int dimensions) : treeStateVars(dimensions) {
		}

		State(const State &) = default;

		State(State &&) = default;

		State &operator=(const State &) = default;

		State &operator=(State &&) = default;

		explicit State(StateVars vars) : treeStateVars(std::move(vars)) {
		}

		const StateVars &getTreeStateVars() const {
			return treeStateVars;
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

		// void move(Control control) {
		// 	const int dimensions = treeStateVars.size();
		// 	assert(control.size() == dimensions);

		// 	for (int i = 0; i < dimensions; ++i) {
		// 		treeStateVars[i] = treeStateVars[i] + control[i];
		// 	}
		// }

#ifdef WITHGRAPHICS

		void draw(const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {
			if (treeStateVars.size() > 2) {
				const auto &identity = OpenGLWrapper::getOpenGLWrapper().getIdentity();

				std::vector<double> pt(treeStateVars.begin(), treeStateVars.begin() + 3);
				pt.push_back(1);
				pt.push_back(0);
				pt.push_back(0);
				pt.push_back(1);
				pt.push_back(1);
				pt.insert(pt.end(), color.getColor().begin(), color.getColor().end());
				pt.insert(pt.end(), identity.begin(), identity.end());

				OpenGLWrapper::getOpenGLWrapper().drawPoints(std::move(pt));
			} else {
				const auto &identity = OpenGLWrapper::getOpenGLWrapper().getIdentity();

				std::vector<double> pt(treeStateVars.begin(), treeStateVars.begin() + 2);
				pt.push_back(0);
				pt.push_back(1);
				pt.push_back(0);
				pt.push_back(0);
				pt.push_back(1);
				pt.push_back(1);
				pt.insert(pt.end(), color.getColor().begin(), color.getColor().end());
				pt.insert(pt.end(), identity.begin(), identity.end());

				OpenGLWrapper::getOpenGLWrapper().drawPoints(std::move(pt));
			}
		}

		void draw2DAbstractEdge(const AbstractState &state, const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {
			const auto &identity = OpenGLWrapper::getOpenGLWrapper().getIdentity();

			std::vector<double> line(treeStateVars.begin(), treeStateVars.begin() + 3);
			line.push_back(1);
			line.push_back(0);
			line.push_back(0);
			line.push_back(1);
			line.push_back(1);
			line.insert(line.end(), color.getColor().begin(), color.getColor().end());
			line.insert(line.end(), identity.begin(), identity.end());

			line.insert(line.end(), state.treeStateVars.begin(), state.treeStateVars.begin() + 3);
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

		static std::vector <State> interpolate(const State &a, const State &b, const double dt) {
			const int dimensions = a.getStateVars().size();
			std::vector <State> intermediateStates;

//			std::cerr << "Dim: " << dimensions << std::endl << " VarSize: "<< b.getStateVars().size() << std::endl;
			BOOST_ASSERT(dimensions == b.getStateVars().size());
			BOOST_ASSERT(dimensions > 0);

			// Find largest difference
			double maxDifference = 0;
//			int maxDifferenceIndex = 0;
			for (int i = 0; i < dimensions; ++i) {
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
			std::vector<double> stepSizes(dimensions);
			for (int i = 0; i < dimensions; ++i) {
				double difference = b.getStateVars()[i] - a.getStateVars()[i];
				stepSizes[i] = difference / numberOfSteps;
			}

			// Generate intermediate states;
			for (int i = 0; i < numberOfSteps; ++i) {

				StateVars intermediateStateVars(dimensions);

				for (int j = 0; j < dimensions; ++j) {
					intermediateStateVars[j] = a.getStateVars()[j] + i * stepSizes[j];
				}

				intermediateStates.emplace_back(std::move(intermediateStateVars));
			}

			return intermediateStates;
		}

		static State getRandomAbstractState(const std::vector <std::pair<double, double>> &bounds) {
			std::vector<double> stateVars;

			//2 is hardcoded right now, it should be using OmniMultiD::abstractDimensions
			for (unsigned int i = 0; i < 2; i++) {
				const auto &lowerUpper = bounds[i];
				std::uniform_real_distribution<double> distribution(lowerUpper.first, lowerUpper.second);
				stateVars.push_back(distribution(GlobalRandomGenerator));
			}

			return State(stateVars);
		}

		static double evaluateDistance(const State &rhs, const State &lhs) {
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

	private:
		StateVars treeStateVars;
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
			const auto &startVars = start.getStateVars();
			const auto &endVars = end.getStateVars();

			if (startVars.size() > 2) {
				OpenGLWrapper::getOpenGLWrapper()
						.drawLine(startVars[0], startVars[1], startVars[2], endVars[0], endVars[1], endVars[2], color);
			} else {
				OpenGLWrapper::getOpenGLWrapper()
						.drawLine(startVars[0], startVars[1], 0, endVars[0], endVars[1], 0, color);
			}

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

	OmniMultiD(const InstanceFileMap &args) : dimensions(args.integerVal("Dimensions")), abstractDimensions(2),
											  workspaceBounds(dimensions) {

		for (int i = 0; i < dimensions; ++i) {
			workspaceBounds[i].first = 0;
			workspaceBounds[i].second = 1;
		}

		const auto &stateVarDomains = getStateVarRanges(workspaceBounds);

		std::uniform_real_distribution<double> distribution(-1, 1);
		workspaceDistributions.resize(stateVarDomains.size(), distribution);

		boost::char_separator<char> sep(" ");
		boost::tokenizer <boost::char_separator<char>> tokens(args.value("Goal Thresholds"), sep);
		for (const auto &token : tokens) {
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
		return dimensions;
	}

	unsigned int getTreeAbstractStateSize() const {
		return abstractDimensions;
	}

	StateVarRanges getStateVarRanges(const WorkspaceBounds &bounds) const {
		return bounds;
	}

	bool isGoal(const State &state, const State &goal) const {
		const auto &targetStateVars = state.getStateVars();
		const auto &goalStateVars = goal.getStateVars();
		const int numberOfLinks = targetStateVars.size();

		BOOST_ASSERT_MSG(numberOfLinks == goalStateVars.size(),
						 "The goal state and the candidate state must have the same dimensionality");

		BOOST_ASSERT_MSG(numberOfLinks == dimensions,
						 "The goal state and the goal state thresholds must have the same dimensionality");

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
			double v = workspaceDistributions[i](GlobalRandomGenerator);
			stateVars[i] = start.getStateVars()[i] + v;
		}

		return steer(start, State(std::move(stateVars)), dt);
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

	Edge constructEdge(const State &start, const State &end) const {
		Control emptyControls;
		double distance = State::evaluateDistance(start, end);
		return Edge(start, end, distance, emptyControls, distance);
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
		return workspaceBounds; // TODO define control bounds
	}

	Control controlFromVector(const Control &controls) const {
		return controls;
	}

	State getRandomStateNearAbstractState(const AbstractState &state, double radius) const {
		StateVars sourceStateVars = state.getStateVars();
		BOOST_ASSERT_MSG(sourceStateVars.size() == 2, "AbstractState should have exactly 2 dimensions");

		sourceStateVars.resize(dimensions);

		// Extend the abstract state with random coordinates
		for (int i = 2; i < dimensions; ++i) {
			sourceStateVars[i]  = workspaceDistributions[i](GlobalRandomGenerator);
		}

		Edge edge = randomSteer(buildState(std::move(sourceStateVars)), zeroToOne(GlobalRandomGenerator) * radius);
		return edge.end;
	}

	State getRandomStateNearState(const State &state, double radius) const {
		auto stateVars = state.getStateVars();
		auto coords = math::randomPointInUnitHyperSphere(stateVars.size());

		double scale = zeroToOne(GlobalRandomGenerator) * radius;
		for(unsigned int i = 0; i < stateVars.size(); ++i) {
			stateVars[i] += coords[i] * scale;
		}

		return buildState(std::move(stateVars));
	}

	State getRandomState() const {
		StateVars stateVars(dimensions);
		for (int i = 0; i < dimensions; ++i) {
			stateVars[i]  = workspaceDistributions[i](GlobalRandomGenerator);
		}
		return buildState(std::move(stateVars));
	}

	AbstractState toAbstractState(const State &s) const {
		StateVars stateVars = s.getStateVars();
		stateVars.resize(abstractDimensions);

		return State(std::move(stateVars));
	}

	inline bool areAbstractEdgesSymmetric() const {
		return true;
	}

	AbstractEdge generateAbstractEdge(const AbstractState &s1, const AbstractState &s2) const {
		std::vector<AbstractState> edge = {s1, s2};
		return edge;
	}

#ifdef WITHGRAPHICS

	void draw() const {
	}

#endif

private:
	const int dimensions, abstractDimensions;
	std::vector<double> goalThresholds;
	WorkspaceBounds workspaceBounds; // TODO remove
	ControlBounds controlBounds;

	mutable std::vector <std::uniform_real_distribution<double>> workspaceDistributions;
	mutable std::vector <std::uniform_real_distribution<double>> controlDistributions;
	mutable std::uniform_real_distribution<double> zeroToOne;
};
