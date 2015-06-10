#pragma once

#include <vector>
#include <random>
#include <boost/thread/barrier.hpp>

class VREPInterface {
public:
	typedef std::vector< std::pair<double, double> > WorkspaceBounds;
	typedef std::vector< std::pair<double, double> > StateVarRanges;

	typedef std::vector<double> StateVars;
	typedef std::vector<double> Control;

	class State {
	public:
		State() {}

		State(const State &s) : stateVars(s.stateVars.begin(), s.stateVars.end()),
		 goalPositionVars(s.goalPositionVars.begin(), s.goalPositionVars.end()),
		 goalOrientationVars(s.goalOrientationVars.begin(), s.goalOrientationVars.end()),
		 poses(s.poses),
		 velocities(s.velocities.begin(), s.velocities.end()),
		 targetVelocities(s.targetVelocities.begin(), s.targetVelocities.end()),
		 targetPositions(s.targetPositions.begin(), s.targetPositions.end()) {}

		State(const StateVars &vars) : stateVars(vars.begin(), vars.end()) {
			stateVars.resize(fullStateSize);
		}

		const StateVars &getStateVars() const { return stateVars; }

		const StateVars &getGoalPositionStateVars() const { return goalPositionVars; }

		const StateVars &getGoalOrientationStateVars() const { return goalOrientationVars; }

		bool equals(const State &s) const {
			for(unsigned int i = 0; i < stateVars.size(); ++i) {
				if(fabs(stateVars[i] - s.stateVars[i]) > 0.000001) {
					return false;
				}
			}
			return true;
		}

		void print() const {
			for(auto v : stateVars) { fprintf(stderr, "%g ", v); }
			fprintf(stderr, "\n");
		}

		StateVars stateVars, goalPositionVars, goalOrientationVars;
		simChar *poses;
		std::vector<double> velocities, targetVelocities, targetPositions;

		static unsigned int fullStateSize;
	};

	class Edge {
	public:
		Edge(const State &s) : start(s), end(s), controls(0), duration(0), safe(false) {}

		Edge(const State &start, const State &end, double cost, const std::vector<double> &controls, double duration, bool safe) : start(start),
			end(end), cost(cost), treeIndex(0), controls(controls.begin(), controls.end()), duration(duration), safe(safe) {}

		Edge(const Edge &e) : start(e.start), end(e.end), cost(e.cost), treeIndex(e.treeIndex), 
			controls(e.controls.begin(), e.controls.end()), duration(e.duration), safe(e.safe) {}

		void print() {
			start.print();
			end.print();
		}

		/* needed for being inserted into NN datastructure */
		const StateVars &getTreeStateVars() const { return end.getStateVars(); }
		int getPointIndex() const { return treeIndex; }
		void setPointIndex(int ptInd) { treeIndex = ptInd; }

		const State start, end;
		double cost, duration;
		std::vector<double> controls;
		int treeIndex;
		bool safe;
	};

	VREPInterface(const InstanceFileMap &args) : simulatorBarrier(2) {
		simLoadScene(args.value("Scene File").c_str());

		std::string agentName = args.value("Agent Handle Name");
		agentHandle = agentName == "EVERYTHING" ? sim_handle_all : simGetObjectHandle(agentName.c_str());

		agentObjectTree = simGetObjectsInTree(agentHandle, sim_handle_all, 0, &agentObjectCount);

		agentCollisionGroupHandle = simGetCollectionHandle(args.value("Agent Collision Group Handle Name").c_str());
		if(agentCollisionGroupHandle < 0) {
			agentCollisionGroupHandle = simGetObjectHandle(args.value("Agent Collision Group Handle Name").c_str());
		}

		collisionCheckAgainstThisGroup = simGetCollectionHandle(args.value("Environment Collision Group Handle Name").c_str());
		if(collisionCheckAgainstThisGroup < 0) {
			collisionCheckAgainstThisGroup = simGetObjectHandle(args.value("Environment Collision Group Handle Name").c_str());
		}

		controllableVelocityJointHandles = getHandleListFromNameList(args.valueList("Controllable Velocity Joint Name List"));
		controllablePositionJointHandles = getHandleListFromNameList(args.valueList("Controllable Position Joint Name List"));

		std::vector<std::string> thresholds = args.valueList("Goal Position Thresholds");
		for(const std::string &thresh : thresholds) {
			goalPositionThresholds.push_back(std::stod(thresh));
		}

		thresholds = args.valueList("Goal Orientation Thresholds");
		for(const std::string &thresh : thresholds) {
			goalOrientationThresholds.push_back(std::stod(thresh));
		}

		std::vector<std::string> box = args.valueList("Sampling Ranges");
		int i = 0;
		for(const std::string &bound : box) {
			int which = i / 2;
			bool isMin = (i % 2) == 0;
			if(isMin) {
				bounds.emplace_back();
				bounds[which].first = std::stod(bound);
			} else {
				bounds[which].second = std::stod(bound);
			}
			i++;
		}

		std::vector<std::string> ranges = args.valueList("Controllable Velocity Joint Ranges");
		i = 0;
		simFloat prev;
		for(const std::string &val : ranges) {
			if((i % 2) == 0) {
				prev = std::stod(val);
			} else {
				simFloat cur = std::stod(val);
				controlBounds.emplace_back(prev, cur);
				controlVelocityDistributions.emplace_back(prev, cur);
			}
			i++;
		}

		ranges = args.valueList("Controllable Position Joint Ranges");
		i = 0;
		for(const std::string &val : ranges) {
			if((i % 2) == 0) {
				prev = std::stod(val);
			} else {
				simFloat cur = std::stod(val);
				controlBounds.emplace_back(prev, cur);
				controlPositionDistributions.emplace_back(prev, cur);
			}
			i++;
		}

		statePositionHandles = getHandleListFromNameList(args.valueList("State Position Name List"));
		stateOrientationHandles = getHandleListFromNameList(args.valueList("State Orientation Name List"));
		stateVelocityHandles = getHandleListFromNameList(args.valueList("State Velocity Name List"));

		statePositionGoalHandle = simGetObjectHandle(args.value("State Position Goal Name").c_str());
		stateOrientationGoalHandle = simGetObjectHandle(args.value("State Orientation Goal Name").c_str());

		State::fullStateSize = treeStateSize = statePositionHandles.size() * 3 + stateOrientationHandles.size() * 3 + stateVelocityHandles.size() * 6;
	}

	~VREPInterface() {}

	unsigned int getTreeStateSize() const {
		return treeStateSize;
	}

	//from the perspective of the environment
	const WorkspaceBounds &getBounds() const {
		return bounds;
	}

	bool safeEdge(const VREPInterface &agent, const Edge &edge, double dt) const {
		return edge.safe;
	}

	bool safePoses(const VREPInterface &agent /*, const std::vector<fcl::Transform3f> &poses*/) const {
		return true;;
	}


	//from the perspective of the agent
	StateVarRanges getStateVarRanges(const WorkspaceBounds &bounds) const {
		return bounds;
	}

	const std::vector< std::pair<double, double> >& getControlBounds() const {
		return controlBounds;
	}

	State buildState(const StateVars &vars) const {
		return State(vars);
	}

	Edge steer(const State &start, const State &goal, double dt) const {
		return randomSteer(start, dt);
	}

	Edge steerWithControl(const State &start, const Control &control, double dt) const {
		return Edge(start);
	}

	Edge randomSteer(const State &start, double dt) const {
		Edge edge(start);
		edge.safe = false;

		loadState(start);

		if(collision()) { return edge; }

		std::vector<double> controls;
		for(unsigned int i = 0; i < controllableVelocityJointHandles.size(); ++i) {
			controls.push_back(controlVelocityDistributions[i](generator));
			simSetJointTargetVelocity(controllableVelocityJointHandles[i], controls.back());
		}

		for(unsigned int i = 0; i < controllablePositionJointHandles.size(); ++i) {
			controls.push_back(controlPositionDistributions[i](generator));
			simSetJointTargetPosition(controllablePositionJointHandles[i], controls.back());
		}

		std::pair<double, bool> result = startSimulation(dt);

		if(result.second || collision()) { return edge; }

		State end;
		saveState(end);

		return Edge(start, end, dt, controls, result.first, true);
	}

	bool isGoal(const State &state, const State &goal) const {
		const StateVars &sPos = state.getGoalPositionStateVars();
		const StateVars &gPos = goal.getGoalPositionStateVars();

		const StateVars &sOri = state.getGoalOrientationStateVars();
		const StateVars &gOri = goal.getGoalOrientationStateVars();

		for(unsigned int i = 0; i < 3; ++i) {
			if(goalPositionThresholds[i] >= 0 && fabs(sPos[i] - gPos[i]) > goalPositionThresholds[i]) {
				return false;
			}
			if(goalOrientationThresholds[i] >= 0 && fabs(sOri[i] - gOri[i]) > goalOrientationThresholds[i]) {
				return false;
			}
		}
		return true;
	}

	void makeStartState(State &start) const {
		saveState(start);
	}

	void animateSolution(const std::vector<const Edge*> &solution) const {
		while(true) {
			for(const Edge* edge : solution) {
				loadState(edge->start);
				unsigned int controlNum = 0;
				const std::vector<double> &controls = edge->controls;
				for(unsigned int i = 0; i < controllableVelocityJointHandles.size(); ++i) {
					simSetJointTargetVelocity(controllableVelocityJointHandles[i], controls[controlNum++]);
				}

				for(unsigned int i = 0; i < controllablePositionJointHandles.size(); ++i) {
					simSetJointTargetPosition(controllablePositionJointHandles[i], controls[controlNum++]);
				}

				startSimulation(edge->duration);
			}
		}
	}

	bool validateSolution(const std::vector<const Edge*> &solution, const State &goal) const {
		for(const Edge* edge : solution) {
			loadState(edge->start);
			unsigned int controlNum = 0;
			const std::vector<double> &controls = edge->controls;
			for(unsigned int i = 0; i < controllableVelocityJointHandles.size(); ++i) {
				simSetJointTargetVelocity(controllableVelocityJointHandles[i], controls[controlNum++]);
			}

			for(unsigned int i = 0; i < controllablePositionJointHandles.size(); ++i) {
				simSetJointTargetPosition(controllablePositionJointHandles[i], controls[controlNum++]);
			}

			std::pair<double, bool> result = startSimulation(edge->duration);

			if(result.second) {
				return false;
			}
		}

		return isGoal(solution[solution.size()-1]->end, goal);
	}

	std::vector<simInt> getHandleListFromNameList(const std::vector<std::string> &list) {
		std::vector<simInt> retList;
		for(const std::string &elem : list) {
			retList.push_back(simGetObjectHandle(elem.c_str()));
		}
		return retList;
	}

	bool collision() const {
		return simCheckCollision(agentCollisionGroupHandle, collisionCheckAgainstThisGroup) == 1;
	}

	void saveState(State& s) const {
		// s.poses = simGetConfigurationTree(sim_handle_all);
		s.poses = simGetConfigurationTree(agentHandle);
		simFloat target;

		simFloat velocity[6];
		for(unsigned int i = 0; i < agentObjectCount; i++) {
			simInt handle = agentObjectTree[i];
			simGetObjectVelocity(handle, velocity, &velocity[3]);

			for(unsigned int j = 0; j < 6; j++) {
				s.velocities.push_back(velocity[j]);
			}

			if(simGetObjectType(handle) == sim_object_joint_type) {
				simGetJointTargetVelocity(handle, &target);
				s.targetVelocities.push_back(target);
				
				simGetJointTargetPosition(handle, &target);
				s.targetPositions.push_back(target);
			}
		}

		simFloat vals[6];

		/* variables used for state comparison */
		for(simInt handle : statePositionHandles) {
			simGetObjectPosition(handle, -1, vals);
			for(unsigned int i = 0; i < 3; ++i)
				s.stateVars.push_back(vals[i]);
		}

		for(simInt handle : stateOrientationHandles) {
			simGetObjectOrientation(handle, -1, vals);
			for(unsigned int i = 0; i < 3; ++i)
				s.stateVars.push_back(vals[i]);
		}

		for(simInt handle : stateVelocityHandles) {
			simGetVelocity(handle, vals, &vals[3]);
			for(unsigned int i = 0; i < 6; ++i)
				s.stateVars.push_back(vals[i]);
		}

		/* variables used for goal checking */
		simGetObjectPosition(statePositionGoalHandle, -1, vals);
		for(unsigned int i = 0; i < 3; ++i)
			s.goalPositionVars.push_back(vals[i]);

		simGetObjectOrientation(stateOrientationGoalHandle, -1, vals);
		for(unsigned int i = 0; i < 3; ++i)
			s.goalOrientationVars.push_back(vals[i]);

	}

	void loadState(const State &s) const {
		unsigned int curTarget = 0;

		simSetConfigurationTree(s.poses);

		for(unsigned int i = 0; i < agentObjectCount; i++) {
			simInt handle = agentObjectTree[i];
			unsigned int indexBase = i * 6;

			simSetObjectFloatParameter(handle, 3000, s.velocities[indexBase]);
			simSetObjectFloatParameter(handle, 3001, s.velocities[indexBase+1]);
			simSetObjectFloatParameter(handle, 3002, s.velocities[indexBase+2]);

			simSetObjectFloatParameter(handle, 3020, s.velocities[indexBase+3]);
			simSetObjectFloatParameter(handle, 3021, s.velocities[indexBase+4]);
			simSetObjectFloatParameter(handle, 3022, s.velocities[indexBase+5]);

			if(simGetObjectType(handle) == sim_object_joint_type) {
				simSetJointTargetVelocity(handle, s.targetVelocities[curTarget]);
				simSetJointTargetPosition(handle, s.targetPositions[curTarget++]);
			}

			simResetDynamicObject(handle);
		}
	}

	std::pair<double, bool> startSimulation(simFloat dt) const {
		simulationDesiredRuntime = dt;

		simulatorBarrier.count_down_and_wait(); // wait until simulator is ready
		//simulator is running
		simulatorBarrier.count_down_and_wait(); // wait until simulator is done

		return std::make_pair(simulationActualRuntime, wasCollision);
	}

	simFloat simulatorReady() {
		simulatorBarrier.count_down_and_wait(); // wait until simulator should start
		wasCollision = false;
		return simulationDesiredRuntime;
	}

	void simulatorDone(simFloat actualDT, bool collided) {
		simulationActualRuntime = actualDT;
		wasCollision = collided;
		
		simulatorBarrier.count_down_and_wait(); // notify simulator is done
	}

	WorkspaceBounds bounds;
	std::vector< std::pair<double, double> > controlBounds;
	std::vector<double> goalPositionThresholds, goalOrientationThresholds;
	int treeStateSize;

	bool wasCollision;

	simInt agentHandle, agentCollisionGroupHandle, collisionCheckAgainstThisGroup, statePositionGoalHandle, stateOrientationGoalHandle;
	simInt* agentObjectTree;
	simInt agentObjectCount;
	
	std::vector<simInt> controllableVelocityJointHandles, controllablePositionJointHandles, statePositionHandles, stateOrientationHandles, stateVelocityHandles;
	mutable std::vector< std::uniform_real_distribution<double> > controlVelocityDistributions, controlPositionDistributions;
	mutable std::default_random_engine generator;

	mutable simFloat simulationDesiredRuntime, simulationActualRuntime;
	mutable boost::barrier simulatorBarrier;
};

unsigned int VREPInterface::State::fullStateSize = 0;