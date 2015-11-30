#pragma once

#include <flann/flann.hpp>
#include <boost/pool/object_pool.hpp>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateProjections.h>
#include <ompl/base/ProjectionEvaluator.h>

#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>


template<class Workspace, class Agent>
class KPIECE {
public:

	class StateSpace : public ompl::base::RealVectorStateSpace {
	public:
		class StateType : public ompl::base::RealVectorStateSpace::StateType {
		public:
			typename Agent::Edge *agentEdge;
			bool valid;
		};


		StateSpace(unsigned int dim) : ompl::base::RealVectorStateSpace(dim) {
			ompl::base::StateSpace::type_ = 2222222;
		}

		void copyState(ompl::base::State *destination, const ompl::base::State *source) const {
			ompl::base::RealVectorStateSpace::copyState(destination, source);

			const StateType *src = source->as<StateType>();
			StateType *dest = destination->as<StateType>();

			dest->valid = src->valid;
			dest->agentEdge = src->agentEdge;
		}

		ompl::base::State *allocState() const {
			auto state = new StateType();
			state->values = new double[dimension_];
			return state;
		}

		void freeState(ompl::base::State *s) const {
			// fprintf(stderr, "freeState called... doing nothing...\n");
		}

		bool equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const {
			fprintf(stderr, "equalStates called\n");
			// exit(1);
			return false;
		}

	};

	KPIECE(const Workspace &workspace, const Agent &agent, const InstanceFileMap &args) :
		workspace(workspace), agent(agent), goalEdge(NULL), samplesGenerated(0), edgesAdded(0), edgesRejected(0) {

		collisionCheckDT = args.doubleVal("Collision Check Delta t");
		dfpair(stdout, "collision check dt", "%g", collisionCheckDT);

		const typename Workspace::WorkspaceBounds &agentStateVarRanges = agent.getStateVarRanges(workspace.getBounds());
		stateSpaceDim = agentStateVarRanges.size();

		StateSpace *baseSpace = new StateSpace(stateSpaceDim);
		ompl::base::RealVectorBounds bounds(stateSpaceDim);
		for(unsigned int i = 0; i < stateSpaceDim; ++i) {
			bounds.setLow(i, agentStateVarRanges[i].first);
			bounds.setHigh(i, agentStateVarRanges[i].second);
		}
		baseSpace->setBounds(bounds);
		ompl::base::StateSpacePtr space = ompl::base::StateSpacePtr(baseSpace);

		const std::vector< std::pair<double, double> > &controlBounds = agent.getControlBounds();
		controlSpaceDim = controlBounds.size();

		ompl::control::RealVectorControlSpace *baseCSpace = new ompl::control::RealVectorControlSpace(space, controlSpaceDim);
		ompl::base::RealVectorBounds cbounds(controlSpaceDim);
		for(unsigned int i = 0; i < controlSpaceDim; ++i) {
			cbounds.setLow(i, controlBounds[i].first);
			cbounds.setHigh(i, controlBounds[i].second);
		}

		baseCSpace->setBounds(cbounds);
		ompl::control::ControlSpacePtr cspace(baseCSpace);

		// construct an instance of  space information from this control space
		spaceInfoPtr = ompl::control::SpaceInformationPtr(new ompl::control::SpaceInformation(space, cspace));

		// set state validity checking for this space
		spaceInfoPtr->setStateValidityChecker(boost::bind(&KPIECE::isStateValid, this, spaceInfoPtr.get(), _1));

		// set the state propagation routine
		spaceInfoPtr->setStatePropagator(boost::bind(&KPIECE::propagate, this, _1, _2, _3, _4));

		pdef = ompl::base::ProblemDefinitionPtr(new ompl::base::ProblemDefinition(spaceInfoPtr));

		spaceInfoPtr->setPropagationStepSize(args.doubleVal("Steering Delta t"));
		dfpair(stdout, "steering dt", "%g", args.doubleVal("Steering Delta t"));
		spaceInfoPtr->setMinMaxControlDuration(stol(args.value("KPIECE Min Control Steps")),stol(args.value("KPIECE Max Control Steps")));
		dfpair(stdout, "min control duration", "%u", stol(args.value("KPIECE Min Control Steps")));
		dfpair(stdout, "max control duration", "%u", stol(args.value("KPIECE Max Control Steps")));

		spaceInfoPtr->setup();

		kpiece = new ompl::control::KPIECE1(spaceInfoPtr);

		kpiece->setGoalBias(args.doubleVal("KPIECE Goal Bias"));
		dfpair(stdout, "goal bias", "%g", args.doubleVal("KPIECE Goal Bias"));

		kpiece->setBorderFraction(args.doubleVal("KPIECE Border Fraction"));
		dfpair(stdout, "border fraction", "%g", args.doubleVal("KPIECE Border Fraction"));

		kpiece->setCellScoreFactor(args.doubleVal("KPIECE Cell Score Good"), args.doubleVal("KPIECE Cell Score Bad"));
		dfpair(stdout, "cell score good", "%g", args.doubleVal("KPIECE Cell Score Good"));
		dfpair(stdout, "cell score bad", "%g", args.doubleVal("KPIECE Cell Score Bad"));

		kpiece->setMaxCloseSamplesCount(stol(args.value("KPIECE Max Close Samples")));
		dfpair(stdout, "max closed samples", "%u", stol(args.value("KPIECE Max Close Samples")));


		ompl::base::RealVectorRandomLinearProjectionEvaluator *projectionEvaluator = new ompl::base::RealVectorRandomLinearProjectionEvaluator(baseSpace, stateSpaceDim);

		ompl::base::ProjectionEvaluatorPtr projectionPtr = ompl::base::ProjectionEvaluatorPtr(projectionEvaluator);

		kpiece->setProjectionEvaluator(projectionPtr);
	}

	~KPIECE() {}

	bool isStateValid(const ompl::control::SpaceInformation *si, const ompl::base::State *state) const {
		return state->as<typename StateSpace::StateType>()->valid;
	}

	void propagate(const ompl::base::State *start, const ompl::control::Control *control, const double duration, ompl::base::State *result) {
		samplesGenerated++;

		const typename StateSpace::StateType *state = start->as<typename StateSpace::StateType>();
		const ompl::control::RealVectorControlSpace::ControlType *realVectorControl = control->as<ompl::control::RealVectorControlSpace::ControlType>();

		std::vector<double> controls(controlSpaceDim);
		for(unsigned int i = 0; i < controlSpaceDim; i++) {
			controls[i] = realVectorControl->values[i];
		}

		typename Agent::Control agentControl = agent.controlFromVector(controls);

		typename Agent::Edge edge = agent.steerWithControl(state->agentEdge->end, agentControl, duration);

		const typename Agent::StateVars &endStateVars = edge.getTreeStateVars();

		typename StateSpace::StateType *resultState = result->as<typename StateSpace::StateType>();

		resultState->agentEdge = new typename Agent::Edge(edge);

		for(unsigned int i = 0; i < stateSpaceDim; ++i) {
			resultState->values[i] = endStateVars[i];
		}

		resultState->valid = workspace.safeEdge(agent, edge, collisionCheckDT);

		if(resultState->valid) {
#ifdef WITHGRAPHICS
			treeEdges.push_back(resultState->agentEdge);
#endif
			edgesAdded++;
		} else {
#ifdef WITHGRAPHICS
			rejectedTreeEdges.push_back(resultState->agentEdge);
#endif
			edgesRejected++;
		}

		resultState->agentEdge->updateParent(state->agentEdge);

		// edge.end.print();

		if(agent.isGoal(edge.end, *agentGoal)) {
			goalEdge = new typename Agent::Edge(edge);
			goalEdge->updateParent(state->agentEdge);
		}
	}

	bool didFindGoal() const {
		return goalEdge != NULL;
	}

	std::vector<const typename Agent::Edge *> query(const typename Agent::State &start, const typename Agent::State &goal, int iterationsAtATime = -1, bool firstInvocation = true) {
		ompl::base::ScopedState<StateSpace> omplStart(spaceInfoPtr);
		omplStart->agentEdge = new typename Agent::Edge(start);
		omplStart->valid = true;

		const typename Agent::StateVars &startStateVars = omplStart->agentEdge->getTreeStateVars();

		for(unsigned int i = 0; i < startStateVars.size(); ++i) {
			omplStart->values[i] = startStateVars[i];
		}

		agentGoal = new typename Agent::State(goal);

		ompl::base::ScopedState<StateSpace> omplGoal(spaceInfoPtr);
		omplGoal->agentEdge = new typename Agent::Edge(start);
		omplGoal->valid = true;

		const typename Agent::StateVars &goalStateVars = omplGoal->agentEdge->getTreeStateVars();;

		omplGoal->values = new double[goalStateVars.size()];

		for(unsigned int i = 0; i < goalStateVars.size(); ++i) {
			omplGoal->values[i] = goalStateVars[i];
		}

		pdef->setStartAndGoalStates(omplStart, omplGoal);

		kpiece->setProblemDefinition(pdef);

		kpiece->setup();

		ompl::base::PlannerTerminationCondition tc = ompl::base::PlannerTerminationCondition(boost::bind(&KPIECE::didFindGoal, this));
		/*ompl::base::plannerOrTerminationCondition(
		ompl::base::timedPlannerTerminationCondition(300),
		ompl::base::PlannerTerminationCondition(boost::bind(&KPIECE::didFindGoal, this))); */

		// ompl::base::PlannerStatus solved =
		kpiece->solve(tc);

		// if(solved) {
		// 	fprintf(stderr, "found goal\n");
		// 	ompl::base::PathPtr path = pdef->getSolutionPath();
		// 	path->print(std::cout);
		// } else {
		// 	fprintf(stderr, "did not find goal\n");
		// }

#ifdef WITHGRAPHICS
		for(const typename Agent::Edge *edge : treeEdges) {
			edge->draw(OpenGLWrapper::Color::Red());
		}

		for(const typename Agent::Edge *edge : rejectedTreeEdges) {
			edge->draw(OpenGLWrapper::Color::Blue());
		}
#endif

		if(goalEdge != NULL) {
			fprintf(stderr, "found goal\n");


			dfpair(stdout, "solution cost", "%g", goalEdge->gCost());
			std::vector<const typename Agent::Edge *> solution;
			solution.push_back(goalEdge);

			unsigned int edgeCount = 1;
			while(solution.back()->parent != NULL) {
				edgeCount++;
				solution.push_back(solution.back()->parent);
			}
			dfpair(stdout, "solution length", "%u", edgeCount);
			return solution;
		}

		dfpair(stdout, "solution cost", "-1");
		dfpair(stdout, "solution length", "-1");

		return std::vector<const typename Agent::Edge *>();
	}

	void dfpairs() const {
		dfpair(stdout, "samples generated", "%u", samplesGenerated);
		dfpair(stdout, "edges added", "%u", edgesAdded);
		dfpair(stdout, "edges rejected", "%u", edgesRejected);
	}


private:
	const Workspace &workspace;
	const Agent &agent;
	typename Agent::State *agentGoal;
	typename Agent::Edge *goalEdge;
	ompl::control::KPIECE1 *kpiece;
	ompl::control::SpaceInformationPtr spaceInfoPtr;
	ompl::base::ProblemDefinitionPtr pdef;
	unsigned int stateSpaceDim, controlSpaceDim;
	double collisionCheckDT;

	unsigned int samplesGenerated, edgesAdded, edgesRejected;
	
	std::vector<const typename Agent::Edge *> treeEdges;
	std::vector<const typename Agent::Edge *> rejectedTreeEdges;
};