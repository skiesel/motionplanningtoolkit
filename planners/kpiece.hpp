#pragma once

#include <flann/flann.hpp>
#include <boost/pool/object_pool.hpp>

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>

template<class Workspace, class Agent, class Sampler, class NN>
class KPIECE {
public:

	typedef typename Agent::State State;
	typedef typename Agent::Edge Edge;

	KPIECE(const Workspace &workspace, const Agent &agent, const InstanceFileMap &args) :
		workspace(workspace), agent(agent) {
			// steeringDT = stod(args.value("Steering Delta t"));
			// collisionCheckDT = stod(args.value("Collision Check Delta t"));

			// set the bounds for the R^2 part of SE(2)
			ompl::base::RealVectorBounds bounds(2);
			bounds.setLow(-1);
			bounds.setHigh(1);

			space = ompl::base::StateSpacePtr(new ompl::base::SE2StateSpace());
			space->as<ompl::base::SE2StateSpace>()->setBounds(bounds);
	
			// create a control space
			ompl::control::ControlSpacePtr cspace(new ompl::control::RealVectorControlSpace(space, 2));

			// set the bounds for the control space
			ompl::base::RealVectorBounds cbounds(2);
			cbounds.setLow(-0.3);
			cbounds.setHigh(0.3);

			cspace->as<ompl::control::RealVectorControlSpace>()->setBounds(cbounds);

			// construct an instance of  space information from this control space
			ompl::control::SpaceInformationPtr si(new ompl::control::SpaceInformation(space, cspace));

			// set state validity checking for this space
			si->setStateValidityChecker(boost::bind(&KPIECE::isStateValid, si.get(),  _1));

			// set the state propagation routine
			si->setStatePropagator(boost::bind(&KPIECE::propagate, _1, _2, _3, _4));

			si->printSettings(std::cout);

			pdef = ompl::base::ProblemDefinitionPtr(new ompl::base::ProblemDefinition(si));

			kpiece = new ompl::control::KPIECE1(si);
		}

	~KPIECE() {
		delete kpiece;
	}

	static bool isStateValid(const ompl::control::SpaceInformation *si, const ompl::base::State *state) {
		// // cast the abstract state type to the type we expect
		// const ompl::base::SE2StateSpace::StateType *se2state = state->as<ompl::base::SE2StateSpace::StateType>();

		// // extract the first component of the state and cast it to what we expect
		// const ompl::base::RealVectorStateSpace::StateType *pos = se2state->as<ompl::base::RealVectorStateSpace::StateType>(0);

		// // extract the second component of the state and cast it to what we expect
		// const ompl::base::SO2StateSpace::StateType *rot = se2state->as<ompl::base::SO2StateSpace::StateType>(1);

		// // check validity of state defined by pos & rot


		// // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
		// return si->satisfiesBounds(state) && (const void*)rot != (const void*)pos;
		return true;
	}

	static void propagate(const ompl::base::State *start, const ompl::control::Control *control, const double duration, ompl::base::State *result) {
		// const ompl::base::SE2StateSpace::StateType *se2state = start->as<ompl::base::SE2StateSpace::StateType>();
		// const double* pos = se2state->as<ompl::base::RealVectorStateSpace::StateType>(0)->values;
		// const double rot = se2state->as<ompl::base::SO2StateSpace::StateType>(1)->value;
		// const double* ctrl = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;

		// result->as<ompl::base::SE2StateSpace::StateType>()->setXY(
		// 	pos[0] + ctrl[0] * duration * cos(rot),
		// 	pos[1] + ctrl[0] * duration * sin(rot)
		// );
		// result->as<ompl::base::SE2StateSpace::StateType>()->setYaw(rot + ctrl[1] * duration);
	}

	void query(const State &start, const State &goal, int iterationsAtATime = -1, bool firstInvocation = true) {
		ompl::base::ScopedState<ompl::base::SE2StateSpace> omplStart(space);
		omplStart->setX(-0.5);
		omplStart->setY(0.0);
		omplStart->setYaw(0.0);

		ompl::base::ScopedState<ompl::base::SE2StateSpace> omplGoal(omplStart);
		omplGoal->setX(0.5);

		pdef->setStartAndGoalStates(start, goal, 0.1);

		kpiece->setProblemDefinition(pdef);

	    kpiece->setup();

		pdef->print(std::cout);

		ompl::base::PlannerTerminationCondition tc = ompl::base::exactSolnPlannerTerminationCondition(pdef);

		ompl::base::PlannerStatus solved = kpiece->solve(tc);

		if (solved) {
			ompl::base::PathPtr path = pdef->getSolutionPath();
			std::cout << "Found solution:" << std::endl;

			path->print(std::cout);
		} else {
			std::cout << "No solution found" << std::endl;
		}
	}

private:
	const Workspace &workspace;
	const Agent &agent;
	ompl::control::KPIECE1 *kpiece;
	ompl::base::StateSpacePtr space;
	ompl::base::ProblemDefinitionPtr pdef;
};