#pragma once

void blimp(const InstanceFileMap &args) {
	typedef Blimp Agent;
	typedef Map3D<Agent> Workspace;

	Agent agent(args);
	Workspace workspace(args);

	/* start and goal states */

	auto startPosition = args.doubleList("Agent Start Location");
	auto startOrientation =  args.doubleList("Agent Start Orientation");
	fcl::Vec3f axis;
	double theta;
	fcl::Quaternion3f startQuaternion(startOrientation[0], startOrientation[1], startOrientation[2], startOrientation[3]);
	startQuaternion.toAxisAngle(axis, theta);
	theta = (theta - 2 * M_PI * std::floor((theta + M_PI) / (2 * M_PI)));

	Agent::State start(startPosition[0], startPosition[1], startPosition[2], theta);

	auto goalPosition = args.doubleList("Agent Goal Location");
	auto goalOrientation = args.doubleList("Agent Goal Orientation");
	fcl::Quaternion3f goalQuaternion(goalOrientation[0], goalOrientation[1], goalOrientation[2], goalOrientation[3]);
	goalQuaternion.toAxisAngle(axis, theta);
	theta = (theta - 2 * M_PI * std::floor((theta + M_PI) / (2 * M_PI)));

	Agent::State goal(goalPosition[0], goalPosition[1], goalPosition[2], theta);

	go<Workspace, Agent>(args, workspace, agent, start, goal);
}

void quadcopter(const InstanceFileMap &args) {
	typedef Quadcopter Agent;
	typedef Map3D<Agent> Workspace;

	Agent agent(args);
	Workspace workspace(args);

	/* start and goal states */

	auto startPosition = args.doubleList("Agent Start Location");	
	Agent::State start(startPosition[0], startPosition[1], startPosition[2]);

	auto goalPosition = args.doubleList("Agent Goal Location");
	Agent::State goal(goalPosition[0], goalPosition[1], goalPosition[2]);

	go<Workspace, Agent>(args, workspace, agent, start, goal);
}

// void snake(const InstanceFileMap &args) {
// 	typedef SnakeTrailers Agent;
// 	typedef Map3D<Agent> Workspace;

// 	Agent agent(args);
// 	Workspace workspace(args);

// 	/* start and goal states */

// 	auto startPosition = args.doubleList("Agent Start Location");
// 	auto startOrientation =  args.doubleList("Agent Start Orientation");

// 	Agent::StateVars startStateVars = startPosition;
// 	startStateVars.push_back(0); //linear v
// 	startStateVars.push_back(0); //angular v

// 	startStateVars.insert(startStateVars.end(), startOrientation.begin(), startOrientation.end());

// 	Agent::State start(startStateVars);

// 	auto goalPosition = args.doubleList("Agent Goal Location");

// 	Agent::State goal(goalPosition);

// 	// go<Workspace, Agent>(args, workspace, agent, start, goal);
// }

// void geometric(const InstanceFileMap &args) {
// 	typedef Geometric Agent;
// 	typedef Map3D<Agent> Workspace;

// 	Agent agent(args);
// 	Workspace workspace(args);

// 	/* start and goal states */

// 	auto startPosition = args.doubleList("Agent Start Location");
// 	auto startOrientation =  args.doubleList("Agent Start Orientation");

// 	Agent::StateVars startStateVars = startPosition;
// 	startStateVars.insert(startStateVars.end(), startOrientation.begin(), startOrientation.end());

// 	Agent::State start(startStateVars);

// 	auto goalPosition = args.doubleList("Agent Goal Location");

// 	Agent::State goal(goalPosition);

// 	// go<Workspace, Agent>(args, workspace, agent, start, goal);
// }

void planarLinkage(const InstanceFileMap &args) {
	assert(false);
	// typedef PlanarLinkage Agent;
	// typedef PlanarLinkage Workspace;

	// PlanarLinkage planarLinkage(args);

	// /* start and goal states */

	// auto startPositionVars = args.doubleList("Agent Start Position");
	// auto goalPositionVars = args.doubleList("Agent Goal Position");

	// Agent::State start(startPositionVars);
	// Agent::State goal(goalPositionVars);

	// go<Workspace, Agent>(args, planarLinkage, planarLinkage, start, goal);
}

void kink(const InstanceFileMap &args) {
	typedef OmniMultiD Agent;
	typedef Kink<Agent> Workspace;

	Agent agent(args);
	Workspace workspace(args);

	/* start and goal states */
	const int dimensions = args.integerVal("Dimensions");
	BOOST_ASSERT_MSG(dimensions > 1, "Number of dimensions must be more than 1.");

	// x, y coordinates
	Agent::StateVars startPositionVars = {0.75, 0};
	Agent::StateVars goalPositionVars = {0.25, 1};

	for (int i = 2; i < dimensions; ++i) {
		startPositionVars.push_back(0.5);
		goalPositionVars.push_back(0.5);
	}

	Agent::State start(startPositionVars);
	Agent::State goal(goalPositionVars);

	go<Workspace, Agent>(args, workspace, agent, start, goal);
}

void narrowPassage(const InstanceFileMap &args, const bool scaleObstacles) {
	typedef OmniMultiD Agent;
	typedef NarrowPassage<Agent> Workspace;

	Agent agent(args);
	Workspace workspace(args, scaleObstacles);

	/* start and goal states */
	const int dimensions = args.integerVal("Dimensions");
	BOOST_ASSERT_MSG(dimensions > 1, "Number of dimensions must be more than 1.");

	// x, y coordinates
	Agent::StateVars startPositionVars = {0, 0};
	Agent::StateVars goalPositionVars = {0, 1};

	for (int i = 2; i < dimensions; ++i) {
		startPositionVars.push_back(0.5);
		goalPositionVars.push_back(0.5);
	}

	Agent::State start(startPositionVars);
	Agent::State goal(goalPositionVars);

	go<Workspace, Agent>(args, workspace, agent, start, goal);
}

void rectangleMap2D(const InstanceFileMap &args) {
	typedef OmniMultiD Agent;
	typedef RectangleMap2D<Agent> Workspace;

	Agent agent(args);
	Workspace workspace(args);

	Agent::State start(args.doubleList("Agent Start Location"));
	Agent::State goal(args.doubleList("Agent Goal Location"));

	go<Workspace, Agent>(args, workspace, agent, start, goal);
}

void pendulum(const InstanceFileMap &args) {

	Pendulum pendulum(args);

#ifdef WITHGRAPHICS
	auto lambda = [&]() {
		pendulum.draw();
		std::cin.ignore();
	};
	OpenGLWrapper::getOpenGLWrapper().runWithCallback(lambda, args);
#endif
}