#pragma once

unsigned int GraphicsIterations = 1000;

template<class Workspace, class Agent>
void go_RRT(const InstanceFileMap &args, const Agent &agent, const Workspace &workspace,
            const typename Agent::State &start, const typename Agent::State &goal) {

	dfpair(stdout, "planner", "%s", "RRT");
	// typedef flann::KDTreeSingleIndexParams KDTreeType;
	typedef flann::KDTreeIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, flann::L2<double>, typename Agent::Edge> KDTree;
	typedef UniformSampler<Workspace, Agent, KDTree> Sampler;
	typedef TreeInterface<Agent, KDTree, Sampler> TreeInterface;
	typedef RRT<Workspace, Agent, TreeInterface> Planner;

	/* planner config */

	KDTreeType kdtreeType(3);
	KDTree kdtree(kdtreeType, agent.getTreeStateSize());
	Sampler sampler(workspace, agent, kdtree);
	TreeInterface treeInterface(kdtree, sampler);
	Planner planner(workspace, agent, treeInterface, args);

#ifdef WITHGRAPHICS
	bool firstInvocation = true;
	auto lambda = [&]() {
		workspace.draw();
		agent.drawMesh(start);
		agent.drawMesh(goal);
		planner.query(start, goal, GraphicsIterations, firstInvocation);
		firstInvocation = false;
	};
	OpenGLWrapper::getOpenGLWrapper().runWithCallback(lambda, args);
#else
	planner.query(start, goal);
	planner.dfpairs();
#endif
}

template<class Workspace, class Agent>
void go_RRTConnect(const InstanceFileMap &args, const Agent &agent, const Workspace &workspace,
                   const typename Agent::State &start, const typename Agent::State &goal) {
	dfpair(stdout, "planner", "%s", "RRT Connect");

	// typedef flann::KDTreeSingleIndexParams KDTreeType;
	typedef flann::KDTreeIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, flann::L2<double>, typename Agent::Edge> KDTree;
	typedef UniformSampler<Workspace, Agent, KDTree> Sampler;
	typedef TreeInterface<Agent, KDTree, Sampler> TreeInterface;
	typedef RRTConnect<Workspace, Agent, TreeInterface> Planner;

	/* planner config */

	KDTreeType kdtreeType(2);
	KDTree kdtree(kdtreeType, agent.getTreeStateSize());
	Sampler sampler(workspace, agent, kdtree);
	TreeInterface treeInterface(kdtree, sampler);
	Planner planner(workspace, agent, treeInterface, args);

#ifdef WITHGRAPHICS
	bool firstInvocation = true;
	auto lambda = [&]() {
		workspace.draw();
		agent.drawMesh(start);
		agent.drawMesh(goal);
		planner.query(start, goal, GraphicsIterations, firstInvocation);
		firstInvocation = false;
	};
	OpenGLWrapper::getOpenGLWrapper().runWithCallback(lambda, args);
#else
	planner.query(start, goal);
	planner.dfpairs();
#endif
}

template<class Workspace, class Agent>
void go_KPIECE(const InstanceFileMap &args, const Agent &agent, const Workspace &workspace,
               const typename Agent::State &start, const typename Agent::State &goal) {
	dfpair(stdout, "planner", "%s", "KPIECE");

	typedef KPIECE<Workspace, Agent> Planner;

	Planner planner(workspace, agent, args);
	planner.query(start, goal);
	planner.dfpairs();
}

template<class Workspace, class Agent>
void go_PPRM(const InstanceFileMap &args, const Agent &agent, const Workspace &workspace,
             const typename Agent::State &start, const typename Agent::State &goal) {
	dfpair(stdout, "planner", "%s", "PPRM");

	typedef LazyPRMLite<Workspace, Agent> PRMLite;
	typedef PlakuTreeInterface<Workspace, Agent, PRMLite> PlakuTreeInterfaceT;
	typedef RRT<Workspace, Agent, PlakuTreeInterfaceT> Plaku;

	unsigned int numberOfPRMVertices = stol(args.value("Number Of PRM Vertices"));
	unsigned int numberOfNearestNeighborEdgeConsiderations = stol(args.value("Nearest Neighbors To Consider In PRM Edge Construction"));
	double prmCollisionCheckDT = args.doubleVal("PRM Collision Check DT");

	PRMLite prmLite(workspace, agent, numberOfPRMVertices, numberOfNearestNeighborEdgeConsiderations, prmCollisionCheckDT);

	double alpha = args.doubleVal("Plaku Alpha Value");
	double b = args.doubleVal("Plaku b Value");
	double stateRadius = args.doubleVal("Plaku PRM State Selection Radius");

	PlakuTreeInterfaceT plakuTreeInterface(workspace, agent, prmLite, start, goal, alpha, b, stateRadius);

	Plaku planner(workspace, agent, plakuTreeInterface, args);
#ifdef WITHGRAPHICS
	bool firstInvocation = true;
	auto lambda = [&]() {
		plakuTreeInterface.draw();
		workspace.draw();
		agent.drawMesh(start);
		agent.drawMesh(goal);

		// plakuTreeInterface.draw();

		planner.query(start, goal, GraphicsIterations, firstInvocation);
		firstInvocation = false;
	};
	OpenGLWrapper::getOpenGLWrapper().runWithCallback(lambda, args);
#else
	planner.query(start, goal);
	planner.dfpairs();
#endif
}

template<class Workspace, class Agent>
void go_SST(const InstanceFileMap &args, const Agent &agent, const Workspace &workspace,
                   const typename Agent::State &start, const typename Agent::State &goal) {
	dfpair(stdout, "planner", "%s", "SST");

	// typedef flann::KDTreeSingleIndexParams KDTreeType;
	typedef flann::KDTreeIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, flann::L2<double>, typename Agent::Edge> KDTree;
	typedef UniformSampler<Workspace, Agent, KDTree> Sampler;
	typedef SST<Workspace, Agent, KDTree, Sampler> TreeInterface;
	typedef RRT<Workspace, Agent, TreeInterface> Planner;

	/* planner config */

	KDTreeType kdtreeType(2);
	KDTree kdtree(kdtreeType, agent.getTreeStateSize());
	Sampler sampler(workspace, agent, kdtree);

	double sstRadius = args.doubleVal("SST Radius");
	double sstResize = args.doubleVal("SST Resize Threshold");

	TreeInterface treeInterface(workspace, agent, kdtree, sampler, sstRadius, sstResize);
	Planner planner(workspace, agent, treeInterface, args);

#ifdef WITHGRAPHICS
	bool firstInvocation = true;
	auto lambda = [&]() {
		workspace.draw();
		agent.drawMesh(start);
		agent.drawMesh(goal);
		planner.query(start, goal, GraphicsIterations, firstInvocation);
		firstInvocation = false;
	};
	OpenGLWrapper::getOpenGLWrapper().runWithCallback(lambda, args);
#else
	planner.query(start, goal);
	planner.dfpairs();
#endif
}

template<class Workspace, class Agent>
void go_SSTGrid(const InstanceFileMap &args, const Agent &agent, const Workspace &workspace,
                   const typename Agent::State &start, const typename Agent::State &goal) {
	dfpair(stdout, "planner", "%s", "SST Grid");

	// typedef flann::KDTreeSingleIndexParams KDTreeType;
	typedef flann::KDTreeIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, flann::L2<double>, typename Agent::Edge> KDTree;
	typedef UniformSampler<Workspace, Agent, KDTree> Sampler;
	typedef SST_Grid<Workspace, Agent, KDTree, Sampler> TreeInterface;
	typedef RRT<Workspace, Agent, TreeInterface> Planner;

	/* planner config */

	KDTreeType kdtreeType(2);
	KDTree kdtree(kdtreeType, agent.getTreeStateSize());
	Sampler sampler(workspace, agent, kdtree);

	double sstRadius = args.doubleVal("SST Radius");
	double sstResize = args.doubleVal("SST Resize Threshold");

	TreeInterface treeInterface(workspace, agent, kdtree, sampler, sstRadius, sstResize);
	Planner planner(workspace, agent, treeInterface, args);

#ifdef WITHGRAPHICS
	bool firstInvocation = true;
	auto lambda = [&]() {
		workspace.draw();
		agent.drawMesh(start);
		agent.drawMesh(goal);
		planner.query(start, goal, GraphicsIterations, firstInvocation);
		firstInvocation = false;
	};
	OpenGLWrapper::getOpenGLWrapper().runWithCallback(lambda, args);
#else
	planner.query(start, goal);
	planner.dfpairs();
#endif
}

template<class Workspace, class Agent>
void go(const InstanceFileMap &args, const Workspace &workspace, const Agent &agent,
        const typename Agent::State &start, const typename Agent::State &goal) {
	clock_t startT = clock();

	std::string planner = args.value("Planner");

	if(planner.compare("RRT") == 0) {
		go_RRT<Workspace, Agent>(args, agent, workspace, start, goal);
	} else if(planner.compare("RRT Connect") == 0) {
		go_RRTConnect<Workspace, Agent>(args, agent, workspace, start, goal);
	} else if(planner.compare("PPRM") == 0) {
		go_PPRM<Workspace, Agent>(args, agent, workspace, start, goal);
	} else if(planner.compare("KPIECE") == 0) {
		go_KPIECE<Workspace, Agent>(args, agent, workspace, start, goal);
	} else if(planner.compare("SST") == 0) {
		go_SST<Workspace, Agent>(args, agent, workspace, start, goal);
	} else if(planner.compare("SST Grid") == 0) {
		go_SSTGrid<Workspace, Agent>(args, agent, workspace, start, goal);
	} else {
		fprintf(stderr, "unreocognized planner: %s\n", planner.c_str());
	}

	clock_t endT = clock();
	dfpair(stdout, "total time solving time", "%g", (double)(endT-startT) / CLOCKS_PER_SEC);
}

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

void snake(const InstanceFileMap &args) {
	typedef SnakeTrailers Agent;
	typedef Map3D<Agent> Workspace;

	Agent agent(args);
	Workspace workspace(args);

	/* start and goal states */

	auto startPosition = args.doubleList("Agent Start Location");
	auto startOrientation =  args.doubleList("Agent Start Orientation");

	Agent::StateVars startStateVars = startPosition;
	startStateVars.push_back(0); //linear v
	startStateVars.push_back(0); //angular v

	startStateVars.insert(startStateVars.end(), startOrientation.begin(), startOrientation.end());

	Agent::State start(startStateVars);

	auto goalPosition = args.doubleList("Agent Goal Location");

	Agent::State goal(goalPosition);

	// go<Workspace, Agent>(args, workspace, agent, start, goal);
}

void geometric(const InstanceFileMap &args) {
	typedef Geometric Agent;
	typedef Map3D<Agent> Workspace;

	Agent agent(args);
	Workspace workspace(args);

	/* start and goal states */

	auto startPosition = args.doubleList("Agent Start Location");
	auto startOrientation =  args.doubleList("Agent Start Orientation");

	Agent::StateVars startStateVars = startPosition;
	startStateVars.insert(startStateVars.end(), startOrientation.begin(), startOrientation.end());

	Agent::State start(startStateVars);

	auto goalPosition = args.doubleList("Agent Goal Location");

	Agent::State goal(goalPosition);

	// go<Workspace, Agent>(args, workspace, agent, start, goal);
}

void planarLinkage(const InstanceFileMap &args) {
	typedef PlanarLinkage Agent;
	typedef PlanarLinkage Workspace;

	PlanarLinkage planarLinkage(args);

	/* start and goal states */

	auto startPositionVars = args.doubleList("Agent Start Position");
	auto goalPositionVars = args.doubleList("Agent Goal Position");

	Agent::State start(startPositionVars);
	Agent::State goal(goalPositionVars);

	start.print();
	goal.print();

	go<Workspace, Agent>(args, planarLinkage, planarLinkage, start, goal);
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

	start.print();
	goal.print();

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

	start.print();
	goal.print();

	go<Workspace, Agent>(args, workspace, agent, start, goal);
}