void blimp_RRT(const InstanceFileMap& args, Blimp& agent, Map3D<Blimp> &workspace,
		typename Blimp::State &start, typename Blimp::State &goal) {

	dfpair(stdout, "planner", "%s", "RRT");

	typedef Blimp Agent;
	typedef Map3D<Agent> Workspace;
	// typedef flann::KDTreeSingleIndexParams KDTreeType;
	typedef flann::KDTreeIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, flann::L2<double>, Agent::Edge> KDTree;
	typedef UniformSampler<Workspace, Agent, KDTree> Sampler;
	typedef TreeInterface<Agent, KDTree, Sampler> TreeInterface;
	typedef RRT<Workspace, Agent, TreeInterface> Planner;

/* planner config */

	KDTreeType kdtreeType(4);
	KDTree kdtree(kdtreeType, agent.getTreeStateSize());
	Sampler sampler(workspace, agent, kdtree);
	TreeInterface treeInterface(kdtree, sampler);
	Planner planner(workspace, agent, treeInterface, args);

	#ifdef WITHGRAPHICS
		bool firstInvocation = true;
		auto lambda = [&](){
			workspace.draw();
			agent.drawMesh(start);
			agent.drawMesh(goal);
			planner.query(start, goal, 100, firstInvocation);
			firstInvocation = false;
		};
		OpenGLWrapper::getOpenGLWrapper().runWithCallback(lambda);
	#else
		planner.query(start, goal);
		planner.dfpairs();
	#endif
}

void blimp_RRTConnect(const InstanceFileMap& args, Blimp& agent, Map3D<Blimp> &workspace,
		typename Blimp::State &start, typename Blimp::State &goal) {
	dfpair(stdout, "planner", "%s", "RRT Connect");

	typedef Blimp Agent;
	typedef Map3D<Agent> Workspace;
	// typedef flann::KDTreeSingleIndexParams KDTreeType;
	typedef flann::KDTreeIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, flann::L2<double>, Agent::Edge> KDTree;
	typedef UniformSampler<Workspace, Agent, KDTree> Sampler;
	typedef TreeInterface<Agent, KDTree, Sampler> TreeInterface;
	typedef RRTConnect<Workspace, Agent, TreeInterface> Planner;

/* planner config */

	KDTreeType kdtreeType(4);
	KDTree kdtree(kdtreeType, agent.getTreeStateSize());
	Sampler sampler(workspace, agent, kdtree);
	TreeInterface treeInterface(kdtree, sampler);
	Planner planner(workspace, agent, treeInterface, args);

	#ifdef WITHGRAPHICS
		bool firstInvocation = true;
		auto lambda = [&](){
			workspace.draw();
			agent.drawMesh(start);
			agent.drawMesh(goal);
			planner.query(start, goal, 100, firstInvocation);
			firstInvocation = false;
		};
		OpenGLWrapper::getOpenGLWrapper().runWithCallback(lambda);
	#else
		planner.query(start, goal);
		planner.dfpairs();
	#endif
}

void blimp_PPRM(const InstanceFileMap& args, Blimp& agent, Map3D<Blimp> &workspace,
		typename Blimp::State &start, typename Blimp::State &goal) {
	dfpair(stdout, "planner", "%s", "PPRM");

	typedef Blimp Agent;
	typedef Map3D<Agent> Workspace;

	typedef PRMLite<Workspace, Agent> PRMLite;	
	typedef PlakuTreeInterface<Workspace, Agent, PRMLite> PlakuTreeInterface;
	typedef RRT<Workspace, Agent, PlakuTreeInterface> Plaku;

	unsigned int numberOfPRMVertices = stol(args.value("Number Of PRM Vertices"));
	unsigned int numberOfNearestNeighborEdgeConsiderations = stol(args.value("Nearest Neighbors To Consider In PRM Edge Construction"));
	double prmCollisionCheckDT = stod(args.value("PRM Collision Check DT"));

	PRMLite prmLite(workspace, agent, start, numberOfPRMVertices, numberOfNearestNeighborEdgeConsiderations, prmCollisionCheckDT);

	double alpha = stod(args.value("Plaku Alpha Value"));
	double b = stod(args.value("Plaku b Value"));
	double stateRadius = stod(args.value("Plaku PRM State Selection Radius"));

	PlakuTreeInterface plakuTreeInterface(workspace, agent, prmLite, start, goal, alpha, b, stateRadius);

	// plakuTreeInterface.draw();

	Plaku planner(workspace, agent, plakuTreeInterface, args);
	#ifdef WITHGRAPHICS
		bool firstInvocation = true;
		auto lambda = [&](){
			workspace.draw();
			agent.drawMesh(start);
			agent.drawMesh(goal);
			// plakuTreeInterface.draw();

			planner.query(start, goal, 100, firstInvocation);
			firstInvocation = false;
		};
		OpenGLWrapper::getOpenGLWrapper().runWithCallback(lambda);
	#else
		planner.query(start, goal);
		planner.dfpairs();
	#endif
}

void blimp(const InstanceFileMap& args) {
	typedef Blimp Agent;
	typedef Map3D<Agent> Workspace;

	Agent agent(args);
	Workspace workspace(args);

/* start and goal states */

	auto startVars = args.doubleList("Agent Start Location");
	fcl::Vec3f axis;
	double theta;
	fcl::Quaternion3f startOrientation(startVars[3], startVars[4], startVars[5], startVars[6]);
	startOrientation.toAxisAngle(axis, theta);
	theta = (theta - 2 * M_PI * std::floor((theta + M_PI) / (2 * M_PI)));

	Agent::State start(startVars[0], startVars[1], startVars[2], theta);
	
	auto goalVars = args.doubleList("Agent Goal Location");
	fcl::Quaternion3f goalOrientation(goalVars[3], goalVars[4], goalVars[5], goalVars[6]);
	goalOrientation.toAxisAngle(axis, theta);
	theta = (theta - 2 * M_PI * std::floor((theta + M_PI) / (2 * M_PI)));

	Agent::State goal(goalVars[0], goalVars[1], goalVars[2], theta);


	std::string planner = args.value("Planner");

	if(planner.compare("RRT") == 0) {
		blimp_RRT(args, agent, workspace, start, goal);
	} else if(planner.compare("RRT Connect") == 0) {
		blimp_RRTConnect(args, agent, workspace, start, goal);
	} else if(planner.compare("PPRM") == 0) {
		blimp_PPRM(args, agent, workspace, start, goal);
	} else {
		fprintf(stderr, "unreocognized planner: %s\n", planner.c_str());
	}

	
}