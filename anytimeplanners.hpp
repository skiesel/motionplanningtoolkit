#pragma once

template<class Planner, class Workspace, class Agent>
void go_COMMONANYTIME(const InstanceFileMap &args, Planner &planner,
		const Workspace &workspace, const Agent &agent,
		const typename Agent::State &start, const typename Agent::State &goal, clock_t startTime) {

#ifdef WITHGRAPHICS
	assert(false);
#else
	planner.query(start, goal, startTime);
	planner.dfpairs();
#endif
}

template<class Workspace, class Agent>
void go_AnytimeRRT(const InstanceFileMap &args, const Agent &agent, const Workspace &workspace,
            const typename Agent::State &start, const typename Agent::State &goal) {

	clock_t startT = clock();

	dfpair(stdout, "planner", "%s", "Anytime RRT");
	// typedef flann::KDTreeSingleIndexParams KDTreeType;
	typedef flann::KDTreeIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, typename Agent::DistanceEvaluator, typename Agent::Edge> KDTree;
	typedef UniformSampler<Workspace, Agent, KDTree> USampler;
	typedef GoalBiasSampler<Agent, USampler> GBSampler;
	typedef TreeInterface<Agent, KDTree, GBSampler> TreeInterface;
	typedef AnytimeRRT<Workspace, Agent, TreeInterface> Planner;

	/* planner config */

	KDTreeType kdtreeType(1);
	KDTree kdtree(kdtreeType, agent.getDistanceEvaluator(), agent.getTreeStateSize());
	USampler uniformsampler(workspace, agent, kdtree);

	double goalBias = args.exists("Goal Bias") ? args.doubleVal("Goal Bias") : 0;
	dfpair(stdout, "goal bias", "%g", goalBias);

	GBSampler goalbiassampler(uniformsampler, goal, goalBias);
	TreeInterface treeInterface(kdtree, goalbiassampler);
	Planner planner(workspace, agent, treeInterface, args);

	go_COMMONANYTIME<Planner, Workspace, Agent>(args, planner, workspace, agent, start, goal, startT);
}

template<class Workspace, class Agent>
void go_AnytimeEST(const InstanceFileMap &args, const Agent &agent, const Workspace &workspace,
            const typename Agent::State &start, const typename Agent::State &goal) {

	clock_t startT = clock();

	dfpair(stdout, "planner", "%s", "Anytime EST");

	typedef AnytimeEST<Workspace, Agent> Planner;

	Planner planner(workspace, agent, args);

	go_COMMONANYTIME<Planner, Workspace, Agent>(args, planner, workspace, agent, start, goal, startT);
}

template<class Workspace, class Agent>
void go_AnytimeBidirectionalEST(const InstanceFileMap &args, const Agent &agent, const Workspace &workspace,
            const typename Agent::State &start, const typename Agent::State &goal) {

	clock_t startT = clock();

	dfpair(stdout, "planner", "%s", "Anytime EST");

	typedef AnytimeBidirectionalEST<Workspace, Agent> Planner;

	Planner planner(workspace, agent, args);

	go_COMMONANYTIME<Planner, Workspace, Agent>(args, planner, workspace, agent, start, goal, startT);
}

template<class Workspace, class Agent>
void go_AnytimePPRM(const InstanceFileMap &args, const Agent &agent, const Workspace &workspace,
             const typename Agent::State &start, const typename Agent::State &goal) {
	clock_t startT = clock();

	dfpair(stdout, "planner", "%s", "PPRM");

	typedef PRMLite<Workspace, Agent> PRMLite;
	typedef PlakuTreeInterface<Workspace, Agent, PRMLite> PlakuTreeInterfaceT;
	typedef AnytimeRRT<Workspace, Agent, PlakuTreeInterfaceT> Planner;

	unsigned int numberOfPRMVertices = stol(args.value("Number Of PRM Vertices"));
	unsigned int numberOfNearestNeighborEdgeConsiderations = stol(args.value("Nearest Neighbors To Consider In PRM Edge Construction"));
	double prmCollisionCheckDT = args.doubleVal("PRM Collision Check DT");

	PRMLite prmLite(workspace, agent, numberOfPRMVertices, numberOfNearestNeighborEdgeConsiderations, prmCollisionCheckDT);

	double alpha = args.doubleVal("Plaku Alpha Value");
	double b = args.doubleVal("Plaku b Value");
	double stateRadius = args.doubleVal("Plaku PRM State Selection Radius");
	double goalBias = args.exists("Goal Bias") ? args.doubleVal("Goal Bias") : 0;
	dfpair(stdout, "goal bias", "%g", goalBias);

	PlakuTreeInterfaceT plakuTreeInterface(workspace, agent, prmLite, start, goal, alpha, b, stateRadius, goalBias);

	Planner planner(workspace, agent, plakuTreeInterface, args);

	go_COMMONANYTIME<Planner, Workspace, Agent>(args, planner, workspace, agent, start, goal, startT);
}

template<class Workspace, class Agent>
void go_SST(const InstanceFileMap &args, const Agent &agent, const Workspace &workspace,
                   const typename Agent::State &start, const typename Agent::State &goal) {

	clock_t startT = clock();

	dfpair(stdout, "planner", "%s", "SST");

	// typedef flann::KDTreeSingleIndexParams KDTreeType;
	typedef flann::KDTreeIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, typename Agent::DistanceEvaluator, typename Agent::Edge> KDTree;
	typedef UniformSampler<Workspace, Agent, KDTree> USampler;
	typedef GoalBiasSampler<Agent, USampler> GBSampler;
	typedef SST<Workspace, Agent, KDTree, GBSampler> TreeInterface;
	typedef AnytimeRRT<Workspace, Agent, TreeInterface> Planner;

	/* planner config */

	KDTreeType kdtreeType(1);
	KDTree kdtree(kdtreeType, agent.getDistanceEvaluator(), agent.getTreeStateSize());
	USampler uniformsampler(workspace, agent, kdtree);

	double goalBias = args.exists("Goal Bias") ? args.doubleVal("Goal Bias") : 0;
	dfpair(stdout, "goal bias", "%g", goalBias);

	GBSampler goalbiassampler(uniformsampler, goal, goalBias);


	double sstRadius = args.doubleVal("SST Radius");
	double sstResize = args.doubleVal("SST Resize Threshold");

	TreeInterface treeInterface(workspace, agent, kdtree, goalbiassampler, sstRadius, sstResize);
	Planner planner(workspace, agent, treeInterface, args);

	go_COMMONANYTIME<Planner, Workspace, Agent>(args, planner, workspace, agent, start, goal, startT);
}

template<class Workspace, class Agent>
void go_SSTGrid(const InstanceFileMap &args, const Agent &agent, const Workspace &workspace,
                   const typename Agent::State &start, const typename Agent::State &goal) {
	
	clock_t startT = clock();

	dfpair(stdout, "planner", "%s", "SST Grid");

	// typedef flann::KDTreeSingleIndexParams KDTreeType;
	typedef flann::KDTreeIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, typename Agent::DistanceEvaluator, typename Agent::Edge> KDTree;
	typedef UniformSampler<Workspace, Agent, KDTree> USampler;
	typedef GoalBiasSampler<Agent, USampler> GBSampler;
	typedef SST_Grid<Workspace, Agent, KDTree, GBSampler> TreeInterface;
	typedef AnytimeRRT<Workspace, Agent, TreeInterface> Planner;

	/* planner config */

	KDTreeType kdtreeType(1);
	KDTree kdtree(kdtreeType, agent.getDistanceEvaluator(), agent.getTreeStateSize());
	USampler uniformsampler(workspace, agent, kdtree);

	double goalBias = args.exists("Goal Bias") ? args.doubleVal("Goal Bias") : 0;
	dfpair(stdout, "goal bias", "%g", goalBias);

	GBSampler goalbiassampler(uniformsampler, goal, goalBias);

	double sstRadius = args.doubleVal("SST Radius");
	double sstResize = args.doubleVal("SST Resize Threshold");

	TreeInterface treeInterface(workspace, agent, kdtree, goalbiassampler, sstRadius, sstResize);
	Planner planner(workspace, agent, treeInterface, args);

	go_COMMONANYTIME<Planner, Workspace, Agent>(args, planner, workspace, agent, start, goal, startT);
}

template<class Workspace, class Agent>
void go_MRRT(const InstanceFileMap &args, const Agent &agent, const Workspace &workspace,
                   const typename Agent::State &start, const typename Agent::State &goal) {

	clock_t startT = clock();

	dfpair(stdout, "planner", "%s", "MRRT");

	typedef NoOpPostProcessor<Workspace, Agent> PostProccesor;
	typedef AnytimeRestartingRRTWithPostProcessing<Workspace, Agent, PostProccesor> Planner;

	PostProccesor postProccesor;

	Planner planner(workspace, agent, postProccesor, args);

	go_COMMONANYTIME<Planner, Workspace, Agent>(args, planner, workspace, agent, start, goal, startT);
}

template<class Workspace, class Agent>
void go_MRRTPlusS(const InstanceFileMap &args, const Agent &agent, const Workspace &workspace,
                   const typename Agent::State &start, const typename Agent::State &goal) {

	clock_t startT = clock();

	dfpair(stdout, "planner", "%s", "MRRT+S");

	typedef SimplePostProcessor<Workspace, Agent> PostProccesor;
	// typedef RestartingRRTWithPostProcessing<Workspace, Agent, PostProccesor> Planner;
	typedef AnytimeRestartingRRTWithPostProcessing<Workspace, Agent, PostProccesor> Planner;

	PostProccesor postProccesor(workspace, agent, args);

	Planner planner(workspace, agent, postProccesor, args);
	
	go_COMMONANYTIME<Planner, Workspace, Agent>(args, planner, workspace, agent, start, goal, startT);
}

template<class Workspace, class Agent>
void go_AORRT(const InstanceFileMap &args, const Agent &agent, const Workspace &workspace,
                   const typename Agent::State &start, const typename Agent::State &goal) {

	clock_t startT = clock();

	dfpair(stdout, "planner", "%s", "AO RRT");
	
	typedef AORRT<Workspace, Agent> Planner;

	Planner planner(workspace, agent, args);
	
	go_COMMONANYTIME<Planner, Workspace, Agent>(args, planner, workspace, agent, start, goal, startT);
}

template<class Workspace, class Agent>
void go_AnytimeSSTPPRM(const InstanceFileMap &args, const Agent &agent, const Workspace &workspace,
                   const typename Agent::State &start, const typename Agent::State &goal) {

	clock_t startT = clock();
	dfpair(stdout, "planner", "%s", "PPRM + SST");

	typedef PRMLite<Workspace, Agent> PRMLite;
	typedef PlakuTreeInterface<Workspace, Agent, PRMLite> PlakuTreeInterfaceT;
	typedef SST<Workspace, Agent, PlakuTreeInterfaceT, PlakuTreeInterfaceT> SSTTreeInterface;
	typedef AnytimeRRT<Workspace, Agent, SSTTreeInterface> Planner;

	unsigned int numberOfPRMVertices = stol(args.value("Number Of PRM Vertices"));
	unsigned int numberOfNearestNeighborEdgeConsiderations = stol(args.value("Nearest Neighbors To Consider In PRM Edge Construction"));
	double prmCollisionCheckDT = args.doubleVal("PRM Collision Check DT");

	PRMLite prmLite(workspace, agent, numberOfPRMVertices, numberOfNearestNeighborEdgeConsiderations, prmCollisionCheckDT);

	double alpha = args.doubleVal("Plaku Alpha Value");
	double b = args.doubleVal("Plaku b Value");
	double stateRadius = args.doubleVal("Plaku PRM State Selection Radius");
	double goalBias = args.exists("Goal Bias") ? args.doubleVal("Goal Bias") : 0;
	dfpair(stdout, "goal bias", "%g", goalBias);

	PlakuTreeInterfaceT plakuTreeInterface(workspace, agent, prmLite, start, goal, alpha, b, stateRadius, goalBias);

	double sstRadius = args.doubleVal("SST Radius");
	double sstResize = args.doubleVal("SST Resize Threshold");

	SSTTreeInterface sstTreeInterface(workspace, agent, plakuTreeInterface, plakuTreeInterface, sstRadius, sstResize);
	
	Planner planner(workspace, agent, sstTreeInterface, args);

	go_COMMONANYTIME<Planner, Workspace, Agent>(args, planner, workspace, agent, start, goal, startT);
}

template<class Workspace, class Agent>
void go_AOEST(const InstanceFileMap &args, const Agent &agent, const Workspace &workspace,
                   const typename Agent::State &start, const typename Agent::State &goal) {

	clock_t startT = clock();

	dfpair(stdout, "planner", "%s", "AO EST");
	
	typedef AOEST<Workspace, Agent> Planner;

	Planner planner(workspace, agent, args);

	go_COMMONANYTIME<Planner, Workspace, Agent>(args, planner, workspace, agent, start, goal, startT);
}

template<class Workspace, class Agent>
void go_AnytimeRestartingEST(const InstanceFileMap &args, const Agent &agent, const Workspace &workspace,
                   const typename Agent::State &start, const typename Agent::State &goal) {

	clock_t startT = clock();

	dfpair(stdout, "planner", "%s", "Anytime Restarting EST");

	typedef AnytimeRestartingEST<Workspace, Agent> Planner;

	Planner planner(workspace, agent, args);

	go_COMMONANYTIME<Planner, Workspace, Agent>(args, planner, workspace, agent, start, goal, startT);
}