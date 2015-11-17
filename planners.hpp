#pragma once

#include <iostream>

template<class Planner, class Workspace, class Agent>
void go_COMMON(const InstanceFileMap &args, Planner &planner,
		const Workspace &workspace, const Agent &agent,
		const typename Agent::State &start, const typename Agent::State &goal) {

#ifdef WITHGRAPHICS
	bool firstIteration = true;
	auto lambda = [&]() {
		// std::cin.ignore();
		start.draw();
		goal.draw();
		agent.drawMesh(start);
		workspace.draw();
		planner.query(start, goal, 1000, firstIteration);
		firstIteration = false;
	};
	OpenGLWrapper::getOpenGLWrapper().runWithCallback(lambda, args);
#else
	planner.query(start, goal);
	planner.dfpairs();
#endif
}

template<class Workspace, class Agent>
void go_RRT(const InstanceFileMap &args, const Agent &agent, const Workspace &workspace,
            const typename Agent::State &start, const typename Agent::State &goal) {

	dfpair(stdout, "planner", "%s", "RRT");
	// typedef flann::KDTreeSingleIndexParams KDTreeType;
	typedef flann::KDTreeIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, typename Agent::DistanceEvaluator, typename Agent::Edge> KDTree;
	typedef UniformSampler<Workspace, Agent, KDTree> USampler;
	typedef GoalBiasSampler<Agent, USampler> GBSampler;
	typedef TreeInterface<Agent, KDTree, GBSampler> TreeInterface;
	typedef RRT<Workspace, Agent, TreeInterface> Planner;

	/* planner config */

	KDTreeType kdtreeType(1);
	KDTree kdtree(kdtreeType, agent.getDistanceEvaluator(), agent.getTreeStateSize());
	USampler uniformsampler(workspace, agent, kdtree);

	double goalBias = args.exists("Goal Bias") ? args.doubleVal("Goal Bias") : 0;
	dfpair(stdout, "goal bias", "%g", goalBias);

	GBSampler goalbiassampler(uniformsampler, goal, goalBias);
	TreeInterface treeInterface(kdtree, goalbiassampler);
	Planner planner(workspace, agent, treeInterface, args);

	go_COMMON<Planner, Workspace, Agent>(args, planner, workspace, agent, start, goal);
}

template<class Workspace, class Agent>
void go_RRTConnect(const InstanceFileMap &args, const Agent &agent, const Workspace &workspace,
            const typename Agent::State &start, const typename Agent::State &goal) {

	dfpair(stdout, "planner", "%s", "RRT-Connect");
	// typedef flann::KDTreeSingleIndexParams KDTreeType;
	typedef flann::KDTreeIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, typename Agent::DistanceEvaluator, typename Agent::Edge> KDTree;
	typedef UniformSampler<Workspace, Agent, KDTree> USampler;
	typedef GoalBiasSampler<Agent, USampler> GBSampler;
	typedef TreeInterface<Agent, KDTree, GBSampler> TreeInterface;
	typedef RRTConnect<Workspace, Agent, TreeInterface> Planner;

	/* planner config */

	KDTreeType kdtreeType(1);
	KDTree kdtree(kdtreeType, agent.getDistanceEvaluator(), agent.getTreeStateSize());
	USampler uniformsampler(workspace, agent, kdtree);

	double goalBias = args.exists("Goal Bias") ? args.doubleVal("Goal Bias") : 0;
	dfpair(stdout, "goal bias", "%g", goalBias);

	GBSampler goalbiassampler(uniformsampler, goal, goalBias);
	TreeInterface treeInterface(kdtree, goalbiassampler);
	Planner planner(workspace, agent, treeInterface, args);

	go_COMMON<Planner, Workspace, Agent>(args, planner, workspace, agent, start, goal);
}

template<class Workspace, class Agent>
void go_FBiasedRRT(const InstanceFileMap &args, const Agent &agent, const Workspace &workspace,
            const typename Agent::State &start, const typename Agent::State &goal) {

	dfpair(stdout, "planner", "%s", "FBiased RRT");
	typedef PRMLite<Workspace, Agent> PRMLite;
	// typedef flann::KDTreeSingleIndexParams KDTreeType;
	typedef flann::KDTreeIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, typename Agent::DistanceEvaluator, typename Agent::Edge> KDTree;
	typedef FBiasedSampler<Workspace, Agent, KDTree, PRMLite> Sampler;
	typedef GoalBiasSampler<Agent, Sampler> GBSampler;
	typedef TreeInterface<Agent, KDTree, GBSampler> TreeInterface;
	typedef RRT<Workspace, Agent, TreeInterface> Planner;

	/* planner config */

	unsigned int numberOfPRMVertices = stol(args.value("Number Of PRM Vertices"));
	unsigned int numberOfNearestNeighborEdgeConsiderations = stol(args.value("Nearest Neighbors To Consider In PRM Edge Construction"));
	double prmCollisionCheckDT = args.doubleVal("PRM Collision Check DT");

	PRMLite prmLite(workspace, agent, numberOfPRMVertices, numberOfNearestNeighborEdgeConsiderations, prmCollisionCheckDT);

	KDTreeType kdtreeType(1);
	KDTree kdtree(kdtreeType, agent.getDistanceEvaluator(), agent.getTreeStateSize());

	double omega = args.doubleVal("FBias Omega");
	double stateRadius = args.doubleVal("FBias State Selection Radius");

	dfpair(stdout, "omega", "%g", omega);
	dfpair(stdout, "state selection radius", "%g", stateRadius);

	Sampler sampler(workspace, agent, kdtree, prmLite, start, goal, stateRadius, omega);

	double goalBias = args.exists("Goal Bias") ? args.doubleVal("Goal Bias") : 0;
	dfpair(stdout, "goal bias", "%g", goalBias);

	GBSampler goalbiassampler(sampler, goal, goalBias);
	TreeInterface treeInterface(kdtree, goalbiassampler);
	Planner planner(workspace, agent, treeInterface, args);

	go_COMMON<Planner, Workspace, Agent>(args, planner, workspace, agent, start, goal);
}

template<class Workspace, class Agent>
void go_FBiasedShellRRT(const InstanceFileMap &args, const Agent &agent, const Workspace &workspace,
            const typename Agent::State &start, const typename Agent::State &goal) {

	dfpair(stdout, "planner", "%s", "FBiased Shell RRT");
	typedef PRMLite<Workspace, Agent> PRMLite;
	// typedef flann::KDTreeSingleIndexParams KDTreeType;
	typedef flann::KDTreeIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, typename Agent::DistanceEvaluator, typename Agent::Edge> KDTree;
	typedef FBiasedShellSampler<Workspace, Agent, KDTree, PRMLite> Sampler;
	// typedef GoalBiasSampler<Agent, Sampler> GBSampler;
	// typedef TreeInterface<Agent, KDTree, GBSampler> TreeInterface;
	typedef Shell<Agent, KDTree, Sampler> TreeInterface;
	typedef RRT<Workspace, Agent, TreeInterface> Planner;

	/* planner config */

	unsigned int numberOfPRMVertices = stol(args.value("Number Of PRM Vertices"));
	unsigned int numberOfNearestNeighborEdgeConsiderations = stol(args.value("Nearest Neighbors To Consider In PRM Edge Construction"));
	double prmCollisionCheckDT = args.doubleVal("PRM Collision Check DT");

	PRMLite prmLite(workspace, agent, numberOfPRMVertices, numberOfNearestNeighborEdgeConsiderations, prmCollisionCheckDT);

	KDTreeType kdtreeType(1);
	KDTree kdtree(kdtreeType, agent.getDistanceEvaluator(), agent.getTreeStateSize());

	double shellPreference = args.doubleVal("Shell Preference");
	dfpair(stdout, "shell preference", "%g", shellPreference);

	double omega = args.doubleVal("FBias Omega");
	double stateRadius = args.doubleVal("FBias State Selection Radius");

	dfpair(stdout, "omega", "%g", omega);
	dfpair(stdout, "state selection radius", "%g", stateRadius);

	double shellDepth = args.doubleVal("Shell Depth");
	dfpair(stdout, "shell depth", "%g", shellDepth);


	Sampler sampler(workspace, agent, kdtree, prmLite, start, goal, stateRadius, shellDepth, omega, shellPreference);

	// double goalBias = args.exists("Goal Bias") ? args.doubleVal("Goal Bias") : 0;
	// dfpair(stdout, "goal bias", "%g", goalBias);

	// GBSampler goalbiassampler(sampler, goal, goalBias);

	TreeInterface treeInterface(kdtree, sampler, shellDepth);
	Planner planner(workspace, agent, treeInterface, args);

	go_COMMON<Planner, Workspace, Agent>(args, planner, workspace, agent, start, goal);
}

template<class Workspace, class Agent>
void go_EST(const InstanceFileMap &args, const Agent &agent, const Workspace &workspace,
                   const typename Agent::State &start, const typename Agent::State &goal) {

	dfpair(stdout, "planner", "%s", "EST");

	typedef EST<Workspace, Agent> Planner;

	Planner planner(workspace, agent, args);

	go_COMMON<Planner, Workspace, Agent>(args, planner, workspace, agent, start, goal);
}

template<class Workspace, class Agent>
void go_ESTBIDIR(const InstanceFileMap &args, const Agent &agent, const Workspace &workspace,
                   const typename Agent::State &start, const typename Agent::State &goal) {

	dfpair(stdout, "planner", "%s", "EST");
	// clock_t startT = clock();

	typedef ESTBidirectional<Workspace, Agent> Planner;

	Planner planner(workspace, agent, args);

	go_COMMON<Planner, Workspace, Agent>(args, planner, workspace, agent, start, goal);
}

template<class Workspace, class Agent>
void go_KPIECE(const InstanceFileMap &args, const Agent &agent, const Workspace &workspace,
               const typename Agent::State &start, const typename Agent::State &goal) {
	dfpair(stdout, "planner", "%s", "KPIECE");

	typedef KPIECE<Workspace, Agent> Planner;

	Planner planner(workspace, agent, args);

	go_COMMON<Planner, Workspace, Agent>(args, planner, workspace, agent, start, goal);
}

template<class Workspace, class Agent>
void go_SSTPPRM(const InstanceFileMap &args, const Agent &agent, const Workspace &workspace,
                   const typename Agent::State &start, const typename Agent::State &goal) {
	dfpair(stdout, "planner", "%s", "SST + PPRM");

	typedef PRMLite<Workspace, Agent> PRMLite;
	typedef PlakuTreeInterface<Workspace, Agent, PRMLite> PlakuTreeInterfaceT;
	typedef SST<Workspace, Agent, PlakuTreeInterfaceT, PlakuTreeInterfaceT> SSTTreeInterface;
	typedef RRT<Workspace, Agent, SSTTreeInterface> Planner;

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

	go_COMMON<Planner, Workspace, Agent>(args, planner, workspace, agent, start, goal);
}

template<class Workspace, class Agent>
void go_SSTGridPPRM(const InstanceFileMap &args, const Agent &agent, const Workspace &workspace,
                   const typename Agent::State &start, const typename Agent::State &goal) {
	dfpair(stdout, "planner", "%s", "SST Grid + PPRM");

	typedef PRMLite<Workspace, Agent> PRMLite;
	typedef PlakuTreeInterface<Workspace, Agent, PRMLite> PlakuTreeInterfaceT;
	typedef SST_Grid<Workspace, Agent, PlakuTreeInterfaceT, PlakuTreeInterfaceT> SSTTreeInterface;
	typedef RRT<Workspace, Agent, SSTTreeInterface> Planner;

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

	go_COMMON<Planner, Workspace, Agent>(args, planner, workspace, agent, start, goal);
}

template<class Workspace, class Agent>
void go_PPRM(const InstanceFileMap &args, const Agent &agent, const Workspace &workspace,
             const typename Agent::State &start, const typename Agent::State &goal) {
	dfpair(stdout, "planner", "%s", "PPRM");

	typedef PRMLite<Workspace, Agent> PRMLite;
	typedef PlakuTreeInterface<Workspace, Agent, PRMLite> PlakuTreeInterfaceT;
	typedef RRT<Workspace, Agent, PlakuTreeInterfaceT> Planner;

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

// #ifdef WITHGRAPHICS
// 	bool firstIteration = true;
// 	auto lambda = [&]() {
// 		plakuTreeInterface.draw();
// 		workspace.draw();
// 	};
// 	OpenGLWrapper::getOpenGLWrapper().runWithCallback(lambda, args);
// #endif

	Planner planner(workspace, agent, plakuTreeInterface, args);

	go_COMMON<Planner, Workspace, Agent>(args, planner, workspace, agent, start, goal);
}

template<class Workspace, class Agent>
void go_NewSearch(const InstanceFileMap &args, const Agent &agent, const Workspace &workspace,
                   const typename Agent::State &start, const typename Agent::State &goal) {
	dfpair(stdout, "planner", "%s", "New Search");

	typedef flann::KDTreeIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, typename Agent::DistanceEvaluator, typename Agent::Edge> KDTree;
	// typedef FrequencyTreeInterface<Agent> RegionManager;
	typedef PRMLite<Workspace, Agent> PRMLite;
	typedef NewTreeInterface<Workspace, Agent, KDTree, PRMLite> TreeInterface;
	typedef RRT<Workspace, Agent, TreeInterface> Planner;

	KDTreeType kdtreeType(1);
	KDTree kdtree(kdtreeType, agent.getDistanceEvaluator(), agent.getTreeStateSize());

	unsigned int numberOfPRMVertices = stol(args.value("Number Of PRM Vertices"));
	unsigned int numberOfNearestNeighborEdgeConsiderations = stol(args.value("Nearest Neighbors To Consider In PRM Edge Construction"));
	double prmCollisionCheckDT = args.doubleVal("PRM Collision Check DT");

	PRMLite prmLite(workspace, agent, numberOfPRMVertices, numberOfNearestNeighborEdgeConsiderations, prmCollisionCheckDT);

	// SimpleBestFirst discreteSearch;

	double stateRadius = args.doubleVal("PRM State Selection Radius");
	double goalBias = args.exists("Goal Bias") ? args.doubleVal("Goal Bias") : 0;
	dfpair(stdout, "goal bias", "%g", goalBias);

	TreeInterface treeInterface(workspace, agent, kdtree, prmLite, start, goal, stateRadius, goalBias);

	Planner planner(workspace, agent, treeInterface, args);

	go_COMMON<Planner, Workspace, Agent>(args, planner, workspace, agent, start, goal);
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
	} else if(planner.compare("FBiased RRT") == 0) {
		go_FBiasedRRT<Workspace, Agent>(args, agent, workspace, start, goal);
	} else if(planner.compare("FBiased Shell RRT") == 0) {
		go_FBiasedShellRRT<Workspace, Agent>(args, agent, workspace, start, goal);
	} else if(planner.compare("New Search") == 0) {
		go_NewSearch<Workspace, Agent>(args, agent, workspace, start, goal);
	} else if(planner.compare("PPRM") == 0) {
		go_PPRM<Workspace, Agent>(args, agent, workspace, start, goal);
	} else if(planner.compare("KPIECE") == 0) {
		go_KPIECE<Workspace, Agent>(args, agent, workspace, start, goal);
	} else if(planner.compare("SST") == 0) {
		go_SST<Workspace, Agent>(args, agent, workspace, start, goal);
	} else if(planner.compare("SST Grid") == 0) {
		go_SSTGrid<Workspace, Agent>(args, agent, workspace, start, goal);
	} else if(planner.compare("SST + PPRM") == 0) {
		go_SSTPPRM<Workspace, Agent>(args, agent, workspace, start, goal);
	} else if(planner.compare("SST Grid + PPRM") == 0) {
		go_SSTGridPPRM<Workspace, Agent>(args, agent, workspace, start, goal);
	} else if(planner.compare("MRRT") == 0) {
		go_MRRT<Workspace, Agent>(args, agent, workspace, start, goal);
	} else if(planner.compare("MRRT+S") == 0) {
		go_MRRTPlusS<Workspace, Agent>(args, agent, workspace, start, goal);
	} else if(planner.compare("AO RRT") == 0) {
		go_AORRT<Workspace, Agent>(args, agent, workspace, start, goal);
	} else if(planner.compare("AO RRT 2") == 0) {
		go_AORRT2<Workspace, Agent>(args, agent, workspace, start, goal);
	} else if(planner.compare("EST") == 0) {
		go_EST<Workspace, Agent>(args, agent, workspace, start, goal);
	} else if(planner.compare("EST Bidirectional") == 0) {
		go_ESTBIDIR<Workspace, Agent>(args, agent, workspace, start, goal);
	} else if(planner.compare("AO EST") == 0) {
		go_AOEST<Workspace, Agent>(args, agent, workspace, start, goal);
	} else if(planner.compare("Anytime RRT") == 0) {
		go_AnytimeRRT<Workspace, Agent>(args, agent, workspace, start, goal);
	} else if(planner.compare("Anytime EST") == 0) {
		go_AnytimeEST<Workspace, Agent>(args, agent, workspace, start, goal);
	} else if(planner.compare("Anytime Restarting EST") == 0) {
		go_AnytimeRestartingEST<Workspace, Agent>(args, agent, workspace, start, goal);
	} else if(planner.compare("Anytime Bidirectional EST") == 0) {
		go_AnytimeBidirectionalEST<Workspace, Agent>(args, agent, workspace, start, goal);
	} else if(planner.compare("Anytime PPRM") == 0) {
		go_AnytimePPRM<Workspace, Agent>(args, agent, workspace, start, goal);
	} else if(planner.compare("Anytime PPRM + SST") == 0) {
		go_AnytimeSSTPPRM<Workspace, Agent>(args, agent, workspace, start, goal);
	} else {
		fprintf(stderr, "unreocognized planner: %s\n", planner.c_str());
	}
	clock_t endT = clock();
	dfpair(stdout, "total solving time", "%g", (double)(endT-startT) / CLOCKS_PER_SEC);
}
