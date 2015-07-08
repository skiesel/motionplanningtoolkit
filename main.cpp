#include <random>
std::default_random_engine GlobalRandomGenerator;

#ifdef WITHGRAPHICS
	#include "utilities/openglwrapper.hpp"
#endif

#include "workspaces/map3d.hpp"

#include "agents/omnidirectional.hpp"
#include "agents/dubins.hpp"
#include "agents/snake_trailers.hpp"
#include "agents/blimp.hpp"

#include "planners/rrt.hpp"
#include "planners/rrtconnect.hpp"
#include "planners/kpiece.hpp"

#include "samplers/uniformsampler.hpp"
#include "samplers/normalsampler.hpp"
#include "samplers/fbiasedsampler.hpp"

#include "tree_interfaces/treeinterface.hpp"
#include "tree_interfaces/plakutreeinterface.hpp"

#include "discretizations/workspace/griddiscretization.hpp"
#include "discretizations/workspace/prmlite.hpp"

#include "utilities/flannkdtreewrapper.hpp"
#include "utilities/instancefilemap.hpp"
#include "utilities/fcl_helpers.hpp"

#include "blimpplanners.hpp"

std::vector<double> parseDoubles(const std::string &str) {
	std::vector<double> values;
	boost::char_separator<char> sep(" ");
	boost::tokenizer< boost::char_separator<char> > tokens(str, sep);
	for(auto token : tokens) {
		values.push_back(std::stod(token));
	}
	return values;
}


// void omnidirectional(const InstanceFileMap& args) {
// 	typedef Omnidirectional Agent;
// 	typedef Map3D<Agent> Workspace;
// 	typedef flann::KDTreeSingleIndexParams KDTreeType;
// 	typedef FLANN_KDTreeWrapper<KDTreeType, flann::L2<double>, Agent::Edge> KDTree;
// 	typedef UniformSampler<Workspace, Agent, KDTree> Sampler;
// 	typedef TreeInterface<Agent, KDTree, Sampler> TreeInterface;
// 	typedef RRT<Workspace, Agent, TreeInterface> Planner;

// 	Agent agent(args);
// 	Workspace workspace(args);

// 	Agent::State start(args.doubleList("Agent Start Location"));
// 	auto goalVars = args.doubleList("Agent Goal Location");
// 	Agent::State goal(goalVars);

// 	KDTreeType kdtreeType;
// 	KDTree kdtree(kdtreeType, agent.getTreeStateSize());

// 	Sampler sampler(workspace, agent, kdtree);

// 	TreeInterface treeInterface(kdtree, sampler);

// 	Planner planner(workspace, agent, treeInterface, args);

// 	#ifdef WITHGRAPHICS
// 		bool firstInvocation = true;
// 		unsigned int i = 0; 
// 		auto lambda = [&](){
// 			agent.draw();
// 			workspace.draw();
// 			planner.query(start, goal, 100, firstInvocation);
// 			firstInvocation = false;
// 		};
// 		OpenGLWrapper::getOpenGLWrapper().runWithCallback(lambda);
// 	#else
// 		planner.query(start, goal);
// 	#endif
// }

// void dubins(const InstanceFileMap& args) {
// 	typedef Dubins Agent;
// 	typedef Map3D<Agent> Workspace;
// 	typedef flann::KDTreeSingleIndexParams KDTreeType;
// 	typedef FLANN_KDTreeWrapper<KDTreeType, flann::L2<double>, Agent::Edge> KDTree;
// 	typedef UniformSampler<Workspace, Agent, KDTree> Sampler;
// 	typedef TreeInterface<Agent, KDTree, Sampler> TreeInterface;
// 	typedef RRT<Workspace, Agent, TreeInterface> Planner;

// 	Agent agent(args);
// 	Workspace workspace(args);

// 	agent.updateBounds(workspace.getBounds());

// 	Agent::State start(args.doubleList("Agent Start Location"));
// 	auto goalVars = args.doubleList("Agent Goal Location");
// 	Agent::State goal(goalVars);

// 	KDTreeType kdtreeType;
// 	KDTree kdtree(kdtreeType, agent.getTreeStateSize());

// 	Sampler sampler(workspace, agent, kdtree);

// 	TreeInterface treeInterface(kdtree, sampler);

// 	Planner planner(workspace, agent, treeInterface, args);

// 	#ifdef WITHGRAPHICS
// 		bool firstInvocation = true;
// 		unsigned int i = 0; 
// 		auto lambda = [&](){
// 			agent.draw();
// 			workspace.draw();
// 			planner.query(start, goal, 100, firstInvocation);
// 			firstInvocation = false;
// 		};
// 		OpenGLWrapper::getOpenGLWrapper().runWithCallback(lambda);
// 	#else
// 		planner.query(start, goal);
// 	#endif
// }

// void snake(const InstanceFileMap& args) {
// 	typedef SnakeTrailers Agent;
// 	typedef Map3D<Agent> Workspace;
// 	typedef flann::KDTreeSingleIndexParams KDTreeType;
// 	typedef FLANN_KDTreeWrapper<KDTreeType, flann::L2<double>, Agent::Edge> KDTree;
// 	typedef UniformSampler<Workspace, Agent, KDTree> Sampler;
// 	typedef TreeInterface<Agent, KDTree, Sampler> TreeInterface;
// 	typedef RRT<Workspace, Agent, TreeInterface> Planner;

// 	Agent agent(args);
// 	Workspace workspace(args);

// 	Agent::State start(args.doubleList("Agent Start Location"));
// 	auto goalVars = args.doubleList("Agent Goal Location");
// 	Agent::State goal(goalVars);

// 	KDTreeType kdtreeType;
// 	KDTree kdtree(kdtreeType, agent.getTreeStateSize());

// 	Sampler sampler(workspace, agent, kdtree);

// 	TreeInterface treeInterface(kdtree, sampler);

// 	Planner planner(workspace, agent, treeInterface, args);

// 	#ifdef WITHGRAPHICS
// 		bool firstInvocation = true;
// 		unsigned int i = 0; 
// 		auto lambda = [&](){
// 			// agent.draw();
// 			// workspace.draw();
// 			planner.query(start, goal, 100, firstInvocation);
// 			firstInvocation = false;
// 		};
// 		OpenGLWrapper::getOpenGLWrapper().runWithCallback(lambda);
// 	#else
// 		planner.query(start, goal);
// 	#endif
// }

int main(int argc, char *argv[]) {
	if(argc < 2) {
		fprintf(stderr, "no instance file provided!\n");
		exit(1);
	}

	InstanceFileMap args(argv[1]);

	GlobalRandomGenerator.seed(stod(args.value("Seed")));
	ompl::RNG::setSeed(stod(args.value("Seed")));

	dfheader(stdout);

	// if(args.value("Agent Type").compare("Omnidirectional") == 0)
	// 	omnidirectional(args);
	// else if(args.value("Agent Type").compare("Dubins") == 0)
	// 	dubins(args);
	// else if(args.value("Agent Type").compare("Snake") == 0)
	// 	snake(args);
	// else
		if(args.value("Agent Type").compare("Blimp") == 0)
		blimp(args);
	else
		fprintf(stderr, "unrecognized Agent Type\n");

	dffooter(stdout);

	return 0;
}