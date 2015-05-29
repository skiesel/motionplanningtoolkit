#ifdef WITHGRAPHICS
	#include "utilities/openglwrapper.hpp"
#endif

#include "workspaces/map3d.hpp"

#include "agents/omnidirectional.hpp"
#include "agents/dubins.hpp"
#include "agents/snake_trailers.hpp"
#include "agents/blimp.hpp"

#include "planners/rrt.hpp"

#include "samplers/uniformsampler.hpp"
#include "samplers/normalsampler.hpp"
#include "samplers/fbiasedsampler.hpp"

#include "discretizations/workspace/griddiscretization.hpp"

#include "utilities/flannkdtreewrapper.hpp"
#include "utilities/instancefilemap.hpp"

std::vector<double> parseDoubles(const std::string &str) {
	std::vector<double> values;
	boost::char_separator<char> sep(" ");
	boost::tokenizer< boost::char_separator<char> > tokens(str, sep);
	for(auto token : tokens) {
		values.push_back(std::stod(token));
	}
	return values;
}


void omnidirectional(const InstanceFileMap& args) {
	typedef Omnidirectional Agent;
	typedef Map3D<Agent> Workspace;
	typedef GridDiscretization<Workspace, Agent> Discretization;
	typedef UniformSampler<Workspace, Agent> Sampler;
	typedef flann::KDTreeSingleIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, flann::L2<double>, Agent::Edge> KDTree;
	typedef RRT<Workspace, Agent, Sampler, KDTree> Planner;

	Agent agent(args);
	Workspace workspace(args);

	Agent::State start(parseDoubles(args.value("Agent Start Location")));
	auto goalVars = parseDoubles(args.value("Agent Goal Location"));
	Agent::State goal(goalVars);

	Sampler sampler(workspace, agent);

	KDTreeType kdtreeType;
	KDTree kdtree(kdtreeType, agent.getTreeStateSize());

	Planner planner(workspace, agent, sampler, kdtree, args);

	#ifdef WITHGRAPHICS
		bool firstInvocation = true;
		unsigned int i = 0; 
		auto lambda = [&](){
			agent.draw();
			workspace.draw();
			planner.query(start, goal, 100, firstInvocation);
			firstInvocation = false;
		};
		OpenGLWrapper::getOpenGLWrapper().runWithCallback(lambda);
	#else
		planner.query(start, goal);
	#endif
}

void dubins(const InstanceFileMap& args) {
	typedef Dubins Agent;
	typedef Map3D<Agent> Workspace;
	typedef GridDiscretization<Workspace, Agent> Discretization;
	typedef UniformSampler<Workspace, Agent> Sampler;
	typedef flann::KDTreeSingleIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, flann::L2<double>, Agent::Edge> KDTree;
	typedef RRT<Workspace, Agent, Sampler, KDTree> Planner;

	Agent agent(args);
	Workspace workspace(args);

	Agent::State start(parseDoubles(args.value("Agent Start Location")));
	auto goalVars = parseDoubles(args.value("Agent Goal Location"));
	Agent::State goal(goalVars);

	Sampler sampler(workspace, agent);

	KDTreeType kdtreeType;
	KDTree kdtree(kdtreeType, agent.getTreeStateSize());

	Planner planner(workspace, agent, sampler, kdtree, args);

	#ifdef WITHGRAPHICS
		bool firstInvocation = true;
		unsigned int i = 0; 
		auto lambda = [&](){
			agent.draw();
			workspace.draw();
			planner.query(start, goal, 100, firstInvocation);
			firstInvocation = false;
		};
		OpenGLWrapper::getOpenGLWrapper().runWithCallback(lambda);
	#else
		planner.query(start, goal);
	#endif
}

void snake(const InstanceFileMap& args) {
	typedef SnakeTrailers Agent;
	typedef Map3D<Agent> Workspace;
	typedef GridDiscretization<Workspace, Agent> Discretization;
	typedef UniformSampler<Workspace, Agent> Sampler;
	typedef flann::KDTreeIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, flann::L2<double>, Agent::Edge> KDTree;
	typedef RRT<Workspace, Agent, Sampler, KDTree> Planner;

	Agent agent(args);
	Workspace workspace(args);

	Agent::State start(parseDoubles(args.value("Agent Start Location")));
	auto goalVars = parseDoubles(args.value("Agent Goal Location"));
	Agent::State goal(goalVars);

	Sampler sampler(workspace, agent);

	KDTreeType kdtreeType;
	KDTree kdtree(kdtreeType, agent.getTreeStateSize());

	Planner planner(workspace, agent, sampler, kdtree, args);

	#ifdef WITHGRAPHICS
		bool firstInvocation = true;
		unsigned int i = 0; 
		auto lambda = [&](){
			// agent.draw();
			// workspace.draw();
			planner.query(start, goal, 100, firstInvocation);
			firstInvocation = false;
		};
		OpenGLWrapper::getOpenGLWrapper().runWithCallback(lambda);
	#else
		planner.query(start, goal);
	#endif
}

void blimp(const InstanceFileMap& args) {
	typedef Blimp Agent;
	typedef Map3D<Agent> Workspace;
	typedef GridDiscretization<Workspace, Agent> Discretization;
	typedef UniformSampler<Workspace, Agent> Sampler;
	typedef flann::KDTreeSingleIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, flann::L2<double>, Agent::Edge> KDTree;
	typedef RRT<Workspace, Agent, Sampler, KDTree> Planner;

	Agent agent(args);
	Workspace workspace(args);

	Agent::State start(parseDoubles(args.value("Agent Start Location")));
	auto goalVars = parseDoubles(args.value("Agent Goal Location"));
	Agent::State goal(goalVars);

	Sampler sampler(workspace, agent);

	KDTreeType kdtreeType;
	KDTree kdtree(kdtreeType, agent.getTreeStateSize());

	Planner planner(workspace, agent, sampler, kdtree, args);

	#ifdef WITHGRAPHICS
		bool firstInvocation = true;
		unsigned int i = 0; 
		auto lambda = [&](){
			agent.draw();
			workspace.draw();
			planner.query(start, goal, 100, firstInvocation);
			firstInvocation = false;
		};
		OpenGLWrapper::getOpenGLWrapper().runWithCallback(lambda);
	#else
		planner.query(start, goal);
	#endif
}

int main(int argc, char *argv[]) {
	if(argc < 2) {
		fprintf(stderr, "no instance file provided!\n");
		exit(1);
	}

	InstanceFileMap args(argv[1]);

	if(args.value("Agent Type").compare("Omnidirectional") == 0)
		omnidirectional(args);
	else if(args.value("Agent Type").compare("Dubins") == 0)
		dubins(args);
	else if(args.value("Agent Type").compare("Snake") == 0)
		snake(args);
	else if(args.value("Agent Type").compare("Blimp") == 0)
		blimp(args);
	else
		fprintf(stderr, "unrecognized Agent Type\n");

	return 0;
}