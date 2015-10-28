#include <random>
std::default_random_engine GlobalRandomGenerator;

#ifdef WITHGRAPHICS
#include "utilities/openglwrapper.hpp"
#endif

#include "utilities/math.hpp"
#include "utilities/flannkdtreewrapper.hpp"
#include "utilities/instancefilemap.hpp"
#include "utilities/fcl_helpers.hpp"
#include "utilities/flann_helpers.hpp"
#include "utilities/heap.hpp"
#include "utilities/datafile.hpp"

#include "workspaces/map3d.hpp"
#include "workspaces/planarlinkage.hpp"
#include "workspaces/kink.hpp"
#include "workspaces/narrowpassage.hpp"
#include "workspaces/rectanglemap2d.hpp"
#include "workspaces/pendulum.hpp"

#include "agents/omnidirectional.hpp"
#include "agents/dubins.hpp"
#include "agents/snake_trailers.hpp"
#include "agents/blimp.hpp"
#include "agents/geometric.hpp"
#include "agents/omnimultid.hpp"

#include "planners/anytimerrt.hpp"
#include "planners/anytimeest.hpp"
#include "planners/anytimebidirectionalest.hpp"
#include "planners/anytimerestartingrrtwithpostprocessing.hpp"
#include "planners/anytimerestartingest.hpp"

#include "planners/rrt.hpp"
#include "planners/rrtconnect.hpp"
#include "planners/reusablerrt.hpp"
#include "planners/kpiece.hpp"
#include "planners/restartingrrtwithpostprocessing.hpp"
#include "planners/aorrt.hpp"
#include "planners/aorrt2.hpp"
#include "planners/est.hpp"
#include "planners/estbidirectional.hpp"
#include "planners/aoest.hpp"

#include "postprocessors/nooppostprocessor.hpp"
#include "postprocessors/simplepostprocessor.hpp"

#include "samplers/uniformsampler.hpp"
#include "samplers/normalsampler.hpp"
#include "samplers/fbiasedsampler.hpp"
#include "samplers/goalbiassampler.hpp"
#include "samplers/probabilitydensityfunction.hpp"

#include "discrete_searches/simplebestfirst.hpp"
#include "discrete_searches/aees.hpp"

#include "tree_interfaces/treeinterface.hpp"
#include "tree_interfaces/plakutreeinterface.hpp"
#include "tree_interfaces/sst.hpp"
#include "tree_interfaces/sst_grid.hpp"
#include "tree_interfaces/frequencytreeinterface.hpp"
#include "tree_interfaces/newtreeinterface.hpp"
#include "tree_interfaces/shell.hpp"

#include "discretizations/workspace/griddiscretization.hpp"
#include "discretizations/workspace/prmlite.hpp"
#include "discretizations/workspace/lazyprmlite.hpp"

#include "anytimeplanners.hpp"
#include "planners.hpp"
#include "domains.hpp"

int main(int argc, char *argv[]) {
	if(argc < 2) {
		fprintf(stderr, "no instance file provided!\n");
		exit(1);
	}

	InstanceFileMap args;
	std::string instFileList;

	for(unsigned int i = 1; i < argc; ++i) {
		args.append(argv[i]);
		instFileList += std::string(argv[i]) + ";";
	}

	double seed = args.doubleVal("Seed");

	GlobalRandomGenerator.seed(seed);
	ompl::RNG::setSeed(seed);

	dfheader(stdout);

	dfpair(stdout, "Instance File List", "%s", instFileList.c_str());

	dfpair(stdout, "Seed", "%g", seed);

	std::string domain = args.value("Agent Type");

	dfpair(stdout, "Agent Type", "%s", domain.c_str());

	if(domain.compare("PlanarLinkage") == 0)
		planarLinkage(args);
	// else if(args.value("Agent Type").compare("Dubins") == 0)
	// 	dubins(args);
	// else if(domain.compare("Snake") == 0)
	// 	snake(args);
	else if(domain.compare("Kink") == 0)
		kink(args);
	else if(domain.compare("NarrowPassageFull") == 0)
		narrowPassage(args, false);
	else if(domain.compare("NarrowPassageHalf") == 0)
		narrowPassage(args, true);
	else if(domain.compare("Rectangle Map") == 0)
		rectangleMap2D(args);
	else if(domain.compare("Pendulum") == 0)
		pendulum(args);
	else if(domain.compare("Blimp") == 0)
		blimp(args);
	// else if(domain.compare("Geometric") == 0)
	// 	geometric(args);
	else
		fprintf(stderr, "unrecognized Agent Type: %s\n", domain.c_str());

	dffooter(stdout);

	return 0;
}