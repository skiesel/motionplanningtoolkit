#include <random>
std::default_random_engine GlobalRandomGenerator;

#ifdef WITHGRAPHICS
#include "utilities/openglwrapper.hpp"
#endif

#include "utilities/math.hpp"

#include "workspaces/map3d.hpp"
#include "workspaces/planarlinkage.hpp"
#include "workspaces/kink.hpp"
#include "workspaces/narrowpassage.hpp"

#include "agents/omnidirectional.hpp"
#include "agents/dubins.hpp"
#include "agents/snake_trailers.hpp"
#include "agents/blimp.hpp"
#include "agents/geometric.hpp"
#include "agents/omnimultid.hpp"

#include "planners/rrt.hpp"
#include "planners/rrtconnect.hpp"
#include "planners/kpiece.hpp"

#include "samplers/uniformsampler.hpp"
#include "samplers/normalsampler.hpp"
#include "samplers/fbiasedsampler.hpp"

#include "tree_interfaces/treeinterface.hpp"
#include "tree_interfaces/plakutreeinterface.hpp"
#include "tree_interfaces/sst.hpp"
#include "tree_interfaces/sst_grid.hpp"

#include "discretizations/workspace/griddiscretization.hpp"
#include "discretizations/workspace/prmlite.hpp"
#include "discretizations/workspace/lazyprmlite.hpp"

#include "utilities/flannkdtreewrapper.hpp"
#include "utilities/instancefilemap.hpp"
#include "utilities/fcl_helpers.hpp"

#include "plannerfunctions.hpp"

std::vector<double> parseDoubles(const std::string &str) {
	std::vector<double> values;
	boost::char_separator<char> sep(" ");
	boost::tokenizer< boost::char_separator<char> > tokens(str, sep);
	for(auto token : tokens) {
		values.push_back(std::stod(token));
	}
	return values;
}

int main(int argc, char *argv[]) {
	if(argc < 2) {
		fprintf(stderr, "no instance file provided!\n");
		exit(1);
	}

	InstanceFileMap args(argv[1]);
	for(unsigned int i = 2; i < argc; ++i) {
		args.append(argv[i]);
	}

	GlobalRandomGenerator.seed(args.doubleVal("Seed"));
	ompl::RNG::setSeed(args.doubleVal("Seed"));

	dfheader(stdout);

	std::string domain = args.value("Agent Type");

	if(domain.compare("PlanarLinkage") == 0)
		planarLinkage(args);
	// else if(args.value("Agent Type").compare("Dubins") == 0)
	// 	dubins(args);
	else if(domain.compare("Snake") == 0)
		snake(args);
	else if(domain.compare("Kink") == 0)
		kink(args);
	else if(domain.compare("NarrowPassageFull") == 0)
		narrowPassage(args, false);
	else if(domain.compare("NarrowPassageHalf") == 0)
		narrowPassage(args, true);
	else if(domain.compare("Blimp") == 0)
		blimp(args);
	else if(domain.compare("Geometric") == 0)
		geometric(args);
	else
		fprintf(stderr, "unrecognized Agent Type: %s\n", domain.c_str());

	dffooter(stdout);

	return 0;
}