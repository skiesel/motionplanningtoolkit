#include <random>
std::default_random_engine GlobalRandomGenerator;

#ifdef WITHGRAPHICS
#include "utilities/openglwrapper.hpp"
#endif

#include <string>

#include "utilities/instancefilemap.hpp"
#include "utilities/meshhandler.hpp"
#include "utilities/math.hpp"
#include "utilities/datafile.hpp"
#include "utilities/flannkdtreewrapper.hpp"

void print(const std::vector<double> &transform) {
	for(unsigned int i = 0; i < 16; i++) {
		if(i % 4 == 0) fprintf(stderr, "\n");
		fprintf(stderr, "%g ", transform[i]);
	}
}

std::vector<double> toDoubleVec(const fcl::Transform3f &transform) {
	std::vector<double> vec(16);

	const fcl::Vec3f &transVec = transform.getTranslation();
	const fcl::Matrix3f &rot = transform.getRotation();

	std::vector<double> transMatrix(16, 0);
	std::vector<double> rotationMatrix(16, 0);

	for(unsigned int i = 0; i < 16; i++) {
		if(i % 4 == i / 4) {
			transMatrix[i] = 1;
			rotationMatrix[i] = 1;
		}
	}

	transMatrix[3] = transVec[0];
	transMatrix[7] = transVec[1];
	transMatrix[11] = transVec[2];

	for(unsigned int i = 0; i < 3; i++) {

		const fcl::Vec3f &row = rot.getRow(i);
		for(unsigned int j = 0; j < 3; j++) {
			//yes, this should be 4
			rotationMatrix[i * 4 + j] = row[j];
		}
	}

	math::multiply(rotationMatrix, transMatrix, vec);

	return vec;
}

std::vector<double> transpose(const std::vector<double> &transform) {
	std::vector<double> transpose(16);

	for(unsigned int i = 0; i < 16; i++) {
		unsigned int row = i / 4;
		unsigned int col = i % 4;
		transpose[i] = transform[col * 4 + row];
	}

	return transpose;
}

void runCollisionTiming(const StaticEnvironmentMeshHandler &env, const SimpleAgentMeshHandler &agent, unsigned int checks, unsigned int frequency) {
	static MeshHandler collisionChecker;

	static InstanceFileMap args;

	std::vector<std::vector<fcl::Transform3f> > poses;
	std::vector<const SimpleAgentMeshHandler *> agentPieces;
	agentPieces.push_back(&agent);

	fcl::Transform3f identity;
	identity.setIdentity();

	poses.emplace_back();
	poses.back().push_back(identity);

	double increment = 1;

	unsigned int collisions = 0;

	double startTime = walltime();

	double intervalStart = walltime(), intervalEnd;
	for(unsigned int i = 0; i < checks; i++) {
		if(i != 0 && i % frequency == 0) {
			intervalEnd = walltime();
			double duration = intervalEnd - intervalStart;
			fprintf(stderr, "interval duration for %u checks: %g\n", frequency, duration);
			fprintf(stderr, "interval rate: %g checks/sec\n", (double)frequency / duration);
			intervalStart = intervalEnd;
		}

		poses.back()[0].setTranslation(fcl::Vec3f(0, 0, increment-=0.01));

		if(collisionChecker.isInCollision(env, agentPieces, poses))
			collisions++;
	}
	double endTime = walltime();

	double duration = endTime - startTime;
	fprintf(stderr, "total time for %u collision checks: %g\n", checks, duration);
	fprintf(stderr, "total rate: %g checks/sec\n", (double)checks / duration);
}

void runCollisionTimings(unsigned int checks, unsigned int frequency) {
	StaticEnvironmentMeshHandler blimpEnv("/Users/skiesel/gopath/src/github.com/skiesel/motionplanningtoolkit/mesh_models/environment_models/blimp_world.dxf", "0 0 0 1 0 0 0");
	SimpleAgentMeshHandler blimpMesh("/Users/skiesel/gopath/src/github.com/skiesel/motionplanningtoolkit/mesh_models/agent_models/simple_blimp2.dxf");

	fprintf(stderr, "\n---------BLIMP MESH TESTING---------\n");

	runCollisionTiming(blimpEnv, blimpMesh, checks, frequency);

	fprintf(stderr, "\n---------CAPSULE TESTING---------\n");

	CapsuleHandler capsule(0.25, 0.5);

	runCollisionTiming(blimpEnv, capsule, checks, frequency);

	fprintf(stderr, "\n---------BOX TESTING---------\n");

	BoxHandler box(0.25, 0.25, 0.5);

	runCollisionTiming(blimpEnv, box, checks, frequency);
}

struct KDNode {
	static KDNode *newRandomPoint(unsigned int dimensionality) {
		KDNode *n = new KDNode(dimensionality);
		for(unsigned int i = 0; i < dimensionality; i++) {
			n->vars[i] = uniformDistribution(GlobalRandomGenerator);
		}
		return n;
	}

	const std::vector<double> &getTreeStateVars() const {
		return vars;
	}

	void setPointIndex(unsigned int index) {
		pointIndex = index;
	}

private:
	KDNode(unsigned int dimensionality) : vars(dimensionality) {}

	std::vector<double> vars;
	int pointIndex;

	static std::uniform_real_distribution<double> uniformDistribution;
};

std::uniform_real_distribution<double> KDNode::uniformDistribution;

void runKDExactTreeTesting(unsigned int insertions, unsigned int inserstionPollingFrequency, unsigned int queries, unsigned int dimensionality) {
}

void runKDTreeInsertionTesting(unsigned int insertions, unsigned int queries, unsigned int frequency, unsigned int dimensionality, unsigned int numTrees, unsigned int traversals) {
	typedef flann::KDTreeIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, flann::L2<double>, KDNode> KDTree;

	GlobalRandomGenerator.seed(2);

	KDTreeType kdtreeType(numTrees);
	KDTree kdtree(kdtreeType, dimensionality);

	std::vector<KDNode *> nodes(insertions);
	for(unsigned int i = 0; i < insertions; i++) {
		nodes[i] = KDNode::newRandomPoint(dimensionality);
	}

	std::vector<KDNode *> queryNodes(queries);
	for(unsigned int i = 0; i < queries; i++) {
		queryNodes[i] = KDNode::newRandomPoint(dimensionality);
	}

	double startTime = walltime();

	double intervalStart = walltime(), intervalEnd;
	for(unsigned int i = 0; i < insertions; i++) {
		if(i != 0 && i % frequency == 0) {
			intervalEnd = walltime();
			double duration = intervalEnd - intervalStart;
			fprintf(stderr, "interval duration for %u insertions: %g\n", frequency, duration);
			fprintf(stderr, "interval rate: %g insertions/sec\n", (double)frequency / duration);
			intervalStart = intervalEnd;
		}
		kdtree.insertPoint(nodes[i]);
	}

	double endTime = walltime();

	double duration = endTime - startTime;
	fprintf(stderr, "total time for %u insertions: %g\n", insertions, duration);
	fprintf(stderr, "total rate: %g insertions/sec\n", (double)insertions / duration);

	startTime = walltime();
	for(unsigned int i = 0; i < queries; i++) {
		if(i != 0 && i % frequency == 0) {
			intervalEnd = walltime();
			double duration = intervalEnd - intervalStart;
			fprintf(stderr, "interval duration for %u queries: %g\n", frequency, duration);
			fprintf(stderr, "interval rate: %g queries/sec\n", (double)frequency / duration);
			intervalStart = intervalEnd;
		}
		kdtree.nearest(queryNodes[i], 0, traversals);
	}
	endTime = walltime();

	duration = endTime - startTime;
	fprintf(stderr, "total time for %u queries: %g\n", queries, duration);
	fprintf(stderr, "total rate: %g queries/sec\n", (double)queries / duration);

	for(unsigned int i = 0; i < insertions; i++) {
		delete nodes[i];
	}
	for(unsigned int i = 0; i < queries; i++) {
		delete queryNodes[i];
	}
}

struct TestData {
	TestData(const std::string &identifier) : identifier(identifier) {}

	void addData(unsigned int treeSize, double rate) {
		data.emplace_back(treeSize, rate);
	}

	void printGoFmtRow() const {
		fprintf(stderr, "\"%s\", plotter.XYs{", identifier.c_str());
		for(unsigned int j = 0; j < data.size(); j++) {
			const auto &pt = data[j];
			if(j < data.size() - 1) {
				fprintf(stderr, "{%u, %g}, ", pt.first, pt.second);
			} else {
				fprintf(stderr, "{%u, %g}", pt.first, pt.second);
			}
		}
		fprintf(stderr, "},\n");
	}

	std::string identifier;
	std::vector<std::pair<unsigned int, double>> data;
};

std::vector<TestData> testData;

template<class KDTree>
void runKDTreeQueryTesting(unsigned int insertions, unsigned int queriesPerRound, unsigned int frequency, unsigned int dimensionality, KDTree &kdtree, unsigned int traversals = 1) {
	GlobalRandomGenerator.seed(2);

	std::vector<KDNode *> nodes(insertions);
	for(unsigned int i = 0; i < insertions; i++) {
		nodes[i] = KDNode::newRandomPoint(dimensionality);
	}

	unsigned int totalQueries = (insertions / frequency) * queriesPerRound;
	std::vector<KDNode *> queryNodes(totalQueries);
	for(unsigned int i = 0; i < totalQueries; i++) {
		queryNodes[i] = KDNode::newRandomPoint(dimensionality);
	}

	unsigned int round = 0;
	for(unsigned int i = 0; i < insertions; i++) {
		if(i != 0 && i % frequency == 0) {
			double startTime = walltime();
			for(unsigned int j = 0; j < queriesPerRound; j++) {
				//traversals can be ignored by some kdtree configurations
				kdtree.nearest(queryNodes[round * queriesPerRound + j], 0, traversals);
			}
			double endTime = walltime();
			double duration = endTime - startTime;

			testData.back().addData(i, (double)queriesPerRound / duration);
			round++;
		}
		kdtree.insertPoint(nodes[i]);
	}

	for(unsigned int i = 0; i < insertions; i++) {
		delete nodes[i];
	}
	for(unsigned int i = 0; i < totalQueries; i++) {
		delete queryNodes[i];
	}
}

void runMultiKDTreeQueryTesting(unsigned int insertions, unsigned int queriesPerRound, unsigned int frequency, unsigned int dimensionality, unsigned int numTrees, unsigned int traversals) {
	typedef flann::KDTreeIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, flann::L2<double>, KDNode> KDTree;

	KDTreeType kdtreeType(numTrees);
	KDTree kdtree(kdtreeType, dimensionality);

	char id[256];
	sprintf(id, "Trees=%u Traversals=%u", numTrees, traversals);

	testData.emplace_back(std::string(id));

	runKDTreeQueryTesting<KDTree>(insertions, queriesPerRound, frequency, dimensionality, kdtree, traversals);
}

void runAutTunedKDTreeQueryTesting(unsigned int insertions, unsigned int queriesPerRound, unsigned int frequency, unsigned int dimensionality,
                                   double precision, double buildWeight, double sampleFraction) {

	typedef flann::KDTreeIndexParams InitialKDTreeType;
	typedef FLANN_KDTreeWrapper<InitialKDTreeType, flann::L2<double>, KDNode> InitialKDTree;

	typedef flann::AutotunedIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, flann::L2<double>, KDNode> KDTree;

	InitialKDTreeType init_kdtreeType(1);
	InitialKDTree init_kdtree(init_kdtreeType, dimensionality);

	KDTreeType kdtreeType(precision, buildWeight, 0, 0);
	KDTree kdtree(kdtreeType, dimensionality);

	char id[256];
	sprintf(id, "Prec=%g BldWt=%g SampFrac=%g", precision, buildWeight, sampleFraction);

	testData.emplace_back(std::string(id));

	GlobalRandomGenerator.seed(2);

	std::vector<KDNode *> nodes(insertions);
	for(unsigned int i = 0; i < insertions; i++) {
		nodes[i] = KDNode::newRandomPoint(dimensionality);
	}

	unsigned int totalQueries = (insertions / frequency) * queriesPerRound;
	std::vector<KDNode *> queryNodes(totalQueries);
	for(unsigned int i = 0; i < totalQueries; i++) {
		queryNodes[i] = KDNode::newRandomPoint(dimensionality);
	}

	unsigned int round = 0;
	double rebuilt = false;
	for(unsigned int i = 0; i < insertions; i++) {
		if(i != 0 && i % frequency == 0) {
			if(rebuilt == false) {
				const auto bootstrappedElems = init_kdtree.getElements();

				for(const auto elem : bootstrappedElems) {
					kdtree.insertPoint(elem);
				}

				kdtree.buildIndex();

				rebuilt = true;
			}

			double startTime = walltime();
			for(unsigned int j = 0; j < queriesPerRound; j++) {
				// kdtree.nearest(queryNodes[round * queriesPerRound + j], 0);
				kdtree.kdtree.knnSearch();
			}
			double endTime = walltime();
			double duration = endTime - startTime;

			testData.back().addData(i, (double)queriesPerRound / duration);
			round++;
		}

		if(!rebuilt) {
			init_kdtree.insertPoint(nodes[i]);
		} else {
			kdtree.insertPoint(nodes[i]);
		}
	}

	for(unsigned int i = 0; i < insertions; i++) {
		delete nodes[i];
	}
	for(unsigned int i = 0; i < totalQueries; i++) {
		delete queryNodes[i];
	}
}

void runKDTreeTests(unsigned int insertions, unsigned int queries, unsigned int frequency,
                    unsigned int minDimensions, unsigned int maxDimensions,
                    unsigned int minTrees, unsigned int maxTrees,
                    unsigned int minTraversals, unsigned int maxTraversals) {

	maxDimensions++;
	maxTrees++;
	maxTraversals++;

	for(unsigned int i = minDimensions; i < maxDimensions; i*=2) {
		// runMultiKDTreeQueryTesting(insertions, queries, frequency, i, 1, flann::FLANN_CHECKS_UNLIMITED);
		// testData.back().printGoFmtRow();
		// for(unsigned int j = minTrees; j < maxTrees; j*=2) {
		// 	for(unsigned int k = minTraversals; k < maxTraversals; k*=2) {
		// 		runMultiKDTreeQueryTesting(insertions, queries, frequency, i, j, k);
		// 		testData.back().printGoFmtRow();
		// 	}
		// }

		// for(unsigned int j = 0; j < 10; j++) {
		// 	for(unsigned int k = 0; k < 10; k++) {
		// 		for(unsigned int l = 0; l < 10; l++) {
		// 			runAutTunedKDTreeQueryTesting(insertions, queries, frequency, i, (double)j/10., (double)k/10., (double)l/10.);
		// 			testData.back().printGoFmtRow();
		// 		}
		// 	}
		// }
	}

	runMultiKDTreeQueryTesting(insertions, queries, frequency, maxDimensions, 1, flann::FLANN_CHECKS_UNLIMITED);
	testData.back().printGoFmtRow();

	runAutTunedKDTreeQueryTesting(insertions, queries, frequency, maxDimensions, 0.9, 0.01, 1.0);
	testData.back().printGoFmtRow();
}

int main(int argc, char *argv[]) {
	// StaticEnvironmentMeshHandler env("/Users/skiesel/gopath/src/github.com/skiesel/motionplanningtoolkit/mesh_models/environment_models/Easy_env.dae", "0 0 0 1 0 0 0");
	// SimpleAgentMeshHandler agent("/Users/skiesel/gopath/src/github.com/skiesel/motionplanningtoolkit/mesh_models/agent_models/Easy_robot.dae");



	// auto lambda = [&](){

	// 	poses.back()[0].setTranslation(fcl::Vec3f(0, 0, increment-=0.01));
	// 	// poses.back()[0].setRotation(fcl::Matrix3f(1,2,3,4,5,6,7,8,9));

	// 	env.draw();
	// 	agent.draw(OpenGLWrapper::Color(), transpose(toDoubleVec(poses.back()[0])));

	// 	if(collisionChecker.isInCollision(env, agentPieces, poses))
	// 		fprintf(stderr, "Collision\n");
	// 	else
	// 		fprintf(stderr, "no collision\n");

	// };
	// OpenGLWrapper::getOpenGLWrapper().runWithCallback(lambda, args);

	unsigned int checks = 1000000;
	unsigned int frequency = checks / 10;

	// runCollisionTimings(checks, frequency);

	// runKDTreeTests(1000000, 1000, 100000, 5, 6, 1, 4, 1, 4);
	runKDTreeTests(1000000, 1000, 100000, 7, 7, 1, 16, 1, 4);

	return 0;
}