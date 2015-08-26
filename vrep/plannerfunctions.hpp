#pragma once

#include "vrepinterface.hpp"
#include "../utilities/instancefilemap.hpp"
#include "../planners/rrt.hpp"
#include "../planners/rrtconnect.hpp"
#include "../planners/kpiece.hpp"
#include "../samplers/uniformsampler.hpp"
#include "../utilities/flannkdtreewrapper.hpp"
#include "../tree_interfaces/treeinterface.hpp"
#include "../tree_interfaces/plakutreeinterface.hpp"
#include "../discretizations/workspace/prmlite.hpp"
#include "../utilities/datafile.hpp"

void solveWithRRT(const VREPInterface *interface, const InstanceFileMap *args, const VREPInterface::State &start, const VREPInterface::State &goal) {
	dfpair(stdout, "planner", "%s", "RRT");

	typedef flann::KDTreeSingleIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, flann::L2<double>, VREPInterface::Edge> KDTree;
	typedef UniformSampler<VREPInterface, VREPInterface, KDTree> UniformSampler;
	typedef TreeInterface<VREPInterface, KDTree, UniformSampler> TreeInterface;
	typedef RRT<VREPInterface, VREPInterface, TreeInterface> RRT;

	KDTreeType kdtreeType;
	KDTree kdtree(kdtreeType, interface->getTreeStateSize());
	UniformSampler uniformSampler(*interface, *interface, kdtree);
	TreeInterface treeInterface(kdtree, uniformSampler);

	RRT rrt(*interface, *interface, treeInterface, *args);
	rrt.query(start, goal);
	rrt.dfpairs();
}

void solveWithRRTConnect(const VREPInterface *interface, const InstanceFileMap *args, const VREPInterface::State &start, const VREPInterface::State &goal) {
	dfpair(stdout, "planner", "%s", "RRT Connect");

	typedef flann::KDTreeSingleIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, flann::L2<double>, VREPInterface::Edge> KDTree;
	typedef UniformSampler<VREPInterface, VREPInterface, KDTree> UniformSampler;
	typedef TreeInterface<VREPInterface, KDTree, UniformSampler> TreeInterface;
	typedef RRTConnect<VREPInterface, VREPInterface, TreeInterface> RRTConnect;

	KDTreeType kdtreeType;
	KDTree kdtree(kdtreeType, interface->getTreeStateSize());
	UniformSampler uniformSampler(*interface, *interface, kdtree);
	TreeInterface treeInterface(kdtree, uniformSampler);

	RRTConnect rrtconnect(*interface, *interface, treeInterface, *args);
	rrtconnect.query(start, goal);
	rrtconnect.dfpairs();
}

void solveWithPlaku(const VREPInterface *interface, const InstanceFileMap *args, const VREPInterface::State &start, const VREPInterface::State &goal) {
	dfpair(stdout, "planner", "%s", "Plaku IROS 2014");

	typedef PRMLite<VREPInterface, VREPInterface> PRMLite;
	typedef PlakuTreeInterface<VREPInterface, VREPInterface, PRMLite> PlakuTreeInterface;
	typedef RRT<VREPInterface, VREPInterface, PlakuTreeInterface> Plaku;

	unsigned int numberOfPRMVertices = stol(args->value("Number Of PRM Vertices"));
	unsigned int numberOfNearestNeighborEdgeConsiderations = stol(args->value("Nearest Neighbors To Consider In PRM Edge Construction"));
	double prmCollisionCheckDT = stod(args->value("PRM Collision Check DT"));

	PRMLite prmLite(*interface, *interface, start, numberOfPRMVertices, numberOfNearestNeighborEdgeConsiderations, prmCollisionCheckDT);

	double alpha = stod(args->value("Plaku Alpha Value"));
	double b = stod(args->value("Plaku b Value"));
	double stateRadius = stod(args->value("Plaku PRM State Selection Radius"));

	PlakuTreeInterface plakuTreeInterface(*interface, *interface, prmLite, start, goal, alpha, b, stateRadius);

	// plakuTreeInterface.draw();

	Plaku plaku(*interface, *interface, plakuTreeInterface, *args);
	plaku.query(start, goal);
}

void solveWithKPIECE(const VREPInterface *interface, const InstanceFileMap *args, const VREPInterface::State &start, const VREPInterface::State &goal) {
	dfpair(stdout, "planner", "%s", "KPIECE");

	KPIECE<VREPInterface, VREPInterface> kpiece(*interface, *interface, *args);
	kpiece.query(start, goal);
	kpiece.dfpairs();
}