#pragma once

#include <random>
#include <unordered_map>
#include <fcl/math/transform.h>

#include "v_repLib.h"

#include "../utilities/flannkdtreewrapper.hpp"
#include "../utilities/datafile.hpp"

template <class Workspace, class Agent>
class PRMLite {
	typedef typename Agent::State State;

	struct VertexZRotationOnly {
		VertexZRotationOnly(const fcl::Transform3f &transform, unsigned int id) : transform(transform), id(id), treeStateVars(4) {
			const fcl::Vec3f &translation = transform.getTranslation();
			const fcl::Quaternion3f &quaternion = transform.getQuatRotation();
			for(unsigned int i = 0; i < 3; ++i)
				treeStateVars[i] = translation[i];

			fcl::Vec3f axis;
			double yaw;
			quaternion.toAxisAngle(axis, yaw);
			treeStateVars[3] = yaw;

			// treeStateVars[3] = quaternion.getX();
			// treeStateVars[4] = quaternion.getY();
			// treeStateVars[5] = quaternion.getZ();
			// treeStateVars[6] = quaternion.getW();
		}

		const std::vector<double>& getTreeStateVars() const {
			return treeStateVars;
		}

		void setPointIndex(unsigned int index) {
			// we aren't going to look things up in the traditional way so we don't need this stored
		}

		fcl::Transform3f transform;
		unsigned int id;
		std::vector<double> treeStateVars;
	};

	struct Edge {
		Edge() {}

		Edge(unsigned int endpoint, double weight) : endpoint(endpoint), weight(weight) {

		}

		std::size_t operator()(const Edge &e) const {
			return e.endpoint;
		}

		bool operator==(const Edge &e) const {
			return e.endpoint == endpoint;
		}

		unsigned int endpoint;
		double weight;
	};

	typedef flann::KDTreeIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, flann::L2<double>, VertexZRotationOnly> KDTree;

public:
	PRMLite(const Workspace &workspace, const Agent &agent, const State &canonicalState, unsigned int numVertices,
		unsigned int edgeSetSize, double collisionCheckDT = 0.1) :
		agent(agent), kdtree(KDTreeType(2), 4), canonicalState(canonicalState) {

		dfpair(stdout, "prm vertex set size", "%lu", numVertices);
		dfpair(stdout, "prm edge set size", "%lu", edgeSetSize);
		dfpair(stdout, "prm edge collision check dt", "%g", collisionCheckDT);
		dfpair(stdout, "prm random quaternion z only", "%s", "true");

		clock_t start = clock();
		clock_t vertexStart = clock();
		
		generateVertices(workspace, agent, numVertices);
		
		clock_t end = clock();
		double time = (double) (end-vertexStart) / CLOCKS_PER_SEC;
		dfpair(stdout, "prm vertex build time", "%g", time);
		clock_t edgeStart = clock();

		generateEdges(workspace, agent, collisionCheckDT, edgeSetSize);
		
		end = clock();
		time = (double) (end-edgeStart) / CLOCKS_PER_SEC;
		dfpair(stdout, "prm edge build time", "%g", time);

		time = (double) (end-start) / CLOCKS_PER_SEC;
		dfpair(stdout, "prm build time", "%g", time);
	}

	void addMoreVertices(unsigned int howManyToAdd) {
		fatal("PRM addMoreVertices not implemented");
	}

	unsigned int getCellCount() const {
		return vertices.size();
	}

	const fcl::Transform3f& getTransform(unsigned int i) const {
		return vertices[i]->transform;
	}

	double getEdgeCostBetweenCells(unsigned int c1, unsigned int c2) const {
		auto vertexAndEdges = edges.find(c1);
		assert(vertexAndEdges != edges.end());

		const auto &edges = vertexAndEdges->second;

		auto edge = edges.find(c2);
		assert(edge != edges.end());

		return edge->second.weight;
	}

	unsigned int getCellId(const typename Agent::State &state) const {
		VertexZRotationOnly v(state.getTransform(), 0);
		auto res = kdtree.nearest(&v, 1, 1);

		assert(res.elements.size() > 0);
		
		return res.elements[0]->id;
	}

	std::vector<unsigned int> getNeighboringCells(unsigned int index) const {
		auto vertexAndEdges = edges.find(index);
		assert(vertexAndEdges != edges.end());

		const auto &edges = vertexAndEdges->second;

		std::vector<unsigned int> ids;
		ids.reserve(edges.size());
		for(const auto &edge : edges) {
			ids.push_back(edge.second.endpoint);
		}
		return ids;
	}

	typename Agent::State getRandomStateNearRegionCenter(unsigned int index, double radius) const {
		fcl::Vec3f point = randomPointInSphere(radius);
		fcl::Quaternion3f rot = getRandomZOnlyQuaternion();
		// fcl::Quaternion3f rot = getRandomQuaternion();

		fcl::Transform3f transform(rot, point);
		transform = vertices[index]->transform * transform; 

		return agent.transformToState(canonicalState, transform, radius);
	}

	void draw(bool drawPoints=true, bool drawLines=false, std::vector<std::vector<double>> colors = std::vector<std::vector<double>>()) const {
		simFloat coords[12];
		for(unsigned int i = 0; i < 6; ++i)
			coords[i] = 0;

		if(drawPoints) {
			auto verticesHandle = simAddDrawingObject(sim_drawing_spherepoints | sim_drawing_itemcolors, 0.05, 0.0, -1, vertices.size(), NULL, NULL, NULL, NULL);
			unsigned int curIndex = 0;
			for(const auto vert : vertices) {
				const auto& trans = vert->transform.getTranslation();

				
				for(unsigned int i = 0; i < 3; ++i) {
					coords[i] = trans[i];
				}

				if(colors.size() > 0) {
					for(unsigned int i = 0; i < 3; ++i) {
						coords[3+i] = colors[curIndex][i];
					}
				}
				simAddDrawingObjectItem(verticesHandle, coords);
				curIndex++;
			}
		}

		if(drawLines) {
			std::vector< std::vector<double> > edgesForVrep;
			for(const auto& edgeSet : edges) {
				std::vector<double> edgeForVrep(6);
				const auto startVertexZRotationOnly = vertices[edgeSet.first];
				
				const auto& trans = startVertexZRotationOnly->transform.getTranslation();
				for(unsigned int i = 0; i < 3; ++i) {
					edgeForVrep[i] = trans[i];
				}

				for(const auto& edge : edgeSet.second) {
					const auto endVertexZRotationOnly = vertices[edge.second.endpoint];
					
					const auto& trans2 = endVertexZRotationOnly->transform.getTranslation();
					for(unsigned int i = 0; i < 3; ++i) {
						edgeForVrep[3+i] = trans2[i];
					}

					edgesForVrep.push_back(edgeForVrep);
				}
			}

			auto edgesHandle = simAddDrawingObject(sim_drawing_lines, 1, 0.0, -1, edgesForVrep.size(), NULL, NULL, NULL, NULL);

			for(const auto &edge : edgesForVrep) {
				for(unsigned int i = 0; i < 6; ++i)
					coords[i] = edge[i];
				simAddDrawingObjectItem(edgesHandle, coords);
			}
		}
	}

	bool edgeExists(unsigned int c1, unsigned int c2) const {
		auto vertexAndEdges = edges.find(c1);
		if(vertexAndEdges == edges.end()) {
			return false;
		}

		const auto &edges = vertexAndEdges->second;

		auto edge = edges.find(c2);
		return edge != edges.end();
	}

private:
	fcl::Vec3f randomPointInSphere(double maxRadius = 1) const {
		double radius = maxRadius * pow(zeroToOne(generator), 1/3);
		double theta = 2 * M_PI * zeroToOne(generator);
		double phi = acos(2 * zeroToOne(generator) - 1);
		double u = cos(phi);

		double t1 = sqrt(1 - u * u);

		return fcl::Vec3f(t1 * cos(theta) * radius, t1 * sin(theta) * radius, u * radius);
	}

	void generateVertices(const Workspace &workspace, const Agent &agent, unsigned int numVertices) {
		vertices.reserve(numVertices);

		auto bounds = workspace.getBounds();

		std::vector< std::uniform_real_distribution<double> > linearDistributions;

		for(auto range : bounds) {
			linearDistributions.emplace_back(range.first, range.second);
		}

		while(vertices.size() < numVertices) {
			fcl::Vec3f translation = getRandomVector(linearDistributions, generator);

			fcl::Quaternion3f rotation = getRandomZOnlyQuaternion();
			// fcl::Quaternion3f rotation = getRandomQuaternion();
			fcl::Transform3f transform(rotation, translation);

			if(workspace.safePose(agent, transform, canonicalState)) {
				auto newVert = new VertexZRotationOnly(transform, vertices.size());
				vertices.push_back(newVert);
				kdtree.insertPoint(newVert);
			} else {
				fprintf(stderr, "vertex collision!\n");
			}
		}
	}

	void generateEdges(const Workspace &workspace, const Agent &agent, double collisionCheckDT, double edgeSetSize) {
		for(unsigned int i = 0; i < vertices.size(); ++i) {
			auto res = kdtree.kNearest(vertices[i], edgeSetSize+1, 0, 1);

			for(const auto endVertexZRotationOnly : res.elements) {

				if(edgeExists(i, endVertexZRotationOnly->id)) {
					continue;
				}
				
				std::vector<fcl::Transform3f> edgeCandidate = interpolate(vertices[i]->transform, endVertexZRotationOnly->transform, collisionCheckDT);

				bool safe = workspace.safePoses(agent, edgeCandidate, canonicalState);

				if(edgeCandidate.size() == 0 || safe) {
					double cost = evaluateTransformDistance(vertices[i]->transform, endVertexZRotationOnly->transform);

					edges[i][endVertexZRotationOnly->id] = Edge(endVertexZRotationOnly->id, cost);
					edges[endVertexZRotationOnly->id][i] = Edge(i, cost); //the reverse interpolation would be symmetric
				} if(!safe) {
					fprintf(stderr, "skipping edge, collision!\n");
				}
			}
		}
	}

	double evaluateTransformDistance(const fcl::Transform3f &t1, const fcl::Transform3f &t2) const {
		const fcl::Vec3f p1 = t1.getTranslation();
		const fcl::Vec3f p2 = t2.getTranslation();
		double dx = p1[0] - p2[0];
		double dy = p1[1] - p2[1];
		double dz = p1[2] - p2[2];

		return sqrt(dx*dx + dy*dy + dz*dz);
	}

	fcl::Vec3f getRandomVector(std::vector< std::uniform_real_distribution<double> > &distributions, std::default_random_engine &generator) const {
		fcl::Vec3f vector;
		for(unsigned int i = 0; i < distributions.size(); ++i) {
			vector[i] = distributions[i](generator);
		}
		return vector;
	}

	std::vector<fcl::Transform3f> interpolate(const fcl::Transform3f &t1, const fcl::Transform3f &t2, double linearStepSize) const {
		std::vector<fcl::Transform3f> interpolationPoints;

		const fcl::Vec3f &v1 = t1.getTranslation();
		const fcl::Vec3f &v2 = t2.getTranslation();

		unsigned int steps = vectorDistance(v1, v2) / linearStepSize;

		// if(steps == 0) {
		// 	fprintf(stderr, "\t%g %g %g  :: %g %g %g\n", v1[0], v1[1], v1[2], v2[0], v2[1], v2[2]);
		// }
		// else
		// 	fprintf(stderr, "%g\n", vectorDistance(v1, v2));

		fcl::Vec3f vecStep = (v2 - v1) / steps;

		const fcl::Quaternion3f &q1 = t1.getQuatRotation();
		const fcl::Quaternion3f &q2 = t2.getQuatRotation();


		// don't bother including the endpoints because this function is only used to check interpolation between points
		// already known to be "safe"

		fcl::Transform3f point(t1);
		for(unsigned int i = 0; i < steps; ++i) {
			point.setTransform(q1, point.getTranslation() + vecStep);
			interpolationPoints.emplace_back(point);
		}

		return interpolationPoints;
	}

	double vectorDistance(const fcl::Vec3f &v1, const fcl::Vec3f &v2) const {
		fcl::Vec3f diff = v1 - v2;
		return sqrt(diff.dot(diff));
	}

	fcl::Quaternion3f normalize(fcl::Quaternion3f &q) const {
		double x = q.getX();
		double y = q.getY();
		double z = q.getZ();
		double w = q.getW();
		double length = sqrt(x * x + y * y + z * z + w * w);

		return fcl::Quaternion3f(x/length, y/length, z/length, w/length);
	}

	// This came from: http://www.sonycsl.co.jp/person/nielsen/visualcomputing/programs/slerp.cpp
	fcl::Quaternion3f slerp(fcl::Quaternion3f q1, fcl::Quaternion3f q2, double lambda) {
		float dotproduct = q1.dot(q2);

		// algorithm adapted from Shoemake's paper
		double lamdaHalf = lambda / 2.0;

		double theta = acos(dotproduct);
		if(theta < 0.0) {
			theta = -theta;
		}

		double st = sin(theta);
		double sut = sin(lamdaHalf * theta);
		double sout = sin((1 - lamdaHalf) * theta);
		double coeff1 = sout/st;
		double coeff2 = sut/st;

		return normalize(q1 * coeff1 + q1 * coeff2);
	}

	fcl::Quaternion3f getRandomZOnlyQuaternion() const {
		double rad = zeroToOne(generator) * 2 * M_PI;
		fcl::Quaternion3f quaternion;
		fcl::Vec3f axis(0,0,1);
		quaternion.fromAxisAngle(axis, rad);
		return quaternion;
	}

	// fcl::Quaternion3f getRandomQuaternion() const {
	// 	double u1 = zeroToOne(generator);
	// 	double u2 = zeroToOne(generator);
	// 	double u3 = zeroToOne(generator);

	// 	return fcl::Quaternion3f(sqrt(1-u1)*sin(2*M_PI*u2),
	// 							 sqrt(1-u1)*cos(2*M_PI*u2),
	// 							 sqrt(u1)*sin(2*M_PI*u3),
	// 							 sqrt(u1)*cos(2*M_PI*u3));
	// }

	const Agent &agent;
	const State &canonicalState;
	std::vector<VertexZRotationOnly*> vertices;
	std::unordered_map<unsigned int, std::unordered_map<unsigned int, Edge>> edges;
	mutable KDTree kdtree;
	mutable std::default_random_engine generator;
	mutable std::uniform_real_distribution<double> zeroToOne;
};