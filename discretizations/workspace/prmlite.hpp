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

	struct Vertex {
		Vertex(const fcl::Transform3f &transform, unsigned int id) : transform(transform), id(id), treeStateVars(7) {
			const fcl::Vec3f &translation = transform.getTranslation();
			const fcl::Quaternion3f &quaternion = transform.getQuatRotation();
			for(unsigned int i = 0; i < 3; ++i)
				treeStateVars[i] = translation[i];
			treeStateVars[3] = quaternion.getX();
			treeStateVars[4] = quaternion.getY();
			treeStateVars[5] = quaternion.getZ();
			treeStateVars[6] = quaternion.getW();
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
	typedef FLANN_KDTreeWrapper<KDTreeType, flann::L2<double>, Vertex> KDTree;

public:
	PRMLite(const Workspace &workspace, const Agent &agent, const State &canonicalState, unsigned int numVertices,
		unsigned int edgeSetSize, double collisionCheckDT = 0.1) :
		agent(agent), kdtree(KDTreeType(2), 7), canonicalState(canonicalState) {

		dfpair(stdout, "prm vertex set size", "%lu", numVertices);
		dfpair(stdout, "prm edge set size", "%lu", edgeSetSize);
		dfpair(stdout, "prm edge collision check dt", "%g", collisionCheckDT);

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

	double getEdgeCostBetweenCells(unsigned int c1, unsigned int c2) const {
		auto vertexAndEdges = edges.find(c1);
		assert(vertexAndEdges != edges.end());

		const auto &edges = vertexAndEdges->second;

		auto edge = edges.find(c2);
		assert(edge != edges.end());

		return edge->second.weight;
	}

	unsigned int getCellId(const typename Agent::State &state) const {
		Vertex v(state.getTransform(), 0);
		auto res = kdtree.nearest(&v);

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
		fcl::Quaternion3f rot = getRandomQuaternion();

		fcl::Transform3f transform(rot, point);
		transform = vertices[index]->transform * transform; 

		return agent.transformToState(canonicalState, transform, radius);
	}

	void draw() const {
		auto verticesHandle = simAddDrawingObject(sim_drawing_points, 5, 0.0, -1, vertices.size(), NULL, NULL, NULL, NULL);
		simFloat coords[6];
		for(const auto vert : vertices) {
			const auto& trans = vert->transform.getTranslation();
			for(unsigned int i = 0; i < 3; ++i)
				coords[i] = trans[i];
			simAddDrawingObjectItem(verticesHandle, coords);
		}

		std::vector< std::vector<double> > edgesForVrep;
		for(const auto& edgeSet : edges) {
			std::vector<double> edgeForVrep(6);
			const auto startVertex = vertices[edgeSet.first];
			
			const auto& trans = startVertex->transform.getTranslation();
			for(unsigned int i = 0; i < 3; ++i)
				edgeForVrep[i] = trans[i];

			for(const auto& edge : edgeSet.second) {
				const auto endVertex = vertices[edge.second.endpoint];
				
				const auto& trans2 = endVertex->transform.getTranslation();
				for(unsigned int i = 3; i < 6; ++i)
					edgeForVrep[i] = trans2[i];

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
			fcl::Quaternion3f rotation = getRandomQuaternion();
			fcl::Transform3f transform(rotation, translation);

			if(workspace.safePose(agent, transform)) {
				auto newVert = new Vertex(transform, vertices.size());
				vertices.push_back(newVert);
				kdtree.insertPoint(newVert);
			}
		}
	}

	void generateEdges(const Workspace &workspace, const Agent &agent, double collisionCheckDT, double edgeSetSize) {
		for(unsigned int i = 0; i < vertices.size(); ++i) {
			auto res = kdtree.kNearest(vertices[i], edgeSetSize, 0, 1);

			for(const auto endVertex : res.elements) {

				if(edgeExists(i, endVertex->id)) {
					continue;
				}
				
				std::vector<fcl::Transform3f> edgeCandidate = interpolate(vertices[i]->transform, endVertex->transform, collisionCheckDT);

				if(edgeCandidate.size() == 0 || workspace.safePoses(agent, edgeCandidate)) {
					double cost = evaluateTransformDistance(vertices[i]->transform, endVertex->transform);

					edges[i][endVertex->id] = Edge(endVertex->id, cost);
					edges[endVertex->id][i] = Edge(i, cost); //the reverse interpolation would be symmetric
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

	fcl::Quaternion3f getRandomQuaternion() const {
		double u1 = zeroToOne(generator);
		double u2 = zeroToOne(generator);
		double u3 = zeroToOne(generator);

		return fcl::Quaternion3f(sqrt(1-u1)*sin(2*M_PI*u2),
								 sqrt(1-u1)*cos(2*M_PI*u2),
								 sqrt(u1)*sin(2*M_PI*u3),
								 sqrt(u1)*cos(2*M_PI*u3));
	}

	const Agent &agent;
	const State &canonicalState;
	std::vector<Vertex*> vertices;
	std::unordered_map<unsigned int, std::unordered_map<unsigned int, Edge>> edges;
	mutable KDTree kdtree;
	mutable std::default_random_engine generator;
	mutable std::uniform_real_distribution<double> zeroToOne;
};