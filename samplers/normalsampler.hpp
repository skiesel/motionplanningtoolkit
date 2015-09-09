#pragma once

#include <random>

template <class Workspace, class Agent, class NN>
class NormalSampler {
	typedef typename Agent::State State;
	typedef typename Agent::Edge Edge;
	typedef typename Agent::StateVars StateVars;
	typedef typename Agent::StateVarRanges StateVarRanges;

public:
	struct Normal {
		Normal(double mean, double stdev) : mean(mean), stdev(stdev) {}
		double mean, stdev;
	};

	NormalSampler(const Workspace &workspace, const Agent &agent, NN &nn, const std::vector<Normal> &normals) : workspace(workspace), agent(agent), nn(nn) {
		for(auto normal : normals) {
			distributions.emplace_back(normal.mean, normal.stdev);
		}
	}

	std::pair<Edge*, State> getTreeSample() const {
		State sample = sampleConfiguration();
		Edge sampleEdge = Edge(sample);
		typename NN::KNNResult result = nn.nearest(&sampleEdge, 0, 1);
		return std::make_pair(result.elements[0], sample);
	}

private:
	State sampleConfiguration() const {
		StateVars vars;
		for(auto distribution : distributions) {
			vars.push_back(distribution(GlobalRandomGenerator));
		}
		return agent.buildState(vars);
	}

	const Workspace &workspace;
	const Agent &agent;
	NN &nn;
	StateVarRanges stateVarDomains;


	std::vector< std::normal_distribution<double> > distributions;
};