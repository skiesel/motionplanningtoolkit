#pragma once

#include <random>

template <class Workspace, class Agent, class NN>
class NormalSampler {
	typedef typename Agent::State State;
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

	State getTreeSample() const {
		auto sample = sampleConfiguration();
		auto sampleEdge = Edge(sample);
		typename NN::KNNResult result = nn.nearest(&sampleEdge);
		return result.elements[0]->end;
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