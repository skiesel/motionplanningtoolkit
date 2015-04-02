#pragma once

#include <random>

template <class Workspace, class Agent>
class NormalSampler {
	typedef typename Agent::State State;
	typedef typename Agent::StateVars StateVars;
	typedef typename Agent::StateVarRanges StateVarRanges;

public:
	struct Normal {
		Normal(double mean, double stdev) : mean(mean), stdev(stdev) {}
		double mean, stdev;
	};

	NormalSampler(const Workspace &workspace, const Agent &agent, const std::vector<Normal> &normals) : workspace(workspace), agent(agent) {
		for(auto normal : normals) {
			distributions.emplace_back(normal.mean, normal.stdev);
		}
	}

	State sampleConfiguration() const {
		StateVars vars;
		for(auto distribution : distributions) {
			vars.push_back(distribution(generator));
		}
		return agent.buildState(vars);
	};
private:
	const Workspace &workspace;
	const Agent &agent;
	StateVarRanges stateVarDomains;

	
	std::vector< std::normal_distribution<double> > distributions;
	mutable std::default_random_engine generator;
};