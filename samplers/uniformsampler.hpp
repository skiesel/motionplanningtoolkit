#pragma once

#include <random>

template <class Workspace, class Agent>
class UniformSampler {
	typedef typename Agent::State State;
	typedef typename Agent::StateVars StateVars;
	typedef typename Agent::StateVarRanges StateVarRanges;

public:
	UniformSampler(const Workspace &workspace, const Agent &agent) : workspace(workspace), agent(agent) {
		stateVarDomains = agent.getStateVarRanges(workspace.getBounds());
		for(auto range : stateVarDomains) {
			distributions.emplace_back(range.first, range.second);
		}
	}

	State sampleConfiguration() const {
		StateVars vars;
		for(auto distribution : distributions) {
			double v = distribution(generator);
			vars.push_back(v);
		}
		return agent.buildState(vars);
	};
private:
	const Workspace &workspace;
	const Agent &agent;
	StateVarRanges stateVarDomains;

	std::vector< std::uniform_real_distribution<double> > distributions;
	mutable std::default_random_engine generator;
};