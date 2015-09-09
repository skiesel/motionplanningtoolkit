#pragma once

template<class Agent, class BackupSampler>
class GoalBiasSampler {
public:
	typedef typename Agent::State State;
	typedef typename Agent::Edge Edge;

	GoalBiasSampler(BackupSampler &backupSampler, const State &goal, double goalBias) : backupSampler(backupSampler), goal(goal), goalBias(goalBias) {}

	std::pair<Edge*, State> getTreeSample() const {
		if(distribution(GlobalRandomGenerator) < goalBias) {
			return std::make_pair(backupSampler.getTreeEdge(goal), goal);
		} else {
			return backupSampler.getTreeSample();
		}
	}

	Edge *getTreeEdge(const State &s) const {
		return backupSampler.getTreeEdge(s);
	}

private:
	double goalBias;
	const State &goal;
	BackupSampler &backupSampler;
	mutable std::uniform_real_distribution<double> distribution;
};