#pragma once

class AbstractTransformState {
public:
	AbstractTransformState() {}
	AbstractTransformState(const AbstractTransformState &s) {}

	const std::vector<double>& getTreeStateVars() const {
		return treeStateVars;
	}

	static AbstractTransformState getRandomAbstractState(const std::vector< std::pair<double, double> > &bounds) {
		AbstractTransformState newState;
		for(const auto &bound : bounds) {
			//It really feels like I'd want this to be created *once* and reused, but I am not sure
			//we can assume that the bounds will always be the same, so it should handle this generically
			std::uniform_real_distribution<double> dist(bound.first, bound.second);
			newState.treeStateVars.emplace_back(dist(GlobalRandomGenerator));
		}
		return newState;
	}

	static double evaluateDistance(const AbstractTransformState &a, const AbstractTransformState &b) {
		assert(a.treeStateVars.size() == b.treeStateVars.size());
		double sum = 0;
		for(unsigned int i = 0; i < a.treeStateVars.size(); ++i) {
			double diff = a.treeStateVars[i] - b.treeStateVars[i];
			sum += diff * diff;
		}
		return sqrt(sum);
	}

	fcl::Transform3f transform;
	std::vector<double> treeStateVars;
};