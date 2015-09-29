#pragma once

template<class Workspace, class Agent>
class NoOpPostProcessor {
public:
	typedef typename Agent::Edge Edge;

	NoOpPostProcessor() {}

	std::vector<const Edge*> postProcess(const std::vector<const Edge*> &path) const {
		return path;
	}
};