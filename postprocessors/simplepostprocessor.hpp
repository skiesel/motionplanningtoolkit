#pragma once

template<class Workspace, class Agent>
class SimplePostProcessor {
public:
	typedef typename Agent::Edge Edge;

	SimplePostProcessor(const Workspace &workspace, const Agent &agent, const InstanceFileMap &args) :
		workspace(workspace), agent(agent) {
		collisionCheckDT = args.doubleVal("Collision Check Delta t");
	}

	std::vector<const Edge*> postProcess(const std::vector<const Edge*> &path) const {
		std::vector<Edge*> newPath;

		for(unsigned int i = 0; i < path.size(); ++i) {
			for(unsigned int j = i + 1; j < path.size(); ++j) {
				Edge edge = agent.constructEdge(newPath.size() == 0 ? path.front()->start : newPath.back()->end, path[j]->end);

				//We are going to ignore cost and assume less edges are going to be cheaper
				if(!workspace.safeEdge(agent, edge, collisionCheckDT)) {

					if(j == i+1) {
						//didn't combine any edges
						Edge *e = new Edge(*path[i]);
						if(newPath.size() > 0) {
							e->updateParent(newPath.back());
						} else {
							e->g = e->cost;
						}

						newPath.push_back(e);
					} else {
						Edge *e = new Edge(agent.constructEdge(newPath.size() == 0 ? path.front()->start : newPath.back()->end, path[j-1]->end));
						if(newPath.size() > 0) {
							e->updateParent(newPath.back());
						} else {
							e->g = e->cost;
						}

						newPath.push_back(e);
						i = j - 2;
					}
					break;
				} else {
					if(j == (path.size() - 1)) {
						Edge *e = new Edge(agent.constructEdge(newPath.back()->end, path.back()->end));
						if(newPath.size() > 0) {
							e->updateParent(newPath.back());
						} else {
							e->g = e->cost;
						}

						newPath.push_back(e);
						i = j;
						break;
					}
				}
			}
		}

		std::vector<const Edge*> constNewPath;
		constNewPath.insert(constNewPath.begin(), newPath.begin(), newPath.end());

		assert(newPath.back()->end.equals(path.back()->end));

		return constNewPath;
	}

private:
	const Workspace &workspace;
	const Agent &agent;
	double collisionCheckDT;
};