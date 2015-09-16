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

		double g = 0;

		for(unsigned int i = 0; i < path.size(); ++i) {
			for(unsigned int lastConsumedEdge = i + 1; lastConsumedEdge < path.size(); ++lastConsumedEdge) {
				Edge edge = agent.constructEdge(path[i]->start, path[lastConsumedEdge]->end);

				//We are going to ignore cost and assume less edges are going to be cheaper
				if(!workspace.safeEdge(agent, edge, collisionCheckDT)) {
					if(lastConsumedEdge == i) {
						//didn't combine any edges
						Edge *e = new Edge(*path[i]);
						g += e->cost;
						e->g = g;
						newPath.push_back(e);
					} else {
						lastConsumedEdge--;
						Edge *e = new Edge(agent.constructEdge(path[i]->start, path[lastConsumedEdge]->end));
						g += e->cost;
						e->g = g;
						newPath.push_back(e);
						i = lastConsumedEdge;
					}
					break;
				}
			}
		}

		return path;
	}

private:
	const Workspace &workspace;
	const Agent &agent;
	double collisionCheckDT;
};