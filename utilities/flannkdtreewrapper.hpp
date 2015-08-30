#pragma once

#include <flann/flann.hpp>

#include <unordered_map>

template<class KDTreeType, class DistanceMetric, class Element>
class FLANN_KDTreeWrapper {
	struct LookupElement {
		LookupElement() : data(NULL), point(NULL) {}
		LookupElement(Element *data, double *point) : data(data), point(point) {}
		LookupElement(const LookupElement &le) : data(le.data), point(le.point) {}
		Element *data;
		double *point;
	};

public:
	typedef flann::Index<DistanceMetric> KDTree;

	FLANN_KDTreeWrapper(const KDTreeType &type, unsigned int dim) : kdtree(type), currentPointIndex(1) {
		flann::Matrix<double> point(new double[dim], 1, dim);
		kdtree.buildIndex(point);
		kdtree.removePoint(0);
	}

	void insertPoint(Element *elem) {
		const std::vector<double> &stateVars = elem->getTreeStateVars();
		double *data = new double[stateVars.size()];
		for(unsigned int i = 0; i < stateVars.size(); i++)
			data[i] = stateVars[i];

		flann::Matrix<double> point(data, 1, stateVars.size());

		kdtree.addPoints(point, 2); //2 is the rebuild threshold

		elem->setPointIndex(currentPointIndex);
		lookup[currentPointIndex] = LookupElement(elem, data);
		++currentPointIndex;
	}

	void removePoint(Element *elem) {
		unsigned int index = elem->getPointIndex();
		if(index == 0) return;

		kdtree.removePoint(index - 1);

		elem->setPointIndex(0);
		lookup.erase(index);
	}

	struct KNNResult {
		std::vector<Element *> elements;
		std::vector<double> distances;
	};

	KNNResult nearest(const Element *elem, double epsilon=0.0, unsigned int traversals=flann::FLANN_CHECKS_UNLIMITED) const {
		return kNearest(elem, 1, epsilon, traversals);
	}

	KNNResult kNearest(const Element *elem, unsigned int k, double epsilon=0.0, unsigned int traversals=flann::FLANN_CHECKS_UNLIMITED) const {
		assert(k > 0);

		auto stateVars = elem->getTreeStateVars();
		flann::Matrix<double> point(stateVars.data(), 1, stateVars.size(), epsilon);

		std::vector< std::vector<int> > indices;
		std::vector< std::vector<double> > distances;

		flann::SearchParams params(traversals, epsilon, false);

		kdtree.knnSearch(point, indices, distances, k, params);

		KNNResult result;

		if(indices[0].size() <= 0) {
			fprintf(stderr, "%zu\n", kdtree.size());
		}

		for(unsigned int i = 0; i < indices.size(); ++i) {
			if(indices[i].size() <= 0) continue;

			for(unsigned int j = 0; j < indices[i].size(); ++j) {

				result.elements.push_back(lookup.at(indices[i][j]).data);
				result.distances.push_back(distances[i][j]);
			}
		}

		return result;
	}

	KNNResult nearestWithin(const Element *elem, double radius, double epsilon=0.0) const {
		return kNearestWithin(elem, radius, 1, epsilon);
	}

	KNNResult kNearestWithin(const Element *elem, double radius, int max_neighbors=-1, double epsilon=0.0) const {
		auto stateVars = elem->getTreeStateVars();
		flann::Matrix<double> point(stateVars.data(), 1, stateVars.size());

		std::vector< std::vector<int> > indices;
		std::vector< std::vector<double> > distances;

		flann::SearchParams params(flann::FLANN_CHECKS_UNLIMITED, epsilon);
		params.max_neighbors = max_neighbors;

		kdtree.radiusSearch(point, indices, distances, radius, params);

		KNNResult result;

		for(unsigned int i = 0; i < indices.size(); ++i) {
			if(indices[i].size() <= 0) continue;

			for(unsigned int j = 0; j < indices[i].size(); ++j) {

				result.elements.push_back(lookup.at(indices[i][j]).data);
				result.distances.push_back(distances[i][j]);
			}
		}

		return result;
	}

	std::vector<Element *> getElements() const {
		std::vector<Element *> elems(lookup.size());
		unsigned int i = 0;
		for(const auto &elem : lookup) {
			elems[i++] = elem.second.data;
		}
		return elems;
	}

private:
	flann::Index<DistanceMetric> kdtree;
	std::unordered_map<int, LookupElement> lookup;
	int currentPointIndex;
};