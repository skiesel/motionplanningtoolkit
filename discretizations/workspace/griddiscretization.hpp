#pragma once

#include <cassert>
#include <bitset>

template <class Workspace, class Agent>
class GridDiscretization {
public:
	GridDiscretization(const Workspace &workspace, const Agent &agent, const std::vector<double> &discretizationSizes) :
		discretizationSizes(discretizationSizes.begin(), discretizationSizes.end()) {
		bounds = workspace.getBounds();
		assert(bounds.size() == discretizationSizes.size());

		unsigned int cellCount = 1;
		for(unsigned int i = 0; i < discretizationSizes.size(); i++) {
			double range = fabs(bounds[i].first - bounds[i].second);
			if(range == 0) {
				dimensions.push_back(1);
			} else {
				dimensions.push_back(ceil(range / discretizationSizes[i]));
			}
			cellCount *= dimensions.back();
		}

		grid.resize(cellCount);

		for(unsigned int i = 0; i < cellCount; i++) {
			auto pt = getGridCenter(i);
			if(workspace.safePoses(agent, agent.getRepresentivePosesForLocation(pt))) {
				grid[i] = true;
			} else {
				grid[i] = false;
			}
		}

		populateGridNeighborOffsets();
	}

	unsigned int getContainingCellId(const std::vector<double> &point) const {
		return getIndex(point);
	}

	unsigned int getCellCount() const {
		return grid.size();
	}

	double getCostBetweenCells(unsigned int c1, unsigned int c2) const {
		double sum = 0;
		auto c1Vec = getGridCoordinates(c1);
		auto c2Vec = getGridCoordinates(c2);
		for(unsigned int i = 0; i < discretizationSizes.size(); ++i) {
			double delta = fabs((double)c1Vec[i] - (double)c2Vec[i]) * (double)discretizationSizes[i];
			sum += delta * delta;
		}
		return sqrt(sum);
	}

	std::vector< std::vector<double> > getCellBoundingHyperRect(unsigned int n) const {
		std::vector< std::vector<double> > bounds(dimensions.size());

		auto center = getGridCenter(n);

		for(unsigned int i = 0; i < center.size(); ++i) {
			double halfDisc = discretizationSizes[i];
			bounds[i].push_back(center[i] - halfDisc);
			bounds[i].push_back(center[i] + halfDisc);
		}

		return bounds;
	}

	std::vector<unsigned int> getNeighbors(unsigned int n) const {
		std::vector<unsigned int> neighbors;

		auto coordinate = getGridCoordinates(n);

		for(const std::vector<int>& offsets : gridNeighborOffsets) {

			std::vector<unsigned int> neighbor(offsets.size());
			bool valid = true;
			for(unsigned int i = 0; i < neighbor.size(); ++i) {
				int coord = (int)coordinate[i] + offsets[i];
				if(coord < 0 || coord >= dimensions[i]) {
					valid = false;
				}
				neighbor[i] = coord;
			}
			unsigned int index = getIndex(neighbor);
			if(valid && grid[index]) {
				neighbors.push_back(index);
			}
		}

		return neighbors;
	}

private:
	std::vector<unsigned int> getGridCoordinates(unsigned int n) const {
		std::vector<unsigned int> coordinate;

		coordinate.push_back(n % dimensions[0]);
		if(dimensions.size() > 1) {
			unsigned int previousDimSizes = 1;
			for(unsigned int i = 1; i < dimensions.size(); i++) {
				previousDimSizes *= dimensions[i];
				coordinate.push_back(n / previousDimSizes % dimensions[i]);
			}
		}

		return coordinate;
	}

	std::vector<double> getGridCenter(unsigned int n) const {
		std::vector<double> point;

		point.push_back(bounds[0].first + (double)(n % dimensions[0]) * discretizationSizes[0] + discretizationSizes[0]  * 0.5);
		if(dimensions.size() > 1) {
			unsigned int previousDimSizes = 1;
			for(unsigned int i = 1; i < dimensions.size(); i++) {
				previousDimSizes *= dimensions[i];
				point.push_back(bounds[i].first + (double)(n / previousDimSizes % dimensions[i]) * discretizationSizes[i] + discretizationSizes[1]  * 0.5);
			}
		}

		return point;
	}

	unsigned int getIndex(const std::vector<double> &point) const {
		unsigned int index = 0;

		for(unsigned int i = 0; i < discretizationSizes.size(); i++) {

			unsigned int which = floor((point[i] - bounds[i].first) / discretizationSizes[i]);

			double offset = 1;
			for(unsigned int j = 0; j < i; j++) {
				offset *= dimensions[j];
			}

			index += which * offset;
		}
		return index;
	}

	unsigned int getIndex(const std::vector<unsigned int> &gridCoordinate) const {
		unsigned int index = 0;

		for(unsigned int i = 0; i < dimensions.size(); i++) {

			double offset = 1;
			for(unsigned int j = 0; j < i; j++) {
				offset *= dimensions[j];
			}

			index += gridCoordinate[i] * offset;
		}
		return index;
	}

	void populateGridNeighborOffsets() {
		std::vector<int> neighbor(dimensions.size());
		populateGridNeighborOffsetsHelper(0, neighbor);
	}
	
	void populateGridNeighborOffsetsHelper(unsigned int coord, std::vector<int> &neighbor) {
		if(coord >= dimensions.size()) {
			bool add = false;
			for(auto v : neighbor) {
				if(v != 0) {
					add = true;
				}
			}
			if(add) {
				gridNeighborOffsets.push_back(neighbor);
			}
			return;
		}

		for(int i = -1; i < 2; ++i) {
			neighbor[coord] = i;
			populateGridNeighborOffsetsHelper(coord+1, neighbor);
		}
	}

	std::vector< std::pair<double, double> > bounds;
	std::vector<bool> grid;
	std::vector<double> discretizationSizes;
	std::vector<unsigned int> dimensions;
	std::vector< std::vector<int> > gridNeighborOffsets;
};