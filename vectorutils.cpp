
#include <vector>
#include <numeric>

#include "vectorutils.h"

std::vector<int> VectorUtils::Flatten(std::vector<std::vector<int> > vec) {
	using namespace std;
	vector<int> v;

	for (int i = 0; i < vec.size(); i++) {
		for (int j = 0; j < vec[i].size(); j++) {
			v.push_back(vec[i][j]);
		}
	}

	return v;
}

std::vector<int> VectorUtils::Filter(std::vector<int> vec, int value) {
	using namespace std;
	vector<int> v;

	for (int i = 0; i < vec.size(); i++) {
		if (vec[i] != value) {
			v.push_back(vec[i]);
		}
	}

	return v;
}

double VectorUtils::Average(std::vector<int> vec) {
	return accumulate(vec.begin(), vec.end(), 0) / static_cast<double>(vec.size());
}
