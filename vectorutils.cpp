
#include <vector>
#include <numeric>

#include "vectorutils.h"

std::vector<double> VectorUtils::Flatten(std::vector<std::vector<double> > vec) {
	using namespace std;
	vector<double> v;

	for (int i = 0; i < vec.size(); i++) {
		for (int j = 0; j < vec[i].size(); j++) {
			v.push_back(vec[i][j]);
		}
	}

	return v;
}

std::vector<double> VectorUtils::Filter(std::vector<double> vec, int value) {
	using namespace std;
	vector<double> v;

	for (int i = 0; i < vec.size(); i++) {
		if (vec[i] != value) {
			v.push_back(vec[i]);
		}
	}

	return v;
}

double VectorUtils::Average(std::vector<double> vec) {
	return accumulate(vec.begin(), vec.end(), 0) / static_cast<double>(vec.size());
}
