#ifndef __VECTORUTILS_H_INCLUDED__
#define __VECTORUTILS_H_INCLUDED__

#include <vector>

class VectorUtils {
	public:
		static std::vector<double> Flatten(std::vector<std::vector<double> > vec);
		static std::vector<double> Filter(std::vector<double> vec, int value);
		static double Average(std::vector<double> vec);
};

#endif