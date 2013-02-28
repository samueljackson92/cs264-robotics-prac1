#ifndef __VECTORUTILS_H_INCLUDED__
#define __VECTORUTILS_H_INCLUDED__

#include <vector>

class VectorUtils {
	public:
		static std::vector<int> Flatten(std::vector<std::vector<int> > vec);
		static std::vector<int> Filter(std::vector<int> vec, int value);
		static double Average(std::vector<int> vec);
};

#endif