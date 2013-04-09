/*
	ProbabilityDist.h
	Description: Simple probability distribution implementation
	Author:	Samuel Jackson (slj11@aber.ac.uk)
	Date:	9/4/13
*/


#ifndef __PROBABILITYDIST_H_INCLUDED__
#define __PROBABILITYDIST_H_INCLUDED__

#include <vector>
#include "point.h"

#define P_HIT 0.6
#define P_MISS 0.2

class ProbabilityDist {
	int start_x, start_y;
	int x_offset, y_offset;
	int dist_height, dist_width;

	std::vector<std::vector<double> > dist;

public:
	ProbabilityDist(int start_x, int start_y, 
		int width, int height);
	void OutputDist();
	void MotionUpdate(int x, int y);
	void SampleUpdate(int x, int y, bool hit);
	void Normalize();
	std::vector<Point> EstimatePosition();
};

#endif