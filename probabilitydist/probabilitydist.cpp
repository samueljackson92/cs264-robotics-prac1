/*
	ProbabilityDist.cpp
	Description: Simple probability distribution implementation
	Author:	Samuel Jackson (slj11@aber.ac.uk)
	Date:	9/4/13
*/

#include <iostream>
#include <iomanip>
#include <cmath>
#include "probabilitydist.h"
#include "point.h"

ProbabilityDist::ProbabilityDist(int start_x, int start_y, int width, int height) {
	this->start_x = start_x;
	this->start_y = start_y;

	this->dist_width = width;
	this->dist_height = height;

	x_offset = 0;
	y_offset = 0;

	double initial_prob = 1.0 / (width * height);
	dist.resize(height);
	for (int i = 0; i < height; i++) {
		dist[i].resize(width, initial_prob);
	}
}

void ProbabilityDist::MotionUpdate(int x, int y) {
	x_offset = x - start_x;
	y_offset = y - start_y;
}

void ProbabilityDist::SampleUpdate(int x, int y, bool hit) {
	double modifier = (hit) ? P_HIT : P_MISS;

	x = x - x_offset;
	y = y - y_offset;

	x = (x%dist_width+dist_width)%dist_width;
	y = (y%dist_height+dist_height)%dist_height;

	std::cout << x << "," << y << std::endl;
	dist[y][x] *= modifier;
}

void ProbabilityDist::Normalize() {
	double sum = 0;
	for (int i = 0; i < dist_height; i++) {
		for (int j = 0; j < dist_width; j++) {
			sum += dist[i][j];
		}
	}

	for (int i = 0; i < dist_height; i++) {
		for (int j = 0; j < dist_width; j++) {
			dist[i][j] /= sum;
		}
	}

}

std::vector<Point> ProbabilityDist::EstimatePosition() {
	double max = 0;
	std::vector<Point> maxes;
	Point p;
	for (int i = 0; i < dist_height; i++) {
		for (int j = 0; j < dist_width; j++) {
			p.SetX(j+x_offset);
			p.SetY(i+y_offset);
			if (dist[i][j] > max) {
				maxes.clear();
				max = dist[i][j];
				maxes.push_back(p);
			} else if(dist[i][j] == max) {
				maxes.push_back(p);
			} 
		}
	}

	for (int i = 0; i<maxes.size(); i++) {
		std::cout << "X: " << maxes[i].GetX() << "Y: " << maxes[i].GetY() << std::endl;
	}
	return maxes;
}

void ProbabilityDist::OutputDist() {
	for (int i = 0; i < dist.size(); i++) {
		for (int j = 0; j < dist[i].size(); j++) {
			std::cout << std::setprecision(3) << dist[i][j] << " ";
		}
		std::cout << std::endl;
	}
}
