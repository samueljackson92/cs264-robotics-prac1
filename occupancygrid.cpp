#include <iostream>
#include <stdio.h>
#include <fstream>
#include <cmath>
#include <stdlib.h>
#include <libplayerc++/playerc++.h> 

#include "vectorutils.h"
#include "occupancygrid.h"

using namespace PlayerCc;

OccupancyGrid::OccupancyGrid() {
	threshold = 0;

	grid_height = EXPANSION_SIZE;
	grid_width = EXPANSION_SIZE;

	start_x = EXPANSION_SIZE/2;
	start_y = EXPANSION_SIZE/2;

	robot_x = start_x*MAP_SCALE + 0.3;
	robot_y = start_y*MAP_SCALE + 0.3;

	grid_x = start_x;
	grid_y = start_y;

	ResizeGrid(grid_width, grid_height);
}

void OccupancyGrid::Init(double x, double y) {
	old_x = x;
	old_y = y;
}

double OccupancyGrid::GetCell(int x, int y) {
	ExpandGrid(x,y);
	return grid[y][x];
}

void OccupancyGrid::SetCell(int x, int y, double value) {
	ExpandGrid(x,y);
	grid[y][x] = value;
}

void OccupancyGrid::IncrementCell(int x, int y) {
	SetCell(x,y, GetCell(x,y)+1);
}

void OccupancyGrid::UpdateBotPosition(double x, double y) {
	double dx = x - old_x;
	double dy = y - old_y;

	std::cout << "dx: " << dx << "dy: " << dy << std::endl; 
	robot_x += dx;
	robot_y += dy;

	old_x = x;
	old_y = y;
}

//Take the postion of the robot and the range of the sensor
void OccupancyGrid::SensorUpdate(double range, double angle) {
	double sensor_x, sensor_y;

	if (range < MAX_RANGE) {

		//new point hit by sensor
		sensor_x = robot_x + (cos(angle) * range);
		sensor_y = robot_y + (sin(angle) * range);

		std::cout << "----------------------------------------" << std::endl;
		std::cout << "Robot Angle: " << angle << std::endl;
		std::cout << "Robot Range: " << range << std::endl;
		std::cout << "Our Position:";
		std::cout << " x: " << robot_x ;
		std::cout << " y: " << robot_y << std::endl;

		std::cout << "Our Range Point:";
		std::cout << " x: " << sensor_x;
		std::cout << " y: " << sensor_y << std::endl;

		grid_x = ScaleToGrid(sensor_x);
		grid_y = ScaleToGrid(sensor_y);

		std::cout << "Range Cell:";
		std::cout << " x: " << grid_x;
		std::cout << " y: " << grid_y << std::endl;

		std::cout << "----------------------------------------" << std::endl;

		double max_grid_r = (MAX_RANGE/MAP_SCALE);
		double range_prob = (max_grid_r-range)/max_grid_r;

		SetCell(grid_x, grid_y, GetCell(grid_x, grid_y) + range_prob);
	}
}

void OccupancyGrid::PrintGrid(){
	using namespace std;

	for (int y = (grid_height-1); y >= 0; y--) {
		for (int x = 0; x < grid_width; x++) {
			int value = GetCell(x, y);
			printf(" %4d", value);
		}
		cout << endl;
	}

	CalculateThreshold();
}

int OccupancyGrid::ScaleToGrid(double num) {
	num = num / MAP_SCALE;
	return (num >= 0) ? floor(num) : ceil(num);
}

void OccupancyGrid::ExpandGrid(double x, double y) {
	//if point falls outside current grid
	int new_width, new_height;
	int x_expand = 0, y_expand = 0;
	if((x < 0 || y < 0) ||(x >= grid_width || y >= grid_height)) {
		
		x_expand = (x < 0 || x >= grid_width) ? EXPANSION_SIZE : 0;
		y_expand = (y < 0 || y >= grid_height) ? EXPANSION_SIZE : 0;

		new_width = grid_width + x_expand;
		new_height = grid_height + y_expand;

		//resize grid to new dimensions
		ResizeGrid(new_width, new_height);
		
		//if we did a negative resize, shift data
		if(x < 0 || y < 0) {
			std::cout << "Moved in Y" << std::endl;
			robot_x += (x_expand*MAP_SCALE);
			robot_y += (y_expand*MAP_SCALE);

			x += EXPANSION_SIZE;
			y += EXPANSION_SIZE;

			for (int i = (grid_width-1); i >= 0; i--) {
				for (int j = (grid_height-1); j >= 0; j--) {
					int value = GetCell(i,j);
					SetCell(i+x_expand,j+y_expand,value);
					SetCell(i,j,0);
				}
			}
		}

		grid_width = new_width;
		grid_height = new_height;
	}
}

void OccupancyGrid::ResizeGrid(int w,  int h) {
	grid.resize(h);
	for (int i = 0; i < h; ++i) {
 		grid[i].resize(w);
 	}
}

void OccupancyGrid::WriteGrid(const char* filename) {
	using namespace std;
	ofstream file;
	file.open(filename);

	for (int y = (grid_height-1); y >= 0; y--) {
		for (int x = 0; x < grid_width; x++) {
			double value = GetCell(x, y);
			file << value;
			if(x < (grid_width-1)) { file << ","; }
		}
		file << "\n";
	}

	file.close();
}

void OccupancyGrid::CalculateThreshold() {
	//calculate the average of all wall points
	using namespace std;
	double average = 0;

	// vector<int> vec = VectorUtils::Flatten(grid);
	// vec = VectorUtils::Filter(vec, 0);
	// average = VectorUtils::Average(vec)/2;

	cout << "Threshold: " << average << endl;
}