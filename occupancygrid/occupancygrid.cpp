#include <iostream>
#include <stdio.h>
#include <fstream>
#include <cmath>
#include <cstdlib>
#include <string>
#include <libplayerc++/playerc++.h> 

#include "../vectorutils/matrixutils.h"
#include "cell.h"
#include "occupancygrid.h"

using namespace PlayerCc;

OccupancyGrid::OccupancyGrid() {
	threshold = 0;

	grid_height = 0;
	grid_width = 0;

	ResizeGrid(EXPANSION_SIZE, EXPANSION_SIZE);

	grid_height = EXPANSION_SIZE;
	grid_width = EXPANSION_SIZE;

	start_x = (EXPANSION_SIZE/2)-1;
	start_y = (EXPANSION_SIZE/2)-1;

	robot_x = (start_x*MAP_SCALE)+MAP_SCALE/2;
	robot_y = (start_y*MAP_SCALE)+MAP_SCALE/2;

	last_expansion = 0;
}

void OccupancyGrid::Init(double x, double y) {
	old_x = x;
	old_y = y;
}

void OccupancyGrid::IncrementCell(int x, int y) {
	SetCellValue(x,y, GetCellValue(x,y)+1);
}

void OccupancyGrid::UpdateBotPosition(double x, double y) {
	double dx = x - old_x;
	double dy = y - old_y;

	robot_x += dx;
	robot_y += dy;

	old_x = x;
	old_y = y;
}

//Take the postion of the robot and the range of the sensor
void OccupancyGrid::SensorUpdate(double range, double angle) {
	double sensor_x, sensor_y;
	int grid_x, grid_y;

	//correct for sensor distance from actual robot position. (+0.3)
	//Also damped slightly.
	range += MAP_SCALE/2;
	if (range < MAX_RANGE) {

		//new point hit by sensor
		sensor_x = robot_x + (cos(angle) * range);
		sensor_y = robot_y + (sin(angle) * range); 

		grid_x = ScaleToGrid(sensor_x);
		grid_y = ScaleToGrid(sensor_y);
		
		ExpandGrid(grid_x, grid_y);

		double max_grid_r = (MAX_RANGE/MAP_SCALE);
		double range_prob = (max_grid_r-range)/max_grid_r;
		double val = GetCellValue(grid_x, grid_y);

		val = (val < 0) ? 0 : val;
		val += range_prob;

		SetCellValue(grid_x, grid_y, val);
	}
}

void OccupancyGrid::PrintGrid(){
	using namespace std;

	for (int y = (grid_height-1); y >= 0; y--) {
		for (int x = 0; x < grid_width; x++) {
			int value = GetCellValue(x, y);
			printf(" %4d", value);
		}
		cout << endl;
	}
}

void OccupancyGrid::PrintFinalGrid(){
	using namespace std;
	double threshold = CalculateThreshold();

	for (int y = (grid_height-1); y >= 0; y--) {
		for (int x = 0; x < grid_width; x++) {
			int value = GetCellValue(x, y);

			if(value < threshold)
				cout << ".";
			else {
				cout << "#";
			}
		}
		cout << endl;
	}

}

void OccupancyGrid::PrintDebug(){
	using namespace std;
	double threshold = CalculateThreshold();

	int rx = ScaleToGrid(robot_x);
	int ry = ScaleToGrid(robot_y);
	cout << rx<<endl;
	cout << ry<<endl;
	for (int y = 0; y < grid_height; y++) {
		for (int x = 0; x < grid_width; x++) {
			Cell* c = GetCell(x, y);

			cout << " (" << c->GetX() << "," << c->GetY() << ")";

			if(rx == x && ry == y) {
				cout << " R";
			} else {
				cout << " .";
			}
		}
		cout << endl;
	}

}

int OccupancyGrid::ScaleToGrid(double num) {
	num = num / MAP_SCALE;
	return (num >= 0) ? floor(num) : ceil(num);
}

double OccupancyGrid::ScaleToWorld(int num) {
	return ((num - start_x) *MAP_SCALE);
}

void OccupancyGrid::ExpandGrid(int& x, int& y) {
	//if point falls outside current grid

	int new_width, new_height;
	int x_expand = 0, y_expand = 0;
	last_expansion = 1;

	if((x < 0 || y < 0) ||(x >= grid_width || y >= grid_height)) {
		
		x_expand = (x < 0 || x >= grid_width) ? EXPANSION_SIZE : 0;
		y_expand = (y < 0 || y >= grid_height) ? EXPANSION_SIZE : 0;

		new_width = grid_width + x_expand;
		new_height = grid_height + y_expand;

		std::cout << "EXPANDING" << std::endl;
		std::cout << "Values: x" << x << " y " << y << std::endl;
 		
		std::cout << new_width << "," << new_height << std::endl;
 		//resize grid to new dimensions
		ResizeGrid(new_width, new_height);
		
		//if we did a negative resize, shift data
		if(x < 0 || y < 0) {
			std::cout << "Y Expand!" << std::endl;

			last_expansion = -1;

			robot_x += (x_expand > 0) ? (x_expand)*MAP_SCALE : 0;
			robot_y += (y_expand > 0) ? (y_expand)*MAP_SCALE : 0;

			for (int j = (grid_height-1); j >= 0; j--) {
				for (int i = (grid_width-1); i >= 0; i--) {

					Cell* old = grid[j][i];
					Cell* newc = grid[j+y_expand][i+x_expand];

					old->SetX(i+x_expand);
					old->SetY(j+y_expand);
					grid[j+y_expand][i+x_expand] = old;
	
					newc->SetX(i);
					newc->SetY(j);
					grid[j][i] = newc;
				}
			}

			x+=x_expand;
			y+=y_expand;
		}



		grid_width = new_width;
		grid_height = new_height;
	}
}

void OccupancyGrid::ResizeGrid(int w, int h) {

	grid.resize(h);
	for (int i = 0; i < h; i++) {
		grid[i].resize(w);
	}

	for(int i = 0; i < h; i++) {
		for (int j =0; j < w; j++) {
			if(i >= grid_height || j >= grid_width) {
				grid[i][j] = new Cell();
				grid[i][j]->SetX(j);
				grid[i][j]->SetY(i);
			}
		}
	}
}

void OccupancyGrid::WriteGrid(std::string filename) {
	using namespace std;
	ofstream file;
	file.open(filename.c_str());

	for (int y = 0; y < grid_height; y++) {
		for (int x = 0; x < grid_width; x++) {
			double value = GetCellValue(x, y);
			file << value;
			if(x < (grid_width-1)) { file << ","; }
		}
		file << "\n";
	}

	file.close();
}

double OccupancyGrid::CalculateThreshold() {
	using namespace std;
	vector<double> vec;

	for(int i = 0; i < grid_height; i++) {
		for(int j = 0; j < grid_width; j++) {
			double val = grid[i][j]->GetValue();
			val = (val < 0) ? 0 : val;
			vec.push_back(val);
		}
	}

	return MatrixUtils::Otsu(vec);
}

Cell* OccupancyGrid::GetCurrentCell() {
	int x = ScaleToGrid(robot_x);
	int y = ScaleToGrid(robot_y);

	return GetCell(x,y);
}

Cell* OccupancyGrid::GetCell(int x, int y) {
	ExpandGrid(x,y);
	return grid[y][x];
}

double OccupancyGrid::GetCellValue(int x, int y) {
	ExpandGrid(x,y);
	return grid[y][x]->GetValue();
}

void OccupancyGrid::SetCellValue(int x, int y, double value) {
	ExpandGrid(x, y);
	grid[y][x]->SetValue(value);
}

int OccupancyGrid::GetGridHeight() {
	return grid_height;
}

int OccupancyGrid::GetGridWidth() {
	return grid_width;
}

void OccupancyGrid::SetRobotPosition(int x, int y) {
	double new_x = (x*MAP_SCALE);
	double new_y = (y*MAP_SCALE);

	double x_diff = robot_x - floor(robot_x);
	double y_diff = robot_y - floor(robot_y);

	new_x += x_diff;
	new_y += y_diff;

	old_x = new_x;
	old_y = new_y;

	robot_x = new_x;
	robot_y = new_y;
}

void OccupancyGrid::LoadValues(const std::vector<std::vector<double> >& map) {
	ResizeGrid(map.size(), map[0].size());

	for(int i =0; i < map.size(); i++) {
		for (int j = 0; j < map[i].size(); j++) {
			SetCellValue(j, i, map[i][j]);
		}
	}
}
