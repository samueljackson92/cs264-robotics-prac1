#include "occupancygrid.h"
#include <iostream>
#include <stdio.h>
#include <cmath>

OccupancyGrid::OccupancyGrid() {
	for (int i = 0; i < MAP_SIZE; i++) {
		for (int j = 0; j < MAP_SIZE; j++) {
			SetCell(i, j, 0);
		}
	}
}

int OccupancyGrid::GetCell(int x, int y) {
	return map[x][(MAP_SIZE-1)-y];
}

void OccupancyGrid::SetCell(int x, int y, int value) {
	map[x][(MAP_SIZE-1)-y] = value;
}

void OccupancyGrid::IncrementCell(int x, int y) {
	SetCell(x,y, GetCell(x,y)+1);
}

//Take the postion of the robot and the range of the sensor
void OccupancyGrid::SensorUpdate(double x, double y, double range, double angle) {
	double sensor_x, sensor_y;
	int grid_x, grid_y;

	if (range < 4) {
		//new point hit by sensor
		sensor_x = x + (cos(angle) * range);
		sensor_y = y + (sin(angle) * range);

		std::cout << "Our Position:" << std::endl;
		std::cout << "x: " << x << std::endl;
		std::cout << "y: " << y << std::endl;

		std::cout << "Our Range Point:" << std::endl;
		std::cout << "x: " << sensor_x << std::endl;
		std::cout << "y: " << sensor_y << std::endl;

		x = ScaleToGrid(x);
		y = ScaleToGrid(y);

		grid_x = ScaleToGrid(sensor_x);
		grid_y = ScaleToGrid(sensor_y);

		std::cout << "Our Cell:" << std::endl;
		std::cout << "x: " << x << std::endl;
		std::cout << "y: " << y << std::endl;

		std::cout << "Range Cell:" << std::endl;
		std::cout << "x: " << grid_x << std::endl;
		std::cout << "y: " << grid_y << std::endl;

		IncrementCell(grid_x, grid_y);
	}
}

void OccupancyGrid::PrintGrid(){
	using namespace std;
	for (int i = 0; i < MAP_SIZE; i++) {
		for (int j = 0; j < MAP_SIZE; j++) {
			printf(" %3d ", map[j][i]);
		}
		cout << endl;
	}
}

int OccupancyGrid::ScaleToGrid(double num) {
	return RoundHalfUp((num + GRID_OFFSET) * MAP_SCALE);
}

int OccupancyGrid::RoundHalfUp(double num) {
	return (num >= 0) ? floor(num+0.6) : ceil(num-0.6);
}