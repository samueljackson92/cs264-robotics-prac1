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
	start_x = 1.0;
	start_y = 1.0;

	old_x = BOT_START_X;
	old_y = BOT_START_X;

	robot_x = start_x;
	robot_y = start_y;
}

int OccupancyGrid::GetCell(int x, int y) {
	return map[(MAP_SIZE-1) - y][x];
}

void OccupancyGrid::SetCell(int x, int y, int value) {
	map[(MAP_SIZE-1) - y][x] = value;
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

	sensor_x = robot_x + (cos(angle) * range);
	sensor_y = robot_y + (sin(angle) * range);

	if (range < 4) {
		//new point hit by sensor
		sensor_x = robot_x + (cos(angle) * range);
		sensor_y = robot_y + (sin(angle) * range);

		std::cout << "Our Position:" << std::endl;
		std::cout << "x: " << robot_x << std::endl;
		std::cout << "y: " << robot_y << std::endl;

		std::cout << "Our Range Point:" << std::endl;
		std::cout << "x: " << sensor_x << std::endl;
		std::cout << "y: " << sensor_y << std::endl;

		grid_x = ScaleToGrid(sensor_x);
		grid_y = ScaleToGrid(sensor_y);

		std::cout << "Range Cell:" << std::endl;
		std::cout << "x: " << grid_x << std::endl;
		std::cout << "y: " << grid_y << std::endl;

		IncrementCell(grid_x, grid_y);
	}
}

void OccupancyGrid::PrintGrid(){
	using namespace std;
	for (int y = (MAP_SIZE-1); y >= 0; y--) {
		for (int x = 0; x < MAP_SIZE; x++) {
			int value = GetCell(x, y);
			printf(" %4d", value);
		}
		cout << endl;
	}
}

int OccupancyGrid::ScaleToGrid(double num) {
	return Round(num/MAP_SCALE);
}

int OccupancyGrid::Round(double num) {
	return (num >= 0) ? floor(num+0.5) : ceil(num-0.5);
}