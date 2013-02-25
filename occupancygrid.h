#ifndef __OCCUPANCYGRID_H_INCLUDED__
#define __OCCUPANCYGRID_H_INCLUDED__

#include <cmath>

#define COORD_SIZE 17
#define MAP_SCALE 0.6
#define BOT_START_X -7
#define BOT_START_Y -7
#define MAP_SIZE (int) (COORD_SIZE / MAP_SCALE)

class OccupancyGrid {
	
	int map[MAP_SIZE][MAP_SIZE];
	double robot_x, robot_y;
	double start_x, start_y;
	double old_x, old_y;
	int grid_x, grid_y;

	public:
		OccupancyGrid();
		int GetCell(int x, int y);
		void SetCell(int x, int y, int value);
		void IncrementCell(int x, int y);
		void SensorUpdate(double range, double angle);
		void PrintGrid();
		void UpdateBotPosition(double x, double y);
	private:
		int ScaleToGrid(double num);
		int Round(double num);
};

#endif