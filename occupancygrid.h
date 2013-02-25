#ifndef __OCCUPANCYGRID_H_INCLUDED__
#define __OCCUPANCYGRID_H_INCLUDED__

#include <cmath>

#define COORD_SIZE 17
#define MAP_SCALE 1
#define MAP_SIZE (int) (COORD_SIZE / MAP_SCALE)
#define GRID_OFFSET MAP_SIZE /2

class OccupancyGrid {
	
	int map[MAP_SIZE][MAP_SIZE];

	public:
		OccupancyGrid();
		int GetCell(int x, int y);
		void SetCell(int x, int y, int value);
		void IncrementCell(int x, int y);
		void SensorUpdate(double x, double y, double range, double angle);
		void PrintGrid();

	private:
		int ScaleToGrid(double num);
		int Round(double num);
};

#endif