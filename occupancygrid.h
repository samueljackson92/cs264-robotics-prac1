#ifndef __OCCUPANCYGRID_H_INCLUDED__
#define __OCCUPANCYGRID_H_INCLUDED__

#define MAP_SIZE 16
#define MAP_SCALE 1
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
		int RoundHalfUp(double num);
};

#endif