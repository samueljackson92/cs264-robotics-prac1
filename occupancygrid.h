#ifndef __OCCUPANCYGRID_H_INCLUDED__
#define __OCCUPANCYGRID_H_INCLUDED__

#include <vector>
#include <string>

#define MAP_SCALE 0.6
#define ROUNDING_OFFSET 0.35
#define EXPANSION_SIZE 6

class OccupancyGrid {
	
	//grid of cells
	std::vector<std::vector<int> > grid;
	double robot_x, robot_y; //internal robot position
	double start_x, start_y; //internal robot start point
	double old_x, old_y;	 //previous internal robot positon
	int grid_x, grid_y;		 //range grid position
	int grid_height, grid_width;
	int threshold;

	public:
		OccupancyGrid(double x, double y);
		int GetCell(int x, int y);
		void SetCell(int x, int y, int value);
		void IncrementCell(int x, int y);
		void SensorUpdate(double range, double angle);
		void PrintGrid();
		void UpdateBotPosition(double x, double y);
		void WriteGrid(const char* filename);
	private:
		void ExpandGrid();
		void CalculateThreshold();
		void ResizeGrid(int w, int h);
		int ScaleToGrid(double num);
};

#endif