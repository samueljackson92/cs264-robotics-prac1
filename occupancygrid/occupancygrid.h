
#ifndef __OCCUPANCYGRID_H_INCLUDED__
#define __OCCUPANCYGRID_H_INCLUDED__

#include <vector>
#include <string>

#define MAP_SCALE 0.6
#define MAX_RANGE 1.8
#define EXPANSION_SIZE 6

#include "cell.h"

class OccupancyGrid {
	
	//grid of cells
	Cell*** grid;
	double robot_x, robot_y; //internal robot position
	double start_x, start_y; //internal robot start point
	double old_x, old_y;	 //previous internal robot positon
	int grid_height, grid_width;
	int threshold;

	public:
		OccupancyGrid();
		void Init(double x, double y);
		double GetCellValue(int x, int y);
		void SetCellValue(int x, int y, double value);
		void IncrementCell(int x, int y);
		void SensorUpdate(double range, double angle);
		void PrintGrid();
		void PrintFinalGrid();
		void PrintDebug();
		void UpdateBotPosition(double x, double y);
		void WriteGrid(const char* filename);
		Cell* GetCurrentCell();
		Cell* GetCell(int x, int y);
		void SetCell(int x, int y, Cell* c);
		double ScaleToWorld(int num);
		int GetGridHeight();
		int GetGridWidth();
	private:
		void ExpandGrid(int& x, int& y);
		double CalculateThreshold();
		void ResizeGrid(int w, int h);
		int ScaleToGrid(double num);
};

#endif