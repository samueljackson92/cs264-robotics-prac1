/*
	Mapper.h
	Description: Robot Mapping system.
	Author:	Samuel Jackson (slj11@aber.ac.uk)
	Date:	1/4/13
*/

#ifndef __MAPPER_H_INCLUDED__
#define __MAPPER_H_INCLUDED__

#include <vector>
#include <map>

#include <libplayerc++/playerc++.h>

#include "pcontroller/pcontroller.h"
#include "occupancygrid/cell.h"
#include "occupancygrid/occupancygrid.h"
#include "probabilitydist/point.h"

using namespace PlayerCc;

class PController;
class Mapper {
	OccupancyGrid grid;
	PlayerClient robot;
	//SonarProxy sp;
	RangerProxy sp;
	Position2dProxy pp;
	PController pc;

	double threshold;
	double robot_x, robot_y;
	double start_x, start_y;
	int map_height, map_width;
	std::vector<std::vector<double> > mapData;

	public:
		Mapper();
		void Start();

		//Localising Functions
		Point Localize();
		std::vector<std::vector<double> > loadData(std::string filename);
		std::vector<double> GetMapNeighbours(int x, int y);
		void RandomWander();

		//Mapping Functions
		void MoveToNextCell(Cell start, Cell goal);
		void UpdateGrid();
		bool vec_contains(std::vector<Cell*> vec, Cell* c);
		std::vector<Cell*> GetNeighbours(Cell* current);
		std::vector<Cell*> FindPath(Cell* current, Cell* goal);
		std::vector<Cell*> ReconstructPath(std::map<Cell*, Cell*> came_from, Cell* current_node);
		~Mapper();
};

#endif