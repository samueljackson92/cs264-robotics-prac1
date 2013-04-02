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


using namespace PlayerCc;

class PController;
class Mapper {
	OccupancyGrid grid;
	PlayerClient robot;
	RangerProxy sp;
	Position2dProxy pp;
	PController pc;

	double start_x, start_y;

	public:
		Mapper();
		void Start();
		void MoveToNextCell(Cell start, Cell goal);
		void UpdateGrid();
		bool vec_contains(std::vector<Cell> vec, Cell c);
		std::vector<Cell> FindPath(Cell start, Cell goal);
		std::vector<Cell> GetNeighbours(Cell current);
		std::vector<Cell> ReconstructPath(std::map<Cell, Cell> came_from, Cell start, Cell current_node);
		~Mapper();

};

#endif