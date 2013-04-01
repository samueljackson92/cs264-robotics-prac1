/*
	Mapper.h
	Description: Robot Mapping system.
	Author:	Samuel Jackson (slj11@aber.ac.uk)
	Date:	1/4/13
*/

#ifndef __MAPPER_H_INCLUDED__
#define __MAPPER_H_INCLUDED__

#include <libplayerc++/playerc++.h>

#include "pcontroller/pcontroller.h"
#include "occupancygrid/occupancygrid.h"

using namespace PlayerCc;

class PController;
class Mapper {
	OccupancyGrid grid;
	PlayerClient robot;
	RangerProxy sp;
	Position2dProxy pp;
	PController pc;

	public:
		Mapper();
		void Start();
		void MoveToNextCell();
		void UpdateGrid();
		vector<Cell> FindPath(Cell start, Cell goal);
		vector<Cell> GetNeighbours(Cell c);
		vector<Cell> ReconstructPath(map<Cell, Cell> came_from, Cell current_node);
		~Mapper();

};

#endif