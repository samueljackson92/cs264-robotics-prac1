
#include "occupancygrid/cell.h"

class ComparePoints{
	Cell goal;
	map<Cell, int> f_score;
	
	ComparePoints(Cell g, map<Cell, int>& fs) 
		: goal(g), f_score(fs) {

	}

	bool operator()(Cell& c1, Cell& c2) {
		return f_score[c1] f_score[c2];
	}

 	static int Distance(Cell c1, Cell c2) {
		return abs(c1.GetX() - c2.GetX()) + abs(c1.GetY() - c2.GetY());
	}
};