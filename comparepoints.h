
#include <map>
#include <cmath>
#include "occupancygrid/cell.h"

class ComparePoints{
	Cell goal;
	std::map<Cell, int> f_score;
	
	public:
		ComparePoints(Cell g, std::map<Cell, int>& fs) 
			: f_score(fs) {
			goal = g;
		}

		bool operator()(Cell& c1, Cell& c2) {
			return f_score[c1] < f_score[c2];
		}

	 	static int Distance(Cell c1, Cell c2) {
			return std::abs(c1.GetX() - c2.GetX()) + std::abs(c1.GetY() - c2.GetY());
		}
};