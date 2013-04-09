/*
	Cell.h
	Description: single cell in grid.
	Author:	Samuel Jackson (slj11@aber.ac.uk)
	Date:	1/4/13
*/

#ifndef __CELL_H_INCLUDED__
#define __CELL_H_INCLUDED__
#include <iostream>
class Cell {
	double value;
	double x,y;
	bool visited;
	bool discovered;
	
	public:
		Cell();
		void SetValue(double val);
		double GetValue();
		double GetX();
		double GetY();
		void SetX(double x);
		void SetY(double y);
		bool IsVisited();
		void SetVisited(bool val);
		bool IsDiscovered();
		void SetDiscovered(bool val);
		bool operator==(const Cell& other) const;
		bool operator!=(const Cell& other) const;
		bool operator<(const Cell& other) const;
		friend std::ostream& operator<<(std::ostream& os, const Cell& c)
		{
		    os << "X: " << c.x << "Y: " << c.y << std::endl;
		    os << "Value: " << c.value << std::endl;
	     	os << "Visited: " << c.visited << std::endl;
	     	os << "Discovered: " << c.discovered << std::endl;
		    return os;
		}
};

#endif