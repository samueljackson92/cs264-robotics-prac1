/*
	Cell.h
	Description: single cell in grid.
	Author:	Samuel Jackson (slj11@aber.ac.uk)
	Date:	1/4/13
*/

#ifndef __CELL_H_INCLUDED__
#define __CELL_H_INCLUDED__

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
		bool operator<(const Cell& other) const;
};

#endif