/*
	Mapper.cpp
	Description: Robot Mapping system.
	Author:	Samuel Jackson (slj11@aber.ac.uk)
	Date:	1/4/13
*/

#define _USE_MATH_DEFINES

#include <vector>
#include <cmath>
#include <queue>
#include <map>
#include <algorithm>


#include "occupancygrid/occupancygrid.h"
#include "comparepoints.h"
#include "mapper.h"

using namespace std;
using namespace PlayerCc;

Mapper::Mapper() : robot("localhost"), sp(&robot,0), 
pp(&robot,0), pc(&robot, &pp, this) {
	robot.Read();

	double x = pp.GetXPos();
	double y = pp.GetYPos();

	//init start position
	start_x = x;
	start_y = y;

	//init occupancy gird.
	grid.Init(x, y);
	
	pp.SetMotorEnable(true);
}

void Mapper::Start() {

	//Gather some inital readings.
	for (int i=0; i<10; i++) {
		UpdateGrid();
	}

	//frontier to store cells tobe explored.
	vector<Cell> frontier;

	//The cell we are currently at
	Cell start = grid.GetCurrentCell();
	grid.SetVisited(start.GetX(), start.GetY(), true);
	grid.SetDiscovered(start.GetX(), start.GetY(), true);

	frontier.push_back(start);

	Cell current = start;

	//while there are still unexpored cells.
	while(!frontier.empty()) {

		//find the 4 neighbours
		vector<Cell> neighbours = GetNeighbours(current);
		for(vector<Cell>::iterator it = neighbours.begin(); it != neighbours.end(); ++it) {
			Cell neighbour = *it;
			if(neighbour.GetValue() == 0) {
				if(!neighbour.IsDiscovered() && !neighbour.IsVisited()) {
					//add new cell to frontier and mark discovered
					grid.SetDiscovered(neighbour.GetX(), neighbour.GetY(), true);
					frontier.push_back(neighbour);
				}
			}
		}

		Cell oldPos = current;
		current = frontier.back();
		frontier.pop_back();

		cout << "AT: " << oldPos << endl;
		cout << "HEADING: " << current << endl;
		MoveToNextCell(oldPos, current);
	}
}

void Mapper::MoveToNextCell(Cell start, Cell goal) {
	double dx = (goal.GetX() - start.GetX()) * MAP_SCALE;
	double dy = (goal.GetY() - start.GetY()) * MAP_SCALE;

	cout << "Oldx: " << pp.GetXPos() << " Oldy: " << pp.GetYPos()  << endl;
	double x = pp.GetXPos() + dx;
	double y = pp.GetYPos() + dy;

	cout << "x: " << x << "y: " << y << endl;
	pc.MoveToPosition(x, y);
}

vector<Cell> Mapper::GetNeighbours(Cell current) {
	vector<Cell> neighbours;
	double x = current.GetX();
	double y = current.GetY();

	neighbours.push_back(grid.GetCell(x+1,y));
	neighbours.push_back(grid.GetCell(x-1,y));
	neighbours.push_back(grid.GetCell(x,y+1));
	neighbours.push_back(grid.GetCell(x,y-1));

	return neighbours;
}

void Mapper::UpdateGrid() {
	robot.Read();
	double x = pp.GetXPos();
	double y = pp.GetYPos();
	double angle = rtod(pp.GetYaw());

	grid.UpdateBotPosition(x,y);
	grid.SensorUpdate(sp[3], dtor(angle+10));
	grid.SensorUpdate(sp[4], dtor(angle-10));

	//side sensors
	grid.SensorUpdate((sp[0] + sp[15])/2, dtor(angle+90));
	grid.SensorUpdate((sp[7] + sp[8])/2, dtor(angle-90));

	//rear sensors
	grid.SensorUpdate(sp[12], dtor(angle + 170));
	grid.SensorUpdate(sp[11], dtor(angle - 170));

	//diagonal sensors
	grid.SensorUpdate(sp[1], dtor(angle + 50));
	grid.SensorUpdate(sp[2], dtor(angle + 30));
	grid.SensorUpdate(sp[5], dtor(angle - 30));
	grid.SensorUpdate(sp[6], dtor(angle - 50));
	

	grid.SensorUpdate(sp[14], dtor(angle + 130));
	grid.SensorUpdate(sp[13], dtor(angle + 150));

	grid.SensorUpdate(sp[10], dtor(angle - 150));
	grid.SensorUpdate(sp[9], dtor(angle - 130));

	// cout << endl;
	// grid.PrintGrid();
	// cout << endl;
}

Mapper::~Mapper(){
	grid.PrintFinalGrid();
	//export grid on destruction.
   	grid.WriteGrid("output.csv");
}

int main(int argc, char *argv[])
{
	Mapper m;
	m.Start();
}

