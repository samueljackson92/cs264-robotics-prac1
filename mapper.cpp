/*
	Mapper.cpp
	Description: Robot Mapping system.
	Author:	Samuel Jackson (slj11@aber.ac.uk)
	Date:	1/4/13
*/

#define _USE_MATH_DEFINES

#include <iostream>
#include <vector>
#include <cmath>
#include <queue>
#include <map>
#include <set>

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

	grid.Init(x, y);
	
	pp.SetMotorEnable(true);
}

void Mapper::Start() {
	//create list of cells on the frointer
	vector<Cell> frontier;

	//get where we currently are.
	Cell current = grid.GetCurrentCell();
	//mark it as visited & discovered
	current.SetDiscovered(true);
	current.SetVisited(true);
	
	//while there are unexplore cells
	while (!frontier.empty()) {
		//find adjacent cells
		vector<Cell> neighbours = GetNeighbours(current);
		for (vector<Cell>::iterator it = neighbours.begin();
			it != neighbours.end(); ++it) {
			if(it->GetValue() == 0) {
				//check if we've already found this neighbour
				if(!it->IsVisted() && !it->IsDiscovered()) {
					//add it to our frontier
					it->SetDiscovered(true);
					frontier.push_back(*it);
				}
			}
		}

		//get next item from frontier
		current = frontier.pop_back();
		Cell ourPos = grid.GetCurrentCell();
		//move to cell
		MoveToNextCell(ourPos, current);
		//Mark as being visited
		current.SetVisited(true);
	}

	cout << "Finished Mapping!" << endl;

}

void Mapper::MoveToNextCell(Cell start, Cell goal) {
	vector<Cell> path = FindPath(start, goal);
	//move along path until goal.
}

vector<Cell> Mapper::FindPath(Cell start, Cell goal) {
	map<Cell, int> f_score;
	map<Cell, int> g_score;
	set<Cell> closed_set;
	set<Cell> in_queue;
	map<Cell, Cell> came_from;

	priority_queue<Cell, vector<Cell>, ComparePoints> frontier(goal, f_score, g_score);
	frontier.push(start);

	g_score[start] = 0;
	f_score[start] = g_score[start] + ComparePoints::Distance(start, goal);
	
	Cell current;
	while (!frontier.empty()) {
		current = frontier.top();
		if(current == goal) {
			return ReconstructPath(came_from, goal);
		}
		closed_set.insert(fronter.pop());
		vector<Cell> neighbours = GetNeighbours(current);
		for (vector<Cell>::iterator it = neighbours.begin();
			it != neighbours.end(); ++it) {
			Cell neighbour = *it;

			int tentative_g_score = g_score[current] + 1;
			if(closed_set.find(neighbour) != closed_set.end()) {
				if(tentative_g_score >= g_score[neighbour]) {
					continue;
				}
			}

			if(in_queue.find(neighbour) == in_queue.end()
				|| tentative_g_score < g_score[(neighbour)]) {
				came_from[neighbour] = current;
				g_score[neighbour] = tentative_g_score;
				f_score[neighbour] = g_score[neighbour] + ComparePoints::Distance(*it, goal);
			
				if(in_queue.find(neighbour) == in_queue.end()) {
					frontier.push(neighbour);
					in_queue.insert(neighbour);
				}
			}
		}
	}

	return NULL;
}

vector<Cell> Mapper::ReconstructPath(map<Cell, Cell> came_from, 
	Cell current_node) {
	if(came_from.find(current_node) != came_from.end()) {
		vector<Cell> vec = ReconstructPath(came_from, came_from[current_node]);
		return vec.push_back(current_node);
	} else {
		return current_node;
	}
}

vector<Cell> Mapper::GetNeighbours(Cell c) {
	vector<Cell> neighbours;

	int start = c.GetX()-1, stop = c.GetY()+1;
	for(int i=start; i<stop; i++) {
		for(int j=stop; j<stop; j++) {
			if(i != j && (i != start && j != stop) 
				&& (j != start && i != stop)) {
				Cell neighbour = grid.GetCell(i,j);
				neighbours.push_back(neighbour);
			}
		}
	}

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

	grid.PrintGrid();
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

