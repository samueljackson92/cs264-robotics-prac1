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
	//create list of cells on the frointer
	vector<Cell*> frontier;

	//gather some intial readings
	for (int i=0; i < 10; i++) {
		UpdateGrid();
	}

	//get where we currently are.
	Cell& c = grid.GetCurrentCell();
	Cell* current = &c;

	current->SetVisited(true);
	current->SetDiscovered(true);

	//find adjacent cells
	vector<Cell*> neighbours = GetNeighbours(*current);
	for (vector<Cell*>::iterator it = neighbours.begin();
		it != neighbours.end(); ++it) {
		cout << **it;
		Cell* neighbour = *it;
		if(neighbour->GetValue() == 0) {
			//check if we've already found this neighbour
			if(!neighbour->IsVisited() && !neighbour->IsDiscovered()) {
				//add it to our frontier
				neighbour->SetDiscovered(true);
				frontier.push_back(neighbour);
			}
		}
	}

	
	//while there are unexplore cells
	while (!frontier.empty()) {

		//get next item from frontier
		current = frontier.back();
		frontier.pop_back();

		//Mark as being visited
		current->SetVisited(true);

		Cell& ourPos = grid.GetCurrentCell();

		//move to cell
		MoveToNextCell(ourPos, *current);

		//find adjacent cells
		vector<Cell*> neighbours = GetNeighbours(*current);
		for (vector<Cell*>::iterator it = neighbours.begin();
			it != neighbours.end(); ++it) {
			cout << **it;
			Cell* neighbour = *it;
			if(neighbour->GetValue() == 0) {
				//check if we've already found this neighbour
				if(!neighbour->IsVisited() && !neighbour->IsDiscovered()) {
					//add it to our frontier
					neighbour->SetDiscovered(true);
					frontier.push_back(neighbour);
				}
			}
		}


	}

	cout << "Finished Mapping!" << endl;

}

void Mapper::MoveToNextCell(Cell start, Cell goal) {

	cout << start.GetX() << ", " << start.GetY() << endl;	
	cout << goal.GetX() << ", " << goal.GetY() << endl;

	//find optimal path from current position to next square.
	vector<Cell> path = FindPath(start, goal);
	cout << "Path Size: " << path.size() <<endl;
	//move along path util we reach next square.
	Cell previous = start;
	for (vector<Cell>::iterator it = path.begin(); it != path.end(); ++it) {
		double dx = (it->GetX()-previous.GetX()) * 0.6;
		double dy = (it->GetY()-previous.GetY()) * 0.6;
		
		cout << "X: " << pp.GetXPos()+ dx << "Y: " << pp.GetYPos() + dy << endl;
		pc.MoveToPosition(pp.GetXPos() + dx, pp.GetYPos() + dy);
		previous = *it;
	}
}

vector<Cell> Mapper::FindPath(Cell start, Cell goal) {
	map<Cell, int> f_score;
	map<Cell, int> g_score;
	vector<Cell> closed_set;
	vector<Cell> in_queue;
	map<Cell, Cell> came_from;
	vector<Cell> path;

	priority_queue<Cell, vector<Cell>, ComparePoints> frontier(ComparePoints(goal, f_score));
	frontier.push(start);

	g_score[start] = 0;
	f_score[start] = g_score[start] + ComparePoints::Distance(start, goal);
	
	Cell current;
	while (!frontier.empty()) {
		current = frontier.top();
		
		if(current == goal) {
			return ReconstructPath(came_from, start, goal);
		}

		closed_set.push_back(current);
		frontier.pop();

		vector<Cell*> neighbours = GetNeighbours(current);

		for (vector<Cell*>::iterator it = neighbours.begin();
			it != neighbours.end(); ++it) {
			Cell neighbour = **it;

			if(neighbour.GetValue() == 0) {

				int tentative_g_score = g_score[current] + 1;
				
				if(vec_contains(closed_set, neighbour)) {
					if(tentative_g_score >= g_score[neighbour]) {
						continue;
					}
				}

				if(!vec_contains(in_queue, neighbour)
					|| tentative_g_score < g_score[(neighbour)]) {
					came_from[neighbour] = current;
					g_score[neighbour] = tentative_g_score;
					f_score[neighbour] = g_score[neighbour] + ComparePoints::Distance(neighbour, goal);
				
					if(!vec_contains(in_queue, neighbour)) {
						frontier.push(neighbour);
						in_queue.push_back(neighbour);
					}
				}
			}
		}
	}

	return path;
}

bool Mapper::vec_contains(vector<Cell> vec, Cell c) {
	return find (vec.begin(), vec.end(), c) != vec.end();
}

vector<Cell> Mapper::ReconstructPath(map<Cell, Cell> came_from, 
	Cell start, Cell current_node) {

	vector<Cell> vec;
	if(current_node == start) {
		return vec;
	} else {
		vec = ReconstructPath(came_from, start, came_from[current_node]);
		vec.push_back(current_node);
		return vec;
	}
}

std::vector<Cell*> Mapper::GetNeighbours(Cell current) {
	vector<Cell*> neighbours;

	int x = current.GetX();
	int y = current.GetY();


	Cell& c = grid.GetCell(x-1,y);
	neighbours.push_back(&c);
	c = grid.GetCell(x+1,y);
	neighbours.push_back(&c);
	c = grid.GetCell(x,y-1);
	neighbours.push_back(&c);
	c = grid.GetCell(x,y+1);
	neighbours.push_back(&c);
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

	//grid.PrintGrid();
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

