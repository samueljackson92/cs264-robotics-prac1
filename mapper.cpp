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
pp(&robot,0), pc(&robot, &pp, &sp, this) {
	robot.Read();

	double x = pp.GetXPos();
	double y = pp.GetYPos();

	//init start position
	robot_x = x;
	robot_y = y;

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
	vector<Cell*> frontier;
	vector<Cell*> path;

	//The cell we are currently at
	Cell* start = grid.GetCurrentCell();
	start->SetDiscovered(true);
	start->SetVisited(true);

	frontier.push_back(start);

	//0 = NORTH, 1 = EAST, 2 = SOUTH, 3 = WEST
	int direction = 1;
	bool backtracking = false;

	Cell* current = start;
	Cell* nextCell;

	//while there are still unexplored cells.
	while(!frontier.empty()) {
		//find the 4 neighbours

		vector<Cell*> neighbours = GetNeighbours(current);

		//select new direction if required.
		for (int i=0; i <= 4; i++) {
			direction = (direction + i) % 4;
			if(neighbours[direction]->GetValue() == 0 && !neighbours[direction]->IsVisited()) {
				//valid cell to move too.
				backtracking = false;
				nextCell = neighbours[direction];
				nextCell->SetDiscovered(true);
				break;
			} else if(i >= 4) {
				//find cell on frontier
				backtracking = true;
			}
		}

		for(vector<Cell*>::iterator it = neighbours.begin(); it != neighbours.end(); ++it) {
			Cell *neighbour = *it;
			if(neighbour->GetValue() == 0) {
				// cout << *neighbour;
				if(!neighbour->IsDiscovered() && !neighbour->IsVisited()) {
					//add new cell to frontier and mark discovered
					neighbour->SetDiscovered(true);
					frontier.push_back(*it);
				}
			}
		}

		cout << "MOVING FROM/TOO:" << endl;
		cout << *current << endl;
		cout << *nextCell << endl;

		if(backtracking) {
			nextCell = frontier.back();
			frontier.pop_back();

			path = FindPath(current, nextCell);

		} else {		
			MoveToNextCell(*current, *nextCell);
			nextCell->SetVisited(true);
			current = nextCell;	
		}

	}
}

vector<Cell*> Mapper::FindPath(Cell* start, Cell* goal) {
	map<Cell*, int> f_score;
	map<Cell*, int> g_score;
	vector<Cell*> closed_set;
	vector<Cell*> in_queue;
	map<Cell*, Cell*> came_from;
	vector<Cell*> path;

	priority_queue<Cell*, vector<Cell*>, ComparePoints> frontier(ComparePoints(goal, f_score));
	frontier.push(start);

	g_score[start] = 0;
	f_score[start] = g_score[start] + ComparePoints::Distance(start, goal);

	Cell* current;
	while (!frontier.empty()) {
		current = frontier.top();
		if(*current == *goal) {
			return ReconstructPath(came_from, goal);
		}
		closed_set.push_back(current);
		frontier.pop();

		vector<Cell*> neighbours = GetNeighbours(current);
		for (vector<Cell*>::iterator it = neighbours.begin();
			it != neighbours.end(); ++it) {
			Cell* neighbour = *it;

			if(neighbour->GetValue() == 0) {
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

bool Mapper::vec_contains(vector<Cell*> vec, Cell* c) {
	return find (vec.begin(), vec.end(), c) != vec.end();
}

vector<Cell*> Mapper::ReconstructPath(map<Cell*, Cell*> came_from, 
	Cell* current_node) {

	vector<Cell*> vec;
	if(came_from.find(current_node) != came_from.end()) {
		vec = ReconstructPath(came_from, came_from[current_node]);
		vec.push_back(current_node);
		return vec;
	} else {
		vec.push_back(current_node);
		return vec;
	}
}

void Mapper::MoveToNextCell(Cell start, Cell goal) {
	double dx = goal.GetX() - start.GetX();
	double dy = goal.GetY() - start.GetY();

	if(grid.GetCell(start.GetX() +(2*dx), start.GetY() +(2*dy))->GetValue() > 0) {
		dx = (dx != 0) ? dx + (0.1/MAP_SCALE) : dx;
		dy = (dy != 0) ? dy + (0.1/MAP_SCALE) : dy;
	}

	robot_x += dx * MAP_SCALE;
	robot_y += dy * MAP_SCALE;

	pc.MoveToPosition(robot_x, robot_y);
}

vector<Cell*> Mapper::GetNeighbours(Cell* current) {
	vector<Cell*> neighbours;

	neighbours.push_back(grid.GetCell(current->GetX(),current->GetY()+1));
	neighbours.push_back(grid.GetCell(current->GetX()+1,current->GetY()));
	neighbours.push_back(grid.GetCell(current->GetX(),current->GetY()-1));
	neighbours.push_back(grid.GetCell(current->GetX()-1,current->GetY()));

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

