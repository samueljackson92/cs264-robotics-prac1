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
#include <sstream>
#include <iostream>
#include <fstream>

#include "occupancygrid/occupancygrid.h"
#include "probabilitydist/point.h"
#include "probabilitydist/probabilitydist.h"
#include "vectorutils/matrixutils.h"
#include "comparepoints.h"
#include "mapper.h"

using namespace std;
using namespace PlayerCc;

#define ROBOT_ADDRESS "lisa.islnet"

Mapper::Mapper() : robot(ROBOT_ADDRESS), sp(&robot,0), 
pp(&robot,0), pc(&robot, &pp, &sp, this) {
	robot.Read();

	double x = pp.GetXPos();
	double y = pp.GetYPos();

	//init start position
	robot_x = x;
	robot_y = y;

	start_x = x;
	start_y = y;

	map_width = 0;
	map_height = 0;

	grid_width = 0;
	grid_height = 0;

	backtracking = false;
	localized = false;

	threshold = 0;

	//init occupancy gird.
	grid.Init(x, y);
	pp.SetMotorEnable(true);
}

void Mapper::Start(bool localize) {
	StartMapping(localize);
}

void Mapper::StartMapping(bool localize) {
	//frontier to store cells to be explored.
	vector<Cell*> frontier;

	
	//init the starting cell
	Cell* start = grid.GetCurrentCell();
	start->SetDiscovered(true);
	start->SetVisited(true);
	start->SetValue(0);

	//add it to the frontier
	frontier.push_back(start);

	Cell* current = start;
	Cell* nextCell = NULL;

	//0 = NORTH, 1 = EAST, 2 = SOUTH, 3 = WEST
	int direction = 1;
	backtracking = false;

	//if we're not attempting to localize
	if(!localize) {
		localized = true;
	}

	//Gather some inital readings.
	for (int i=0; i<10; i++) {
		UpdateGrid();
	}

	Point myLocation;

	//if we haven't localized yet, attempt to localize
	if(!localized) {
		myLocation = Localize();
	}

	//while there are still unexplored cells.
	//or we haven't yet managed to localize
	while((!frontier.empty() && !localize) || (!frontier.empty() && !localized)) {
		current->SetValue(0);

		//find the 4 neighbours
		threshold = grid.CalculateThreshold()/2;

		vector<Cell*> neighbours = GetNeighbours(current);

		//select new direction if required.
		for (int i=0; i <= 4; i++) {
			direction = (direction + i) % 4;
			if(neighbours[direction]->GetValue() <= threshold && !neighbours[direction]->IsVisited()) {
				//valid cell to move too.
				backtracking = false;
				nextCell = neighbours[direction];
				nextCell->SetValue(0);
				nextCell->SetDiscovered(true);
				break;
			} else if(i >= 4) {
				//find cell on frontier
				backtracking = true;
			}
		}

		//add neighbours to frontier for later examination
		for(vector<Cell*>::iterator it = neighbours.begin(); it != neighbours.end(); ++it) {
			Cell *neighbour = *it;
			if(neighbour->GetValue() <= threshold) {
				// cout << *neighbour;
				if(!neighbour->IsDiscovered() && !neighbour->IsVisited()) {
					//add new cell to frontier and mark discovered
					neighbour->SetValue(0);
					neighbour->SetDiscovered(true);
					frontier.push_back(*it);
				}
			}
		}

		cout << "MOVING FROM/TOO:" << endl;
		cout << *current << endl;
		cout << *nextCell << endl;

		//if we are heading backwards
		if(backtracking) {
			//get next non visited cell on frontier.
			while(!frontier.empty()) {
				nextCell = frontier.back();
				frontier.pop_back();

				if(!nextCell->IsVisited() && nextCell->GetValue() <= threshold)  {
					//find path from this square to us.
					MoveToCell(current, nextCell);
					current = nextCell;
					break;
				}
			}
		} else {
			//just move to the next cell.
			MoveToNextCell(*current, *nextCell);
			nextCell->SetVisited(true);
			current = nextCell;
		}

		//if we're not localized, attempt to
		if(!localized) {
			myLocation = Localize();
		}

	}

	//if we we're trying to localize, move to a hiding spot
	if(localize && localized) {
		cout << "Moving to hiding spot!!" << endl;
		grid.SetRobotPosition(myLocation.GetX(), myLocation.GetY());
		Hide();
	}
}

void Mapper::Hide() {
	Point hidingSpot;
	vector<vector<double> > rawMap = ml.LoadMap(mapName);
	hidingSpot = ml.FindHidingSpots(mapData);

	grid.LoadValues(rawMap);
	
	threshold = grid.CalculateThreshold();

	Cell* current = grid.GetCurrentCell();
	Cell* goal = grid.GetCell(hidingSpot.GetX(), hidingSpot.GetY());
	MoveToCell(current, goal);
}


vector<Cell*> Mapper::FindPath(Cell* start, Cell* goal) {
	//define map of f scores, g scores
	map<Cell*, int> f_score;
	map<Cell*, int> g_score;

	//close and open sets
	vector<Cell*> closed_set;
	vector<Cell*> in_queue;

	//map for paths reoonstruction
	map<Cell*, Cell*> came_from;

	//finished path to follow
	vector<Cell*> path;

	//Priority queue of cells in frontier
	//ordered shortest distance to goal first
	priority_queue<Cell*, vector<Cell*>, ComparePoints> frontier(ComparePoints(goal, f_score));

	//add where we are to frontier
	frontier.push(start);

	//Init start cell scores
	g_score[start] = 0;
	f_score[start] = g_score[start] + ComparePoints::Distance(start, goal);

	Cell* current;

	//while there are still potential pathways
	while (!frontier.empty()) {
		//get next best looking cell
		current = frontier.top();

		//if its the goal, we're done!
		//Reconstruct the path and return it
		if(*current == *goal) {
			return ReconstructPath(came_from, goal);
		}

		//else add it to closed set and examine it
		closed_set.push_back(current);
		frontier.pop();

		//get the neighbours of the current cell
		vector<Cell*> neighbours = GetNeighbours(current);
		for (vector<Cell*>::iterator it = neighbours.begin();
			it != neighbours.end(); ++it) {
			Cell* neighbour = *it;

			//check if the neighbour is a valid cell
			if(neighbour->GetValue() <= threshold) {
				int tentative_g_score = g_score[current] + 1;

				//if we've already examined it and it's score isn't any better just forget
				//and contiune with next neighbour
				if(vec_contains(closed_set, neighbour)) {
					if(tentative_g_score >= g_score[neighbour]) {
						continue;
					}
				}

				//if its not already in the queue to be looked at
				//or has a shorter path score
				if(!vec_contains(in_queue, neighbour)
					|| tentative_g_score < g_score[(neighbour)]) {
					came_from[neighbour] = current;

					//compute the new scores for this neighbour
					g_score[neighbour] = tentative_g_score;
					f_score[neighbour] = g_score[neighbour] + ComparePoints::Distance(neighbour, goal);

					//if it's not already queued, queue it!
					if(!vec_contains(in_queue, neighbour)) {
						frontier.push(neighbour);
						in_queue.push_back(neighbour);
					}
				}
			}
		}
	}

	//return the empty path if no path found
	return path;
}

void Mapper::MoveToCell(Cell* start, Cell* goal) {
	vector<Cell*> path;

	Cell* current = start;
	Cell* nextCell;

	path = FindPath(start, goal);

	cout << "Path to follow:" << endl;
	for (int i=0; i<path.size(); i++) {
		cout << *path[i];
	}

	for(int i = 1; i < path.size(); i++) {
		nextCell = path[i];
		MoveToNextCell(*current, *nextCell);
		nextCell->SetVisited(true);
		current = nextCell;
	}
}

bool Mapper::vec_contains(vector<Cell*> vec, Cell* c) {
	return find (vec.begin(), vec.end(), c) != vec.end();
}

vector<Cell*> Mapper::ReconstructPath(map<Cell*, Cell*> came_from, 
	Cell* current_node) {

	vector<Cell*> vec;
	//if current node in the map
	if(came_from.find(current_node) != came_from.end()) {
		//recursively build the map
		vec = ReconstructPath(came_from, came_from[current_node]);
		//add this node to the end of the path
		vec.push_back(current_node);
		return vec;
	} else {
		//just add it to the end of the path
		vec.push_back(current_node);
		return vec;
	}
}

void Mapper::MoveToNextCell(Cell start, Cell goal) {
	double dx = goal.GetX() - start.GetX();
	double dy = goal.GetY() - start.GetY();

	dx *= MAP_SCALE;
	dy *= MAP_SCALE;

	robot_x += dx;
	robot_y += dy;

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

	if(!backtracking) {
		grid.SensorUpdate(sp[3], dtor(angle+10));
		grid.SensorUpdate(sp[4], dtor(angle-10));

		//side sensors
		grid.SensorUpdate(sp[0], dtor(angle+90));
		grid.SensorUpdate(sp[15], dtor(angle+90));
		grid.SensorUpdate(sp[7], dtor(angle-90));
		grid.SensorUpdate(sp[8], dtor(angle-90));

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

		cout << endl;
		//grid.PrintGrid();
		cout << endl;
	}
}

void Mapper::RandomWander() {
	double speed = 0;
	double turnrate = 0;

	robot.Read();

	if(sp[3] < 0.6 || sp[4] < 0.6) {
		int direction = (sp[3]<sp[4]) ? -1 : 1;
	} else if((sp[0] + sp[1]) < (sp[6] + sp[7])) {
		turnrate = dtor(-15); // turn 20 degrees per second
	} else {
		turnrate = dtor(15);
	}

	if(sp[3] < 0.6 || sp[4] < 0.6) {
		speed = 0;
	} else {
		speed = 0.150;
	}

	UpdateGrid();

	pp.SetSpeed(speed, turnrate);
}

Point Mapper::Localize() {
	Point me;
	me.SetX(-1);
	me.SetY(-1);

	//only attempt to localize when the grid expands some more
	//(or at the start)
	if(grid_height != grid.GetGridHeight() 
		|| grid_width != grid.GetGridWidth()) {

		int matches = 0;

		//loop over exisitng map
		for(int i =0; i < map_height - grid.GetGridHeight(); i++) {
			for (int j =0; j < map_width - grid.GetGridWidth(); j++) {

				int count = 0;
				//at each point, check if our grid matches
				for(int k = 0; k < grid.GetGridHeight(); k++) {
					for (int l = 0; l < grid.GetGridWidth(); l++) {
						double mval = mapData[i+k][j+l];
						double gval = grid.GetCell(l,k)->GetValue();

						//check if cells match
						if((gval == 0 && mval == 0) || (gval > 0 && mval > 0)) {
							count++;

						//count it as matched if it's unvisited
						} else if(gval < 0) {
							count++;
						}
					}
				}

				//if we match every square
				if(count == (grid.GetGridWidth()*grid.GetGridHeight())) {
					Cell* loc = grid.GetCurrentCell();
					me.SetX(loc->GetX()+j);
					me.SetY(loc->GetY()+i);
					matches++;
				}
			}
		}

		grid_width = grid.GetGridWidth();
		grid_height = grid.GetGridHeight();

		cout << matches << endl;

		//if we have exactly one match we're localized
		if(matches == 1) {
			localized = true;
		}
	}

	cout << *grid.GetCurrentCell();
	cout << "Point x: " << me.GetX() << " y: " << me.GetY() << endl;

	//return the point we think we're at
	return me;
}

vector<double> Mapper::GetMapNeighbours(int x, int y) {
	vector<double> neighbours;

	if(x-1 >= 0) {
		neighbours.push_back(mapData[y][x-1]);
	}
	if(x+1 < map_width) {
		neighbours.push_back(mapData[y][x+1]);
	}
	if(y-1 >= 0) {
		neighbours.push_back(mapData[y-1][x]);
	}
	if(y+1 < map_height) {
		neighbours.push_back(mapData[y+1][x]);
	}

	return neighbours;
}

void Mapper::LoadMapData(std::string filename) {
	mapName = filename;
	vector<vector<double> > rawMap = ml.LoadMap(filename);
	vector<int> vec;
	
	for(int i = 0; i < rawMap.size(); i++) {
		for(int j = 0; j < rawMap[i].size(); j++) {
			double val = rawMap[i][j];
			val = (val < 0) ? 0 : val;
			vec.push_back(val);
		}
	}

	threshold = MatrixUtils::Otsu(vec);
	mapData = ml.ConvertToBinaryMap(rawMap, threshold);
	map_height = mapData.size();

	if (map_height > 0) {
		map_width = mapData[0].size();	
	}
	
	for (int i = 0; i< mapData.size(); i++) {
	  for(int j=0; j<mapData[i].size();j++) {
	    cout << mapData[i][j];
	  }
	  cout << endl;
	}
}

void Mapper::SaveMap(std::string filename) {
	grid.WriteGrid(filename);
}

void Mapper::FindRobot(std::string o, std::string n) {
	mapData = ml.ConvertToBinaryMap(ml.LoadMap(n), threshold);
	vector<vector<int> > oldMap = ml.ConvertToBinaryMap(ml.LoadMap(o), threshold);
	vector<Point> pos = ml.FindNewCells(oldMap, mapData);

	cout << "Found robot positions at: " << endl;
	for (int i = 0; i < pos.size(); i++) {
		cout << " - x: " << pos[i].GetX() << " y: " << pos[i].GetY() << endl;
	}
}

Mapper::~Mapper(){
    grid.PrintFinalGrid();
}
