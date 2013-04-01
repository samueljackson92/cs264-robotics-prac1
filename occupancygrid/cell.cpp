/*
	Cell.cpp
	Description: single cell in grid.
	Author:	Samuel Jackson (slj11@aber.ac.uk)
	Date:	1/4/13
*/

#include "cell.h"
#include <cmath>

Cell::Cell() {
	this->x = 0;
	this->y = 0;
	this->visited = false;
}

void Cell::SetValue(double val) {
	value = val;
}

double Cell::GetValue() {
	return value;
}

bool Cell::IsVisited() {
	return visited;
}

void Cell::SetVisited(bool val) {
	this->visited = val;
}

bool Cell::IsDiscovered() {
	return discovered;
}

void Cell::SetDiscovered(bool val) {
	this->discovered = val;
}

double Cell::GetX() {
	return x;
}

double Cell::GetY() {
	return y;
}

void Cell::SetX(double x) {
	this->x = x;
} 

void Cell::SetY(double y) {
	this->y = y;
}

bool Cell::operator==(const Cell& other) const {
	return (x == other.x && y == other.y);
}
bool Cell::operator<(const Cell& other) const {
	return (x+y) < (other.x+other.y);
}