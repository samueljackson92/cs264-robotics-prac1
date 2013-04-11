
#include <iostream>

#include "probabilitydist/point.h"
#include "mapper.h"

using namespace std;

int main(int argc, char *argv[])
{
	Point p;
	Mapper m;
	p = m.Localize("good_output.csv");

	cout << p.GetX() << "," << p.GetY() << endl;
}