
#include <string>
#include <iostream>
#include <vector>

#include "probabilitydist/point.h"
#include "mapper.h"

using namespace std;

int main(int argc, char *argv[])
{
	if(argc == 3) {
		string old_map(argv[1]);
		string new_map(argv[2]);
		Mapper mapper;
		mapper.Start(false);
		mapper.SaveMap(new_map);
		mapper.FindRobot(old_map, new_map);

	} else {
		cout << "Please supply the filename of the map to load." << endl;
		cout << "Please supply the filename of the map to save." << endl;
	}
}