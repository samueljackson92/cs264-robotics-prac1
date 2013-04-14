
#include <string>
#include <iostream>

#include "probabilitydist/point.h"
#include "mapper.h"

using namespace std;

int main(int argc, char *argv[])
{
	if(argc == 2) {
		string filename(argv[1]);
		Mapper mapper;

		mapper.LoadMapData(filename);
		mapper.Start(true);

	} else {
		cout << "Please supply the filename of the map to load." << endl;
	}
}