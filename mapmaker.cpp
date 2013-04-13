
#include <string>
#include <iostream>
#include "mapper.h"

using namespace std;

int main(int argc, char *argv[])
{
	if(argc == 2) {
		string filename(argv[1]);
		Mapper mapper;

		mapper.Start(false);
		mapper.SaveMap(filename);
		
	} else {
		cout << "Please supply the filename of the map to save." << endl;
	}
}