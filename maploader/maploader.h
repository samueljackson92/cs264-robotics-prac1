#ifndef __MAPLOADER_H_INCLUDED__
#define __MAPLOADER_H_INCLUDED__

#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>

#include "../probabilitydist/point.h"

class MapLoader {
	public:
		std::vector<std::vector<int> > LoadBinaryMap(std::string filename);
		std::vector<std::vector<double> > LoadMap(std::string filename);
		std::vector<Point> FindNewCells(std::vector<std::vector<int> > map1, 
											std::vector<std::vector<int> >map2);
		std::vector<std::vector<int> > ConvertToBinaryMap();
		double StringToDouble(const std::string& s );
};

#endif