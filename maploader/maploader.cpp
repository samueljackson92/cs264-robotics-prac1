
#include <vector>
#include <string>

#include "../probabilitydist/point.h"
#include "maploader.h"

std::vector<std::vector<int> > MapLoader::LoadBinaryMap(std::string filename) {
	std::ifstream f;
	std::vector<int> vec;
	std::vector<std::vector<int> > mapData;
	std::string line;
	std::string field;

	f.open(filename.c_str());

   	while (getline(f,line)) {
        vec.clear();
        std::stringstream ss(line);

        while (getline(ss,field,',')) {
        	double num = StringToDouble(field);
        	int val = (num > 0) ? 1 : 0;
            vec.push_back(val);
        }
        mapData.push_back(vec);
    }

	f.close();

	return mapData;
}

std::vector<std::vector<double> > MapLoader::LoadMap(std::string filename) {
	std::ifstream f;
	std::vector<double> vec;
	std::vector<std::vector<double> > mapData;
	std::string line;
	std::string field;

	f.open(filename.c_str());

   	while (getline(f,line)) {
        vec.clear();
        std::stringstream ss(line);

        while (getline(ss,field,',')) {
            vec.push_back(StringToDouble(field));
        }
        mapData.push_back(vec);
    }

	f.close();

	return mapData;
}

std::vector<Point> MapLoader::FindNewCells(std::vector<std::vector<int> > map1, 
	std::vector<std::vector<int> >map2) {

	std::vector<Point> diffPoints;
	for(int i=0; i< map1.size(); i++) {
		for (int j=0; j < map1[i].size(); j++) {
			if(map1[i][j] != map2[i][j]) {
				Point p(j,i);
				diffPoints.push_back(p);
			}
		}
	}

	return diffPoints;
}

std::vector<std::vector<int> > ConvertToBinaryMap(
	std::vector<std::vector<double> > mapData, double threshold) {
	std::vector<std::vector<int> > newMap;

	for(int i =0; i < mapData.size(); i++) {
		for(int j =0; j < mapData.size(); j++) {
			newMap[i][j] = (mapData[i][j] > threshold) ? 1 : 0;
		}
	}

	return newMap;
}

double MapLoader::StringToDouble(const std::string& s )
{
	std::istringstream i(s);
	double x;
	if (!(i >> x)) {
		return 0;
	}
	return x;
}