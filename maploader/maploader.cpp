
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

std::vector<Point> MapLoader::FindNewCells(const std::vector<std::vector<int> >& map1, 
	const std::vector<std::vector<int> >& map2) {

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
	const std::vector<std::vector<double> >& mapData, double threshold) {
	std::vector<std::vector<int> > newMap;

	for(int i =0; i < mapData.size(); i++) {
		for(int j =0; j < mapData.size(); j++) {
			newMap[i][j] = (mapData[i][j] > threshold) ? 1 : 0;
		}
	}

	return newMap;
}

std::vector<Point> MapLoader::FindHidingSpots(const std::vector<std::vector<int> >& mapData, int threshold) {
	std::vector<Point> spots;
	for (int i =0; i<mapData.size(); i++) {
		for (int j = 0; j<mapData[i].size(); j++) {
			//if valid square
			if(mapData[i][j] <= threshold) {
				std::vector<double> neighbours = GetMapNeighbours(mapData,j,i);
				int count = 0;

				//has four neighbours surrounding it
				if(neighbours.size() == 4) {
					//iterate over neighours, count walls
					for(std::vector<double>::iterator it = neighbours.begin(); it != neighbours.end(); ++it) {
						if (*it > threshold) {
							count++;
						}
					}
				}

				//if it has three walls, good hiding spot
				if (count == 3) {
					Point p(j,i);
					spots.push_back(p);
				}
			}
		}
	}

	return spots;
}

std::vector<double> MapLoader::GetMapNeighbours(const std::vector<std::vector<int> >& mapData, int x, int y) {
	std::vector<double> neighbours;

	if(x-1 >= 0) {
		neighbours.push_back(mapData[y][x-1]);
	}
	if(x+1 < mapData[0].size()) {
		neighbours.push_back(mapData[y][x+1]);
	}
	if(y-1 >= 0) {
		neighbours.push_back(mapData[y-1][x]);
	}
	if(y-1 < mapData.size()) {
		neighbours.push_back(mapData[y+1][x]);
	}

	return neighbours;
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