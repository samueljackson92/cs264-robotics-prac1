
#include <vector>
#include <string>

#include "../probabilitydist/point.h"
#include "maploader.h"

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

std::vector<std::vector<int> > MapLoader::ConvertToBinaryMap(
	const std::vector<std::vector<double> >& mapData, double threshold) {

	std::vector<std::vector<int> > newMap;
	newMap.resize(mapData.size());
	for(int i =0; i < mapData.size(); i++) {
		for(int j =0; j < mapData[i].size(); j++) {
			int val;
			if(mapData[i][j] >= 0) {
				val = (mapData[i][j] > threshold) ? 1 : 0;
			} else {
				val = -1;
			}

			newMap[i].push_back(val);
		}
	}

	return newMap;
}

Point MapLoader::FindHidingSpots(const std::vector<std::vector<int> >& mapData) {
	int minScore = 16;
	Point p;
	for (int i =0; i< mapData.size(); i++) {
		for (int j = 0; j< mapData[i].size(); j++) {
			
			int score = 0;

			//if valid square
			if(mapData[i][j] == 0) {
				for(int k = j; k <= j+4; k++) {
					if(j+4 >= mapData[i].size()) {
						break;
					} else if (mapData[i][k] <= 0) {
						score++;
					} else {
						break;
					}
				}

				for(int k = i; k <= i+4; k++) {
					if(i+4 >= mapData.size()) {
						break;
					} else if (mapData[k][j] <= 0) {
						score++;
					} else {
						break;
					}
				}

				for(int k = j; k >= j-4; k--) {
					if(j-4 < 0) {
						break;
					} else if (mapData[i][k] <= 0) {
						score++;
					} else {
						break;
					}
				}

				for(int k = i; k >= i-4; k--) {
					if(i-4 < 0) {
						break;
					} else if (mapData[k][j] <= 0) {
						score++;
					} else {
						break;
					}
				}

				if(score <= minScore) {
					minScore = score;
					p.SetX(j);
					p.SetY(i);
				}
			}
		}
	}

	return p;
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