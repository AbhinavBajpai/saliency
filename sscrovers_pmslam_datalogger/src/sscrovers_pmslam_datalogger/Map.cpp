#include "Map.h"

Map::Map()
{}

Map::~Map()
{}

void Map::addLandmark(Landmark lm)
{
	map.push_back(lm);
}
