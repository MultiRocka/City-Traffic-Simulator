#pragma once

#include <vector>
#include "core/City.h"

using namespace std;

class PathFinder {
public:
    vector<int> findShortestRoute(const City& city, int startRoadId, int endRoadId) const;
    vector<int> findRouteThroughIntersections(const City& city, int startRoadId, int endIntersectionId) const;
};
