#pragma once

#include "core/City.h"
#include "ui/RenderSnapshot.h"

using namespace std;

class CityRenderer {
public:
    void buildRoads(const City& city, RenderSnapshot& snapshot) const;
    void buildIntersections(const City& city, RenderSnapshot& snapshot) const;
};
