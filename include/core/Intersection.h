#pragma once

#include <vector>
#include "core/Vector2D.h"

using namespace std;

class Intersection {
public:
    Intersection();
    Intersection(int id, const Vector2D& position);

    int getId() const;
    Vector2D getPosition() const;

    void addConnectedRoadId(int roadId);
    const vector<int>& getConnectedRoadIds() const;

private:
    int id;
    Vector2D position;
    vector<int> connectedRoadIds;
};
