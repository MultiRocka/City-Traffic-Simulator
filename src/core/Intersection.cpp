#include "core/Intersection.h"

using namespace std;

Intersection::Intersection() : id(-1), position(0.0, 0.0) {
}

Intersection::Intersection(int id, const Vector2D& position) : id(id), position(position) {
}

int Intersection::getId() const {
    return id;
}

Vector2D Intersection::getPosition() const {
    return position;
}

void Intersection::addConnectedRoadId(int roadId) {
    connectedRoadIds.push_back(roadId);
}

const vector<int>& Intersection::getConnectedRoadIds() const {
    return connectedRoadIds;
}
