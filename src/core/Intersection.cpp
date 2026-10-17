#include "core/Intersection.h"

using namespace std;

Intersection::Intersection()
    : id(-1), position(0.0, 0.0), priorityMode(PriorityMode::None) {
}

Intersection::Intersection(int id, const Vector2D& position)
    : id(id), position(position), priorityMode(PriorityMode::None) {
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

const string& Intersection::getSourceRef() const {
    return sourceRef;
}

void Intersection::setSourceRef(const string& ref) {
    sourceRef = ref;
}

PriorityMode Intersection::getPriorityMode() const {
    return priorityMode;
}

void Intersection::setPriorityMode(PriorityMode mode) {
    priorityMode = mode;
}

const vector<int>& Intersection::getPriorityRoadIds() const {
    return priorityRoadIds;
}

void Intersection::setPriorityRoadIds(const vector<int>& roadIds) {
    priorityRoadIds = roadIds;
}
