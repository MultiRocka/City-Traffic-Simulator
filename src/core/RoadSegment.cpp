#include "core/RoadSegment.h"

using namespace std;

RoadSegment::RoadSegment()
    : id(-1),
      fromIntersectionId(-1),
      toIntersectionId(-1),
      length(0.0),
      speedLimit(0.0),
      laneCount(1),
      allowAutoDemand(true) {
}

RoadSegment::RoadSegment(int id, int fromIntersectionId, int toIntersectionId, double length, double speedLimit)
    : id(id),
      fromIntersectionId(fromIntersectionId),
      toIntersectionId(toIntersectionId),
      length(length),
      speedLimit(speedLimit),
      laneCount(1),
      allowAutoDemand(true) {
}

int RoadSegment::getId() const {
    return id;
}

int RoadSegment::getFromIntersectionId() const {
    return fromIntersectionId;
}

int RoadSegment::getToIntersectionId() const {
    return toIntersectionId;
}

double RoadSegment::getLength() const {
    return length;
}

double RoadSegment::getSpeedLimit() const {
    return speedLimit;
}

int RoadSegment::getLaneCount() const {
    return laneCount;
}

void RoadSegment::setLaneCount(int newLaneCount) {
    laneCount = newLaneCount;
}

const vector<Vector2D>& RoadSegment::getIntermediatePoints() const {
    return intermediatePoints;
}

void RoadSegment::setIntermediatePoints(const vector<Vector2D>& points) {
    intermediatePoints = points;
}

const string& RoadSegment::getSourceRef() const {
    return sourceRef;
}

void RoadSegment::setSourceRef(const string& ref) {
    sourceRef = ref;
}

bool RoadSegment::getAllowAutoDemand() const {
    return allowAutoDemand;
}

void RoadSegment::setAllowAutoDemand(bool allowed) {
    allowAutoDemand = allowed;
}
