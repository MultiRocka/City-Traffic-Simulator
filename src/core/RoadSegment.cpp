#include "core/RoadSegment.h"

using namespace std;

RoadSegment::RoadSegment() : id(-1), fromIntersectionId(-1), toIntersectionId(-1), length(0.0), speedLimit(0.0) {
}

RoadSegment::RoadSegment(int id, int fromIntersectionId, int toIntersectionId, double length, double speedLimit)
    : id(id),
      fromIntersectionId(fromIntersectionId),
      toIntersectionId(toIntersectionId),
      length(length),
      speedLimit(speedLimit) {
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
