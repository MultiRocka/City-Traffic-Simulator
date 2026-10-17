#pragma once

#include <string>
#include <vector>
#include "core/Vector2D.h"

using namespace std;

// Units in core: length in meters, speedLimit in m/s.
// One RoadSegment is one directed edge (one way). Opposite travel = a separate RoadSegment.
// laneCount is lanes for THIS direction only — never treat opposite directions as lanes.
class RoadSegment {
public:
    RoadSegment();
    RoadSegment(int id, int fromIntersectionId, int toIntersectionId, double length, double speedLimit);

    int getId() const;
    int getFromIntersectionId() const;
    int getToIntersectionId() const;
    double getLength() const;
    double getSpeedLimit() const;

    int getLaneCount() const;
    void setLaneCount(int newLaneCount);

    const vector<Vector2D>& getIntermediatePoints() const;
    void setIntermediatePoints(const vector<Vector2D>& points);

    const string& getSourceRef() const;
    void setSourceRef(const string& ref);

    bool getAllowAutoDemand() const;
    void setAllowAutoDemand(bool allowed);

private:
    int id;
    int fromIntersectionId;
    int toIntersectionId;
    double length;
    double speedLimit;
    int laneCount;
    vector<Vector2D> intermediatePoints;
    string sourceRef;
    bool allowAutoDemand;
};
