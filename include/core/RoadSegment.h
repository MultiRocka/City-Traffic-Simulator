#pragma once

class RoadSegment {
public:
    RoadSegment();
    RoadSegment(int id, int fromIntersectionId, int toIntersectionId, double length, double speedLimit);

    int getId() const;
    int getFromIntersectionId() const;
    int getToIntersectionId() const;
    double getLength() const;
    double getSpeedLimit() const;

private:
    int id;
    int fromIntersectionId;
    int toIntersectionId;
    double length;
    double speedLimit;
};
