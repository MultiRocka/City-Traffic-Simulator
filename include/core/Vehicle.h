#pragma once

using namespace std;

class Vehicle {
public:
    Vehicle();
    Vehicle(int id, int roadSegmentId, double positionOnRoad, double speed);

    int getId() const;
    int getRoadSegmentId() const;
    double getPositionOnRoad() const;
    double getSpeed() const;
    bool isStopped() const;

    void setRoadSegmentId(int newRoadSegmentId);
    void setPositionOnRoad(double newPositionOnRoad);
    void setSpeed(double newSpeed);
    void stop();

    void update(double dt, double roadLength);

private:
    int id;
    int roadSegmentId;
    double positionOnRoad;
    double speed;
    bool stopped;
};
