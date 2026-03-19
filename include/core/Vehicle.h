#pragma once

#include <vector>

using namespace std;

class Vehicle {
public:
    Vehicle();
    Vehicle(int id, double maxSpeed);

    int getId() const;
    double getPositionOnRoad() const;
    double getSpeed() const;
    double getMaxSpeed() const;
    bool isStopped() const;
    bool isFinished() const;
    int getCurrentRoadSegmentId() const;
    int getNextRoadSegmentId() const;
    int getCurrentRoadIndex() const;

    void setSpeed(double newSpeed);
    void setRoute(const vector<int>& newRoute);

    void update(double dt, double roadLength, double maxAllowedPosition);
    bool moveToNextRoad();
    void finish();

private:
    int id;
    vector<int> route;
    int currentRoadIndex;
    double positionOnRoad;
    double speed;
    double maxSpeed;
    bool stopped;
    bool finished;
};
