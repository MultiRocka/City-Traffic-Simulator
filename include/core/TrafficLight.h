#pragma once

using namespace std;

class TrafficLight {
public:
    TrafficLight();
    TrafficLight(int id, int roadSegmentId, double greenDuration, double redDuration, bool startGreen);
    TrafficLight(int id, int roadSegmentId, int intersectionId, bool phaseControlled);

    void update(double dt);
    void setGreen(bool isGreenState);
    void setPhaseControlled(bool controlled);

    int getId() const;
    int getRoadSegmentId() const;
    int getIntersectionId() const;
    bool isGreen() const;
    bool isRed() const;
    bool isPhaseControlled() const;
    double getTimer() const;

private:
    int id;
    int roadSegmentId;
    int intersectionId;
    double greenDuration;
    double redDuration;
    double timer;
    bool green;
    bool phaseControlled;
};
