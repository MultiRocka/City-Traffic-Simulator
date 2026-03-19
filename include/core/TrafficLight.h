#pragma once

using namespace std;

class TrafficLight {
public:
    TrafficLight();
    TrafficLight(int id, int roadSegmentId, double greenDuration, double redDuration, bool startGreen);

    void update(double dt);

    int getId() const;
    int getRoadSegmentId() const;
    bool isGreen() const;
    bool isRed() const;
    double getTimer() const;

private:
    int id;
    int roadSegmentId;
    double greenDuration;
    double redDuration;
    double timer;
    bool green;
};
