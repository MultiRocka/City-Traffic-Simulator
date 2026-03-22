#pragma once

using namespace std;

class PedestrianCrossing {
public:
    PedestrianCrossing();
    PedestrianCrossing(int id, int roadSegmentId, double positionOnRoad, double inactiveDuration, double activeDuration, bool startActive);

    void update(double dt);

    int getId() const;
    int getRoadSegmentId() const;
    double getPositionOnRoad() const;
    bool isActive() const;
    double getTimer() const;

private:
    int id;
    int roadSegmentId;
    double positionOnRoad;
    double inactiveDuration;
    double activeDuration;
    double timer;
    bool active;
};
