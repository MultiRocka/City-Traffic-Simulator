#include "core/PedestrianCrossing.h"

using namespace std;

PedestrianCrossing::PedestrianCrossing()
    : id(-1), roadSegmentId(-1), positionOnRoad(0.0), inactiveDuration(5.0), activeDuration(3.0), timer(0.0), active(false) {
}

PedestrianCrossing::PedestrianCrossing(int id, int roadSegmentId, double positionOnRoad, double inactiveDuration, double activeDuration, bool startActive)
    : id(id),
      roadSegmentId(roadSegmentId),
      positionOnRoad(positionOnRoad),
      inactiveDuration(inactiveDuration),
      activeDuration(activeDuration),
      timer(0.0),
      active(startActive) {
}

void PedestrianCrossing::update(double dt) {
    timer += dt;

    if (active && timer >= activeDuration) {
        active = false;
        timer = 0.0;
    } else if (!active && timer >= inactiveDuration) {
        active = true;
        timer = 0.0;
    }
}

int PedestrianCrossing::getId() const {
    return id;
}

int PedestrianCrossing::getRoadSegmentId() const {
    return roadSegmentId;
}

double PedestrianCrossing::getPositionOnRoad() const {
    return positionOnRoad;
}

bool PedestrianCrossing::isActive() const {
    return active;
}

double PedestrianCrossing::getTimer() const {
    return timer;
}
