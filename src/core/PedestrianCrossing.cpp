#include "core/PedestrianCrossing.h"

using namespace std;

PedestrianCrossing::PedestrianCrossing()
    : id(-1),
      roadSegmentId(-1),
      positionOnRoad(0.0),
      inactiveDuration(5.0),
      activeDuration(3.0),
      timer(0.0),
      active(false),
      mode(CrossingMode::Timed),
      walkSpeed(1.4),
      entityOccupying(false) {
}

PedestrianCrossing::PedestrianCrossing(
    int id,
    int roadSegmentId,
    double positionOnRoad,
    double inactiveDuration,
    double activeDuration,
    bool startActive
)
    : id(id),
      roadSegmentId(roadSegmentId),
      positionOnRoad(positionOnRoad),
      inactiveDuration(inactiveDuration),
      activeDuration(activeDuration),
      timer(0.0),
      active(startActive),
      mode(CrossingMode::Timed),
      walkSpeed(1.4),
      entityOccupying(false) {
}

void PedestrianCrossing::update(double dt) {
    if (mode != CrossingMode::Timed) {
        // Spawn cadence uses inactiveDuration via PedestrianManager; isActive is entity-driven.
        active = entityOccupying;
        return;
    }

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
    if (mode == CrossingMode::Timed) {
        return active;
    }
    return entityOccupying;
}

double PedestrianCrossing::getTimer() const {
    return timer;
}

double PedestrianCrossing::getInactiveDuration() const {
    return inactiveDuration;
}

double PedestrianCrossing::getActiveDuration() const {
    return activeDuration;
}

CrossingMode PedestrianCrossing::getMode() const {
    return mode;
}

double PedestrianCrossing::getWalkSpeed() const {
    return walkSpeed;
}

void PedestrianCrossing::setMode(CrossingMode newMode) {
    mode = newMode;
    if (mode != CrossingMode::Timed) {
        active = entityOccupying;
        timer = 0.0;
    }
}

void PedestrianCrossing::setWalkSpeed(double metersPerSecond) {
    walkSpeed = metersPerSecond > 0.05 ? metersPerSecond : 0.05;
}

void PedestrianCrossing::setEntityOccupying(bool occupying) {
    entityOccupying = occupying;
    if (mode != CrossingMode::Timed) {
        active = occupying;
    }
}
