#include "core/Vehicle.h"
#include <algorithm>

using namespace std;

Vehicle::Vehicle()
    : id(-1), currentRoadIndex(0), positionOnRoad(0.0), speed(0.0), maxSpeed(0.0), stopped(false), finished(false) {
}

Vehicle::Vehicle(int id, double maxSpeed)
    : id(id), currentRoadIndex(0), positionOnRoad(0.0), speed(maxSpeed), maxSpeed(maxSpeed), stopped(false), finished(false) {
}

int Vehicle::getId() const {
    return id;
}

double Vehicle::getPositionOnRoad() const {
    return positionOnRoad;
}

double Vehicle::getSpeed() const {
    return speed;
}

double Vehicle::getMaxSpeed() const {
    return maxSpeed;
}

bool Vehicle::isStopped() const {
    return stopped;
}

bool Vehicle::isFinished() const {
    return finished;
}

int Vehicle::getCurrentRoadSegmentId() const {
    if (route.empty() || currentRoadIndex < 0 || currentRoadIndex >= static_cast<int>(route.size())) {
        return -1;
    }

    return route[currentRoadIndex];
}

int Vehicle::getNextRoadSegmentId() const {
    if (route.empty()) {
        return -1;
    }

    if (currentRoadIndex + 1 >= static_cast<int>(route.size())) {
        return -1;
    }

    return route[currentRoadIndex + 1];
}

int Vehicle::getCurrentRoadIndex() const {
    return currentRoadIndex;
}

void Vehicle::setSpeed(double newSpeed) {
    speed = newSpeed;
    maxSpeed = newSpeed;
}

void Vehicle::setRoute(const vector<int>& newRoute) {
    route = newRoute;
    currentRoadIndex = 0;
    positionOnRoad = 0.0;
    speed = maxSpeed;
    stopped = false;
    finished = route.empty();
}

void Vehicle::update(double dt, double roadLength, double maxAllowedPosition) {
    if (finished) {
        return;
    }

    if (maxAllowedPosition < positionOnRoad) {
        maxAllowedPosition = positionOnRoad;
    }

    double oldPosition = positionOnRoad;
    double desiredPosition = positionOnRoad + maxSpeed * dt;

    positionOnRoad = min(desiredPosition, maxAllowedPosition);

    double movedDistance = positionOnRoad - oldPosition;

    if (movedDistance <= 0.0001) {
        speed = 0.0;
        stopped = true;
    } else {
        speed = movedDistance / dt;
        stopped = false;
    }

    if (positionOnRoad >= roadLength) {
        positionOnRoad = roadLength;
        speed = 0.0;
        stopped = true;
    }
}

bool Vehicle::moveToNextRoad() {
    if (currentRoadIndex + 1 >= static_cast<int>(route.size())) {
        return false;
    }

    currentRoadIndex++;
    positionOnRoad = 0.0;
    speed = maxSpeed;
    stopped = false;

    return true;
}

void Vehicle::finish() {
    finished = true;
    stopped = true;
    speed = 0.0;
}
