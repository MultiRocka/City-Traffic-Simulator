#include "core/Vehicle.h"

using namespace std;

Vehicle::Vehicle() : id(-1), roadSegmentId(-1), positionOnRoad(0.0), speed(0.0), stopped(false) {
}

Vehicle::Vehicle(int id, int roadSegmentId, double positionOnRoad, double speed)
    : id(id), roadSegmentId(roadSegmentId), positionOnRoad(positionOnRoad), speed(speed), stopped(false) {
}

int Vehicle::getId() const {
    return id;
}

int Vehicle::getRoadSegmentId() const {
    return roadSegmentId;
}

double Vehicle::getPositionOnRoad() const {
    return positionOnRoad;
}

double Vehicle::getSpeed() const {
    return speed;
}

bool Vehicle::isStopped() const {
    return stopped;
}

void Vehicle::setRoadSegmentId(int newRoadSegmentId) {
    roadSegmentId = newRoadSegmentId;
}

void Vehicle::setPositionOnRoad(double newPositionOnRoad) {
    positionOnRoad = newPositionOnRoad;
}

void Vehicle::setSpeed(double newSpeed) {
    speed = newSpeed;
}

void Vehicle::stop() {
    speed = 0.0;
    stopped = true;
}

void Vehicle::update(double dt, double roadLength) {
    if (stopped) {
        return;
    }

    positionOnRoad += speed * dt;

    if (positionOnRoad >= roadLength) {
        positionOnRoad = roadLength;
        stop();
    }
}
