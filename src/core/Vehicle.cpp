#include "core/Vehicle.h"
#include <algorithm>
#include <cmath>

using namespace std;

Vehicle::Vehicle()
    : id(-1),
      currentRoadIndex(0),
      positionOnRoad(0.0),
      speed(0.0),
      maxSpeed(0.0),
      desiredSpeed(0.0),
      length(kDefaultLength),
      maxAcceleration(kDefaultMaxAcceleration),
      maxDeceleration(kDefaultMaxDeceleration),
      laneIndex(0),
      preferLane(-1),
      laneChangeCooldown(0.0),
      stopped(false),
      finished(false) {
}

Vehicle::Vehicle(int id, double maxSpeed)
    : id(id),
      currentRoadIndex(0),
      positionOnRoad(0.0),
      speed(0.0),
      maxSpeed(maxSpeed),
      desiredSpeed(maxSpeed),
      length(kDefaultLength),
      maxAcceleration(kDefaultMaxAcceleration),
      maxDeceleration(kDefaultMaxDeceleration),
      laneIndex(0),
      preferLane(-1),
      laneChangeCooldown(0.0),
      stopped(false),
      finished(false) {
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

double Vehicle::getDesiredSpeed() const {
    return desiredSpeed;
}

double Vehicle::getLength() const {
    return length;
}

double Vehicle::getMaxAcceleration() const {
    return maxAcceleration;
}

double Vehicle::getMaxDeceleration() const {
    return maxDeceleration;
}

int Vehicle::getLaneIndex() const {
    return laneIndex;
}

int Vehicle::getPreferLane() const {
    return preferLane;
}

double Vehicle::getLaneChangeCooldown() const {
    return laneChangeCooldown;
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
}

void Vehicle::setPositionOnRoad(double newPosition) {
    positionOnRoad = max(0.0, newPosition);
}

void Vehicle::setDesiredSpeed(double newDesiredSpeed) {
    desiredSpeed = max(0.0, newDesiredSpeed);
}

void Vehicle::setMaxSpeed(double newMaxSpeed) {
    maxSpeed = max(0.0, newMaxSpeed);
}

void Vehicle::setLength(double newLength) {
    length = max(0.1, newLength);
}

void Vehicle::setMaxAcceleration(double newMaxAcceleration) {
    maxAcceleration = max(0.1, newMaxAcceleration);
}

void Vehicle::setMaxDeceleration(double newMaxDeceleration) {
    maxDeceleration = max(0.1, newMaxDeceleration);
}

void Vehicle::setLaneIndex(int newLaneIndex) {
    laneIndex = max(0, newLaneIndex);
}

void Vehicle::setPreferLane(int newPreferLane) {
    preferLane = newPreferLane;
}

void Vehicle::setLaneChangeCooldown(double seconds) {
    laneChangeCooldown = max(0.0, seconds);
}

void Vehicle::setRoute(const vector<int>& newRoute) {
    route = newRoute;
    currentRoadIndex = 0;
    positionOnRoad = 0.0;
    speed = 0.0;
    stopped = false;
    finished = route.empty();
}

void Vehicle::clampLaneIndex(int laneCount) {
    if (laneCount < 1) {
        laneIndex = 0;
        return;
    }

    if (laneIndex < 0) {
        laneIndex = 0;
    } else if (laneIndex >= laneCount) {
        laneIndex = laneCount - 1;
    }
}

void Vehicle::applyLaneChange(int targetLane, double cooldownSeconds) {
    laneIndex = max(0, targetLane);
    laneChangeCooldown = max(0.0, cooldownSeconds);
}

double Vehicle::computeTargetSpeed(double maxAllowedPosition, double speedLimit) const {
    // Cruise never exceeds min(desired, physical max, road limit).
    double cruiseSpeed = min(desiredSpeed, min(maxSpeed, speedLimit));
    double gap = maxAllowedPosition - positionOnRoad;

    if (gap <= 0.05) {
        return 0.0;
    }

    if (gap >= reactionDistance) {
        return cruiseSpeed;
    }

    const double comfortGap = 0.3;
    double usableRange = reactionDistance - comfortGap;

    if (usableRange <= 0.0) {
        return cruiseSpeed * 0.5;
    }

    double ratio = (gap - comfortGap) / usableRange;
    ratio = max(0.0, min(1.0, ratio));

    return cruiseSpeed * (0.35 + 0.65 * sqrt(ratio));
}

void Vehicle::update(double dt, double roadLength, double maxAllowedPosition, double speedLimit, double minVehicleGap) {
    (void)minVehicleGap;

    if (finished) {
        return;
    }

    if (laneChangeCooldown > 0.0) {
        laneChangeCooldown = max(0.0, laneChangeCooldown - dt);
    }

    if (maxAllowedPosition < positionOnRoad) {
        maxAllowedPosition = positionOnRoad;
    }

    double targetSpeed = computeTargetSpeed(maxAllowedPosition, speedLimit);

    if (speed < targetSpeed) {
        speed = min(speed + maxAcceleration * dt, targetSpeed);
    } else {
        speed = max(speed - maxDeceleration * dt, targetSpeed);
    }

    double oldPosition = positionOnRoad;
    double desiredPosition = positionOnRoad + speed * dt;
    positionOnRoad = min(desiredPosition, maxAllowedPosition);

    double movedDistance = positionOnRoad - oldPosition;
    double gapToConstraint = maxAllowedPosition - positionOnRoad;

    if (gapToConstraint <= 0.15 && speed < 0.3) {
        speed = 0.0;
        stopped = true;
    } else if (movedDistance <= 0.0001 && gapToConstraint <= 0.25) {
        speed = 0.0;
        stopped = true;
    } else {
        if (dt > 0.0001) {
            speed = movedDistance / dt;
        }
        stopped = false;
    }

    if (positionOnRoad > roadLength) {
        positionOnRoad = roadLength;
    }
}

bool Vehicle::moveToNextRoad() {
    if (currentRoadIndex + 1 >= static_cast<int>(route.size())) {
        return false;
    }

    currentRoadIndex++;
    positionOnRoad = 0.0;
    // Preserve incoming speed (including 0 when stopped). No artificial boost.
    stopped = (speed <= 0.0001);

    return true;
}

void Vehicle::finish() {
    finished = true;
    stopped = true;
    speed = 0.0;
}
