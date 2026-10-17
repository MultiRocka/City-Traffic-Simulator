#include "core/Pedestrian.h"

Pedestrian::Pedestrian()
    : id(-1),
      crossingId(-1),
      roadSegmentId(-1),
      state(PedestrianState::Waiting),
      crossProgress(0.0),
      crossSpeed(1.4),
      crossWidth(3.5) {
}

Pedestrian::Pedestrian(int id, int crossingId, int roadSegmentId, double crossSpeed)
    : id(id),
      crossingId(crossingId),
      roadSegmentId(roadSegmentId),
      state(PedestrianState::Waiting),
      crossProgress(0.0),
      crossSpeed(crossSpeed > 0.0 ? crossSpeed : 1.4),
      crossWidth(3.5) {
}

int Pedestrian::getId() const {
    return id;
}

int Pedestrian::getCrossingId() const {
    return crossingId;
}

int Pedestrian::getRoadSegmentId() const {
    return roadSegmentId;
}

PedestrianState Pedestrian::getState() const {
    return state;
}

double Pedestrian::getCrossProgress() const {
    return crossProgress;
}

double Pedestrian::getCrossSpeed() const {
    return crossSpeed;
}

double Pedestrian::getCrossWidth() const {
    return crossWidth;
}

void Pedestrian::setState(PedestrianState newState) {
    state = newState;
}

void Pedestrian::setCrossWidth(double widthMeters) {
    crossWidth = widthMeters > 0.1 ? widthMeters : 0.1;
}

void Pedestrian::setCrossProgress(double meters) {
    crossProgress = meters;
    if (crossProgress < 0.0) {
        crossProgress = 0.0;
    }
}

void Pedestrian::update(double dt) {
    if (state != PedestrianState::Crossing || dt <= 0.0) {
        return;
    }

    crossProgress += crossSpeed * dt;
    if (crossProgress >= crossWidth) {
        crossProgress = crossWidth;
        state = PedestrianState::Finished;
    }
}

bool Pedestrian::isOnRoadway() const {
    return state == PedestrianState::Crossing;
}

bool Pedestrian::isWaiting() const {
    return state == PedestrianState::Waiting;
}

bool Pedestrian::isFinished() const {
    return state == PedestrianState::Finished;
}
