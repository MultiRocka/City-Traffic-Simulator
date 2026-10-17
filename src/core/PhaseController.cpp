#include "core/PhaseController.h"

using namespace std;

PhaseController::PhaseController() : id(-1), intersectionId(-1), currentPhaseIndex(0), timer(0.0) {
}

PhaseController::PhaseController(int id, int intersectionId)
    : id(id), intersectionId(intersectionId), currentPhaseIndex(0), timer(0.0) {
}

void PhaseController::addPhase(const Phase& phase) {
    phases.push_back(phase);
}

void PhaseController::update(double dt, bool holdPhase) {
    if (phases.empty()) {
        return;
    }

    if (holdPhase) {
        return;
    }

    timer += dt;

    if (timer >= phases[currentPhaseIndex].getDuration()) {
        timer = 0.0;
        currentPhaseIndex = (currentPhaseIndex + 1) % static_cast<int>(phases.size());
    }
}

int PhaseController::getId() const {
    return id;
}

int PhaseController::getIntersectionId() const {
    return intersectionId;
}

int PhaseController::getCurrentPhaseId() const {
    if (phases.empty()) {
        return -1;
    }

    return phases[currentPhaseIndex].getId();
}

bool PhaseController::isRoadGreen(int roadSegmentId) const {
    if (phases.empty()) {
        return true;
    }

    return phases[currentPhaseIndex].isRoadGreen(roadSegmentId);
}
