#pragma once

#include <vector>
#include "core/Phase.h"

using namespace std;

class PhaseController {
public:
    PhaseController();
    PhaseController(int id, int intersectionId);

    void addPhase(const Phase& phase);
    // When holdPhase is true, timer freezes so a pedestrian still on the roadway can clear.
    void update(double dt, bool holdPhase = false);

    int getId() const;
    int getIntersectionId() const;
    int getCurrentPhaseId() const;
    bool isRoadGreen(int roadSegmentId) const;

private:
    int id;
    int intersectionId;
    vector<Phase> phases;
    int currentPhaseIndex;
    double timer;
};
