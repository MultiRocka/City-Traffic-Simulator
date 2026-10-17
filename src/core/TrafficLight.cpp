#include "core/TrafficLight.h"

using namespace std;

TrafficLight::TrafficLight()
    : id(-1),
      roadSegmentId(-1),
      intersectionId(-1),
      greenDuration(5.0),
      redDuration(5.0),
      timer(0.0),
      green(true),
      phaseControlled(false) {
}

TrafficLight::TrafficLight(int id, int roadSegmentId, double greenDuration, double redDuration, bool startGreen)
    : id(id),
      roadSegmentId(roadSegmentId),
      intersectionId(-1),
      greenDuration(greenDuration),
      redDuration(redDuration),
      timer(0.0),
      green(startGreen),
      phaseControlled(false) {
}

TrafficLight::TrafficLight(int id, int roadSegmentId, int intersectionId, bool phaseControlled)
    : id(id),
      roadSegmentId(roadSegmentId),
      intersectionId(intersectionId),
      greenDuration(0.0),
      redDuration(0.0),
      timer(0.0),
      green(false),
      phaseControlled(phaseControlled) {
}

void TrafficLight::update(double dt) {
    if (phaseControlled) {
        return;
    }

    timer += dt;

    if (green && timer >= greenDuration) {
        green = false;
        timer = 0.0;
    } else if (!green && timer >= redDuration) {
        green = true;
        timer = 0.0;
    }
}

void TrafficLight::setGreen(bool isGreenState) {
    green = isGreenState;
}

void TrafficLight::setPhaseControlled(bool controlled) {
    phaseControlled = controlled;
}

int TrafficLight::getId() const {
    return id;
}

int TrafficLight::getRoadSegmentId() const {
    return roadSegmentId;
}

int TrafficLight::getIntersectionId() const {
    return intersectionId;
}

bool TrafficLight::isGreen() const {
    return green;
}

bool TrafficLight::isRed() const {
    return !green;
}

bool TrafficLight::isPhaseControlled() const {
    return phaseControlled;
}

double TrafficLight::getTimer() const {
    return timer;
}
