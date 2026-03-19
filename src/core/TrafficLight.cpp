#include "core/TrafficLight.h"

using namespace std;

TrafficLight::TrafficLight()
    : id(-1), roadSegmentId(-1), greenDuration(5.0), redDuration(5.0), timer(0.0), green(true) {
}

TrafficLight::TrafficLight(int id, int roadSegmentId, double greenDuration, double redDuration, bool startGreen)
    : id(id),
      roadSegmentId(roadSegmentId),
      greenDuration(greenDuration),
      redDuration(redDuration),
      timer(0.0),
      green(startGreen) {
}

void TrafficLight::update(double dt) {
    timer += dt;

    if (green && timer >= greenDuration) {
        green = false;
        timer = 0.0;
    } else if (!green && timer >= redDuration) {
        green = true;
        timer = 0.0;
    }
}

int TrafficLight::getId() const {
    return id;
}

int TrafficLight::getRoadSegmentId() const {
    return roadSegmentId;
}

bool TrafficLight::isGreen() const {
    return green;
}

bool TrafficLight::isRed() const {
    return !green;
}

double TrafficLight::getTimer() const {
    return timer;
}
