#include "core/Simulation.h"

using namespace std;

Simulation::Simulation() : running(false), currentTime(0.0) {
}

void Simulation::start() {
    running = true;
}

void Simulation::stop() {
    running = false;
}

void Simulation::update(double dt, const City& city) {
    if (!running) {
        return;
    }

    currentTime += dt;

    for (auto& vehicle : vehicles) {
        int roadId = vehicle.getRoadSegmentId();
        double roadLength = 0.0;

        for (const auto& roadSegment : city.getRoadSegments()) {
            if (roadSegment.getId() == roadId) {
                roadLength = roadSegment.getLength();
                break;
            }
        }

        if (roadLength > 0.0) {
            vehicle.update(dt, roadLength);
        }
    }
}

bool Simulation::isRunning() const {
    return running;
}

double Simulation::getTime() const {
    return currentTime;
}

void Simulation::addVehicle(const Vehicle& vehicle) {
    vehicles.push_back(vehicle);
}

vector<Vehicle>& Simulation::getVehicles() {
    return vehicles;
}

const vector<Vehicle>& Simulation::getVehicles() const {
    return vehicles;
}
