#include "core/Simulation.h"
#include <algorithm>

using namespace std;

static bool isGreenForRoad(const vector<TrafficLight>& trafficLights, int roadId) {
    for (const auto& light : trafficLights) {
        if (light.getRoadSegmentId() == roadId) {
            return light.isGreen();
        }
    }

    return true;
}

static bool hasRedLightForRoad(const vector<TrafficLight>& trafficLights, int roadId) {
    for (const auto& light : trafficLights) {
        if (light.getRoadSegmentId() == roadId) {
            return light.isRed();
        }
    }

    return false;
}

static bool canEnterRoad(int nextRoadId, const vector<Vehicle>& vehicles, double minVehicleGap) {
    for (const auto& vehicle : vehicles) {
        if (!vehicle.isFinished() &&
            vehicle.getCurrentRoadSegmentId() == nextRoadId &&
            vehicle.getPositionOnRoad() < minVehicleGap) {
            return false;
        }
    }

    return true;
}

Simulation::Simulation()
    : running(false), currentTime(0.0), minVehicleGap(3.0) {
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

    for (auto& trafficLight : trafficLights) {
        trafficLight.update(dt);
    }

    for (auto& generator : generators) {
        generator.update(dt, vehicles);
    }

    for (const auto& roadSegment : city.getRoadSegments()) {
        vector<Vehicle*> vehiclesOnRoad;

        for (auto& vehicle : vehicles) {
            if (!vehicle.isFinished() && vehicle.getCurrentRoadSegmentId() == roadSegment.getId()) {
                vehiclesOnRoad.push_back(&vehicle);
            }
        }

        sort(vehiclesOnRoad.begin(), vehiclesOnRoad.end(),
            [](Vehicle* a, Vehicle* b) {
                return a->getPositionOnRoad() > b->getPositionOnRoad();
            });

        for (int i = 0; i < static_cast<int>(vehiclesOnRoad.size()); i++) {
            double maxAllowedPosition = roadSegment.getLength();

            if (hasRedLightForRoad(trafficLights, roadSegment.getId())) {
                maxAllowedPosition = min(maxAllowedPosition, roadSegment.getLength() - 2.0);
            }

            if (i > 0) {
                maxAllowedPosition = min(maxAllowedPosition, vehiclesOnRoad[i - 1]->getPositionOnRoad() - minVehicleGap);
            }

            if (maxAllowedPosition < vehiclesOnRoad[i]->getPositionOnRoad()) {
                maxAllowedPosition = vehiclesOnRoad[i]->getPositionOnRoad();
            }

            vehiclesOnRoad[i]->update(dt, roadSegment.getLength(), maxAllowedPosition);
        }
    }

    for (auto& vehicle : vehicles) {
        if (vehicle.isFinished()) {
            continue;
        }

        int currentRoadId = vehicle.getCurrentRoadSegmentId();
        const RoadSegment* currentRoad = city.findRoadSegmentById(currentRoadId);

        if (currentRoad == nullptr) {
            continue;
        }

        if (vehicle.getPositionOnRoad() < currentRoad->getLength()) {
            continue;
        }

        int nextRoadId = vehicle.getNextRoadSegmentId();

        if (nextRoadId == -1) {
            vehicle.finish();
            continue;
        }

        if (isGreenForRoad(trafficLights, currentRoadId) && canEnterRoad(nextRoadId, vehicles, minVehicleGap)) {
            vehicle.moveToNextRoad();
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

void Simulation::addVehicleGenerator(const VehicleGenerator& generator) {
    generators.push_back(generator);
}

void Simulation::setMinVehicleGap(double newMinVehicleGap) {
    minVehicleGap = newMinVehicleGap;
}

void Simulation::addTrafficLight(const TrafficLight& trafficLight) {
    trafficLights.push_back(trafficLight);
}

vector<TrafficLight>& Simulation::getTrafficLights() {
    return trafficLights;
}

const vector<TrafficLight>& Simulation::getTrafficLights() const {
    return trafficLights;
}
