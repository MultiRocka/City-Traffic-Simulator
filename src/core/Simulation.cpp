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

string Simulation::determineStopReason(const Vehicle& vehicle, const vector<Vehicle>& vehicles, const vector<TrafficLight>& trafficLights, const City& city) const {
    int roadId = vehicle.getCurrentRoadSegmentId();

    if (roadId == -1) {
        return "unknown";
    }

    const RoadSegment* road = city.findRoadSegmentById(roadId);

    if (road == nullptr) {
        return "unknown";
    }

    double roadLength = road->getLength();
    double position = vehicle.getPositionOnRoad();

    for (const auto& light : trafficLights) {
        if (light.getRoadSegmentId() == roadId && light.isRed()) {
            if (position >= roadLength - 3.0) {
                return "red_light";
            }
        }
    }

    for (const auto& otherVehicle : vehicles) {
        if (otherVehicle.getId() == vehicle.getId()) {
            continue;
        }

        if (otherVehicle.isFinished()) {
            continue;
        }

        if (otherVehicle.getCurrentRoadSegmentId() != roadId) {
            continue;
        }

        if (otherVehicle.getPositionOnRoad() > position) {
            double gap = otherVehicle.getPositionOnRoad() - position;

            if (gap <= minVehicleGap + 0.5) {
                return "vehicle_ahead";
            }
        }
    }

    if (position >= roadLength) {
        if (vehicle.getNextRoadSegmentId() == -1) {
            return "route_finished";
        }

        return "intersection_blocked";
    }

    return "unknown";
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

    for (const auto& vehicle : vehicles) {
        if (knownVehicleIds.find(vehicle.getId()) == knownVehicleIds.end()) {
            knownVehicleIds.insert(vehicle.getId());
            previousStoppedState[vehicle.getId()] = vehicle.isStopped();
            previousRoadState[vehicle.getId()] = vehicle.getCurrentRoadSegmentId();

            eventLog.addEvent(
                currentTime,
                "vehicle_spawned",
                vehicle.getId(),
                vehicle.getCurrentRoadSegmentId(),
                "generator_spawn",
                "Vehicle spawned in simulation"
            );
        }
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

            eventLog.addEvent(
                currentTime,
                "vehicle_finished",
                vehicle.getId(),
                currentRoadId,
                "route_finished",
                "Vehicle finished its route"
            );

            continue;
        }

        if (isGreenForRoad(trafficLights, currentRoadId) && canEnterRoad(nextRoadId, vehicles, minVehicleGap)) {
            vehicle.moveToNextRoad();

            eventLog.addEvent(
                currentTime,
                "vehicle_entered_road",
                vehicle.getId(),
                vehicle.getCurrentRoadSegmentId(),
                "green_and_space_available",
                "Vehicle entered next road segment"
            );
        }
    }

    for (const auto& vehicle : vehicles) {
        if (vehicle.isFinished()) {
            continue;
        }

        int vehicleId = vehicle.getId();
        bool oldStopped = previousStoppedState[vehicleId];
        bool newStopped = vehicle.isStopped();

        if (!oldStopped && newStopped) {
            eventLog.addEvent(
                currentTime,
                "vehicle_stopped",
                vehicleId,
                vehicle.getCurrentRoadSegmentId(),
                determineStopReason(vehicle, vehicles, trafficLights, city),
                "Vehicle stopped"
            );
        }

        if (oldStopped && !newStopped) {
            eventLog.addEvent(
                currentTime,
                "vehicle_resumed",
                vehicleId,
                vehicle.getCurrentRoadSegmentId(),
                "obstacle_cleared",
                "Vehicle resumed movement"
            );
        }

        int oldRoadId = previousRoadState[vehicleId];
        int newRoadId = vehicle.getCurrentRoadSegmentId();

        if (oldRoadId != newRoadId) {
            eventLog.addEvent(
                currentTime,
                "vehicle_changed_road",
                vehicleId,
                newRoadId,
                "route_progress",
                "Vehicle changed road segment"
            );
        }

        previousStoppedState[vehicleId] = newStopped;
        previousRoadState[vehicleId] = newRoadId;
    }

    statisticsCollector.update(dt, currentTime, vehicles);
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

void Simulation::setStatisticsOutputFiles(const string& timelineFile, const string& summaryFile) {
    statisticsCollector.setOutputFiles(timelineFile, summaryFile);
}

void Simulation::exportStatisticsSummary() const {
    statisticsCollector.exportSummary();
}

const StatisticsCollector& Simulation::getStatisticsCollector() const {
    return statisticsCollector;
}

void Simulation::setEventLogOutputFile(const string& filePath) {
    eventLog.setOutputFile(filePath);
}

void Simulation::exportEventLog() const {
    eventLog.exportToJson();
}

const EventLog& Simulation::getEventLog() const {
    return eventLog;
}
