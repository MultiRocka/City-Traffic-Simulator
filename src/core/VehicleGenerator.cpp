#include "core/VehicleGenerator.h"
#include "core/City.h"
#include "core/LaneBlockage.h"
#include <algorithm>
#include <cmath>

using namespace std;

VehicleGenerator::VehicleGenerator()
    : spawnInterval(1.0),
      vehicleSpeed(10.0),
      targetVehicleCount(0),
      generatedCount(0),
      timeSinceLastSpawn(0.0),
      spawnSafeDistance(10.0),
      rngState(defaultSeed),
      configuredSeed(defaultSeed),
      laneCountHint(1) {
}

VehicleGenerator::VehicleGenerator(double spawnInterval, double vehicleSpeed, int targetVehicleCount, double spawnSafeDistance)
    : spawnInterval(spawnInterval),
      vehicleSpeed(vehicleSpeed),
      targetVehicleCount(targetVehicleCount),
      generatedCount(0),
      timeSinceLastSpawn(0.0),
      spawnSafeDistance(spawnSafeDistance),
      rngState(defaultSeed),
      configuredSeed(defaultSeed),
      laneCountHint(1) {
}

void VehicleGenerator::setRoute(const vector<int>& newRoute) {
    route = newRoute;
}

const vector<int>& VehicleGenerator::getRoute() const {
    return route;
}

void VehicleGenerator::setSeed(uint32_t seed) {
    configuredSeed = seed;
    rngState = seed;
}

uint32_t VehicleGenerator::getSeed() const {
    return configuredSeed;
}

void VehicleGenerator::setLaneCountHint(int laneCount) {
    laneCountHint = max(1, laneCount);
}

double VehicleGenerator::getSpawnInterval() const {
    return spawnInterval;
}

void VehicleGenerator::setSpawnInterval(double intervalSeconds) {
    spawnInterval = max(0.05, intervalSeconds);
}

int VehicleGenerator::getGeneratedCount() const {
    return generatedCount;
}

uint32_t VehicleGenerator::nextRandom() {
    // Numerical Recipes LCG — deterministic, no <random> dependency.
    rngState = rngState * 1664525u + 1013904223u;
    return rngState;
}

double VehicleGenerator::nextUnitRandom() {
    return static_cast<double>(nextRandom()) / 4294967296.0;
}

void VehicleGenerator::assignVariedTraits(Vehicle& vehicle) {
    // Desired speed: 80%–100% of generator base so open-road speeds differ across vehicles.
    const double desiredFactor = 0.80 + 0.20 * nextUnitRandom();
    const double desired = vehicleSpeed * desiredFactor;
    vehicle.setMaxSpeed(vehicleSpeed);
    vehicle.setDesiredSpeed(desired);

    // Length ~4.0–5.5 m, accel/decel mild variety around Stage 0 defaults.
    vehicle.setLength(4.0 + 1.5 * nextUnitRandom());
    vehicle.setMaxAcceleration(3.2 + 1.2 * nextUnitRandom());
    vehicle.setMaxDeceleration(4.8 + 1.4 * nextUnitRandom());
}

bool VehicleGenerator::canSpawn(
    const vector<Vehicle>& vehicles,
    int laneIndex,
    const LaneBlockageManager* laneBlocks,
    double currentTime
) const {
    if (route.empty()) {
        return false;
    }

    int firstRoadId = route[0];

    for (const auto& vehicle : vehicles) {
        if (!vehicle.isFinished() &&
            vehicle.getCurrentRoadSegmentId() == firstRoadId &&
            vehicle.getLaneIndex() == laneIndex &&
            // Clear front + body so a new spawn at 0 does not overlap (length-aware).
            vehicle.getPositionOnRoad() < spawnSafeDistance + vehicle.getLength()) {
            return false;
        }
    }

    // Do not spawn into an active lane blockage near the entrance.
    if (laneBlocks != nullptr) {
        for (const auto& block : laneBlocks->getBlockages()) {
            if (!block.isActive(currentTime) ||
                block.roadId != firstRoadId ||
                block.laneIndex != laneIndex) {
                continue;
            }
            if (block.positionAlongRoad < spawnSafeDistance + 5.5) {
                return false;
            }
        }
    }

    return true;
}

void VehicleGenerator::update(
    double dt,
    vector<Vehicle>& vehicles,
    int& nextVehicleId,
    const City* city,
    const LaneBlockageManager* laneBlocks,
    double currentTime
) {
    if (generatedCount >= targetVehicleCount) {
        return;
    }

    timeSinceLastSpawn += dt;

    if (timeSinceLastSpawn < spawnInterval) {
        return;
    }

    int lanes = max(1, laneCountHint);
    if (city != nullptr && !route.empty()) {
        const RoadSegment* firstRoad = city->findRoadSegmentById(route[0]);
        if (firstRoad != nullptr) {
            lanes = max(1, firstRoad->getLaneCount());
        }
    }
    int chosenLane = -1;

    // Prefer a free lane; fall back to scanning all lanes.
    for (int attempt = 0; attempt < lanes; ++attempt) {
        int lane = static_cast<int>(nextRandom() % static_cast<uint32_t>(lanes));
        if (canSpawn(vehicles, lane, laneBlocks, currentTime)) {
            chosenLane = lane;
            break;
        }
    }

    if (chosenLane < 0) {
        for (int lane = 0; lane < lanes; ++lane) {
            if (canSpawn(vehicles, lane, laneBlocks, currentTime)) {
                chosenLane = lane;
                break;
            }
        }
    }

    if (chosenLane < 0) {
        return;
    }

    Vehicle vehicle(nextVehicleId, vehicleSpeed);
    vehicle.setRoute(route);
    assignVariedTraits(vehicle);
    vehicle.setLaneIndex(chosenLane);

    vehicles.push_back(vehicle);

    nextVehicleId++;
    generatedCount++;
    timeSinceLastSpawn = 0.0;
}
