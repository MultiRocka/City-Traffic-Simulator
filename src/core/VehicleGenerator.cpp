#include "core/VehicleGenerator.h"

using namespace std;

static int globalVehicleId = 1;

VehicleGenerator::VehicleGenerator()
    : spawnInterval(1.0), vehicleSpeed(10.0), targetVehicleCount(0),
      generatedCount(0), timeSinceLastSpawn(0.0), spawnSafeDistance(10.0) {
}

VehicleGenerator::VehicleGenerator(double spawnInterval, double vehicleSpeed, int targetVehicleCount, double spawnSafeDistance)
    : spawnInterval(spawnInterval), vehicleSpeed(vehicleSpeed),
      targetVehicleCount(targetVehicleCount), generatedCount(0),
      timeSinceLastSpawn(0.0), spawnSafeDistance(spawnSafeDistance) {
}

void VehicleGenerator::setRoute(const vector<int>& newRoute) {
    route = newRoute;
}

int VehicleGenerator::getGeneratedCount() const {
    return generatedCount;
}

bool VehicleGenerator::canSpawn(const vector<Vehicle>& vehicles) const {
    if (route.empty()) {
        return false;
    }

    int firstRoadId = route[0];

    for (const auto& vehicle : vehicles) {
        if (!vehicle.isFinished() &&
            vehicle.getCurrentRoadSegmentId() == firstRoadId &&
            vehicle.getPositionOnRoad() < spawnSafeDistance) {
            return false;
        }
    }

    return true;
}

void VehicleGenerator::update(double dt, vector<Vehicle>& vehicles) {
    if (generatedCount >= targetVehicleCount) {
        return;
    }

    timeSinceLastSpawn += dt;

    if (timeSinceLastSpawn < spawnInterval) {
        return;
    }

    if (!canSpawn(vehicles)) {
        return;
    }

    Vehicle vehicle(globalVehicleId, vehicleSpeed);
    vehicle.setRoute(route);

    vehicles.push_back(vehicle);

    globalVehicleId++;
    generatedCount++;
    timeSinceLastSpawn = 0.0;
}
