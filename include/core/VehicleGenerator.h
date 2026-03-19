#pragma once

#include <vector>
#include "core/Vehicle.h"

using namespace std;

class VehicleGenerator {
public:
    VehicleGenerator();
    VehicleGenerator(double spawnInterval, double vehicleSpeed, int targetVehicleCount, double spawnSafeDistance);

    void setRoute(const vector<int>& newRoute);
    void update(double dt, vector<Vehicle>& vehicles);

    int getGeneratedCount() const;

private:
    bool canSpawn(const vector<Vehicle>& vehicles) const;

    double spawnInterval;
    double vehicleSpeed;
    int targetVehicleCount;
    int generatedCount;
    double timeSinceLastSpawn;
    double spawnSafeDistance;
    vector<int> route;
};
