#pragma once

#include <cstdint>
#include <vector>
#include "core/Vehicle.h"

class City;
class LaneBlockageManager;

using namespace std;

// Deterministic spawn variety (Stage 2).
// Seed policy: defaultSeed = 20261013. Call setSeed before update() for reproducible runs.
// Each spawned vehicle advances an internal LCG; traits are assigned once at spawn (never re-rolled).
//
// Demand modes:
// - Finite quota: targetVehicleCount > 0 → stop after that many successful spawns.
// - Continuous: targetVehicleCount <= 0 → keep attempting until Stop (respect maxActive).
class VehicleGenerator {
public:
    static constexpr uint32_t defaultSeed = 20261013u;

    VehicleGenerator();
    VehicleGenerator(double spawnInterval, double vehicleSpeed, int targetVehicleCount, double spawnSafeDistance);

    void setRoute(const vector<int>& newRoute);
    const vector<int>& getRoute() const;
    void setSeed(uint32_t seed);
    uint32_t getSeed() const;
    void setLaneCountHint(int laneCount); // used when city is not passed to update()

    // Spawn cadence in seconds between attempts (Stage 6 tempo scaling).
    double getSpawnInterval() const;
    void setSpawnInterval(double intervalSeconds);

    int getTargetVehicleCount() const;
    void setTargetVehicleCount(int count); // <= 0 → continuous
    bool isContinuous() const;

    // If city is non-null, spawn laneCount is taken from the first route road (clamped ≥ 1).
    // Optional laneBlocks refuses spawn into an active closed span near the entrance.
    // activeCount / maxActive: soft concurrent cap. maxActive <= 0 → unlimited.
    // When at cap, increments deferredDueToActiveCap (not failedSpawnAttempts) and does not
    // consume the spawn interval so the next free slot can fill immediately.
    void update(
        double dt,
        vector<Vehicle>& vehicles,
        int& nextVehicleId,
        const City* city = nullptr,
        const LaneBlockageManager* laneBlocks = nullptr,
        double currentTime = 0.0,
        int* activeCount = nullptr,
        int maxActive = 0
    );

    int getGeneratedCount() const;
    // Cadence ticks where spawn was due but every entrance lane was blocked
    // (occupancy / entrance blockage). Does not include idle waits before interval,
    // and does not include soft maxActive deferrals.
    int getFailedSpawnAttempts() const;
    // Spawn was due and under quota, but global active vehicle cap was reached.
    int getDeferredDueToActiveCap() const;

private:
    bool canSpawn(
        const vector<Vehicle>& vehicles,
        int laneIndex,
        const LaneBlockageManager* laneBlocks = nullptr,
        double currentTime = 0.0
    ) const;
    uint32_t nextRandom();
    double nextUnitRandom(); // [0, 1)
    void assignVariedTraits(Vehicle& vehicle);

    double spawnInterval;
    double vehicleSpeed; // base desired/max cruise used as center of variety
    int targetVehicleCount;
    int generatedCount;
    int failedSpawnAttempts;
    int deferredDueToActiveCap;
    double timeSinceLastSpawn;
    double spawnSafeDistance;
    vector<int> route;
    uint32_t rngState;
    uint32_t configuredSeed;
    int laneCountHint;
};
