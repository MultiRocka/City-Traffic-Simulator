#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>
#include "core/City.h"
#include "core/Pedestrian.h"
#include "core/PedestrianCrossing.h"
#include "core/PhaseController.h"
#include "core/RoadSegment.h"
#include "core/TrafficLight.h"
#include "core/Vehicle.h"

using namespace std;

// Owns visible pedestrians, spawn cadence, multi-lane entry safety, and
// seeded once-per-encounter yield refusal (Stage 4).
class PedestrianManager {
public:
    static constexpr uint32_t defaultSeed = 20261013u;
    static constexpr double defaultYieldRefusalProbability = 0.05;

    PedestrianManager();

    void clear();
    void setSeed(uint32_t seed);
    uint32_t getSeed() const;
    void setYieldRefusalProbability(double probability);
    double getYieldRefusalProbability() const;

    // Stage 6: >1 requests more often; scales inactiveDuration used as spawn interval.
    void setRequestTempoMultiplier(double multiplier);
    double getRequestTempoMultiplier() const;

    const vector<Pedestrian>& getPedestrians() const;
    vector<Pedestrian>& getPedestrians();

    // Spawn / enter / cross / finish; sync Timed visuals; update entity occupancy.
    void update(
        double dt,
        const City& city,
        vector<PedestrianCrossing>& crossings,
        const vector<Vehicle>& vehicles,
        const vector<TrafficLight>& trafficLights
    );

    // True if this vehicle must stop for a waiting (off-road) ped at crossing.
    // Rolls refusal once per (vehicleId, crossingId) encounter when probability > 0.
    bool mustYieldToWaitingPedestrian(int vehicleId, int crossingId) const;

    // Non-const: may record a refusal decision for this encounter.
    bool mustYieldToWaitingPedestrian(int vehicleId, int crossingId);

    bool hasPedestrianOnRoadway(int crossingId) const;
    bool hasWaitingPedestrian(int crossingId) const;

    // Cap position for vehicle approaching crossings on this road (or +inf).
    // Sets *stopReasonOut to "pedestrian_on_road" or "pedestrian_crossing" when capping.
    double computeCrossingStopCap(
        const Vehicle& vehicle,
        const RoadSegment& road,
        const vector<PedestrianCrossing>& crossings,
        const vector<TrafficLight>& trafficLights,
        string* stopReasonOut = nullptr
    );

    // True when a signalized phase controller should not advance (ped still on roadway).
    bool shouldHoldPhaseController(
        const PhaseController& controller,
        const vector<PedestrianCrossing>& crossings,
        const vector<TrafficLight>& trafficLights
    ) const;

    // Road crosswalk width from lane count (meters).
    static double crosswalkWidthForRoad(const RoadSegment& road);

    // Test/scenario helper: place a waiting pedestrian at a crossing (no auto-spawn wait).
    int spawnWaitingPedestrian(const PedestrianCrossing& crossing, const City& city);

private:
    vector<Pedestrian> pedestrians;
    uint32_t rngState;
    uint32_t configuredSeed;
    double yieldRefusalProbability;
    double requestTempoMultiplier;
    int nextPedestrianId;

    // Spawn timers for Signalized / Unsignalized (Timed uses crossing timer).
    map<int, double> requestTimers;
    // Timed: which crossings already have a synced visible ped for current active window.
    map<int, int> timedActivePedByCrossing;

    // Encounter key (vehicleId, crossingId) -> refused to yield (true) or will yield (false).
    map<pair<int, int>, bool> encounterRefusal;

    uint32_t nextRandom();
    double nextUnitRandom();

    void pruneFinished();
    void syncEntityOccupancy(vector<PedestrianCrossing>& crossings);
    void spawnIfNeeded(
        PedestrianCrossing& crossing,
        const City& city,
        const vector<Vehicle>& vehicles,
        const vector<TrafficLight>& trafficLights
    );
    void syncTimedPedestrian(
        PedestrianCrossing& crossing,
        const City& city
    );
    bool isWalkAllowed(
        const PedestrianCrossing& crossing,
        const vector<TrafficLight>& trafficLights
    ) const;
    bool isSafeToEnter(
        const PedestrianCrossing& crossing,
        const City& city,
        const vector<Vehicle>& vehicles
    ) const;
    bool vehicleInConflictZone(
        const Vehicle& vehicle,
        const PedestrianCrossing& crossing
    ) const;
    void clearStaleEncounters(const vector<Vehicle>& vehicles, const vector<PedestrianCrossing>& crossings);
    void tryEnterWaiting(
        const City& city,
        const vector<Vehicle>& vehicles,
        const vector<TrafficLight>& trafficLights,
        const vector<PedestrianCrossing>& crossings
    );
};
