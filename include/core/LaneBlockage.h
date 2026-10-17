#pragma once

#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>
#include "core/City.h"
#include "core/EventLog.h"
#include "core/RoadSegment.h"
#include "core/Vehicle.h"

using namespace std;

class CollisionDetector;

// Temporary closure of one lane on one directed RoadSegment.
struct LaneBlockage {
    int id = -1;
    int roadId = -1;
    int laneIndex = 0;
    double positionAlongRoad = 0.0; // start of closed span (m)
    double length = 5.0;            // span along road (m)
    double startTime = 0.0;
    double duration = 10.0;

    bool hasLoggedStart = false;
    bool hasLoggedEnd = false;

    bool isActive(double currentTime) const;
    bool isExpired(double currentTime) const;
    double endPosition() const;
    double stopLine() const; // approach stop before span
};

// Optional random blockage generator (Stage 6 can expose these knobs).
struct RandomBlockConfig {
    bool enabled = false;
    double minInterval = 25.0;
    double maxInterval = 70.0;
    double minDuration = 8.0;
    double maxDuration = 30.0;
    double minLength = 4.0;
    double maxLength = 12.0;
    // Place blocks in this fraction of road length (avoid termini).
    double minPositionFraction = 0.25;
    double maxPositionFraction = 0.75;
    uint32_t seed = 20261013u;
};

// Owns lane blocks, random generation, stop caps, and zipper grants near merges.
class LaneBlockageManager {
public:
    static constexpr uint32_t defaultSeed = 20261013u;

    LaneBlockageManager();

    void clear();
    void setSeed(uint32_t seed);
    uint32_t getSeed() const;

    void setRandomConfig(const RandomBlockConfig& config);
    const RandomBlockConfig& getRandomConfig() const;

    const vector<LaneBlockage>& getBlockages() const;
    vector<LaneBlockage>& getBlockages();

    // Manual add. Rejects invalid geometry or overlap with a vehicle body on that lane.
    // Returns id, or -1 on reject. Does not delete/teleport vehicles.
    int addBlockage(
        int roadId,
        int laneIndex,
        double positionAlongRoad,
        double length,
        double startTime,
        double duration,
        const City& city,
        const vector<Vehicle>& vehicles
    );

    bool clearBlockage(int blockageId);
    void clearAllBlockages();

    // Activate/expire logging, prune expired, optional random spawn, refresh zipper grants.
    void update(
        double dt,
        double currentTime,
        const City& city,
        const vector<Vehicle>& vehicles,
        EventLog& eventLog,
        const CollisionDetector* collisionDetector = nullptr
    );

    // Cap for this vehicle on its current road/lane, or +inf.
    // Includes closed-lane stop line and open-lane zipper wait near merge.
    double computeStopCap(
        const Vehicle& vehicle,
        const RoadSegment& road,
        double currentTime,
        string* stopReasonOut = nullptr
    ) const;

    // True if span [position, position] on road/lane intersects an active block.
    bool isPositionBlocked(int roadId, int laneIndex, double position, double currentTime) const;

    // True if vehicle body would overlap any active/scheduled-soon block span.
    bool wouldOverlapVehicle(
        int roadId,
        int laneIndex,
        double spanStart,
        double spanEnd,
        const vector<Vehicle>& vehicles
    ) const;

    // Closed-lane vehicles with zipper grant may LC into an open lane (safe gap required).
    // Call after update(); uses existing CollisionDetector::canChangeLane / cooldown.
    void tryEscapeLaneChanges(
        double currentTime,
        const City& city,
        vector<Vehicle>& vehicles,
        const CollisionDetector& collisionDetector,
        EventLog& eventLog
    );

    bool hasActiveBlockOnLane(int roadId, int laneIndex, double currentTime) const;
    bool areAllLanesBlocked(const RoadSegment& road, double currentTime) const;
    // True if an active block closes the entrance zone of this lane (spawn / junction entry).
    bool blocksEntrance(int roadId, int laneIndex, double currentTime, double clearance = 8.0) const;

    // Zipper grant for this vehicle this tick (merge zone only).
    bool hasZipperGrant(int vehicleId) const;

private:
    vector<LaneBlockage> blockages;
    RandomBlockConfig randomConfig;
    uint32_t rngState;
    uint32_t configuredSeed;
    int nextBlockageId;
    double timeUntilNextRandom;

    // Per blockage: last lane that received the merge slot (-1 = none yet).
    map<int, int> zipperLastGrantedLane;
    // Vehicle ids allowed to take the merge this tick.
    set<int> zipperGrantedVehicleIds;
    // Open-lane vehicles that must wait at the merge line this tick.
    set<int> zipperWaitingOpenLaneIds;

    uint32_t nextRandom();
    double nextUnitRandom();
    double nextRange(double minValue, double maxValue);

    void logStateChanges(double currentTime, EventLog& eventLog);
    void pruneExpired(double currentTime);
    void maybeSpawnRandom(double dt, double currentTime, const City& city, const vector<Vehicle>& vehicles);
    void refreshZipperGrants(
        double currentTime,
        const City& city,
        const vector<Vehicle>& vehicles,
        const CollisionDetector* collisionDetector
    );

    vector<int> openLanesForBlock(const LaneBlockage& block, const RoadSegment& road, double currentTime) const;
    const Vehicle* headInMergeZone(
        int roadId,
        int laneIndex,
        double blockStart,
        const vector<Vehicle>& vehicles
    ) const;
};
