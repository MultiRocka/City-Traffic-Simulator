#pragma once

#include <string>
#include <vector>
#include "core/Vehicle.h"
#include "core/TrafficLight.h"
#include "core/PedestrianCrossing.h"
#include "core/RoadSegment.h"
#include "core/City.h"
#include "core/IntersectionRules.h"

using namespace std;

class PedestrianManager;

namespace SimConstants {
    constexpr double RED_LIGHT_STOP_DISTANCE = 1.0;
    constexpr double CROSSING_STOP_DISTANCE = 1.5;
    constexpr double ROAD_END_EPSILON = 0.01;
    constexpr double STOP_REASON_TOLERANCE = 0.5;
    constexpr double STOPPED_GAP_TOLERANCE = 0.2;
    constexpr double CONSTRAINT_MATCH_EPSILON = 0.05;
    // Lateral world offset per lane (meters). Rendering uses this via layout.
    constexpr double LANE_WIDTH = 3.5;
    constexpr double LANE_CHANGE_COOLDOWN = 1.5;
    constexpr double LANE_BLOCK_STOP_DISTANCE = 1.5;
    // Only near the closed-lane merge — not whole-road zipper / normal preferLane LC.
    constexpr double ZIPPER_MERGE_ZONE = 22.0;
}

class LaneBlockageManager;

class CollisionDetector {
public:
    explicit CollisionDetector(double minVehicleGap = 3.0);

    void setMinVehicleGap(double newMinVehicleGap);
    double getMinVehicleGap() const;

    bool isGreenForRoad(const vector<TrafficLight>& trafficLights, int roadId) const;
    bool hasRedLightForRoad(const vector<TrafficLight>& trafficLights, int roadId) const;

    // Entry clearance on a specific lane of nextRoadId (lane-aware for multi-lane).
    bool canEnterRoad(int nextRoadId, int laneIndex, const vector<Vehicle>& vehicles) const;
    // Convenience: lane 0 (single-lane / legacy callers).
    bool canEnterRoad(int nextRoadId, const vector<Vehicle>& vehicles) const;

    // True when position has effectively reached roadLength (tiny epsilon allowed).
    bool isAtIntersectionEntry(double positionOnRoad, double roadLength) const;

    // Nearest vehicle ahead on same road AND same lane. Returns nullptr if none.
    const Vehicle* findPredecessor(
        const Vehicle& vehicle,
        const vector<Vehicle>& vehicles
    ) const;

    // Same-road helper: `sameRoad` is typically front-most-first for that road only.
    // Scans only that list (O(road occupancy)) instead of the full fleet.
    const Vehicle* findPredecessorOnRoad(
        const Vehicle& vehicle,
        const vector<Vehicle*>& sameRoad
    ) const;

    // Front of ego may not pass leader.front - leader.length - minGap.
    // positionOnRoad is the front reference for each vehicle.
    double computeLeaderStopPosition(const Vehicle& leader) const;

    // Safe LC onto targetLane of the same directed road: room ahead/behind, no overlap.
    // Requires cooldown cleared and road.laneCount > 1. Stage 5 can reuse for closed-lane escape.
    bool canChangeLane(
        const Vehicle& vehicle,
        int targetLane,
        const RoadSegment& road,
        const vector<Vehicle>& vehicles
    ) const;

    // Longitudinal body intervals [front-length, front] with minGap padding.
    bool vehiclesOverlapLongitudinally(
        double frontA,
        double lengthA,
        double frontB,
        double lengthB
    ) const;

    // vehicleAheadPosition < 0 means no leader; otherwise leader front + length used for gap.
    // Optional pedManager applies on-road / waiting-yield / Timed caps (Stage 4).
    // Optional laneBlocks applies closed-lane stop + zipper wait caps (Stage 5).
    double computeMaxAllowedPosition(
        const RoadSegment& roadSegment,
        const vector<TrafficLight>& trafficLights,
        const vector<PedestrianCrossing>& pedestrianCrossings,
        double vehiclePosition,
        double vehicleAheadPosition,
        double vehicleAheadLength = 0.0,
        PedestrianManager* pedManager = nullptr,
        const Vehicle* subjectVehicle = nullptr,
        LaneBlockageManager* laneBlocks = nullptr,
        double currentTime = 0.0
    ) const;

    string determineStopReason(
        const Vehicle& vehicle,
        const City& city,
        const vector<TrafficLight>& trafficLights,
        const vector<PedestrianCrossing>& pedestrianCrossings,
        const vector<Vehicle>& vehicles,
        const IntersectionRules* intersectionRules = nullptr,
        PedestrianManager* pedManager = nullptr,
        LaneBlockageManager* laneBlocks = nullptr,
        double currentTime = 0.0
    ) const;

private:
    double minVehicleGap;
};
