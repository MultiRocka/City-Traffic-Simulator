#pragma once

// Shared Stage-4 credible freeze checks: stop reason must match a current obstacle.
// Do not delete vehicles to improve metrics — only classify stuck vehicles.

#include "core/City.h"
#include "core/CollisionDetector.h"
#include "core/IntersectionRules.h"
#include "core/LaneBlockage.h"
#include "core/PedestrianManager.h"
#include "core/Simulation.h"
#include "core/Vehicle.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <vector>

namespace credible_freeze {

inline bool hasLeaderAhead(const Vehicle& vehicle, const std::vector<Vehicle>& vehicles) {
    for (const auto& other : vehicles) {
        if (other.getId() == vehicle.getId() || other.isFinished()) {
            continue;
        }
        if (other.getCurrentRoadSegmentId() == vehicle.getCurrentRoadSegmentId() &&
            other.getLaneIndex() == vehicle.getLaneIndex() &&
            other.getPositionOnRoad() > vehicle.getPositionOnRoad() + 0.01) {
            return true;
        }
    }
    return false;
}

inline bool hasActiveCrossingObstacle(
    const Vehicle& vehicle,
    const City& city,
    Simulation& sim,
    const std::string& reason
) {
    const RoadSegment* road = city.findRoadSegmentById(vehicle.getCurrentRoadSegmentId());
    if (road == nullptr) {
        return false;
    }

    // Prefer the same cap PedestrianManager uses (waiting yield + on-road).
    std::string pedReason;
    const double pedCap = sim.getPedestrianManager().computeCrossingStopCap(
        vehicle,
        *road,
        sim.getPedestrianCrossings(),
        sim.getTrafficLights(),
        &pedReason
    );
    if (std::isfinite(pedCap)) {
        if (reason == "pedestrian_on_road") {
            return pedReason == "pedestrian_on_road" || pedReason.empty();
        }
        if (reason == "pedestrian_crossing") {
            return pedReason == "pedestrian_crossing" ||
                   pedReason == "pedestrian_on_road" ||
                   pedReason.empty();
        }
    }

    const int roadId = vehicle.getCurrentRoadSegmentId();
    for (const auto& crossing : sim.getPedestrianCrossings()) {
        if (crossing.getRoadSegmentId() != roadId) {
            continue;
        }
        const bool ahead = vehicle.getPositionOnRoad() < crossing.getPositionOnRoad();
        const bool onRoad = sim.getPedestrianManager().hasPedestrianOnRoadway(crossing.getId());
        if (reason == "pedestrian_on_road") {
            if (onRoad && (ahead || crossing.isActive())) {
                return true;
            }
        } else if (reason == "pedestrian_crossing") {
            if ((crossing.isActive() || onRoad) && ahead) {
                return true;
            }
        }
    }
    return false;
}

inline bool hasLaneBlockObstacle(
    const Vehicle& vehicle,
    const City& city,
    const Simulation& sim,
    const std::string& reason
) {
    const RoadSegment* road = city.findRoadSegmentById(vehicle.getCurrentRoadSegmentId());
    if (road == nullptr) {
        return false;
    }
    std::string reported;
    const double cap = sim.getLaneBlockageManager().computeStopCap(
        vehicle, *road, sim.getTime(), &reported
    );
    if (!std::isfinite(cap)) {
        return false;
    }
    if (reason == "lane_blockage") {
        return reported == "lane_blockage" || reported.empty();
    }
    if (reason == "zipper_wait") {
        return reported == "zipper_wait";
    }
    return false;
}

// True when determineStopReason is backed by a matching live obstacle / terminal state.
inline bool reasonMatchesCurrentObstacle(
    const Vehicle& vehicle,
    const City& city,
    Simulation& sim,
    CollisionDetector& detector
) {
    const std::string reason = detector.determineStopReason(
        vehicle,
        city,
        sim.getTrafficLights(),
        sim.getPedestrianCrossings(),
        sim.getVehicles(),
        &sim.getIntersectionRules(),
        &sim.getPedestrianManager(),
        &sim.getLaneBlockageManager(),
        sim.getTime()
    );

    if (reason.empty()) {
        return false;
    }

    if (reason == "destination_reached" || reason == "route_invalid") {
        return true;
    }

    if (reason == "vehicle_ahead") {
        return hasLeaderAhead(vehicle, sim.getVehicles());
    }

    if (reason == "red_light") {
        return detector.hasRedLightForRoad(sim.getTrafficLights(), vehicle.getCurrentRoadSegmentId());
    }

    if (reason == "pedestrian_crossing" || reason == "pedestrian_on_road") {
        return hasActiveCrossingObstacle(vehicle, city, sim, reason);
    }

    if (reason == "lane_blockage" || reason == "zipper_wait") {
        return hasLaneBlockObstacle(vehicle, city, sim, reason);
    }

    if (reason == "blocked_next_road" ||
        reason == "yield_priority" ||
        reason == "intersection_conflict") {
        const RoadSegment* road = city.findRoadSegmentById(vehicle.getCurrentRoadSegmentId());
        if (road == nullptr) {
            return false;
        }
        const int nextId = vehicle.getNextRoadSegmentId();
        if (nextId < 0) {
            return reason == "blocked_next_road" && false;
        }

        int entryLane = vehicle.getLaneIndex();
        const RoadSegment* nextRoad = city.findRoadSegmentById(nextId);
        if (nextRoad != nullptr) {
            entryLane = std::min(vehicle.getLaneIndex(), std::max(0, nextRoad->getLaneCount() - 1));
        }
        const bool spaceBlocked = !detector.canEnterRoad(nextId, entryLane, sim.getVehicles());
        const bool atEntry =
            detector.isAtIntersectionEntry(vehicle.getPositionOnRoad(), road->getLength());

        if (reason == "blocked_next_road") {
            if (spaceBlocked) {
                return true;
            }
            // Still waiting at the stop line while junction/space logic says wait.
            if (atEntry && city.areRoadsConnected(vehicle.getCurrentRoadSegmentId(), nextId)) {
                const JunctionBlockReason block = sim.getIntersectionRules().evaluateEntry(
                    vehicle,
                    city,
                    sim.getTrafficLights(),
                    sim.getVehicles(),
                    !spaceBlocked
                );
                return block != JunctionBlockReason::None;
            }
            return false;
        }

        if (!atEntry || !city.areRoadsConnected(vehicle.getCurrentRoadSegmentId(), nextId)) {
            return false;
        }
        const JunctionBlockReason block = sim.getIntersectionRules().evaluateEntry(
            vehicle,
            city,
            sim.getTrafficLights(),
            sim.getVehicles(),
            true
        );
        if (reason == "yield_priority") {
            return block == JunctionBlockReason::YieldPriority;
        }
        return block == JunctionBlockReason::IntersectionConflict;
    }

    return false;
}

inline int countUnexplainedFreezes(
    const City& city,
    Simulation& sim,
    const std::map<int, int>& stuckTicks,
    int stuckThresholdTicks
) {
    CollisionDetector detector(4.0);
    detector.setMinVehicleGap(4.0);
    int unexplained = 0;

    for (const auto& entry : stuckTicks) {
        if (entry.second < stuckThresholdTicks) {
            continue;
        }

        const Vehicle* v = nullptr;
        for (const auto& vehicle : sim.getVehicles()) {
            if (vehicle.getId() == entry.first) {
                v = &vehicle;
                break;
            }
        }
        if (v == nullptr) {
            continue;
        }

        if (!reasonMatchesCurrentObstacle(*v, city, sim, detector)) {
            ++unexplained;
        }
    }
    return unexplained;
}

inline int countActiveReservations(const Simulation& sim) {
    int count = 0;
    for (const auto& entry : sim.getIntersectionRules().getReservations()) {
        for (const auto& reservation : entry.second) {
            if (reservation.isActive()) {
                ++count;
            }
        }
    }
    return count;
}

// Reservations whose vehicle is finished or no longer present are stale.
inline int countStaleReservations(const Simulation& sim) {
    int stale = 0;
    for (const auto& entry : sim.getIntersectionRules().getReservations()) {
        for (const auto& reservation : entry.second) {
            if (!reservation.isActive()) {
                continue;
            }
            bool foundAlive = false;
            for (const auto& vehicle : sim.getVehicles()) {
                if (vehicle.getId() == reservation.vehicleId && !vehicle.isFinished()) {
                    foundAlive = true;
                    break;
                }
            }
            if (!foundAlive) {
                ++stale;
            }
        }
    }
    return stale;
}

} // namespace credible_freeze
