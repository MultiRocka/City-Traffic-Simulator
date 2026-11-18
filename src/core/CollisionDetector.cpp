#include "core/CollisionDetector.h"
#include "core/LaneBlockage.h"
#include "core/PedestrianManager.h"
#include <algorithm>
#include <cmath>
#include <limits>

using namespace std;

CollisionDetector::CollisionDetector(double minVehicleGap)
    : minVehicleGap(minVehicleGap) {
}

void CollisionDetector::setMinVehicleGap(double newMinVehicleGap) {
    minVehicleGap = newMinVehicleGap;
}

double CollisionDetector::getMinVehicleGap() const {
    return minVehicleGap;
}

bool CollisionDetector::isGreenForRoad(const vector<TrafficLight>& trafficLights, int roadId) const {
    for (const auto& light : trafficLights) {
        if (light.getRoadSegmentId() == roadId) {
            return light.isGreen();
        }
    }

    return true;
}

bool CollisionDetector::hasRedLightForRoad(const vector<TrafficLight>& trafficLights, int roadId) const {
    for (const auto& light : trafficLights) {
        if (light.getRoadSegmentId() == roadId) {
            return light.isRed();
        }
    }

    return false;
}

bool CollisionDetector::canEnterRoad(int nextRoadId, int laneIndex, const vector<Vehicle>& vehicles) const {
    for (const auto& vehicle : vehicles) {
        if (vehicle.isFinished() ||
            vehicle.getCurrentRoadSegmentId() != nextRoadId ||
            vehicle.getLaneIndex() != laneIndex) {
            continue;
        }

        // Block if the occupant still occupies the entrance zone.
        if (vehicle.getPositionOnRoad() < minVehicleGap + vehicle.getLength()) {
            return false;
        }
    }

    return true;
}

bool CollisionDetector::canEnterRoad(int nextRoadId, const vector<Vehicle>& vehicles) const {
    return canEnterRoad(nextRoadId, 0, vehicles);
}

bool CollisionDetector::isAtIntersectionEntry(double positionOnRoad, double roadLength) const {
    return positionOnRoad >= roadLength - SimConstants::ROAD_END_EPSILON;
}

const Vehicle* CollisionDetector::findPredecessor(
    const Vehicle& vehicle,
    const vector<Vehicle>& vehicles
) const {
    const Vehicle* best = nullptr;
    const int roadId = vehicle.getCurrentRoadSegmentId();
    const int lane = vehicle.getLaneIndex();
    const double position = vehicle.getPositionOnRoad();

    for (const auto& other : vehicles) {
        if (other.getId() == vehicle.getId() || other.isFinished()) {
            continue;
        }

        if (other.getCurrentRoadSegmentId() != roadId || other.getLaneIndex() != lane) {
            continue;
        }

        if (other.getPositionOnRoad() <= position) {
            continue;
        }

        if (best == nullptr || other.getPositionOnRoad() < best->getPositionOnRoad()) {
            best = &other;
        }
    }

    return best;
}

const Vehicle* CollisionDetector::findPredecessorOnRoad(
    const Vehicle& vehicle,
    const vector<Vehicle*>& sameRoad
) const {
    const Vehicle* best = nullptr;
    const int lane = vehicle.getLaneIndex();
    const double position = vehicle.getPositionOnRoad();

    for (const Vehicle* otherPtr : sameRoad) {
        if (otherPtr == nullptr) {
            continue;
        }
        const Vehicle& other = *otherPtr;
        if (other.getId() == vehicle.getId() || other.isFinished()) {
            continue;
        }
        if (other.getLaneIndex() != lane) {
            continue;
        }
        if (other.getPositionOnRoad() <= position) {
            continue;
        }
        if (best == nullptr || other.getPositionOnRoad() < best->getPositionOnRoad()) {
            best = &other;
        }
    }

    return best;
}

double CollisionDetector::computeLeaderStopPosition(const Vehicle& leader) const {
    return leader.getPositionOnRoad() - leader.getLength() - minVehicleGap;
}

bool CollisionDetector::vehiclesOverlapLongitudinally(
    double frontA,
    double lengthA,
    double frontB,
    double lengthB
) const {
    const double rearA = frontA - lengthA;
    const double rearB = frontB - lengthB;
    return !(frontA + minVehicleGap <= rearB || frontB + minVehicleGap <= rearA);
}

bool CollisionDetector::canChangeLane(
    const Vehicle& vehicle,
    int targetLane,
    const RoadSegment& road,
    const vector<Vehicle>& vehicles
) const {
    if (road.getLaneCount() <= 1) {
        return false;
    }

    if (targetLane < 0 || targetLane >= road.getLaneCount()) {
        return false;
    }

    if (targetLane == vehicle.getLaneIndex()) {
        return false;
    }

    if (vehicle.getLaneChangeCooldown() > 0.0) {
        return false;
    }

    if (vehicle.getCurrentRoadSegmentId() != road.getId()) {
        return false;
    }

    const double egoFront = vehicle.getPositionOnRoad();
    const double egoLength = vehicle.getLength();

    for (const auto& other : vehicles) {
        if (other.getId() == vehicle.getId() || other.isFinished()) {
            continue;
        }

        if (other.getCurrentRoadSegmentId() != road.getId() || other.getLaneIndex() != targetLane) {
            continue;
        }

        if (vehiclesOverlapLongitudinally(
                egoFront, egoLength,
                other.getPositionOnRoad(), other.getLength())) {
            return false;
        }
    }

    return true;
}

double CollisionDetector::computeMaxAllowedPosition(
    const RoadSegment& roadSegment,
    const vector<TrafficLight>& trafficLights,
    const vector<PedestrianCrossing>& pedestrianCrossings,
    double vehiclePosition,
    double vehicleAheadPosition,
    double vehicleAheadLength,
    PedestrianManager* pedManager,
    const Vehicle* subjectVehicle,
    LaneBlockageManager* laneBlocks,
    double currentTime
) const {
    double maxAllowedPosition = roadSegment.getLength();

    if (hasRedLightForRoad(trafficLights, roadSegment.getId())) {
        maxAllowedPosition = min(
            maxAllowedPosition,
            roadSegment.getLength() - SimConstants::RED_LIGHT_STOP_DISTANCE
        );
    }

    // Legacy Timed active flag (and entity occupancy for non-Timed).
    for (const auto& crossing : pedestrianCrossings) {
        if (crossing.getRoadSegmentId() != roadSegment.getId() || !crossing.isActive()) {
            continue;
        }

        if (vehiclePosition >= crossing.getPositionOnRoad()) {
            continue;
        }

        maxAllowedPosition = min(
            maxAllowedPosition,
            crossing.getPositionOnRoad() - SimConstants::CROSSING_STOP_DISTANCE
        );
    }

    // Stage 4: on-road peds + unsignalized waiting yield (refusal-aware).
    if (pedManager != nullptr && subjectVehicle != nullptr) {
        const double pedCap = pedManager->computeCrossingStopCap(
            *subjectVehicle,
            roadSegment,
            pedestrianCrossings,
            trafficLights,
            nullptr
        );
        if (isfinite(pedCap)) {
            maxAllowedPosition = min(maxAllowedPosition, pedCap);
        }
    }

    // Stage 5: lane blockage stop + zipper wait near merge.
    if (laneBlocks != nullptr && subjectVehicle != nullptr) {
        const double blockCap = laneBlocks->computeStopCap(
            *subjectVehicle,
            roadSegment,
            currentTime,
            nullptr
        );
        if (isfinite(blockCap)) {
            maxAllowedPosition = min(maxAllowedPosition, blockCap);
        }
    }

    if (vehicleAheadPosition >= 0.0) {
        maxAllowedPosition = min(
            maxAllowedPosition,
            vehicleAheadPosition - vehicleAheadLength - minVehicleGap
        );
    }

    if (maxAllowedPosition < vehiclePosition) {
        maxAllowedPosition = vehiclePosition;
    }

    return maxAllowedPosition;
}

string CollisionDetector::determineStopReason(
    const Vehicle& vehicle,
    const City& city,
    const vector<TrafficLight>& trafficLights,
    const vector<PedestrianCrossing>& pedestrianCrossings,
    const vector<Vehicle>& vehicles,
    const IntersectionRules* intersectionRules,
    PedestrianManager* pedManager,
    LaneBlockageManager* laneBlocks,
    double currentTime
) const {
    int roadId = vehicle.getCurrentRoadSegmentId();

    if (roadId == -1) {
        return "route_invalid";
    }

    const RoadSegment* road = city.findRoadSegmentById(roadId);

    if (road == nullptr) {
        return "route_invalid";
    }

    double roadLength = road->getLength();
    double position = vehicle.getPositionOnRoad();
    int nextRoadId = vehicle.getNextRoadSegmentId();
    int entryLane = vehicle.getLaneIndex();
    if (nextRoadId != -1) {
        const RoadSegment* nextRoad = city.findRoadSegmentById(nextRoadId);
        if (nextRoad != nullptr) {
            entryLane = min(vehicle.getLaneIndex(), max(0, nextRoad->getLaneCount() - 1));
        }
    }

    const Vehicle* leader = findPredecessor(vehicle, vehicles);

    const double inf = numeric_limits<double>::infinity();
    double redCap = inf;
    if (hasRedLightForRoad(trafficLights, roadId)) {
        redCap = roadLength - SimConstants::RED_LIGHT_STOP_DISTANCE;
    }

    double crossingCap = inf;
    string crossingReason = "pedestrian_crossing";
    for (const auto& crossing : pedestrianCrossings) {
        if (crossing.getRoadSegmentId() != roadId || !crossing.isActive()) {
            continue;
        }

        if (position >= crossing.getPositionOnRoad()) {
            continue;
        }

        crossingCap = min(
            crossingCap,
            crossing.getPositionOnRoad() - SimConstants::CROSSING_STOP_DISTANCE
        );
        if (pedManager != nullptr && pedManager->hasPedestrianOnRoadway(crossing.getId())) {
            crossingReason = "pedestrian_on_road";
        }
    }

    if (pedManager != nullptr) {
        string pedReason;
        const double pedCap = pedManager->computeCrossingStopCap(
            vehicle,
            *road,
            pedestrianCrossings,
            trafficLights,
            &pedReason
        );
        if (isfinite(pedCap) && pedCap <= crossingCap) {
            crossingCap = pedCap;
            if (!pedReason.empty()) {
                crossingReason = pedReason;
            }
        }
    }

    double aheadCap = inf;
    if (leader != nullptr) {
        aheadCap = computeLeaderStopPosition(*leader);
    }

    double blockCap = inf;
    string blockReason = "lane_blockage";
    if (laneBlocks != nullptr) {
        string reason;
        const double cap = laneBlocks->computeStopCap(vehicle, *road, currentTime, &reason);
        if (isfinite(cap)) {
            blockCap = cap;
            if (!reason.empty()) {
                blockReason = reason;
            }
        }
    }

    const double roadCap = roadLength;
    const double matchEps = SimConstants::CONSTRAINT_MATCH_EPSILON;
    const double rawMaxAllowed = min(
        roadCap,
        min(redCap, min(crossingCap, min(aheadCap, blockCap)))
    );

    auto nearlyEqual = [&](double a, double b) {
        return isfinite(a) && isfinite(b) && abs(a - b) <= matchEps;
    };

    auto intersectionWaitReason = [&]() -> string {
        if (nextRoadId == -1) {
            return "destination_reached";
        }

        if (!city.areRoadsConnected(roadId, nextRoadId)) {
            return "route_invalid";
        }

        if (!isGreenForRoad(trafficLights, roadId)) {
            return "red_light";
        }

        if (!canEnterRoad(nextRoadId, entryLane, vehicles)) {
            return "blocked_next_road";
        }

        if (intersectionRules != nullptr) {
            const JunctionBlockReason block = intersectionRules->evaluateEntry(
                vehicle,
                city,
                trafficLights,
                vehicles,
                true
            );
            if (block == JunctionBlockReason::YieldPriority ||
                block == JunctionBlockReason::IntersectionConflict) {
                return IntersectionRules::blockReasonToStopReason(block);
            }
        }

        return "blocked_next_road";
    };

    if (isAtIntersectionEntry(position, roadLength) && nearlyEqual(rawMaxAllowed, roadCap)) {
        return intersectionWaitReason();
    }

    if (nearlyEqual(redCap, rawMaxAllowed)) {
        return "red_light";
    }

    if (nearlyEqual(crossingCap, rawMaxAllowed)) {
        return crossingReason;
    }

    if (nearlyEqual(blockCap, rawMaxAllowed)) {
        return blockReason;
    }

    if (nearlyEqual(aheadCap, rawMaxAllowed)) {
        return "vehicle_ahead";
    }

    if (isAtIntersectionEntry(position, roadLength)) {
        return intersectionWaitReason();
    }

    if (nextRoadId != -1 && !city.areRoadsConnected(roadId, nextRoadId)) {
        return "route_invalid";
    }

    if (isfinite(blockCap)) {
        return blockReason;
    }

    if (isfinite(aheadCap)) {
        return "vehicle_ahead";
    }

    if (isfinite(redCap)) {
        return "red_light";
    }

    if (isfinite(crossingCap)) {
        return crossingReason;
    }

    return intersectionWaitReason();
}
