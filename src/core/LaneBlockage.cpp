#include "core/LaneBlockage.h"
#include "core/CollisionDetector.h"
#include <algorithm>
#include <cmath>
#include <limits>

using namespace std;

bool LaneBlockage::isActive(double currentTime) const {
    return currentTime + 1e-9 >= startTime && currentTime < startTime + duration;
}

bool LaneBlockage::isExpired(double currentTime) const {
    return currentTime + 1e-9 >= startTime + duration;
}

double LaneBlockage::endPosition() const {
    return positionAlongRoad + length;
}

double LaneBlockage::stopLine() const {
    return positionAlongRoad - SimConstants::LANE_BLOCK_STOP_DISTANCE;
}

LaneBlockageManager::LaneBlockageManager()
    : rngState(defaultSeed),
      configuredSeed(defaultSeed),
      nextBlockageId(1),
      timeUntilNextRandom(0.0) {
    randomConfig.seed = defaultSeed;
}

void LaneBlockageManager::clear() {
    blockages.clear();
    zipperLastGrantedLane.clear();
    zipperGrantedVehicleIds.clear();
    zipperWaitingOpenLaneIds.clear();
    nextBlockageId = 1;
    timeUntilNextRandom = 0.0;
    rngState = configuredSeed;
    randomConfig = RandomBlockConfig{};
    randomConfig.seed = configuredSeed;
}

void LaneBlockageManager::setSeed(uint32_t seed) {
    configuredSeed = seed;
    rngState = seed;
    randomConfig.seed = seed;
}

uint32_t LaneBlockageManager::getSeed() const {
    return configuredSeed;
}

void LaneBlockageManager::setRandomConfig(const RandomBlockConfig& config) {
    randomConfig = config;
    configuredSeed = config.seed;
    rngState = config.seed;
}

const RandomBlockConfig& LaneBlockageManager::getRandomConfig() const {
    return randomConfig;
}

const vector<LaneBlockage>& LaneBlockageManager::getBlockages() const {
    return blockages;
}

vector<LaneBlockage>& LaneBlockageManager::getBlockages() {
    return blockages;
}

uint32_t LaneBlockageManager::nextRandom() {
    rngState = rngState * 1664525u + 1013904223u;
    return rngState;
}

double LaneBlockageManager::nextUnitRandom() {
    return static_cast<double>(nextRandom()) / 4294967296.0;
}

double LaneBlockageManager::nextRange(double minValue, double maxValue) {
    if (maxValue <= minValue) {
        return minValue;
    }
    return minValue + (maxValue - minValue) * nextUnitRandom();
}

bool LaneBlockageManager::wouldOverlapVehicle(
    int roadId,
    int laneIndex,
    double spanStart,
    double spanEnd,
    const vector<Vehicle>& vehicles
) const {
    const double start = min(spanStart, spanEnd);
    const double end = max(spanStart, spanEnd);

    for (const auto& vehicle : vehicles) {
        if (vehicle.isFinished() ||
            vehicle.getCurrentRoadSegmentId() != roadId ||
            vehicle.getLaneIndex() != laneIndex) {
            continue;
        }

        const double front = vehicle.getPositionOnRoad();
        const double rear = front - vehicle.getLength();
        if (!(front < start || rear > end)) {
            return true;
        }
    }

    return false;
}

int LaneBlockageManager::addBlockage(
    int roadId,
    int laneIndex,
    double positionAlongRoad,
    double length,
    double startTime,
    double duration,
    const City& city,
    const vector<Vehicle>& vehicles
) {
    const RoadSegment* road = city.findRoadSegmentById(roadId);
    if (road == nullptr || length <= 0.0 || duration <= 0.0) {
        return -1;
    }

    if (laneIndex < 0 || laneIndex >= road->getLaneCount()) {
        return -1;
    }

    if (positionAlongRoad < 0.0 || positionAlongRoad >= road->getLength()) {
        return -1;
    }

    const double clampedLength = min(length, road->getLength() - positionAlongRoad);
    if (clampedLength <= 0.0) {
        return -1;
    }

    if (wouldOverlapVehicle(
            roadId,
            laneIndex,
            positionAlongRoad,
            positionAlongRoad + clampedLength,
            vehicles)) {
        return -1;
    }

    LaneBlockage block;
    block.id = nextBlockageId++;
    block.roadId = roadId;
    block.laneIndex = laneIndex;
    block.positionAlongRoad = positionAlongRoad;
    block.length = clampedLength;
    block.startTime = startTime;
    block.duration = duration;
    blockages.push_back(block);
    return block.id;
}

bool LaneBlockageManager::clearBlockage(int blockageId) {
    const auto it = remove_if(
        blockages.begin(),
        blockages.end(),
        [blockageId](const LaneBlockage& block) {
            return block.id == blockageId;
        });
    if (it == blockages.end()) {
        return false;
    }
    blockages.erase(it, blockages.end());
    zipperLastGrantedLane.erase(blockageId);
    return true;
}

void LaneBlockageManager::clearAllBlockages() {
    blockages.clear();
    zipperLastGrantedLane.clear();
    zipperGrantedVehicleIds.clear();
    zipperWaitingOpenLaneIds.clear();
}

bool LaneBlockageManager::hasActiveBlockOnLane(int roadId, int laneIndex, double currentTime) const {
    for (const auto& block : blockages) {
        if (block.roadId == roadId &&
            block.laneIndex == laneIndex &&
            block.isActive(currentTime)) {
            return true;
        }
    }
    return false;
}

bool LaneBlockageManager::areAllLanesBlocked(const RoadSegment& road, double currentTime) const {
    const int lanes = max(1, road.getLaneCount());
    for (int lane = 0; lane < lanes; ++lane) {
        if (!hasActiveBlockOnLane(road.getId(), lane, currentTime)) {
            return false;
        }
    }
    return true;
}

bool LaneBlockageManager::blocksEntrance(
    int roadId,
    int laneIndex,
    double currentTime,
    double clearance
) const {
    for (const auto& block : blockages) {
        if (!block.isActive(currentTime) ||
            block.roadId != roadId ||
            block.laneIndex != laneIndex) {
            continue;
        }
        if (block.positionAlongRoad < clearance) {
            return true;
        }
    }
    return false;
}

bool LaneBlockageManager::isPositionBlocked(
    int roadId,
    int laneIndex,
    double position,
    double currentTime
) const {
    for (const auto& block : blockages) {
        if (!block.isActive(currentTime) ||
            block.roadId != roadId ||
            block.laneIndex != laneIndex) {
            continue;
        }
        if (position >= block.positionAlongRoad && position <= block.endPosition()) {
            return true;
        }
    }
    return false;
}

bool LaneBlockageManager::hasZipperGrant(int vehicleId) const {
    return zipperGrantedVehicleIds.find(vehicleId) != zipperGrantedVehicleIds.end();
}

vector<int> LaneBlockageManager::openLanesForBlock(
    const LaneBlockage& block,
    const RoadSegment& road,
    double currentTime
) const {
    vector<int> open;
    const int lanes = max(1, road.getLaneCount());
    for (int lane = 0; lane < lanes; ++lane) {
        if (lane == block.laneIndex) {
            continue;
        }

        bool blocked = false;
        for (const auto& other : blockages) {
            if (!other.isActive(currentTime) ||
                other.roadId != block.roadId ||
                other.laneIndex != lane) {
                continue;
            }
            if (other.positionAlongRoad <= block.positionAlongRoad &&
                other.endPosition() >= block.positionAlongRoad) {
                blocked = true;
                break;
            }
        }
        if (!blocked) {
            open.push_back(lane);
        }
    }
    return open;
}

const Vehicle* LaneBlockageManager::headInMergeZone(
    int roadId,
    int laneIndex,
    double blockStart,
    const vector<Vehicle>& vehicles
) const {
    const double zoneStart = blockStart - SimConstants::ZIPPER_MERGE_ZONE;
    const Vehicle* best = nullptr;

    for (const auto& vehicle : vehicles) {
        if (vehicle.isFinished() ||
            vehicle.getCurrentRoadSegmentId() != roadId ||
            vehicle.getLaneIndex() != laneIndex) {
            continue;
        }

        const double pos = vehicle.getPositionOnRoad();
        if (pos < zoneStart || pos >= blockStart - 1e-6) {
            continue;
        }

        if (best == nullptr || pos > best->getPositionOnRoad()) {
            best = &vehicle;
        }
    }

    return best;
}

void LaneBlockageManager::logStateChanges(double currentTime, EventLog& eventLog) {
    for (auto& block : blockages) {
        if (block.isActive(currentTime) && !block.hasLoggedStart) {
            block.hasLoggedStart = true;
            eventLog.addEvent(
                currentTime,
                "lane_blockage_started",
                -1,
                block.roadId,
                "lane_blockage",
                "Lane blockage started on lane " + to_string(block.laneIndex)
            );
        }

        if (block.isExpired(currentTime) && block.hasLoggedStart && !block.hasLoggedEnd) {
            block.hasLoggedEnd = true;
            eventLog.addEvent(
                currentTime,
                "lane_blockage_ended",
                -1,
                block.roadId,
                "lane_blockage_cleared",
                "Lane blockage ended on lane " + to_string(block.laneIndex)
            );
            zipperLastGrantedLane.erase(block.id);
        }
    }
}

void LaneBlockageManager::pruneExpired(double currentTime) {
    blockages.erase(
        remove_if(
            blockages.begin(),
            blockages.end(),
            [currentTime](const LaneBlockage& block) {
                return block.isExpired(currentTime) && block.hasLoggedEnd;
            }),
        blockages.end()
    );
}

void LaneBlockageManager::maybeSpawnRandom(
    double dt,
    double currentTime,
    const City& city,
    const vector<Vehicle>& vehicles
) {
    if (!randomConfig.enabled) {
        return;
    }

    timeUntilNextRandom -= dt;
    if (timeUntilNextRandom > 0.0) {
        return;
    }

    timeUntilNextRandom = nextRange(randomConfig.minInterval, randomConfig.maxInterval);

    const auto& roads = city.getRoadSegments();
    if (roads.empty()) {
        return;
    }

    const size_t roadIndex = static_cast<size_t>(nextRandom() % static_cast<uint32_t>(roads.size()));
    const RoadSegment& road = roads[roadIndex];
    const int lanes = max(1, road.getLaneCount());
    const int lane = static_cast<int>(nextRandom() % static_cast<uint32_t>(lanes));

    const double frac = nextRange(randomConfig.minPositionFraction, randomConfig.maxPositionFraction);
    const double position = max(0.0, min(road.getLength() * 0.95, road.getLength() * frac));
    const double length = min(
        nextRange(randomConfig.minLength, randomConfig.maxLength),
        max(0.5, road.getLength() - position)
    );
    const double duration = nextRange(randomConfig.minDuration, randomConfig.maxDuration);

    addBlockage(
        road.getId(),
        lane,
        position,
        length,
        currentTime,
        duration,
        city,
        vehicles
    );
}

void LaneBlockageManager::refreshZipperGrants(
    double currentTime,
    const City& city,
    const vector<Vehicle>& vehicles,
    const CollisionDetector* collisionDetector
) {
    zipperGrantedVehicleIds.clear();
    zipperWaitingOpenLaneIds.clear();

    for (const auto& block : blockages) {
        if (!block.isActive(currentTime)) {
            continue;
        }

        const RoadSegment* road = city.findRoadSegmentById(block.roadId);
        if (road == nullptr || road->getLaneCount() <= 1) {
            continue;
        }

        const vector<int> openLanes = openLanesForBlock(block, *road, currentTime);
        if (openLanes.empty()) {
            continue;
        }

        const Vehicle* closedHead = headInMergeZone(
            block.roadId,
            block.laneIndex,
            block.positionAlongRoad,
            vehicles
        );

        vector<pair<int, const Vehicle*>> openHeads;
        for (int openLane : openLanes) {
            const Vehicle* head = headInMergeZone(
                block.roadId,
                openLane,
                block.positionAlongRoad,
                vehicles
            );
            if (head != nullptr) {
                openHeads.push_back({openLane, head});
            }
        }

        // Empty closed queue → open lane does not wait for a ghost car.
        if (closedHead == nullptr) {
            for (const auto& entry : openHeads) {
                zipperGrantedVehicleIds.insert(entry.second->getId());
            }
            continue;
        }

        // Empty open queue → closed lane may merge without waiting for ghost open traffic.
        if (openHeads.empty()) {
            zipperGrantedVehicleIds.insert(closedHead->getId());
            continue;
        }

        auto closedCanMerge = [&]() -> bool {
            if (collisionDetector == nullptr) {
                return true;
            }
            for (int openLane : openLanes) {
                if (collisionDetector->canChangeLane(*closedHead, openLane, *road, vehicles)) {
                    return true;
                }
            }
            return false;
        };

        // Both queues present → fair alternate; no monopoly.
        int lastLane = -1;
        const auto lastIt = zipperLastGrantedLane.find(block.id);
        if (lastIt != zipperLastGrantedLane.end()) {
            lastLane = lastIt->second;
        }

        const bool preferClosed = (lastLane != block.laneIndex);
        if (preferClosed) {
            if (closedCanMerge()) {
                zipperGrantedVehicleIds.insert(closedHead->getId());
                zipperLastGrantedLane[block.id] = block.laneIndex;
                for (const auto& entry : openHeads) {
                    zipperWaitingOpenLaneIds.insert(entry.second->getId());
                }
            } else {
                // No safe gap → closed waits at block; open proceeds (do not stall for ghost merge).
                for (const auto& entry : openHeads) {
                    zipperGrantedVehicleIds.insert(entry.second->getId());
                }
            }
        } else {
            const Vehicle* bestOpen = openHeads.front().second;
            int bestLane = openHeads.front().first;
            for (const auto& entry : openHeads) {
                if (entry.second->getPositionOnRoad() > bestOpen->getPositionOnRoad()) {
                    bestOpen = entry.second;
                    bestLane = entry.first;
                }
            }
            zipperGrantedVehicleIds.insert(bestOpen->getId());
            zipperLastGrantedLane[block.id] = bestLane;
            for (const auto& entry : openHeads) {
                if (entry.second->getId() != bestOpen->getId()) {
                    zipperWaitingOpenLaneIds.insert(entry.second->getId());
                }
            }
        }
    }
}

void LaneBlockageManager::update(
    double dt,
    double currentTime,
    const City& city,
    const vector<Vehicle>& vehicles,
    EventLog& eventLog,
    const CollisionDetector* collisionDetector
) {
    logStateChanges(currentTime, eventLog);
    pruneExpired(currentTime);
    maybeSpawnRandom(dt, currentTime, city, vehicles);
    logStateChanges(currentTime, eventLog);
    refreshZipperGrants(currentTime, city, vehicles, collisionDetector);
}

double LaneBlockageManager::computeStopCap(
    const Vehicle& vehicle,
    const RoadSegment& road,
    double currentTime,
    string* stopReasonOut
) const {
    double cap = numeric_limits<double>::infinity();
    const int roadId = road.getId();
    const int lane = vehicle.getLaneIndex();
    const double position = vehicle.getPositionOnRoad();

    for (const auto& block : blockages) {
        if (!block.isActive(currentTime) || block.roadId != roadId) {
            continue;
        }

        if (block.laneIndex != lane) {
            continue;
        }

        if (position >= block.positionAlongRoad) {
            if (position <= block.endPosition()) {
                cap = min(cap, position);
                if (stopReasonOut != nullptr) {
                    *stopReasonOut = "lane_blockage";
                }
            }
            continue;
        }

        const double stopAt = block.stopLine();
        if (stopAt < cap) {
            cap = stopAt;
            if (stopReasonOut != nullptr) {
                *stopReasonOut = "lane_blockage";
            }
        }
    }

    if (zipperWaitingOpenLaneIds.find(vehicle.getId()) != zipperWaitingOpenLaneIds.end()) {
        for (const auto& block : blockages) {
            if (!block.isActive(currentTime) || block.roadId != roadId) {
                continue;
            }
            if (block.laneIndex == lane) {
                continue;
            }
            const double mergeLine = block.stopLine();
            if (position < block.positionAlongRoad && mergeLine < cap) {
                cap = mergeLine;
                if (stopReasonOut != nullptr) {
                    *stopReasonOut = "zipper_wait";
                }
            }
        }
    }

    return cap;
}

void LaneBlockageManager::tryEscapeLaneChanges(
    double currentTime,
    const City& city,
    vector<Vehicle>& vehicles,
    const CollisionDetector& collisionDetector,
    EventLog& eventLog
) {
    for (auto& vehicle : vehicles) {
        if (vehicle.isFinished()) {
            continue;
        }

        if (zipperGrantedVehicleIds.find(vehicle.getId()) == zipperGrantedVehicleIds.end()) {
            continue;
        }

        const RoadSegment* road = city.findRoadSegmentById(vehicle.getCurrentRoadSegmentId());
        if (road == nullptr || road->getLaneCount() <= 1) {
            continue;
        }

        bool onClosedLane = false;
        double blockStart = 0.0;
        LaneBlockage closedBlock;
        for (const auto& block : blockages) {
            if (!block.isActive(currentTime)) {
                continue;
            }
            if (block.roadId == vehicle.getCurrentRoadSegmentId() &&
                block.laneIndex == vehicle.getLaneIndex()) {
                onClosedLane = true;
                blockStart = block.positionAlongRoad;
                closedBlock = block;
                break;
            }
        }

        if (!onClosedLane) {
            continue;
        }

        const double pos = vehicle.getPositionOnRoad();
        if (pos < blockStart - SimConstants::ZIPPER_MERGE_ZONE || pos >= blockStart) {
            continue;
        }

        const vector<int> openLanes = openLanesForBlock(closedBlock, *road, currentTime);

        int bestTarget = -1;
        int bestDist = 999;
        for (int lane : openLanes) {
            const int dist = abs(lane - vehicle.getLaneIndex());
            if (dist < bestDist) {
                bestDist = dist;
                bestTarget = lane;
            }
        }

        if (bestTarget < 0) {
            continue;
        }

        if (!collisionDetector.canChangeLane(vehicle, bestTarget, *road, vehicles)) {
            continue;
        }

        vehicle.applyLaneChange(bestTarget, SimConstants::LANE_CHANGE_COOLDOWN);
        eventLog.addEvent(
            currentTime,
            "vehicle_lane_change",
            vehicle.getId(),
            vehicle.getCurrentRoadSegmentId(),
            "lane_blockage_escape",
            "Vehicle changed lane to escape blockage"
        );
    }
}
