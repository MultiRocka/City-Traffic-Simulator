#include "core/PedestrianManager.h"
#include "core/CollisionDetector.h"
#include "core/RoadSegment.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

using namespace std;

namespace {
constexpr double kStoppedSpeed = 0.35;
// Crosswalk strip only — must stay tighter than CROSSING_STOP_DISTANCE so
// cars legally stopped at the stop line are not treated as occupying the roadway.
constexpr double kConflictHalfWidth = 0.75;
}

PedestrianManager::PedestrianManager()
    : rngState(defaultSeed),
      configuredSeed(defaultSeed),
      yieldRefusalProbability(defaultYieldRefusalProbability),
      requestTempoMultiplier(1.0),
      nextPedestrianId(1) {
}

void PedestrianManager::clear() {
    pedestrians.clear();
    rngState = configuredSeed;
    nextPedestrianId = 1;
    requestTimers.clear();
    timedActivePedByCrossing.clear();
    encounterRefusal.clear();
}

void PedestrianManager::setSeed(uint32_t seed) {
    configuredSeed = seed;
    rngState = seed;
}

uint32_t PedestrianManager::getSeed() const {
    return configuredSeed;
}

void PedestrianManager::setRequestTempoMultiplier(double multiplier) {
    if (multiplier < 0.1) {
        multiplier = 0.1;
    }
    if (multiplier > 10.0) {
        multiplier = 10.0;
    }
    requestTempoMultiplier = multiplier;
}

double PedestrianManager::getRequestTempoMultiplier() const {
    return requestTempoMultiplier;
}

void PedestrianManager::setYieldRefusalProbability(double probability) {
    if (probability < 0.0) {
        probability = 0.0;
    }
    if (probability > 1.0) {
        probability = 1.0;
    }
    yieldRefusalProbability = probability;
}

double PedestrianManager::getYieldRefusalProbability() const {
    return yieldRefusalProbability;
}

const vector<Pedestrian>& PedestrianManager::getPedestrians() const {
    return pedestrians;
}

vector<Pedestrian>& PedestrianManager::getPedestrians() {
    return pedestrians;
}

uint32_t PedestrianManager::nextRandom() {
    rngState = rngState * 1664525u + 1013904223u;
    return rngState;
}

double PedestrianManager::nextUnitRandom() {
    return static_cast<double>(nextRandom()) / 4294967296.0;
}

double PedestrianManager::crosswalkWidthForRoad(const RoadSegment& road) {
    const int lanes = max(1, road.getLaneCount());
    return static_cast<double>(lanes) * SimConstants::LANE_WIDTH;
}

void PedestrianManager::pruneFinished() {
    pedestrians.erase(
        remove_if(
            pedestrians.begin(),
            pedestrians.end(),
            [](const Pedestrian& ped) { return ped.isFinished(); }
        ),
        pedestrians.end()
    );
}

bool PedestrianManager::hasPedestrianOnRoadway(int crossingId) const {
    for (const auto& ped : pedestrians) {
        if (ped.getCrossingId() == crossingId && ped.isOnRoadway()) {
            return true;
        }
    }
    return false;
}

bool PedestrianManager::hasWaitingPedestrian(int crossingId) const {
    for (const auto& ped : pedestrians) {
        if (ped.getCrossingId() == crossingId && ped.isWaiting()) {
            return true;
        }
    }
    return false;
}

bool PedestrianManager::mustYieldToWaitingPedestrian(int vehicleId, int crossingId) const {
    if (!hasWaitingPedestrian(crossingId) || hasPedestrianOnRoadway(crossingId)) {
        return false;
    }

    const auto key = make_pair(vehicleId, crossingId);
    const auto it = encounterRefusal.find(key);
    if (it != encounterRefusal.end()) {
        return !it->second;
    }

    // Undecided yet: conservative yield until the mutable path records a roll.
    return true;
}

bool PedestrianManager::mustYieldToWaitingPedestrian(int vehicleId, int crossingId) {
    if (!hasWaitingPedestrian(crossingId)) {
        return false;
    }
    // Ped already on roadway is handled separately (always stop).
    if (hasPedestrianOnRoadway(crossingId)) {
        return false;
    }

    const auto key = make_pair(vehicleId, crossingId);
    auto it = encounterRefusal.find(key);
    if (it == encounterRefusal.end()) {
        const bool refused = nextUnitRandom() < yieldRefusalProbability;
        encounterRefusal[key] = refused;
        it = encounterRefusal.find(key);
    }

    return !it->second;
}

bool PedestrianManager::vehicleInConflictZone(
    const Vehicle& vehicle,
    const PedestrianCrossing& crossing
) const {
    if (vehicle.isFinished() || vehicle.getCurrentRoadSegmentId() != crossing.getRoadSegmentId()) {
        return false;
    }

    const double front = vehicle.getPositionOnRoad();
    const double rear = front - vehicle.getLength();
    const double zoneStart = crossing.getPositionOnRoad() - kConflictHalfWidth;
    const double zoneEnd = crossing.getPositionOnRoad() + kConflictHalfWidth;

    return rear < zoneEnd && front > zoneStart;
}

bool PedestrianManager::isSafeToEnter(
    const PedestrianCrossing& crossing,
    const City& city,
    const vector<Vehicle>& vehicles
) const {
    const RoadSegment* road = city.findRoadSegmentById(crossing.getRoadSegmentId());
    if (road == nullptr) {
        return false;
    }

    const double stopLine = crossing.getPositionOnRoad() - SimConstants::CROSSING_STOP_DISTANCE;
    const int laneCount = max(1, road->getLaneCount());

    // Safety across ALL lanes: any lane with a conflicting vehicle blocks entry.
    for (int lane = 0; lane < laneCount; ++lane) {
        for (const auto& vehicle : vehicles) {
            if (vehicle.isFinished() ||
                vehicle.getCurrentRoadSegmentId() != crossing.getRoadSegmentId() ||
                vehicle.getLaneIndex() != lane) {
                continue;
            }

            // Never step into a vehicle already in the strip (moving or stopped on roadway).
            if (vehicleInConflictZone(vehicle, crossing)) {
                return false;
            }

            const double pos = vehicle.getPositionOnRoad();
            const double rear = pos - vehicle.getLength();
            const double crossingPos = crossing.getPositionOnRoad();

            // Car already past the stop line into/through the strip: let it finish.
            // (Stopped AT the stop line is OK — that is the yield position.)
            if (pos > stopLine + 0.1 && rear < crossingPos + kConflictHalfWidth) {
                return false;
            }

            if (pos <= stopLine + 0.1) {
                // Still approaching: must be fully stopped at the yield point (not idle at spawn).
                if (vehicle.getSpeed() > kStoppedSpeed) {
                    return false;
                }
                if (pos < stopLine - 3.0) {
                    return false;
                }
            }
        }
    }

    return true;
}

bool PedestrianManager::isWalkAllowed(
    const PedestrianCrossing& crossing,
    const vector<TrafficLight>& trafficLights
) const {
    if (crossing.getMode() != CrossingMode::Signalized) {
        return true;
    }

    bool hasLight = false;
    for (const auto& light : trafficLights) {
        if (light.getRoadSegmentId() != crossing.getRoadSegmentId()) {
            continue;
        }
        hasLight = true;
        // Pedestrian walk when vehicles are held (red).
        if (light.isRed()) {
            return true;
        }
    }

    // No light → treat as unsignalized walk permission.
    return !hasLight;
}

void PedestrianManager::syncEntityOccupancy(vector<PedestrianCrossing>& crossings) {
    for (auto& crossing : crossings) {
        crossing.setEntityOccupying(hasPedestrianOnRoadway(crossing.getId()));
    }
}

void PedestrianManager::syncTimedPedestrian(PedestrianCrossing& crossing, const City& city) {
    if (crossing.getMode() != CrossingMode::Timed) {
        return;
    }

    const int crossingId = crossing.getId();
    if (crossing.isActive()) {
        auto it = timedActivePedByCrossing.find(crossingId);
        if (it != timedActivePedByCrossing.end()) {
            // Keep existing ped; ensure still Crossing.
            for (auto& ped : pedestrians) {
                if (ped.getId() == it->second && ped.getCrossingId() == crossingId) {
                    if (ped.isWaiting()) {
                        ped.setState(PedestrianState::Crossing);
                    }
                    return;
                }
            }
            timedActivePedByCrossing.erase(it);
        }

        const RoadSegment* road = city.findRoadSegmentById(crossing.getRoadSegmentId());
        const double width = road != nullptr ? crosswalkWidthForRoad(*road) : SimConstants::LANE_WIDTH;
        // Pace walk to roughly fill the active window (visible sync with timer).
        double speed = crossing.getWalkSpeed();
        if (crossing.getActiveDuration() > 0.05) {
            speed = max(speed, width / crossing.getActiveDuration());
        }

        Pedestrian ped(nextPedestrianId++, crossingId, crossing.getRoadSegmentId(), speed);
        ped.setCrossWidth(width);
        ped.setState(PedestrianState::Crossing);
        timedActivePedByCrossing[crossingId] = ped.getId();
        pedestrians.push_back(ped);
    } else {
        auto it = timedActivePedByCrossing.find(crossingId);
        if (it != timedActivePedByCrossing.end()) {
            for (auto& ped : pedestrians) {
                if (ped.getId() == it->second) {
                    ped.setState(PedestrianState::Finished);
                }
            }
            timedActivePedByCrossing.erase(it);
        }
    }
}

void PedestrianManager::spawnIfNeeded(
    PedestrianCrossing& crossing,
    const City& city,
    const vector<Vehicle>& vehicles,
    const vector<TrafficLight>& trafficLights
) {
    if (crossing.getMode() == CrossingMode::Timed) {
        return;
    }

    // One live ped per crossing at a time.
    for (const auto& ped : pedestrians) {
        if (ped.getCrossingId() == crossing.getId() && !ped.isFinished()) {
            return;
        }
    }

    double& timer = requestTimers[crossing.getId()];
    const double interval = max(0.5, crossing.getInactiveDuration() / max(0.1, requestTempoMultiplier));
    if (timer < interval) {
        return;
    }

    // Never spawn under a moving car in the conflict zone.
    for (const auto& vehicle : vehicles) {
        if (vehicleInConflictZone(vehicle, crossing) && vehicle.getSpeed() > kStoppedSpeed) {
            return;
        }
    }

    const RoadSegment* road = city.findRoadSegmentById(crossing.getRoadSegmentId());
    const double width = road != nullptr ? crosswalkWidthForRoad(*road) : SimConstants::LANE_WIDTH;

    Pedestrian ped(nextPedestrianId++, crossing.getId(), crossing.getRoadSegmentId(), crossing.getWalkSpeed());
    ped.setCrossWidth(width);
    ped.setState(PedestrianState::Waiting);
    pedestrians.push_back(ped);
    timer = 0.0;

    (void)trafficLights;
}

void PedestrianManager::tryEnterWaiting(
    const City& city,
    const vector<Vehicle>& vehicles,
    const vector<TrafficLight>& trafficLights,
    const vector<PedestrianCrossing>& crossings
) {
    for (auto& ped : pedestrians) {
        if (!ped.isWaiting()) {
            continue;
        }

        const PedestrianCrossing* crossing = nullptr;
        for (const auto& candidate : crossings) {
            if (candidate.getId() == ped.getCrossingId()) {
                crossing = &candidate;
                break;
            }
        }
        if (crossing == nullptr) {
            continue;
        }

        if (!isWalkAllowed(*crossing, trafficLights)) {
            continue;
        }

        if (!isSafeToEnter(*crossing, city, vehicles)) {
            continue;
        }

        // If any approaching vehicle refused, do not step in front of it.
        bool blockedByRefusal = false;
        const double stopLine = crossing->getPositionOnRoad() - SimConstants::CROSSING_STOP_DISTANCE;
        for (const auto& vehicle : vehicles) {
            if (vehicle.isFinished() ||
                vehicle.getCurrentRoadSegmentId() != crossing->getRoadSegmentId()) {
                continue;
            }
            if (vehicle.getPositionOnRoad() >= stopLine) {
                continue;
            }
            const auto key = make_pair(vehicle.getId(), crossing->getId());
            const auto it = encounterRefusal.find(key);
            if (it != encounterRefusal.end() && it->second) {
                blockedByRefusal = true;
                break;
            }
        }
        if (blockedByRefusal) {
            continue;
        }

        ped.setState(PedestrianState::Crossing);
    }
}

void PedestrianManager::clearStaleEncounters(
    const vector<Vehicle>& vehicles,
    const vector<PedestrianCrossing>& crossings
) {
    for (auto it = encounterRefusal.begin(); it != encounterRefusal.end();) {
        const int vehicleId = it->first.first;
        const int crossingId = it->first.second;

        const Vehicle* vehicle = nullptr;
        for (const auto& candidate : vehicles) {
            if (candidate.getId() == vehicleId && !candidate.isFinished()) {
                vehicle = &candidate;
                break;
            }
        }

        const PedestrianCrossing* crossing = nullptr;
        for (const auto& candidate : crossings) {
            if (candidate.getId() == crossingId) {
                crossing = &candidate;
                break;
            }
        }

        bool clear = false;
        if (vehicle == nullptr || crossing == nullptr) {
            clear = true;
        } else if (vehicle->getCurrentRoadSegmentId() != crossing->getRoadSegmentId()) {
            clear = true;
        } else if (vehicle->getPositionOnRoad() > crossing->getPositionOnRoad() + kConflictHalfWidth) {
            clear = true;
        } else if (!hasWaitingPedestrian(crossingId) && !hasPedestrianOnRoadway(crossingId)) {
            clear = true;
        }

        if (clear) {
            it = encounterRefusal.erase(it);
        } else {
            ++it;
        }
    }
}

bool PedestrianManager::shouldHoldPhaseController(
    const PhaseController& controller,
    const vector<PedestrianCrossing>& crossings,
    const vector<TrafficLight>& trafficLights
) const {
    for (const auto& crossing : crossings) {
        if (crossing.getMode() != CrossingMode::Signalized) {
            continue;
        }
        if (!hasPedestrianOnRoadway(crossing.getId())) {
            continue;
        }

        // Hold if this controller owns a light on the crossing's road and walk is currently allowed.
        for (const auto& light : trafficLights) {
            if (light.getRoadSegmentId() != crossing.getRoadSegmentId()) {
                continue;
            }
            if (!light.isPhaseControlled()) {
                continue;
            }
            if (light.getIntersectionId() == controller.getIntersectionId() && light.isRed()) {
                return true;
            }
        }
    }
    return false;
}

double PedestrianManager::computeCrossingStopCap(
    const Vehicle& vehicle,
    const RoadSegment& road,
    const vector<PedestrianCrossing>& crossings,
    const vector<TrafficLight>& trafficLights,
    string* stopReasonOut
) {
    double cap = numeric_limits<double>::infinity();
    string reason;

    for (const auto& crossing : crossings) {
        if (crossing.getRoadSegmentId() != road.getId()) {
            continue;
        }
        if (vehicle.getPositionOnRoad() >= crossing.getPositionOnRoad()) {
            continue;
        }

        const double stopPos = crossing.getPositionOnRoad() - SimConstants::CROSSING_STOP_DISTANCE;
        bool block = false;
        string localReason;

        if (hasPedestrianOnRoadway(crossing.getId())) {
            block = true;
            localReason = "pedestrian_on_road";
        } else if (crossing.getMode() == CrossingMode::Timed && crossing.isActive()) {
            block = true;
            localReason = "pedestrian_crossing";
        } else if (crossing.getMode() == CrossingMode::Unsignalized &&
                   hasWaitingPedestrian(crossing.getId())) {
            if (mustYieldToWaitingPedestrian(vehicle.getId(), crossing.getId())) {
                block = true;
                localReason = "pedestrian_crossing";
            }
        } else if (crossing.getMode() == CrossingMode::Signalized &&
                   hasWaitingPedestrian(crossing.getId())) {
            // Waiting peds do not override a vehicle green; red is handled by lights.
            (void)trafficLights;
        }

        // Timed active already covered; also block for entity occupying flagged via isActive.
        if (!block && crossing.getMode() != CrossingMode::Timed && crossing.isActive()) {
            block = true;
            localReason = "pedestrian_on_road";
        }

        if (block && stopPos < cap) {
            cap = stopPos;
            reason = localReason;
        }
    }

    if (stopReasonOut != nullptr && isfinite(cap)) {
        *stopReasonOut = reason;
    }

    return cap;
}

int PedestrianManager::spawnWaitingPedestrian(const PedestrianCrossing& crossing, const City& city) {
    for (const auto& ped : pedestrians) {
        if (ped.getCrossingId() == crossing.getId() && !ped.isFinished()) {
            return ped.getId();
        }
    }

    const RoadSegment* road = city.findRoadSegmentById(crossing.getRoadSegmentId());
    const double width = road != nullptr ? crosswalkWidthForRoad(*road) : SimConstants::LANE_WIDTH;
    Pedestrian ped(nextPedestrianId++, crossing.getId(), crossing.getRoadSegmentId(), crossing.getWalkSpeed());
    ped.setCrossWidth(width);
    ped.setState(PedestrianState::Waiting);
    const int id = ped.getId();
    pedestrians.push_back(ped);
    return id;
}

void PedestrianManager::update(
    double dt,
    const City& city,
    vector<PedestrianCrossing>& crossings,
    const vector<Vehicle>& vehicles,
    const vector<TrafficLight>& trafficLights
) {
    for (auto& crossing : crossings) {
        if (crossing.getMode() != CrossingMode::Timed) {
            requestTimers[crossing.getId()] += dt;
        }
    }

    for (auto& crossing : crossings) {
        syncTimedPedestrian(crossing, city);
        spawnIfNeeded(crossing, city, vehicles, trafficLights);
    }

    tryEnterWaiting(city, vehicles, trafficLights, crossings);

    for (auto& ped : pedestrians) {
        ped.update(dt);
    }

    pruneFinished();
    syncEntityOccupancy(crossings);
    clearStaleEncounters(vehicles, crossings);
}
