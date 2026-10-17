#include "core/Simulation.h"
#include "core/CollisionDetector.h"
#include <algorithm>

using namespace std;

Simulation::Simulation()
    : running(false),
      currentTime(0.0),
      vehicleSpawnRateMultiplier(1.0),
      minVehicleGap(2.5),
      nextVehicleId(1),
      collisionDetector(minVehicleGap) {
}

void Simulation::start() {
    running = true;
}

void Simulation::stop() {
    running = false;
    pruneFinishedVehicles();
}

void Simulation::clear() {
    running = false;
    currentTime = 0.0;
    vehicles.clear();
    generators.clear();
    vehicleSpawnBaselines.clear();
    vehicleSpawnRateMultiplier = 1.0;
    minVehicleGap = 2.5;
    collisionDetector.setMinVehicleGap(minVehicleGap);
    intersectionRules.clear();
    nextVehicleId = 1;
    trafficLights.clear();
    phaseControllers.clear();
    pedestrianCrossings.clear();
    pedestrianManager.clear();
    laneBlockageManager.clear();
    statisticsCollector.resetRuntime();
    eventLog.clear();
    knownVehicleIds.clear();
    knownInvalidRoutes.clear();
    previousStoppedState.clear();
    previousRoadState.clear();
    previousCrossingState.clear();
}

void Simulation::syncPhaseControlledLights() {
    for (auto& trafficLight : trafficLights) {
        if (!trafficLight.isPhaseControlled()) {
            continue;
        }

        bool isGreen = false;

        for (const auto& controller : phaseControllers) {
            if (controller.getIntersectionId() == trafficLight.getIntersectionId()) {
                isGreen = controller.isRoadGreen(trafficLight.getRoadSegmentId());
                break;
            }
        }

        trafficLight.setGreen(isGreen);
    }
}

void Simulation::pruneFinishedVehicles() {
    for (const auto& vehicle : vehicles) {
        if (vehicle.isFinished()) {
            intersectionRules.releaseVehicle(vehicle.getId());
        }
    }

    vehicles.erase(
        remove_if(vehicles.begin(), vehicles.end(), [](const Vehicle& vehicle) {
            return vehicle.isFinished();
        }),
        vehicles.end()
    );
}

void Simulation::update(double dt, const City& city) {
    if (!running) {
        return;
    }

    currentTime += dt;

    for (auto& phaseController : phaseControllers) {
        const bool hold = pedestrianManager.shouldHoldPhaseController(
            phaseController,
            pedestrianCrossings,
            trafficLights
        );
        phaseController.update(dt, hold);
    }

    syncPhaseControlledLights();

    for (auto& trafficLight : trafficLights) {
        trafficLight.update(dt);
    }

    for (auto& crossing : pedestrianCrossings) {
        crossing.update(dt);
    }

    pedestrianManager.update(dt, city, pedestrianCrossings, vehicles, trafficLights);

    laneBlockageManager.update(dt, currentTime, city, vehicles, eventLog, &collisionDetector);
    laneBlockageManager.tryEscapeLaneChanges(
        currentTime,
        city,
        vehicles,
        collisionDetector,
        eventLog
    );

    for (auto& crossing : pedestrianCrossings) {
        if (previousCrossingState.find(crossing.getId()) == previousCrossingState.end()) {
            previousCrossingState[crossing.getId()] = crossing.isActive();
        } else {
            bool oldState = previousCrossingState[crossing.getId()];
            bool newState = crossing.isActive();

            if (!oldState && newState) {
                eventLog.addEvent(
                    currentTime,
                    "pedestrian_crossing_started",
                    -1,
                    crossing.getRoadSegmentId(),
                    "pedestrian_request",
                    "Pedestrian started crossing"
                );
            }

            if (oldState && !newState) {
                eventLog.addEvent(
                    currentTime,
                    "pedestrian_crossing_finished",
                    -1,
                    crossing.getRoadSegmentId(),
                    "pedestrian_finished",
                    "Pedestrian finished crossing"
                );
            }

            previousCrossingState[crossing.getId()] = newState;
        }
    }

    for (auto& generator : generators) {
        generator.update(dt, vehicles, nextVehicleId, &city, &laneBlockageManager, currentTime);
    }

    for (const auto& vehicle : vehicles) {
        if (knownVehicleIds.find(vehicle.getId()) == knownVehicleIds.end()) {
            knownVehicleIds.insert(vehicle.getId());
            previousStoppedState[vehicle.getId()] = vehicle.isStopped();
            previousRoadState[vehicle.getId()] = vehicle.getCurrentRoadSegmentId();

            eventLog.addEvent(
                currentTime,
                "vehicle_spawned",
                vehicle.getId(),
                vehicle.getCurrentRoadSegmentId(),
                "generator_spawn",
                "Vehicle spawned in simulation"
            );
        }
    }

    // Optional preferLane: try one intentional LC per vehicle when cooldown allows.
    // Not zipper — blockage escape LC runs earlier via LaneBlockageManager.
    for (auto& vehicle : vehicles) {
        if (vehicle.isFinished() || vehicle.getPreferLane() < 0 || vehicle.getLaneChangeCooldown() > 0.0) {
            continue;
        }

        if (vehicle.getPreferLane() == vehicle.getLaneIndex()) {
            continue;
        }

        tryLaneChange(vehicle.getId(), vehicle.getPreferLane(), city);
    }

    for (const auto& roadSegment : city.getRoadSegments()) {
        vector<Vehicle*> vehiclesOnRoad;

        for (auto& vehicle : vehicles) {
            if (!vehicle.isFinished() && vehicle.getCurrentRoadSegmentId() == roadSegment.getId()) {
                // Keep laneIndex valid for this directed segment.
                vehicle.clampLaneIndex(roadSegment.getLaneCount());
                vehiclesOnRoad.push_back(&vehicle);
            }
        }

        // Front-most first; predecessor lookup is still lane-aware via findPredecessor.
        sort(vehiclesOnRoad.begin(), vehiclesOnRoad.end(),
            [](Vehicle* a, Vehicle* b) {
                return a->getPositionOnRoad() > b->getPositionOnRoad();
            });

        for (Vehicle* vehiclePtr : vehiclesOnRoad) {
            const Vehicle* leader = collisionDetector.findPredecessor(*vehiclePtr, vehicles);
            double aheadPosition = -1.0;
            double aheadLength = 0.0;

            if (leader != nullptr) {
                aheadPosition = leader->getPositionOnRoad();
                aheadLength = leader->getLength();
            }

            double maxAllowedPosition = collisionDetector.computeMaxAllowedPosition(
                roadSegment,
                trafficLights,
                pedestrianCrossings,
                vehiclePtr->getPositionOnRoad(),
                aheadPosition,
                aheadLength,
                &pedestrianManager,
                vehiclePtr,
                &laneBlockageManager,
                currentTime
            );

            vehiclePtr->update(
                dt,
                roadSegment.getLength(),
                maxAllowedPosition,
                roadSegment.getSpeedLimit(),
                minVehicleGap
            );
        }
    }

    intersectionRules.clearStaleReservations(city, vehicles);

    // Collect vehicles ready at stop lines, then grant per intersection (priority / conflict).
    map<int, vector<int>> candidatesByIntersection;

    for (auto& vehicle : vehicles) {
        if (vehicle.isFinished()) {
            continue;
        }

        int currentRoadId = vehicle.getCurrentRoadSegmentId();
        const RoadSegment* currentRoad = city.findRoadSegmentById(currentRoadId);

        if (currentRoad == nullptr) {
            continue;
        }

        if (!collisionDetector.isAtIntersectionEntry(vehicle.getPositionOnRoad(), currentRoad->getLength())) {
            intersectionRules.clearWaiting(vehicle.getId());
            continue;
        }

        int nextRoadId = vehicle.getNextRoadSegmentId();

        if (nextRoadId == -1) {
            intersectionRules.releaseVehicle(vehicle.getId());
            vehicle.finish();

            eventLog.addEvent(
                currentTime,
                "vehicle_finished",
                vehicle.getId(),
                currentRoadId,
                "destination_reached",
                "Vehicle finished its route"
            );

            continue;
        }

        if (!city.areRoadsConnected(currentRoadId, nextRoadId)) {
            if (knownInvalidRoutes.find(vehicle.getId()) == knownInvalidRoutes.end()) {
                knownInvalidRoutes.insert(vehicle.getId());

                eventLog.addEvent(
                    currentTime,
                    "vehicle_stopped",
                    vehicle.getId(),
                    currentRoadId,
                    "route_invalid",
                    "Vehicle route has invalid road connection"
                );
            }

            continue;
        }

        const RoadSegment* nextRoad = city.findRoadSegmentById(nextRoadId);
        int entryLane = vehicle.getLaneIndex();
        if (nextRoad != nullptr) {
            entryLane = min(vehicle.getLaneIndex(), max(0, nextRoad->getLaneCount() - 1));
        }

        // Lights + entrance space are hard gates. Yield/priority among peers is resolved below
        // so circular yield-to-right can break via wait time / bearing (not vehicleId).
        if (!collisionDetector.isGreenForRoad(trafficLights, currentRoadId) ||
            !collisionDetector.canEnterRoad(nextRoadId, entryLane, vehicles) ||
            laneBlockageManager.blocksEntrance(nextRoadId, entryLane, currentTime)) {
            intersectionRules.noteWaiting(vehicle.getId(), currentTime);
            continue;
        }

        candidatesByIntersection[currentRoad->getToIntersectionId()].push_back(vehicle.getId());
    }

    for (auto& entry : candidatesByIntersection) {
        const int intersectionId = entry.first;
        const vector<int> grants = intersectionRules.resolveGrants(
            intersectionId,
            entry.second,
            city,
            trafficLights,
            vehicles,
            currentTime
        );

        set<int> grantedIds(grants.begin(), grants.end());
        for (int vehicleId : entry.second) {
            if (grantedIds.find(vehicleId) == grantedIds.end()) {
                intersectionRules.noteWaiting(vehicleId, currentTime);
            }
        }

        for (int vehicleId : grants) {
            Vehicle* vehiclePtr = nullptr;
            for (auto& vehicle : vehicles) {
                if (vehicle.getId() == vehicleId && !vehicle.isFinished()) {
                    vehiclePtr = &vehicle;
                    break;
                }
            }
            if (vehiclePtr == nullptr) {
                continue;
            }

            const int currentRoadId = vehiclePtr->getCurrentRoadSegmentId();
            const int nextRoadId = vehiclePtr->getNextRoadSegmentId();
            const RoadSegment* nextRoad = city.findRoadSegmentById(nextRoadId);
            int entryLane = vehiclePtr->getLaneIndex();
            if (nextRoad != nullptr) {
                entryLane = min(vehiclePtr->getLaneIndex(), max(0, nextRoad->getLaneCount() - 1));
            }

            // Re-check space: green does not skip conflict clearance / entrance room.
            if (!collisionDetector.isGreenForRoad(trafficLights, currentRoadId) ||
                !collisionDetector.canEnterRoad(nextRoadId, entryLane, vehicles) ||
                laneBlockageManager.blocksEntrance(nextRoadId, entryLane, currentTime)) {
                intersectionRules.noteWaiting(vehicleId, currentTime);
                continue;
            }

            const MovementKey movement = intersectionRules.makeMovement(city, *vehiclePtr);
            intersectionRules.reserve(intersectionId, *vehiclePtr, movement);
            intersectionRules.clearWaiting(vehicleId);

            vehiclePtr->moveToNextRoad();
            if (nextRoad != nullptr) {
                vehiclePtr->clampLaneIndex(nextRoad->getLaneCount());
            }

            eventLog.addEvent(
                currentTime,
                "vehicle_entered_road",
                vehiclePtr->getId(),
                vehiclePtr->getCurrentRoadSegmentId(),
                "junction_cleared",
                "Vehicle entered next road segment"
            );
        }
    }

    for (const auto& vehicle : vehicles) {
        if (vehicle.isFinished()) {
            continue;
        }

        int vehicleId = vehicle.getId();
        bool oldStopped = previousStoppedState[vehicleId];
        bool newStopped = vehicle.isStopped();

        if (!oldStopped && newStopped) {
            eventLog.addEvent(
                currentTime,
                "vehicle_stopped",
                vehicleId,
                vehicle.getCurrentRoadSegmentId(),
                collisionDetector.determineStopReason(
                    vehicle,
                    city,
                    trafficLights,
                    pedestrianCrossings,
                    vehicles,
                    &intersectionRules,
                    &pedestrianManager,
                    &laneBlockageManager,
                    currentTime
                ),
                "Vehicle stopped"
            );
        }

        if (oldStopped && !newStopped) {
            eventLog.addEvent(
                currentTime,
                "vehicle_resumed",
                vehicleId,
                vehicle.getCurrentRoadSegmentId(),
                "obstacle_cleared",
                "Vehicle resumed movement"
            );
        }

        int oldRoadId = previousRoadState[vehicleId];
        int newRoadId = vehicle.getCurrentRoadSegmentId();

        if (oldRoadId != newRoadId) {
            eventLog.addEvent(
                currentTime,
                "vehicle_changed_road",
                vehicleId,
                newRoadId,
                "route_progress",
                "Vehicle changed road segment"
            );
        }

        previousStoppedState[vehicleId] = newStopped;
        previousRoadState[vehicleId] = newRoadId;
    }

    statisticsCollector.update(dt, currentTime, vehicles);
    pruneFinishedVehicles();
}

bool Simulation::isRunning() const {
    return running;
}

double Simulation::getTime() const {
    return currentTime;
}

void Simulation::addVehicle(const Vehicle& vehicle) {
    vehicles.push_back(vehicle);
    nextVehicleId = max(nextVehicleId, vehicle.getId() + 1);
}

bool Simulation::tryLaneChange(int vehicleId, int targetLane, const City& city) {
    Vehicle* subject = nullptr;
    for (auto& vehicle : vehicles) {
        if (vehicle.getId() == vehicleId && !vehicle.isFinished()) {
            subject = &vehicle;
            break;
        }
    }

    if (subject == nullptr) {
        return false;
    }

    const RoadSegment* road = city.findRoadSegmentById(subject->getCurrentRoadSegmentId());
    if (road == nullptr) {
        return false;
    }

    if (!collisionDetector.canChangeLane(*subject, targetLane, *road, vehicles)) {
        return false;
    }

    subject->applyLaneChange(targetLane, SimConstants::LANE_CHANGE_COOLDOWN);

    eventLog.addEvent(
        currentTime,
        "vehicle_lane_change",
        subject->getId(),
        subject->getCurrentRoadSegmentId(),
        "lane_change",
        "Vehicle changed lane on current road"
    );

    return true;
}

const CollisionDetector& Simulation::getCollisionDetector() const {
    return collisionDetector;
}

const IntersectionRules& Simulation::getIntersectionRules() const {
    return intersectionRules;
}

IntersectionRules& Simulation::getIntersectionRules() {
    return intersectionRules;
}

vector<Vehicle>& Simulation::getVehicles() {
    return vehicles;
}

const vector<Vehicle>& Simulation::getVehicles() const {
    return vehicles;
}

void Simulation::addVehicleGenerator(const VehicleGenerator& generator) {
    generators.push_back(generator);
    vehicleSpawnBaselines.push_back(generator.getSpawnInterval());
}

vector<VehicleGenerator>& Simulation::getVehicleGenerators() {
    return generators;
}

const vector<VehicleGenerator>& Simulation::getVehicleGenerators() const {
    return generators;
}

void Simulation::captureVehicleSpawnBaselines() {
    vehicleSpawnBaselines.clear();
    vehicleSpawnBaselines.reserve(generators.size());
    for (const auto& generator : generators) {
        vehicleSpawnBaselines.push_back(generator.getSpawnInterval());
    }
}

void Simulation::setVehicleSpawnRateMultiplier(double rate) {
    if (rate < 0.1) {
        rate = 0.1;
    }
    if (rate > 10.0) {
        rate = 10.0;
    }
    vehicleSpawnRateMultiplier = rate;

    if (vehicleSpawnBaselines.size() != generators.size()) {
        captureVehicleSpawnBaselines();
    }

    for (size_t i = 0; i < generators.size(); ++i) {
        const double base = (i < vehicleSpawnBaselines.size()) ? vehicleSpawnBaselines[i] : generators[i].getSpawnInterval();
        generators[i].setSpawnInterval(base / vehicleSpawnRateMultiplier);
    }
}

double Simulation::getVehicleSpawnRateMultiplier() const {
    return vehicleSpawnRateMultiplier;
}

void Simulation::setPedestrianRequestTempo(double tempo) {
    pedestrianManager.setRequestTempoMultiplier(tempo);
}

double Simulation::getPedestrianRequestTempo() const {
    return pedestrianManager.getRequestTempoMultiplier();
}

void Simulation::setMinVehicleGap(double newMinVehicleGap) {
    minVehicleGap = newMinVehicleGap;
    collisionDetector.setMinVehicleGap(newMinVehicleGap);
}

void Simulation::addTrafficLight(const TrafficLight& trafficLight) {
    trafficLights.push_back(trafficLight);
}

vector<TrafficLight>& Simulation::getTrafficLights() {
    return trafficLights;
}

const vector<TrafficLight>& Simulation::getTrafficLights() const {
    return trafficLights;
}

void Simulation::addPhaseController(const PhaseController& phaseController) {
    phaseControllers.push_back(phaseController);
}

vector<PhaseController>& Simulation::getPhaseControllers() {
    return phaseControllers;
}

const vector<PhaseController>& Simulation::getPhaseControllers() const {
    return phaseControllers;
}

void Simulation::addPedestrianCrossing(const PedestrianCrossing& pedestrianCrossing) {
    pedestrianCrossings.push_back(pedestrianCrossing);
    previousCrossingState[pedestrianCrossing.getId()] = pedestrianCrossing.isActive();
}

vector<PedestrianCrossing>& Simulation::getPedestrianCrossings() {
    return pedestrianCrossings;
}

const vector<PedestrianCrossing>& Simulation::getPedestrianCrossings() const {
    return pedestrianCrossings;
}

PedestrianManager& Simulation::getPedestrianManager() {
    return pedestrianManager;
}

const PedestrianManager& Simulation::getPedestrianManager() const {
    return pedestrianManager;
}

void Simulation::setPedestrianYieldRefusalProbability(double probability) {
    pedestrianManager.setYieldRefusalProbability(probability);
}

void Simulation::setPedestrianSeed(uint32_t seed) {
    pedestrianManager.setSeed(seed);
}

int Simulation::addLaneBlockage(
    int roadId,
    int laneIndex,
    double positionAlongRoad,
    double length,
    double startTime,
    double duration,
    const City& city
) {
    return laneBlockageManager.addBlockage(
        roadId,
        laneIndex,
        positionAlongRoad,
        length,
        startTime,
        duration,
        city,
        vehicles
    );
}

bool Simulation::clearLaneBlockage(int blockageId) {
    int roadId = -1;
    for (const auto& block : laneBlockageManager.getBlockages()) {
        if (block.id == blockageId) {
            roadId = block.roadId;
            break;
        }
    }

    const bool removed = laneBlockageManager.clearBlockage(blockageId);
    if (removed) {
        eventLog.addEvent(
            currentTime,
            "lane_blockage_ended",
            -1,
            roadId,
            "lane_blockage_cleared",
            "Lane blockage cleared manually"
        );
    }
    return removed;
}

void Simulation::clearAllLaneBlockages() {
    if (!laneBlockageManager.getBlockages().empty()) {
        eventLog.addEvent(
            currentTime,
            "lane_blockage_ended",
            -1,
            -1,
            "lane_blockage_cleared",
            "All lane blockages cleared"
        );
    }
    laneBlockageManager.clearAllBlockages();
}

const vector<LaneBlockage>& Simulation::getLaneBlockages() const {
    return laneBlockageManager.getBlockages();
}

LaneBlockageManager& Simulation::getLaneBlockageManager() {
    return laneBlockageManager;
}

const LaneBlockageManager& Simulation::getLaneBlockageManager() const {
    return laneBlockageManager;
}

void Simulation::setRandomLaneBlockConfig(const RandomBlockConfig& config) {
    laneBlockageManager.setRandomConfig(config);
}

const RandomBlockConfig& Simulation::getRandomLaneBlockConfig() const {
    return laneBlockageManager.getRandomConfig();
}

void Simulation::setLaneBlockSeed(uint32_t seed) {
    laneBlockageManager.setSeed(seed);
}

void Simulation::setStatisticsOutputFiles(const string& timelineFile, const string& summaryFile) {
    statisticsCollector.setOutputFiles(timelineFile, summaryFile);
}

void Simulation::exportStatisticsSummary() const {
    statisticsCollector.exportAll();
}

const StatisticsCollector& Simulation::getStatisticsCollector() const {
    return statisticsCollector;
}

void Simulation::setEventLogOutputFile(const string& filePath) {
    eventLog.setOutputFile(filePath);
}

void Simulation::exportEventLog() const {
    eventLog.exportToJson();
}

const EventLog& Simulation::getEventLog() const {
    return eventLog;
}
