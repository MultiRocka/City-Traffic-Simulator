#pragma once

#include <vector>
#include <string>
#include <map>
#include <set>
#include <cstdint>
#include "core/Vehicle.h"
#include "core/City.h"
#include "core/VehicleGenerator.h"
#include "core/TrafficLight.h"
#include "core/StatisticsCollector.h"
#include "core/EventLog.h"
#include "core/PedestrianCrossing.h"
#include "core/PedestrianManager.h"
#include "core/LaneBlockage.h"
#include "core/CollisionDetector.h"
#include "core/IntersectionRules.h"
#include "core/PhaseController.h"

using namespace std;

class Simulation {
public:
    Simulation();

    void start();
    void stop();
    void clear();
    void update(double dt, const City& city);

    bool isRunning() const;
    double getTime() const;

    void addVehicle(const Vehicle& vehicle);
    vector<Vehicle>& getVehicles();
    const vector<Vehicle>& getVehicles() const;

    void addVehicleGenerator(const VehicleGenerator& generator);
    vector<VehicleGenerator>& getVehicleGenerators();
    const vector<VehicleGenerator>& getVehicleGenerators() const;
    void setMinVehicleGap(double newMinVehicleGap);

    // Soft concurrent active-vehicle cap (Etap 3). 0 = unlimited.
    // Generators defer when at cap; vehicles are never deleted to free slots.
    void setMaxActiveVehicles(int limit);
    int getMaxActiveVehicles() const;
    int countActiveVehicles() const;

    // Stage 6: scale all generator spawn intervals relative to captured baselines.
    // rate > 1 → more frequent spawns (shorter intervals). Clamped to [0.1, 10].
    void setVehicleSpawnRateMultiplier(double rate);
    double getVehicleSpawnRateMultiplier() const;
    void captureVehicleSpawnBaselines();

    void setPedestrianRequestTempo(double tempo);
    double getPedestrianRequestTempo() const;

    // Intentional lane change on the vehicle's current directed road.
    // Returns false if unsafe, invalid lane, single-lane road, or cooldown active.
    bool tryLaneChange(int vehicleId, int targetLane, const City& city);

    const CollisionDetector& getCollisionDetector() const;
    const IntersectionRules& getIntersectionRules() const;
    IntersectionRules& getIntersectionRules();

    void addTrafficLight(const TrafficLight& trafficLight);
    vector<TrafficLight>& getTrafficLights();
    const vector<TrafficLight>& getTrafficLights() const;

    void addPhaseController(const PhaseController& phaseController);
    vector<PhaseController>& getPhaseControllers();
    const vector<PhaseController>& getPhaseControllers() const;

    void addPedestrianCrossing(const PedestrianCrossing& pedestrianCrossing);
    vector<PedestrianCrossing>& getPedestrianCrossings();
    const vector<PedestrianCrossing>& getPedestrianCrossings() const;

    PedestrianManager& getPedestrianManager();
    const PedestrianManager& getPedestrianManager() const;
    void setPedestrianYieldRefusalProbability(double probability);
    void setPedestrianSeed(uint32_t seed);

    // Stage 5: temporary lane blockage (manual + optional random). Opposite directed
    // RoadSegment is never touched unless a separate block is added on it.
    int addLaneBlockage(
        int roadId,
        int laneIndex,
        double positionAlongRoad,
        double length,
        double startTime,
        double duration,
        const City& city
    );
    bool clearLaneBlockage(int blockageId);
    void clearAllLaneBlockages();
    const vector<LaneBlockage>& getLaneBlockages() const;
    LaneBlockageManager& getLaneBlockageManager();
    const LaneBlockageManager& getLaneBlockageManager() const;
    void setRandomLaneBlockConfig(const RandomBlockConfig& config);
    const RandomBlockConfig& getRandomLaneBlockConfig() const;
    void setLaneBlockSeed(uint32_t seed);

    void setStatisticsOutputFiles(const string& timelineFile, const string& summaryFile);
    void exportStatisticsSummary() const;
    const StatisticsCollector& getStatisticsCollector() const;

    void setEventLogOutputFile(const string& filePath);
    void exportEventLog() const;
    const EventLog& getEventLog() const;

private:
    bool running;
    double currentTime;
    vector<Vehicle> vehicles;
    vector<VehicleGenerator> generators;
    vector<double> vehicleSpawnBaselines;
    double vehicleSpawnRateMultiplier;
    double minVehicleGap;
    int maxActiveVehicles;
    int nextVehicleId;
    vector<TrafficLight> trafficLights;
    vector<PhaseController> phaseControllers;
    vector<PedestrianCrossing> pedestrianCrossings;
    PedestrianManager pedestrianManager;
    LaneBlockageManager laneBlockageManager;
    StatisticsCollector statisticsCollector;
    EventLog eventLog;
    CollisionDetector collisionDetector;
    IntersectionRules intersectionRules;

    set<int> knownVehicleIds;
    set<int> knownInvalidRoutes;
    map<int, bool> previousStoppedState;
    map<int, int> previousRoadState;
    map<int, bool> previousCrossingState;

    void syncPhaseControlledLights();
    void pruneFinishedVehicles();
};

