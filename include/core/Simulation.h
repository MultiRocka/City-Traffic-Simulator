#pragma once

#include <vector>
#include <string>
#include <map>
#include <set>
#include "core/Vehicle.h"
#include "core/City.h"
#include "core/VehicleGenerator.h"
#include "core/TrafficLight.h"
#include "core/StatisticsCollector.h"
#include "core/EventLog.h"
#include "core/PedestrianCrossing.h"

using namespace std;

class Simulation {
public:
    Simulation();

    void start();
    void stop();
    void update(double dt, const City& city);

    bool isRunning() const;
    double getTime() const;

    void addVehicle(const Vehicle& vehicle);
    vector<Vehicle>& getVehicles();
    const vector<Vehicle>& getVehicles() const;

    void addVehicleGenerator(const VehicleGenerator& generator);
    void setMinVehicleGap(double newMinVehicleGap);

    void addTrafficLight(const TrafficLight& trafficLight);
    vector<TrafficLight>& getTrafficLights();
    const vector<TrafficLight>& getTrafficLights() const;

    void addPedestrianCrossing(const PedestrianCrossing& pedestrianCrossing);
    vector<PedestrianCrossing>& getPedestrianCrossings();
    const vector<PedestrianCrossing>& getPedestrianCrossings() const;

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
    double minVehicleGap;
    vector<TrafficLight> trafficLights;
    vector<PedestrianCrossing> pedestrianCrossings;
    StatisticsCollector statisticsCollector;
    EventLog eventLog;

    set<int> knownVehicleIds;
    map<int, bool> previousStoppedState;
    map<int, int> previousRoadState;
    map<int, bool> previousCrossingState;

    string determineStopReason(const Vehicle& vehicle, const City& city) const;
};
