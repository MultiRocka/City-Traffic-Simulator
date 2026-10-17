#pragma once

#include <string>
#include <vector>
#include <set>
#include "core/Vehicle.h"

using namespace std;

struct StatisticsSnapshot {
    double time;
    int totalVehiclesSeen;
    int activeVehicles;
    int movingVehicles;
    int stoppedVehicles;
    int finishedVehicles;
    double averageSpeed;
    double averageMovingSpeed;
    double totalStoppedTime;
    int maxActiveVehicles;
};

class StatisticsCollector {
public:
    StatisticsCollector();

    void setOutputFiles(const string& timelineFile, const string& summaryFile);
    void reset();
    void resetRuntime();
    void update(double dt, double currentTime, const vector<Vehicle>& vehicles);
    void exportAll() const;

    const StatisticsSnapshot& getLastSnapshot() const;

private:
    string formatCsvDouble(double value) const;
    void writeTimeline() const;
    void writeSummary() const;

    string timelineFilePath;
    string summaryFilePath;

    set<int> seenVehicleIds;
    set<int> finishedVehicleIds;

    double totalStoppedTime;
    double accumulatedAverageSpeed;
    double accumulatedAverageMovingSpeed;
    int tickCount;
    int maxActiveVehicles;

    StatisticsSnapshot lastSnapshot;
    vector<StatisticsSnapshot> timeline;
};
