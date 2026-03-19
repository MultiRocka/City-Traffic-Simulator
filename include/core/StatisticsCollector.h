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
    void update(double dt, double currentTime, const vector<Vehicle>& vehicles);
    void exportSummary() const;

    const StatisticsSnapshot& getLastSnapshot() const;

private:
    void writeTimelineHeaderIfNeeded();
    void appendTimelineRow(const StatisticsSnapshot& snapshot);

    string timelineFilePath;
    string summaryFilePath;
    bool timelineHeaderWritten;

    set<int> seenVehicleIds;
    set<int> finishedVehicleIds;

    double totalStoppedTime;
    double accumulatedAverageSpeed;
    double accumulatedAverageMovingSpeed;
    int tickCount;
    int maxActiveVehicles;

    StatisticsSnapshot lastSnapshot;
};
