#include "core/StatisticsCollector.h"
#include <fstream>
#include <iomanip>
#include <sstream>
#include <algorithm>

using namespace std;

StatisticsCollector::StatisticsCollector() {
    reset();
}

void StatisticsCollector::setOutputFiles(const string& timelineFile, const string& summaryFile) {
    timelineFilePath = timelineFile;
    summaryFilePath = summaryFile;
}

void StatisticsCollector::reset() {
    timelineFilePath = "simulation_timeline.csv";
    summaryFilePath = "simulation_summary.csv";
    resetRuntime();
}

void StatisticsCollector::resetRuntime() {
    seenVehicleIds.clear();
    finishedVehicleIds.clear();
    timeline.clear();

    totalStoppedTime = 0.0;
    accumulatedAverageSpeed = 0.0;
    accumulatedAverageMovingSpeed = 0.0;
    tickCount = 0;
    maxActiveVehicles = 0;

    lastSnapshot = {0.0, 0, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0};
}

string StatisticsCollector::formatCsvDouble(double value) const {
    ostringstream stream;
    stream << fixed << setprecision(2) << value;

    string text = stream.str();
    replace(text.begin(), text.end(), '.', ',');

    return text;
}

void StatisticsCollector::update(double dt, double currentTime, const vector<Vehicle>& vehicles) {
    int activeVehicles = 0;
    int movingVehicles = 0;
    int stoppedVehicles = 0;

    double speedSum = 0.0;
    double movingSpeedSum = 0.0;

    for (const auto& vehicle : vehicles) {
        seenVehicleIds.insert(vehicle.getId());

        if (vehicle.isFinished()) {
            finishedVehicleIds.insert(vehicle.getId());
            continue;
        }

        activeVehicles++;
        speedSum += vehicle.getSpeed();

        if (vehicle.isStopped()) {
            stoppedVehicles++;
            totalStoppedTime += dt;
        } else {
            movingVehicles++;
            movingSpeedSum += vehicle.getSpeed();
        }
    }

    if (activeVehicles > maxActiveVehicles) {
        maxActiveVehicles = activeVehicles;
    }

    double averageSpeed = 0.0;
    if (activeVehicles > 0) {
        averageSpeed = speedSum / activeVehicles;
    }

    double averageMovingSpeed = 0.0;
    if (movingVehicles > 0) {
        averageMovingSpeed = movingSpeedSum / movingVehicles;
    }

    accumulatedAverageSpeed += averageSpeed;
    accumulatedAverageMovingSpeed += averageMovingSpeed;
    tickCount++;

    lastSnapshot.time = currentTime;
    lastSnapshot.totalVehiclesSeen = static_cast<int>(seenVehicleIds.size());
    lastSnapshot.activeVehicles = activeVehicles;
    lastSnapshot.movingVehicles = movingVehicles;
    lastSnapshot.stoppedVehicles = stoppedVehicles;
    lastSnapshot.finishedVehicles = static_cast<int>(finishedVehicleIds.size());
    lastSnapshot.averageSpeed = averageSpeed;
    lastSnapshot.averageMovingSpeed = averageMovingSpeed;
    lastSnapshot.totalStoppedTime = totalStoppedTime;
    lastSnapshot.maxActiveVehicles = maxActiveVehicles;

    timeline.push_back(lastSnapshot);
}

void StatisticsCollector::writeTimeline() const {
    ofstream file(timelineFilePath);

    file << "sep=;\n";
    file << "time;total_vehicles_seen;active_vehicles;moving_vehicles;stopped_vehicles;finished_vehicles;average_speed;average_moving_speed;total_stopped_time;max_active_vehicles\n";

    for (const auto& snapshot : timeline) {
        file << formatCsvDouble(snapshot.time) << ";"
             << snapshot.totalVehiclesSeen << ";"
             << snapshot.activeVehicles << ";"
             << snapshot.movingVehicles << ";"
             << snapshot.stoppedVehicles << ";"
             << snapshot.finishedVehicles << ";"
             << formatCsvDouble(snapshot.averageSpeed) << ";"
             << formatCsvDouble(snapshot.averageMovingSpeed) << ";"
             << formatCsvDouble(snapshot.totalStoppedTime) << ";"
             << snapshot.maxActiveVehicles
             << "\n";
    }

    file.close();
}

void StatisticsCollector::writeSummary() const {
    ofstream file(summaryFilePath);

    double overallAverageSpeed = 0.0;
    double overallAverageMovingSpeed = 0.0;

    if (tickCount > 0) {
        overallAverageSpeed = accumulatedAverageSpeed / tickCount;
        overallAverageMovingSpeed = accumulatedAverageMovingSpeed / tickCount;
    }

    file << "sep=;\n";
    file << "metric;value\n";
    file << "simulation_time;" << formatCsvDouble(lastSnapshot.time) << "\n";
    file << "total_vehicles_seen;" << lastSnapshot.totalVehiclesSeen << "\n";
    file << "active_vehicles;" << lastSnapshot.activeVehicles << "\n";
    file << "finished_vehicles;" << lastSnapshot.finishedVehicles << "\n";
    file << "moving_vehicles;" << lastSnapshot.movingVehicles << "\n";
    file << "stopped_vehicles;" << lastSnapshot.stoppedVehicles << "\n";
    file << "overall_average_speed;" << formatCsvDouble(overallAverageSpeed) << "\n";
    file << "overall_average_moving_speed;" << formatCsvDouble(overallAverageMovingSpeed) << "\n";
    file << "total_stopped_time;" << formatCsvDouble(totalStoppedTime) << "\n";
    file << "max_active_vehicles;" << maxActiveVehicles << "\n";

    file.close();
}

void StatisticsCollector::exportAll() const {
    writeTimeline();
    writeSummary();
}

const StatisticsSnapshot& StatisticsCollector::getLastSnapshot() const {
    return lastSnapshot;
}
