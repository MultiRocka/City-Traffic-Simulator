#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <iomanip>
#include <cstdlib>
#include "core/Simulation.h"
#include "core/City.h"
#include "core/Vehicle.h"
#include "core/PedestrianCrossing.h"

using namespace std;

inline void clearConsolePreview() {
    system("cls");
}

inline void putChar(vector<string>& grid, int x, int y, char c) {
    if (y >= 0 && y < static_cast<int>(grid.size()) &&
        x >= 0 && x < static_cast<int>(grid[y].size())) {
        grid[y][x] = c;
    }
}

inline int mapRange(double value, double maxValue, int startValue, int endValue) {
    if (maxValue <= 0.0) {
        return startValue;
    }

    double ratio = value / maxValue;

    if (ratio < 0.0) {
        ratio = 0.0;
    }

    if (ratio > 1.0) {
        ratio = 1.0;
    }

    return startValue + static_cast<int>(ratio * (endValue - startValue));
}

inline char getVehicleChar(int id) {
    return static_cast<char>('0' + (id % 10));
}

inline string getVehicleStatusText(const Vehicle& vehicle) {
    if (vehicle.isFinished()) {
        return "FINISHED";
    }

    if (vehicle.isStopped()) {
        return "STOPPED";
    }

    return "MOVING";
}

inline void renderConsolePreview(const City& city, const Simulation& sim) {
    static set<int> knownVehicleIds;

    clearConsolePreview();

    vector<string> grid(15, string(43, ' '));

    for (int x = 2; x <= 18; x++) {
        putChar(grid, x, 7, '-');
    }

    for (int x = 20; x <= 38; x++) {
        putChar(grid, x, 7, '-');
    }

    for (int y = 1; y <= 6; y++) {
        putChar(grid, 20, y, '|');
    }

    putChar(grid, 20, 7, '+');

    for (const auto& light : sim.getTrafficLights()) {
        if (light.getRoadSegmentId() == 1) {
            putChar(grid, 18, 7, light.isGreen() ? 'G' : 'R');
        }

        if (light.getRoadSegmentId() == 2) {
            putChar(grid, 20, 6, light.isGreen() ? 'G' : 'R');
        }
    }

    const RoadSegment* road1 = city.findRoadSegmentById(1);
    const RoadSegment* road2 = city.findRoadSegmentById(2);
    const RoadSegment* road3 = city.findRoadSegmentById(3);

    for (const auto& crossing : sim.getPedestrianCrossings()) {
        char crossingChar = crossing.isActive() ? 'P' : 'x';

        if (crossing.getRoadSegmentId() == 1 && road1 != nullptr) {
            int x = mapRange(crossing.getPositionOnRoad(), road1->getLength(), 2, 17);
            putChar(grid, x, 7, crossingChar);
        }

        if (crossing.getRoadSegmentId() == 2 && road2 != nullptr) {
            int y = mapRange(crossing.getPositionOnRoad(), road2->getLength(), 1, 5);
            putChar(grid, 20, y, crossingChar);
        }

        if (crossing.getRoadSegmentId() == 3 && road3 != nullptr) {
            int x = mapRange(crossing.getPositionOnRoad(), road3->getLength(), 21, 38);
            putChar(grid, x, 7, crossingChar);
        }
    }

    for (const auto& vehicle : sim.getVehicles()) {
        if (vehicle.isFinished()) {
            continue;
        }

        char symbol = getVehicleChar(vehicle.getId());

        if (vehicle.getCurrentRoadSegmentId() == 1 && road1 != nullptr) {
            int x = mapRange(vehicle.getPositionOnRoad(), road1->getLength(), 2, 17);
            putChar(grid, x, 7, symbol);
        }

        if (vehicle.getCurrentRoadSegmentId() == 2 && road2 != nullptr) {
            int y = mapRange(vehicle.getPositionOnRoad(), road2->getLength(), 1, 5);
            putChar(grid, 20, y, symbol);
        }

        if (vehicle.getCurrentRoadSegmentId() == 3 && road3 != nullptr) {
            int x = mapRange(vehicle.getPositionOnRoad(), road3->getLength(), 21, 38);
            putChar(grid, x, 7, symbol);
        }
    }

    cout << "Easy Rider - Intersection Preview" << endl;
    cout << "Time: " << fixed << setprecision(1) << sim.getTime() << " s" << endl;
    cout << "========================================" << endl;

    for (const auto& row : grid) {
        cout << row << endl;
    }

    cout << "========================================" << endl;
    cout << "Legend: 1/2/3... = vehicles, G = green, R = red, P = active pedestrian, x = inactive crossing" << endl;
    cout << endl;
    cout << "Vehicle details:" << endl;

    if (sim.getVehicles().empty()) {
        cout << "No vehicles yet." << endl;
    } else {
        for (const auto& vehicle : sim.getVehicles()) {
            bool isNew = knownVehicleIds.find(vehicle.getId()) == knownVehicleIds.end();

            cout << getVehicleChar(vehicle.getId()) << " (id=" << vehicle.getId() << ")";

            if (isNew) {
                cout << " [NEW]";
                knownVehicleIds.insert(vehicle.getId());
            }

            cout << " | road: " << vehicle.getCurrentRoadSegmentId()
                 << " | roadIndex: " << vehicle.getCurrentRoadIndex()
                 << " | pos: " << fixed << setprecision(1) << vehicle.getPositionOnRoad() << " m"
                 << " | speed: " << fixed << setprecision(1) << vehicle.getSpeed() << " m/s"
                 << " | status: " << getVehicleStatusText(vehicle)
                 << endl;
        }
    }

    cout << endl;
    cout << "Crossings:" << endl;

    if (sim.getPedestrianCrossings().empty()) {
        cout << "No pedestrian crossings." << endl;
    } else {
        for (const auto& crossing : sim.getPedestrianCrossings()) {
            cout << "Crossing " << crossing.getId()
                 << " | road: " << crossing.getRoadSegmentId()
                 << " | pos: " << fixed << setprecision(1) << crossing.getPositionOnRoad() << " m"
                 << " | state: " << (crossing.isActive() ? "ACTIVE" : "INACTIVE")
                 << endl;
        }
    }
}
