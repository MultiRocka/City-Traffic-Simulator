#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <iomanip>
#include <cstdlib>
#include "ui/RenderSnapshot.h"
#include "ui/CityLayout.h"

using namespace std;

inline void clearConsolePreview() {
#ifdef _WIN32
    system("cls");
#else
    system("clear");
#endif
}

inline char getVehicleChar(int id) {
    return static_cast<char>('0' + (id % 10));
}

inline string getVehicleStatusText(const VehicleRenderInfo& vehicle) {
    if (vehicle.finished) {
        return "FINISHED";
    }

    if (vehicle.stopped) {
        return "STOPPED";
    }

    return "MOVING";
}

inline int worldToGridX(double worldX, const WorldBounds& bounds, int gridWidth) {
  double span = bounds.maxX - bounds.minX;

  if (span <= 0.0) {
    return gridWidth / 2;
  }

  double ratio = (worldX - bounds.minX) / span;
  return static_cast<int>(ratio * (gridWidth - 1));
}

inline int worldToGridY(double worldY, const WorldBounds& bounds, int gridHeight) {
  double span = bounds.maxY - bounds.minY;

  if (span <= 0.0) {
    return gridHeight / 2;
  }

  double ratio = (worldY - bounds.minY) / span;
  return static_cast<int>((1.0 - ratio) * (gridHeight - 1));
}

inline void drawLine(vector<string>& grid, int x1, int y1, int x2, int y2, char symbol) {
  int dx = abs(x2 - x1);
  int dy = abs(y2 - y1);
  int steps = max(dx, dy);

  if (steps == 0) {
    if (y1 >= 0 && y1 < static_cast<int>(grid.size()) &&
        x1 >= 0 && x1 < static_cast<int>(grid[y1].size())) {
      grid[y1][x1] = symbol;
    }
    return;
  }

  for (int step = 0; step <= steps; step++) {
    double t = static_cast<double>(step) / steps;
    int x = x1 + static_cast<int>((x2 - x1) * t);
    int y = y1 + static_cast<int>((y2 - y1) * t);

    if (y >= 0 && y < static_cast<int>(grid.size()) &&
        x >= 0 && x < static_cast<int>(grid[y].size())) {
      grid[y][x] = symbol;
    }
  }
}

inline void renderConsolePreview(const RenderSnapshot& snapshot) {
    const int gridWidth = 50;
    const int gridHeight = 20;
    vector<string> grid(gridHeight, string(gridWidth, ' '));

    CityLayout layout;
    WorldBounds bounds = layout.getWorldBoundsFromSnapshot(snapshot);

    for (const auto& road : snapshot.roads) {
        int x1 = worldToGridX(road.start.x, bounds, gridWidth);
        int y1 = worldToGridY(road.start.y, bounds, gridHeight);
        int x2 = worldToGridX(road.end.x, bounds, gridWidth);
        int y2 = worldToGridY(road.end.y, bounds, gridHeight);
        drawLine(grid, x1, y1, x2, y2, '-');
    }

    for (const auto& intersection : snapshot.intersections) {
        int x = worldToGridX(intersection.position.x, bounds, gridWidth);
        int y = worldToGridY(intersection.position.y, bounds, gridHeight);

        if (y >= 0 && y < gridHeight && x >= 0 && x < gridWidth) {
            grid[y][x] = '+';
        }
    }

    for (const auto& light : snapshot.trafficLights) {
        int x = worldToGridX(light.worldPosition.x, bounds, gridWidth);
        int y = worldToGridY(light.worldPosition.y, bounds, gridHeight);

        if (y >= 0 && y < gridHeight && x >= 0 && x < gridWidth) {
            grid[y][x] = light.green ? 'G' : 'R';
        }
    }

    for (const auto& crossing : snapshot.crossings) {
        int x = worldToGridX(crossing.worldPosition.x, bounds, gridWidth);
        int y = worldToGridY(crossing.worldPosition.y, bounds, gridHeight);

        if (y >= 0 && y < gridHeight && x >= 0 && x < gridWidth) {
            grid[y][x] = crossing.active ? 'P' : 'x';
        }
    }

    for (const auto& vehicle : snapshot.vehicles) {
        if (vehicle.finished) {
            continue;
        }

        int x = worldToGridX(vehicle.worldPosition.x, bounds, gridWidth);
        int y = worldToGridY(vehicle.worldPosition.y, bounds, gridHeight);

        if (y >= 0 && y < gridHeight && x >= 0 && x < gridWidth) {
            grid[y][x] = getVehicleChar(vehicle.vehicleId);
        }
    }

    cout << "Easy Rider - " << snapshot.cityName << endl;
    cout << "Time: " << fixed << setprecision(1) << snapshot.simulationTime << " s" << endl;
    cout << string(50, '=') << endl;

    for (const auto& row : grid) {
        cout << row << endl;
    }

    cout << string(50, '=') << endl;
    cout << "Legend: 0-9 = vehicles, G = green, R = red, P = active crossing, x = inactive crossing" << endl;
    cout << endl;
    cout << "Vehicles:" << endl;

    if (snapshot.vehicles.empty()) {
        cout << "No vehicles yet." << endl;
    } else {
        for (const auto& vehicle : snapshot.vehicles) {
            cout << getVehicleChar(vehicle.vehicleId) << " (id=" << vehicle.vehicleId << ")"
                 << " | road: " << vehicle.roadSegmentId
                 << " | pos: " << fixed << setprecision(1) << vehicle.positionOnRoad << " m"
                 << " | speed: " << fixed << setprecision(1) << vehicle.speed << " m/s"
                 << " | status: " << getVehicleStatusText(vehicle)
                 << endl;
        }
    }

    cout << endl;
    cout << "Crossings:" << endl;

    if (snapshot.crossings.empty()) {
        cout << "No pedestrian crossings." << endl;
    } else {
        for (const auto& crossing : snapshot.crossings) {
            cout << "Crossing " << crossing.crossingId
                 << " | road: " << crossing.roadSegmentId
                 << " | state: " << (crossing.active ? "ACTIVE" : "INACTIVE")
                 << endl;
        }
    }

    cout << endl;
    cout << "Pedestrians:" << endl;
    if (snapshot.pedestrians.empty()) {
        cout << "No pedestrians." << endl;
    } else {
        for (const auto& ped : snapshot.pedestrians) {
            cout << "Ped " << ped.pedestrianId
                 << " | crossing: " << ped.crossingId
                 << " | road: " << ped.roadSegmentId
                 << " | state: " << (ped.state == 1 ? "CROSSING" : "WAITING")
                 << " | progress: " << fixed << setprecision(1) << ped.crossProgress << " m"
                 << endl;
        }
    }

    cout << endl;
    cout << "Lane blockages:" << endl;
    if (snapshot.laneBlockages.empty()) {
        cout << "No lane blockages." << endl;
    } else {
        for (const auto& block : snapshot.laneBlockages) {
            cout << "Block " << block.blockageId
                 << " | road: " << block.roadSegmentId
                 << " | lane: " << block.laneIndex
                 << " | pos: " << fixed << setprecision(1) << block.positionAlongRoad << " m"
                 << " | len: " << block.length << " m"
                 << " | state: " << (block.active ? "ACTIVE" : "INACTIVE")
                 << endl;
        }
    }
}
