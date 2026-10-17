#include "ui/RenderComposer.h"
#include "ui/LaneBlockageRenderer.h"
#include <algorithm>
#include <cmath>

using namespace std;

Vector2D RenderComposer::lerpPosition(const Vector2D& from, const Vector2D& to, double alpha) {
    alpha = max(0.0, min(1.0, alpha));

    return Vector2D(
        from.x + (to.x - from.x) * alpha,
        from.y + (to.y - from.y) * alpha
    );
}

RenderSnapshot RenderComposer::buildSnapshot(const City& city, const Simulation& simulation) const {
    RenderSnapshot snapshot;
    snapshot.simulationTime = simulation.getTime();
    snapshot.cityName = city.getName();

    cityRenderer.buildRoads(city, snapshot);
    cityRenderer.buildIntersections(city, snapshot);
    vehicleRenderer.buildVehicles(city, simulation, snapshot);
    trafficLightRenderer.buildTrafficLights(city, simulation, snapshot);
    pedestrianRenderer.buildCrossings(city, simulation, snapshot);
    pedestrianRenderer.buildPedestrians(city, simulation, snapshot);

    LaneBlockageRenderer laneBlockageRenderer;
    laneBlockageRenderer.buildLaneBlockages(city, simulation, snapshot);

    return snapshot;
}

RenderSnapshot RenderComposer::interpolate(const RenderSnapshot& previous, const RenderSnapshot& current, double alpha) const {
    RenderSnapshot display = current;

    if (alpha >= 1.0 || previous.vehicles.empty()) {
        for (auto& vehicle : display.vehicles) {
            vehicle.previousWorldPosition = vehicle.worldPosition;
        }
        return display;
    }

    if (alpha <= 0.0) {
        display.vehicles = previous.vehicles;
        for (auto& vehicle : display.vehicles) {
            vehicle.previousWorldPosition = vehicle.worldPosition;
        }
        return display;
    }

    for (auto& vehicle : display.vehicles) {
        if (vehicle.finished) {
            continue;
        }

        const VehicleRenderInfo* previousVehicle = nullptr;

        for (const auto& candidate : previous.vehicles) {
            if (candidate.vehicleId == vehicle.vehicleId) {
                previousVehicle = &candidate;
                break;
            }
        }

        if (previousVehicle == nullptr) {
            vehicle.previousWorldPosition = vehicle.worldPosition;
            continue;
        }

        vehicle.previousWorldPosition = previousVehicle->worldPosition;
        vehicle.worldPosition = lerpPosition(previousVehicle->worldPosition, vehicle.worldPosition, alpha);
        vehicle.positionOnRoad = previousVehicle->positionOnRoad + (vehicle.positionOnRoad - previousVehicle->positionOnRoad) * alpha;
        vehicle.speed = previousVehicle->speed + (vehicle.speed - previousVehicle->speed) * alpha;
        vehicle.stopped = vehicle.speed < 0.2;
    }

    display.simulationTime = previous.simulationTime + (current.simulationTime - previous.simulationTime) * alpha;
    return display;
}
