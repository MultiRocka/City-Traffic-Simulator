#include "ui/VehicleRenderer.h"

using namespace std;

void VehicleRenderer::buildVehicles(const City& city, const Simulation& simulation, RenderSnapshot& snapshot) const {
    for (const auto& vehicle : simulation.getVehicles()) {
        if (vehicle.isFinished()) {
            continue;
        }

        VehicleRenderInfo vehicleInfo;
        vehicleInfo.vehicleId = vehicle.getId();
        vehicleInfo.roadSegmentId = vehicle.getCurrentRoadSegmentId();
        vehicleInfo.laneIndex = vehicle.getLaneIndex();
        vehicleInfo.positionOnRoad = vehicle.getPositionOnRoad();
        vehicleInfo.speed = vehicle.getSpeed();
        vehicleInfo.stopped = vehicle.isStopped();
        vehicleInfo.finished = false;

        int laneCount = 1;
        const RoadSegment* road = city.findRoadSegmentById(vehicle.getCurrentRoadSegmentId());
        if (road != nullptr) {
            laneCount = road->getLaneCount();
        }

        vehicleInfo.worldPosition = layout.getRoadPosition(
            city,
            vehicle.getCurrentRoadSegmentId(),
            vehicle.getPositionOnRoad(),
            vehicle.getLaneIndex(),
            laneCount
        );
        vehicleInfo.previousWorldPosition = vehicleInfo.worldPosition;

        snapshot.vehicles.push_back(vehicleInfo);
    }
}
