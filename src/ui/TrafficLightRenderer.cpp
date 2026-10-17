#include "ui/TrafficLightRenderer.h"

using namespace std;

void TrafficLightRenderer::buildTrafficLights(const City& city, const Simulation& simulation, RenderSnapshot& snapshot) const {
    for (const auto& trafficLight : simulation.getTrafficLights()) {
        const RoadSegment* road = city.findRoadSegmentById(trafficLight.getRoadSegmentId());

        if (road == nullptr) {
            continue;
        }

        TrafficLightRenderInfo lightInfo;
        lightInfo.lightId = trafficLight.getId();
        lightInfo.roadSegmentId = trafficLight.getRoadSegmentId();
        lightInfo.green = trafficLight.isGreen();
        lightInfo.worldPosition = layout.getRoadPosition(
            city,
            trafficLight.getRoadSegmentId(),
            road->getLength() - 1.0
        );

        snapshot.trafficLights.push_back(lightInfo);
    }
}
