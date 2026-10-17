#include "ui/CityRenderer.h"

using namespace std;

void CityRenderer::buildRoads(const City& city, RenderSnapshot& snapshot) const {
    for (const auto& roadSegment : city.getRoadSegments()) {
        const Intersection* fromIntersection = city.findIntersectionById(roadSegment.getFromIntersectionId());
        const Intersection* toIntersection = city.findIntersectionById(roadSegment.getToIntersectionId());

        if (fromIntersection == nullptr || toIntersection == nullptr) {
            continue;
        }

        RoadRenderInfo roadInfo;
        roadInfo.roadId = roadSegment.getId();
        roadInfo.start = fromIntersection->getPosition();
        roadInfo.end = toIntersection->getPosition();
        roadInfo.length = roadSegment.getLength();
        roadInfo.laneCount = roadSegment.getLaneCount();
        roadInfo.centerline.push_back(roadInfo.start);
        for (const auto& point : roadSegment.getIntermediatePoints()) {
            roadInfo.centerline.push_back(point);
        }
        roadInfo.centerline.push_back(roadInfo.end);

        snapshot.roads.push_back(roadInfo);
    }
}

void CityRenderer::buildIntersections(const City& city, RenderSnapshot& snapshot) const {
    for (const auto& intersection : city.getIntersections()) {
        IntersectionRenderInfo intersectionInfo;
        intersectionInfo.intersectionId = intersection.getId();
        intersectionInfo.position = intersection.getPosition();
        snapshot.intersections.push_back(intersectionInfo);
    }
}
