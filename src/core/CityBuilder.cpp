#include "core/CityBuilder.h"

#include <cmath>
#include <sstream>
#include <unordered_set>

using namespace std;

double CityBuilder::polylineLengthTolerance = 1.0;

static double computePolylineLength(
    const Vector2D& from,
    const vector<Vector2D>& intermediates,
    const Vector2D& to
) {
    double total = 0.0;
    Vector2D previous = from;

    for (const auto& point : intermediates) {
        total += previous.distanceTo(point);
        previous = point;
    }

    total += previous.distanceTo(to);
    return total;
}

bool CityBuilder::build(const MapDescription& description, City& outCity, string& error) {
    outCity = City(description.name.empty() ? "Unnamed City" : description.name);
    error.clear();

    unordered_set<int> intersectionIds;
    unordered_set<int> roadIds;

    for (const auto& intersectionDesc : description.intersections) {
        if (intersectionDesc.id < 0) {
            error = "intersection id must be non-negative";
            outCity = City();
            return false;
        }

        if (!intersectionIds.insert(intersectionDesc.id).second) {
            ostringstream message;
            message << "duplicate intersection id " << intersectionDesc.id;
            error = message.str();
            outCity = City();
            return false;
        }

        Intersection intersection(intersectionDesc.id, intersectionDesc.position);
        intersection.setSourceRef(intersectionDesc.sourceRef);
        intersection.setPriorityMode(intersectionDesc.priorityMode);
        intersection.setPriorityRoadIds(intersectionDesc.priorityRoadIds);
        outCity.addIntersection(intersection);
    }

    for (const auto& roadDesc : description.roads) {
        if (roadDesc.id < 0) {
            error = "road id must be non-negative";
            outCity = City();
            return false;
        }

        if (!roadIds.insert(roadDesc.id).second) {
            ostringstream message;
            message << "duplicate road id " << roadDesc.id;
            error = message.str();
            outCity = City();
            return false;
        }

        if (roadDesc.length <= 0.0) {
            ostringstream message;
            message << "road " << roadDesc.id << " has invalid length (must be > 0)";
            error = message.str();
            outCity = City();
            return false;
        }

        const int laneCount = roadDesc.laneCount < 0 ? 1 : roadDesc.laneCount;
        if (laneCount < 1) {
            ostringstream message;
            message << "road " << roadDesc.id << " has invalid laneCount (must be >= 1)";
            error = message.str();
            outCity = City();
            return false;
        }

        if (outCity.findIntersectionById(roadDesc.fromIntersectionId) == nullptr) {
            ostringstream message;
            message << "road " << roadDesc.id << " references missing from-intersection "
                    << roadDesc.fromIntersectionId;
            error = message.str();
            outCity = City();
            return false;
        }

        if (outCity.findIntersectionById(roadDesc.toIntersectionId) == nullptr) {
            ostringstream message;
            message << "road " << roadDesc.id << " references missing to-intersection "
                    << roadDesc.toIntersectionId;
            error = message.str();
            outCity = City();
            return false;
        }

        if (!roadDesc.intermediatePoints.empty()) {
            const Intersection* fromIntersection = outCity.findIntersectionById(roadDesc.fromIntersectionId);
            const Intersection* toIntersection = outCity.findIntersectionById(roadDesc.toIntersectionId);
            const double geometryLength = computePolylineLength(
                fromIntersection->getPosition(),
                roadDesc.intermediatePoints,
                toIntersection->getPosition()
            );

            if (fabs(geometryLength - roadDesc.length) > polylineLengthTolerance) {
                ostringstream message;
                message << "road " << roadDesc.id
                        << " length does not match polyline geometry within tolerance";
                error = message.str();
                outCity = City();
                return false;
            }
        }

        RoadSegment road(
            roadDesc.id,
            roadDesc.fromIntersectionId,
            roadDesc.toIntersectionId,
            roadDesc.length,
            roadDesc.speedLimit
        );
        road.setLaneCount(laneCount);
        road.setIntermediatePoints(roadDesc.intermediatePoints);
        road.setSourceRef(roadDesc.sourceRef);
        road.setAllowAutoDemand(roadDesc.allowAutoDemand);
        outCity.addRoadSegment(road);
    }

    unordered_set<int> crossingIds;

    for (const auto& crossingDesc : description.crossings) {
        if (crossingDesc.id < 0) {
            error = "crossing id must be non-negative";
            outCity = City();
            return false;
        }

        if (!crossingIds.insert(crossingDesc.id).second) {
            ostringstream message;
            message << "duplicate crossing id " << crossingDesc.id;
            error = message.str();
            outCity = City();
            return false;
        }

        const RoadSegment* road = outCity.findRoadSegmentById(crossingDesc.roadSegmentId);
        if (road == nullptr) {
            ostringstream message;
            message << "crossing " << crossingDesc.id << " references missing road "
                    << crossingDesc.roadSegmentId;
            error = message.str();
            outCity = City();
            return false;
        }

        if (crossingDesc.positionOnRoad < 0.0 || crossingDesc.positionOnRoad > road->getLength()) {
            ostringstream message;
            message << "crossing " << crossingDesc.id << " position out of range for road "
                    << road->getId();
            error = message.str();
            outCity = City();
            return false;
        }
    }

    for (const auto& connectionDesc : description.connections) {
        if (outCity.findRoadSegmentById(connectionDesc.fromRoadId) == nullptr ||
            outCity.findRoadSegmentById(connectionDesc.toRoadId) == nullptr) {
            ostringstream message;
            message << "connection references missing road(s): "
                    << connectionDesc.fromRoadId << " -> " << connectionDesc.toRoadId;
            error = message.str();
            outCity = City();
            return false;
        }

        if (!outCity.areRoadsGeometricallyConnected(connectionDesc.fromRoadId, connectionDesc.toRoadId)) {
            ostringstream message;
            message << "invalid connection (roads not geometrically connected): "
                    << connectionDesc.fromRoadId << " -> " << connectionDesc.toRoadId;
            error = message.str();
            outCity = City();
            return false;
        }

        if (!connectionDesc.allowed) {
            outCity.disallowConnection(connectionDesc.fromRoadId, connectionDesc.toRoadId);
        }
    }

    return true;
}
