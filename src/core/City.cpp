#include "core/City.h"

using namespace std;

RoadConnection::RoadConnection() : fromRoadId(-1), toRoadId(-1) {
}

RoadConnection::RoadConnection(int fromId, int toId) : fromRoadId(fromId), toRoadId(toId) {
}

uint64_t City::connectionKey(int fromRoadId, int toRoadId) {
    return (static_cast<uint64_t>(static_cast<uint32_t>(fromRoadId)) << 32) |
           static_cast<uint32_t>(toRoadId);
}

City::City() : name("Unnamed City") {
}

City::City(const string& name) : name(name) {
}

void City::setName(const string& newName) {
    name = newName;
}

string City::getName() const {
    return name;
}

void City::addIntersection(const Intersection& intersection) {
    intersectionIndexById[intersection.getId()] = intersections.size();
    intersections.push_back(intersection);
}

void City::addRoadSegment(const RoadSegment& roadSegment) {
    roadIndexById[roadSegment.getId()] = roadSegments.size();
    roadSegments.push_back(roadSegment);

    roadsFromIntersection[roadSegment.getFromIntersectionId()].push_back(roadSegment.getId());
    roadsToIntersection[roadSegment.getToIntersectionId()].push_back(roadSegment.getId());

    Intersection* fromIntersection = findIntersectionById(roadSegment.getFromIntersectionId());
    Intersection* toIntersection = findIntersectionById(roadSegment.getToIntersectionId());

    if (fromIntersection != nullptr) {
        fromIntersection->addConnectedRoadId(roadSegment.getId());
    }

    if (toIntersection != nullptr) {
        toIntersection->addConnectedRoadId(roadSegment.getId());
    }
}

const vector<Intersection>& City::getIntersections() const {
    return intersections;
}

const vector<RoadSegment>& City::getRoadSegments() const {
    return roadSegments;
}

Intersection* City::findIntersectionById(int id) {
    const auto it = intersectionIndexById.find(id);
    if (it == intersectionIndexById.end()) {
        return nullptr;
    }
    return &intersections[it->second];
}

RoadSegment* City::findRoadSegmentById(int id) {
    const auto it = roadIndexById.find(id);
    if (it == roadIndexById.end()) {
        return nullptr;
    }
    return &roadSegments[it->second];
}

const Intersection* City::findIntersectionById(int id) const {
    const auto it = intersectionIndexById.find(id);
    if (it == intersectionIndexById.end()) {
        return nullptr;
    }
    return &intersections[it->second];
}

const RoadSegment* City::findRoadSegmentById(int id) const {
    const auto it = roadIndexById.find(id);
    if (it == roadIndexById.end()) {
        return nullptr;
    }
    return &roadSegments[it->second];
}

void City::disallowConnection(int fromRoadId, int toRoadId) {
    if (isConnectionDisallowed(fromRoadId, toRoadId)) {
        return;
    }

    disallowedConnections.push_back(RoadConnection(fromRoadId, toRoadId));
    disallowedConnectionKeys.insert(connectionKey(fromRoadId, toRoadId));
}

bool City::isConnectionDisallowed(int fromRoadId, int toRoadId) const {
    return disallowedConnectionKeys.count(connectionKey(fromRoadId, toRoadId)) != 0;
}

const vector<RoadConnection>& City::getDisallowedConnections() const {
    return disallowedConnections;
}

bool City::areRoadsGeometricallyConnected(int fromRoadId, int toRoadId) const {
    const RoadSegment* fromRoad = findRoadSegmentById(fromRoadId);
    const RoadSegment* toRoad = findRoadSegmentById(toRoadId);

    if (fromRoad == nullptr || toRoad == nullptr) {
        return false;
    }

    return fromRoad->getToIntersectionId() == toRoad->getFromIntersectionId();
}

bool City::areRoadsConnected(int fromRoadId, int toRoadId) const {
    if (!areRoadsGeometricallyConnected(fromRoadId, toRoadId)) {
        return false;
    }

    return !isConnectionDisallowed(fromRoadId, toRoadId);
}

bool City::isValidRoute(const vector<int>& route) const {
    if (route.empty()) {
        return false;
    }

    for (int i = 0; i + 1 < static_cast<int>(route.size()); i++) {
        if (!areRoadsConnected(route[i], route[i + 1])) {
            return false;
        }
    }

    return true;
}

vector<int> City::getOutgoingRoadIds(int roadId) const {
    vector<int> outgoingRoadIds;
    const RoadSegment* road = findRoadSegmentById(roadId);

    if (road == nullptr) {
        return outgoingRoadIds;
    }

    const int intersectionId = road->getToIntersectionId();
    const auto it = roadsFromIntersection.find(intersectionId);
    if (it == roadsFromIntersection.end()) {
        return outgoingRoadIds;
    }

    for (int candidateId : it->second) {
        if (candidateId != roadId && areRoadsConnected(roadId, candidateId)) {
            outgoingRoadIds.push_back(candidateId);
        }
    }

    return outgoingRoadIds;
}

vector<int> City::getIncomingRoadIds(int roadId) const {
    vector<int> incomingRoadIds;
    const RoadSegment* road = findRoadSegmentById(roadId);

    if (road == nullptr) {
        return incomingRoadIds;
    }

    const int intersectionId = road->getFromIntersectionId();
    const auto it = roadsToIntersection.find(intersectionId);
    if (it == roadsToIntersection.end()) {
        return incomingRoadIds;
    }

    for (int candidateId : it->second) {
        if (candidateId != roadId && areRoadsConnected(candidateId, roadId)) {
            incomingRoadIds.push_back(candidateId);
        }
    }

    return incomingRoadIds;
}

vector<int> City::getRoadIdsAtIntersection(int intersectionId) const {
    const Intersection* intersection = findIntersectionById(intersectionId);

    if (intersection != nullptr) {
        return intersection->getConnectedRoadIds();
    }

    return {};
}
