#include "core/City.h"

using namespace std;

RoadConnection::RoadConnection() : fromRoadId(-1), toRoadId(-1) {
}

RoadConnection::RoadConnection(int fromId, int toId) : fromRoadId(fromId), toRoadId(toId) {
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
    intersections.push_back(intersection);
}

void City::addRoadSegment(const RoadSegment& roadSegment) {
    roadSegments.push_back(roadSegment);

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
    for (auto& intersection : intersections) {
        if (intersection.getId() == id) {
            return &intersection;
        }
    }

    return nullptr;
}

RoadSegment* City::findRoadSegmentById(int id) {
    for (auto& roadSegment : roadSegments) {
        if (roadSegment.getId() == id) {
            return &roadSegment;
        }
    }

    return nullptr;
}

const Intersection* City::findIntersectionById(int id) const {
    for (const auto& intersection : intersections) {
        if (intersection.getId() == id) {
            return &intersection;
        }
    }

    return nullptr;
}

const RoadSegment* City::findRoadSegmentById(int id) const {
    for (const auto& roadSegment : roadSegments) {
        if (roadSegment.getId() == id) {
            return &roadSegment;
        }
    }

    return nullptr;
}

void City::disallowConnection(int fromRoadId, int toRoadId) {
    if (isConnectionDisallowed(fromRoadId, toRoadId)) {
        return;
    }

    disallowedConnections.push_back(RoadConnection(fromRoadId, toRoadId));
}

bool City::isConnectionDisallowed(int fromRoadId, int toRoadId) const {
    for (const auto& connection : disallowedConnections) {
        if (connection.fromRoadId == fromRoadId && connection.toRoadId == toRoadId) {
            return true;
        }
    }

    return false;
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

    int intersectionId = road->getToIntersectionId();

    for (const auto& candidateRoad : roadSegments) {
        if (candidateRoad.getFromIntersectionId() == intersectionId &&
            candidateRoad.getId() != roadId &&
            areRoadsConnected(roadId, candidateRoad.getId())) {
            outgoingRoadIds.push_back(candidateRoad.getId());
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

    int intersectionId = road->getFromIntersectionId();

    for (const auto& candidateRoad : roadSegments) {
        if (candidateRoad.getToIntersectionId() == intersectionId &&
            candidateRoad.getId() != roadId &&
            areRoadsConnected(candidateRoad.getId(), roadId)) {
            incomingRoadIds.push_back(candidateRoad.getId());
        }
    }

    return incomingRoadIds;
}

vector<int> City::getRoadIdsAtIntersection(int intersectionId) const {
    vector<int> roadIds;
    const Intersection* intersection = findIntersectionById(intersectionId);

    if (intersection != nullptr) {
        return intersection->getConnectedRoadIds();
    }

    return roadIds;
}
