#pragma once

#include <string>
#include <vector>
#include "core/Intersection.h"
#include "core/RoadSegment.h"

using namespace std;

// Simple turn restriction: fromRoadId -> toRoadId.
struct RoadConnection {
    int fromRoadId;
    int toRoadId;

    RoadConnection();
    RoadConnection(int fromId, int toId);
};

class City {
public:
    City();
    City(const string& name);

    void setName(const string& newName);
    string getName() const;

    void addIntersection(const Intersection& intersection);
    void addRoadSegment(const RoadSegment& roadSegment);

    const vector<Intersection>& getIntersections() const;
    const vector<RoadSegment>& getRoadSegments() const;

    Intersection* findIntersectionById(int id);
    RoadSegment* findRoadSegmentById(int id);

    const Intersection* findIntersectionById(int id) const;
    const RoadSegment* findRoadSegmentById(int id) const;

    // Default: any geometrically connected pair (shared intersection) is allowed.
    // disallowedConnections override that default.
    void disallowConnection(int fromRoadId, int toRoadId);
    bool isConnectionDisallowed(int fromRoadId, int toRoadId) const;
    const vector<RoadConnection>& getDisallowedConnections() const;

    bool areRoadsGeometricallyConnected(int fromRoadId, int toRoadId) const;
    bool areRoadsConnected(int fromRoadId, int toRoadId) const;
    bool isValidRoute(const vector<int>& route) const;

    vector<int> getOutgoingRoadIds(int roadId) const;
    vector<int> getIncomingRoadIds(int roadId) const;
    vector<int> getRoadIdsAtIntersection(int intersectionId) const;

private:
    string name;
    vector<Intersection> intersections;
    vector<RoadSegment> roadSegments;
    vector<RoadConnection> disallowedConnections;
};
