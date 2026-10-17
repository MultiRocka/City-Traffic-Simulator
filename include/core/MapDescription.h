#pragma once

#include <string>
#include <vector>
#include "core/Intersection.h"
#include "core/Vector2D.h"

using namespace std;

// Plain data boundary for building a City (synthetic or future OSM importer).
// Units: length meters, speedLimit m/s, positions meters, times seconds.
// Empty sourceRef means synthetic/local (not from an external map source).

struct MapIntersectionDesc {
    int id = -1;
    Vector2D position;
    string sourceRef;
    PriorityMode priorityMode = PriorityMode::None;
    vector<int> priorityRoadIds;
};

struct MapRoadDesc {
    int id = -1;
    int fromIntersectionId = -1;
    int toIntersectionId = -1;
    double length = 0.0;       // meters — authoritative for motion
    double speedLimit = 0.0;   // m/s
    // -1 = unset → CityBuilder defaults to 1. Explicit values must be >= 1.
    int laneCount = -1;
    vector<Vector2D> intermediatePoints; // optional polyline (rendering); not used as length source
    string sourceRef;
    // When false, OsmScenarioBuilder excludes this road from auto-generated demand routes
    // (destination / private-adjacent / conditional access). Still present for topology.
    bool allowAutoDemand = true;
};

struct MapCrossingDesc {
    int id = -1;
    int roadSegmentId = -1;
    double positionOnRoad = 0.0; // meters along road; must be in [0, road.length]
    double inactiveDuration = 5.0;
    double activeDuration = 3.0;
    bool startActive = false;
};

// Override default "all geometric turns allowed".
// allowed=false → disallowed turn. allowed=true → must be geometrically valid (no-op for whitelist).
struct MapConnectionDesc {
    int fromRoadId = -1;
    int toRoadId = -1;
    bool allowed = true;
};

struct MapDescription {
    string name;
    vector<MapIntersectionDesc> intersections;
    vector<MapRoadDesc> roads;
    vector<MapCrossingDesc> crossings;     // validated against roads; not stored on City
    vector<MapConnectionDesc> connections; // turn overrides
};
