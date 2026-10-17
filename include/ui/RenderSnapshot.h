#pragma once

#include <string>
#include <vector>
#include "core/Vector2D.h"

using namespace std;

struct RoadRenderInfo {
    int roadId;
    Vector2D start;
    Vector2D end;
    double length;
    int laneCount;
    // Full centerline polyline including start/end (intermediates when present).
    vector<Vector2D> centerline;
};

struct VehicleRenderInfo {
    int vehicleId;
    int roadSegmentId;
    int laneIndex;
    Vector2D worldPosition;
    Vector2D previousWorldPosition;
    double positionOnRoad;
    double speed;
    bool stopped;
    bool finished;
};

struct TrafficLightRenderInfo {
    int lightId;
    int roadSegmentId;
    Vector2D worldPosition;
    bool green;
};

struct CrossingRenderInfo {
    int crossingId;
    int roadSegmentId;
    Vector2D worldPosition;
    bool active;
};

struct PedestrianRenderInfo {
    int pedestrianId;
    int crossingId;
    int roadSegmentId;
    Vector2D worldPosition;
    // 0 = waiting off-road, 1 = crossing on roadway
    int state;
    double crossProgress;
};

struct IntersectionRenderInfo {
    int intersectionId;
    Vector2D position;
};

struct LaneBlockageRenderInfo {
    int blockageId;
    int roadSegmentId;
    int laneIndex;
    Vector2D startWorld;
    Vector2D endWorld;
    // Optional zipper merge approach marker (same lane).
    Vector2D mergeStartWorld;
    bool hasMergeZone;
    double positionAlongRoad;
    double length;
    double startTime;
    double duration;
    bool active;
};

struct RenderSnapshot {
    double simulationTime;
    string cityName;
    vector<RoadRenderInfo> roads;
    vector<VehicleRenderInfo> vehicles;
    vector<TrafficLightRenderInfo> trafficLights;
    vector<CrossingRenderInfo> crossings;
    vector<PedestrianRenderInfo> pedestrians;
    vector<LaneBlockageRenderInfo> laneBlockages;
    vector<IntersectionRenderInfo> intersections;
};
