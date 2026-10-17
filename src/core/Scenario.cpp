#include "core/Scenario.h"
#include "core/CityBuilder.h"
#include "core/MapDescription.h"
#include "core/VehicleGenerator.h"
#include "core/TrafficLight.h"
#include "core/PedestrianCrossing.h"
#include "core/Phase.h"
#include "core/PhaseController.h"
#include "core/PathFinder.h"
#include "core/Vector2D.h"

using namespace std;

static int addRoadDesc(
    MapDescription& map,
    int& nextRoadId,
    int fromId,
    int toId,
    double length,
    double speedLimit,
    int laneCount = -1,
    const vector<Vector2D>& intermediatePoints = {}
) {
    MapRoadDesc road;
    road.id = nextRoadId++;
    road.fromIntersectionId = fromId;
    road.toIntersectionId = toId;
    road.length = length;
    road.speedLimit = speedLimit;
    // laneCount unset (-1) → CityBuilder defaults to 1; empty sourceRef → synthetic/local
    road.laneCount = laneCount;
    road.intermediatePoints = intermediatePoints;
    map.roads.push_back(road);
    return road.id;
}

static void addJunctionController(Simulation& sim, int controllerId, int intersectionId, int phase1RoadId, int phase2RoadId) {
    PhaseController controller(controllerId, intersectionId);
    controller.addPhase(Phase(1, 5.0, {phase1RoadId}));
    controller.addPhase(Phase(2, 5.0, {phase2RoadId}));
    sim.addPhaseController(controller);

    sim.addTrafficLight(TrafficLight(controllerId * 10 + 1, phase1RoadId, intersectionId, true));
    sim.addTrafficLight(TrafficLight(controllerId * 10 + 2, phase2RoadId, intersectionId, true));
}

static void addIntersection(MapDescription& map, int id, double x, double y, PriorityMode mode = PriorityMode::None) {
    MapIntersectionDesc node;
    node.id = id;
    node.position = Vector2D(x, y);
    node.priorityMode = mode;
    map.intersections.push_back(node);
}

MapDescription Scenario::buildDemoMapDescription() {
    // Compact demo city exercising the MapDescription → CityBuilder contract.
    // Road/intersection ids are local to this description; core rules must not special-case them.
    MapDescription map;
    map.name = "Demo City";

    // 1 WestIn, 2 PrioJ, 3 SigJ, 4 EastOut, 5 SouthIn, 6 NorthMid, 7 CurveEnd, 8 NorthWest, 9 SideIn
    addIntersection(map, 1, 0.0, 40.0);
    addIntersection(map, 2, 60.0, 40.0, PriorityMode::PriorityRoad);
    addIntersection(map, 3, 120.0, 40.0, PriorityMode::Signalized);
    addIntersection(map, 4, 190.0, 40.0);
    addIntersection(map, 5, 120.0, 0.0);
    addIntersection(map, 6, 120.0, 90.0, PriorityMode::YieldToRight);
    addIntersection(map, 7, 175.0, 95.0);
    addIntersection(map, 8, 0.0, 90.0);
    addIntersection(map, 9, 60.0, 0.0);

    int nextRoadId = 1;

    // One-way multi-lane west approach (lane-block / zipper demo site).
    const int westApproach = addRoadDesc(map, nextRoadId, 1, 2, 60.0, 7.0, 2);
    const int midCorridor = addRoadDesc(map, nextRoadId, 2, 3, 60.0, 6.0, 2);
    const int eastExit = addRoadDesc(map, nextRoadId, 3, 4, 70.0, 8.5, 1);
    const int southEntry = addRoadDesc(map, nextRoadId, 5, 3, 40.0, 5.0, 1);
    addRoadDesc(map, nextRoadId, 9, 2, 40.0, 4.5, 1); // side entry (PriorityRoad subordinate)

    // Two-way pair (separate directed edges) on the north–south link at SigJ.
    addRoadDesc(map, nextRoadId, 3, 6, 50.0, 5.5, 1);
    addRoadDesc(map, nextRoadId, 6, 3, 50.0, 5.5, 1);

    // Two-way east–west pair on the northern corridor.
    addRoadDesc(map, nextRoadId, 6, 8, 120.0, 4.0, 1);
    addRoadDesc(map, nextRoadId, 8, 6, 120.0, 4.0, 1);

    // Non-straight polyline: geometry length must match authoritative length within tolerance.
    const Vector2D curveFrom(120.0, 90.0);
    const Vector2D curveMid(148.0, 110.0);
    const Vector2D curveTo(175.0, 95.0);
    const double curveLen =
        curveFrom.distanceTo(curveMid) + curveMid.distanceTo(curveTo);
    addRoadDesc(map, nextRoadId, 6, 7, curveLen, 4.5, 1, {curveMid});

    const Vector2D exitFrom(175.0, 95.0);
    const Vector2D exitTo(190.0, 40.0);
    addRoadDesc(map, nextRoadId, 7, 4, exitFrom.distanceTo(exitTo), 6.0, 1);

    // PriorityRoad: west through-approach has priority; sideEntry is subordinate.
    for (auto& intersection : map.intersections) {
        if (intersection.id == 2) {
            intersection.priorityRoadIds = {westApproach};
        }
    }

    MapCrossingDesc crossingMid;
    crossingMid.id = 1;
    crossingMid.roadSegmentId = midCorridor;
    crossingMid.positionOnRoad = 22.0;
    crossingMid.inactiveDuration = 6.0;
    crossingMid.activeDuration = 3.0;
    map.crossings.push_back(crossingMid);

    MapCrossingDesc crossingSouth;
    crossingSouth.id = 2;
    crossingSouth.roadSegmentId = southEntry;
    crossingSouth.positionOnRoad = 16.0;
    crossingSouth.inactiveDuration = 7.0;
    crossingSouth.activeDuration = 3.0;
    map.crossings.push_back(crossingSouth);

    MapCrossingDesc crossingEast;
    crossingEast.id = 3;
    crossingEast.roadSegmentId = eastExit;
    crossingEast.positionOnRoad = 18.0;
    crossingEast.inactiveDuration = 5.0;
    crossingEast.activeDuration = 2.5;
    map.crossings.push_back(crossingEast);

    return map;
}

void Scenario::setupDemoCity(City& city, Simulation& sim) {
    MapDescription map = buildDemoMapDescription();

    string error;
    if (!CityBuilder::build(map, city, error)) {
        city = City("Demo City (build failed)");
        return;
    }

    // Resolve road ids from the built city by topology (no magic id hardcoding in rules).
    auto findDirected = [&](int fromId, int toId) -> int {
        for (const auto& road : city.getRoadSegments()) {
            if (road.getFromIntersectionId() == fromId && road.getToIntersectionId() == toId) {
                return road.getId();
            }
        }
        return -1;
    };

    const int westApproach = findDirected(1, 2);
    const int midCorridor = findDirected(2, 3);
    const int eastExit = findDirected(3, 4);
    const int southEntry = findDirected(5, 3);
    const int sideEntry = findDirected(9, 2);
    const int northEastBound = findDirected(8, 6);
    const int curveRoad = findDirected(6, 7);
    const int curveToExit = findDirected(7, 4);

    PathFinder pathFinder;
    auto routeOrFallback = [&](int startRoadId, int endIntersectionId, const vector<int>& fallback) {
        vector<int> route = pathFinder.findRouteThroughIntersections(city, startRoadId, endIntersectionId);
        return route.empty() ? fallback : route;
    };

    VehicleGenerator generatorWest(2.2, 6.5, 14, 7.0);
    generatorWest.setSeed(VehicleGenerator::defaultSeed + 11u);
    generatorWest.setLaneCountHint(2);
    generatorWest.setRoute(routeOrFallback(westApproach, 4, {westApproach, midCorridor, eastExit}));

    VehicleGenerator generatorSouth(3.0, 5.0, 10, 5.5);
    generatorSouth.setSeed(VehicleGenerator::defaultSeed + 12u);
    generatorSouth.setRoute(routeOrFallback(southEntry, 4, {southEntry, eastExit}));

    VehicleGenerator generatorSide(3.5, 4.5, 8, 5.0);
    generatorSide.setSeed(VehicleGenerator::defaultSeed + 13u);
    generatorSide.setRoute(routeOrFallback(sideEntry, 4, {sideEntry, midCorridor, eastExit}));

    VehicleGenerator generatorNorth(3.2, 4.5, 8, 5.0);
    generatorNorth.setSeed(VehicleGenerator::defaultSeed + 14u);
    generatorNorth.setRoute(routeOrFallback(
        northEastBound, 4, {northEastBound, curveRoad, curveToExit}
    ));

    sim.addVehicleGenerator(generatorWest);
    sim.addVehicleGenerator(generatorSouth);
    sim.addVehicleGenerator(generatorSide);
    sim.addVehicleGenerator(generatorNorth);
    sim.setMinVehicleGap(4.0);

    // Signalized junction: mid corridor vs south entry.
    if (midCorridor >= 0 && southEntry >= 0) {
        addJunctionController(sim, 1, 3, midCorridor, southEntry);
    }

    for (const auto& crossingDesc : map.crossings) {
        sim.addPedestrianCrossing(PedestrianCrossing(
            crossingDesc.id,
            crossingDesc.roadSegmentId,
            crossingDesc.positionOnRoad,
            crossingDesc.inactiveDuration,
            crossingDesc.activeDuration,
            crossingDesc.startActive
        ));
    }

    // Pedestrian RNG for deterministic encounters on the demo map.
    sim.setPedestrianSeed(VehicleGenerator::defaultSeed + 50u);
    sim.setLaneBlockSeed(VehicleGenerator::defaultSeed + 60u);
}

void Scenario::setupIntersectionTest(City& city, Simulation& sim) {
    MapDescription map;
    map.name = "City Network Test";

    const double spacing = 35.0;
    int nextRoadId = 1;

    for (int col = 0; col < 4; col++) {
        MapIntersectionDesc south;
        south.id = 1 + col;
        south.position = Vector2D(col * spacing, 0.0);
        map.intersections.push_back(south);

        MapIntersectionDesc mid;
        mid.id = 5 + col;
        mid.position = Vector2D(col * spacing, spacing);
        map.intersections.push_back(mid);

        MapIntersectionDesc north;
        north.id = 9 + col;
        north.position = Vector2D(col * spacing, spacing * 2.0);
        map.intersections.push_back(north);
    }

    auto addNode = [&](int id, double x, double y) {
        MapIntersectionDesc node;
        node.id = id;
        node.position = Vector2D(x, y);
        map.intersections.push_back(node);
    };

    addNode(13, -30.0, 0.0);
    addNode(14, -30.0, spacing);
    addNode(15, -30.0, spacing * 2.0);
    addNode(16, spacing * 4.0, spacing);
    addNode(17, spacing * 1.5, -25.0);
    addNode(18, spacing * 2.5, spacing * 3.0);

    // Preserve historical road-id assignment order.
    int westLow = addRoadDesc(map, nextRoadId, 13, 1, 30.0, 5.0);
    // 2-lane approach feeds the mid corridor (Stage 2 parallel-lane demo).
    int westMid = addRoadDesc(map, nextRoadId, 14, 5, 30.0, 5.0, 2);
    int westHigh = addRoadDesc(map, nextRoadId, 15, 9, 30.0, 5.0);
    int southEntry = addRoadDesc(map, nextRoadId, 17, 2, 25.0, 4.5);
    int northEntry = addRoadDesc(map, nextRoadId, 18, 11, 25.0, 4.5);
    int eastExit = addRoadDesc(map, nextRoadId, 8, 16, 30.0, 6.0);

    int horizontalSouth[3];
    int horizontalMid[3];
    int horizontalNorth[3];

    for (int col = 0; col < 3; col++) {
        horizontalSouth[col] = addRoadDesc(map, nextRoadId, 1 + col, 2 + col, spacing, 6.0);
        // Mid E–W corridor is 2-lane (Stage 2 demo of parallel independent lanes).
        horizontalMid[col] = addRoadDesc(map, nextRoadId, 5 + col, 6 + col, spacing, 6.0, 2);
        horizontalNorth[col] = addRoadDesc(map, nextRoadId, 9 + col, 10 + col, spacing, 6.0);
    }

    int verticalLow[4];
    int verticalHigh[4];

    for (int col = 0; col < 4; col++) {
        verticalLow[col] = addRoadDesc(map, nextRoadId, 1 + col, 5 + col, spacing, 5.0);
        verticalHigh[col] = addRoadDesc(map, nextRoadId, 5 + col, 9 + col, spacing, 5.0);
    }

    // Stage 3 priority wiring (MapDescription → CityBuilder → Intersection).
    // Signalized junctions already have phase controllers below.
    for (auto& intersection : map.intersections) {
        if (intersection.id == 6 || intersection.id == 10) {
            intersection.priorityMode = PriorityMode::Signalized;
        } else if (intersection.id == 2) {
            // East–west through-road has priority; southEntry is subordinate.
            intersection.priorityMode = PriorityMode::PriorityRoad;
            intersection.priorityRoadIds = {horizontalSouth[0]};
        } else if (intersection.id == 5) {
            intersection.priorityMode = PriorityMode::YieldToRight;
        }
    }

    MapCrossingDesc crossing1;
    crossing1.id = 1;
    crossing1.roadSegmentId = horizontalMid[1];
    crossing1.positionOnRoad = 12.0;
    crossing1.inactiveDuration = 6.0;
    crossing1.activeDuration = 3.0;
    map.crossings.push_back(crossing1);

    MapCrossingDesc crossing2;
    crossing2.id = 2;
    crossing2.roadSegmentId = verticalHigh[1];
    crossing2.positionOnRoad = 10.0;
    crossing2.inactiveDuration = 7.0;
    crossing2.activeDuration = 3.0;
    map.crossings.push_back(crossing2);

    MapCrossingDesc crossing3;
    crossing3.id = 3;
    crossing3.roadSegmentId = eastExit;
    crossing3.positionOnRoad = 8.0;
    crossing3.inactiveDuration = 5.0;
    crossing3.activeDuration = 2.5;
    map.crossings.push_back(crossing3);

    string error;
    if (!CityBuilder::build(map, city, error)) {
        city = City("City Network Test (build failed)");
        return;
    }

    PathFinder pathFinder;

    auto routeOrFallback = [&](int startRoadId, int endIntersectionId, const vector<int>& fallback) {
        vector<int> route = pathFinder.findRouteThroughIntersections(city, startRoadId, endIntersectionId);
        return route.empty() ? fallback : route;
    };

    VehicleGenerator generatorWestLow(2.5, 5.0, 12, 6.0);
    generatorWestLow.setSeed(VehicleGenerator::defaultSeed + 1u);
    generatorWestLow.setRoute(routeOrFallback(westLow, 16, {westLow, horizontalSouth[0], horizontalSouth[1], horizontalSouth[2], eastExit}));

    VehicleGenerator generatorWestMid(2.0, 4.5, 15, 6.0);
    generatorWestMid.setSeed(VehicleGenerator::defaultSeed + 2u);
    generatorWestMid.setLaneCountHint(2); // first road is 1-lane; mid corridor is 2-lane after transition
    generatorWestMid.setRoute(routeOrFallback(westMid, 16, {westMid, horizontalMid[0], horizontalMid[1], eastExit}));

    VehicleGenerator generatorWestHigh(3.0, 4.0, 10, 6.0);
    generatorWestHigh.setSeed(VehicleGenerator::defaultSeed + 3u);
    generatorWestHigh.setRoute(routeOrFallback(westHigh, 16, {westHigh, horizontalNorth[0], horizontalNorth[1], eastExit}));

    VehicleGenerator generatorSouth(3.5, 4.0, 10, 5.0);
    generatorSouth.setSeed(VehicleGenerator::defaultSeed + 4u);
    generatorSouth.setRoute(routeOrFallback(southEntry, 16, {southEntry, verticalLow[1], verticalHigh[1], horizontalNorth[1], eastExit}));

    VehicleGenerator generatorNorth(4.0, 3.5, 8, 5.0);
    generatorNorth.setSeed(VehicleGenerator::defaultSeed + 5u);
    generatorNorth.setRoute(routeOrFallback(northEntry, 16, {northEntry, verticalHigh[2], horizontalMid[2], eastExit}));

    sim.addVehicleGenerator(generatorWestLow);
    sim.addVehicleGenerator(generatorWestMid);
    sim.addVehicleGenerator(generatorWestHigh);
    sim.addVehicleGenerator(generatorSouth);
    sim.addVehicleGenerator(generatorNorth);
    sim.setMinVehicleGap(4.0);

    addJunctionController(sim, 1, 6, horizontalMid[0], verticalLow[1]);
    addJunctionController(sim, 2, 10, horizontalNorth[0], verticalHigh[1]);

    for (const auto& crossingDesc : map.crossings) {
        sim.addPedestrianCrossing(PedestrianCrossing(
            crossingDesc.id,
            crossingDesc.roadSegmentId,
            crossingDesc.positionOnRoad,
            crossingDesc.inactiveDuration,
            crossingDesc.activeDuration,
            crossingDesc.startActive
        ));
    }
}
