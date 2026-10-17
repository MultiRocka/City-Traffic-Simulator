#include "core/City.h"
#include "core/CityBuilder.h"
#include "core/MapDescription.h"
#include "core/RoadSegment.h"
#include "core/Vector2D.h"

#include <cmath>
#include <iostream>
#include <string>

using namespace std;

namespace {

int g_failures = 0;
int g_passed = 0;

void expect(bool condition, const string& name) {
    if (condition) {
        ++g_passed;
        cout << "PASS  " << name << "\n";
    } else {
        ++g_failures;
        cout << "FAIL  " << name << "\n";
    }
}

MapDescription makeValidTwoWayMap() {
    MapDescription map;
    map.name = "Synthetic Two-Way";

    MapIntersectionDesc a;
    a.id = 1;
    a.position = Vector2D(0.0, 0.0);
    map.intersections.push_back(a);

    MapIntersectionDesc b;
    b.id = 2;
    b.position = Vector2D(100.0, 0.0);
    b.priorityMode = PriorityMode::PriorityRoad;
    b.priorityRoadIds = {10, 11};
    map.intersections.push_back(b);

    MapRoadDesc ab;
    ab.id = 10;
    ab.fromIntersectionId = 1;
    ab.toIntersectionId = 2;
    ab.length = 100.0;
    ab.speedLimit = 13.9; // m/s
    ab.intermediatePoints = {Vector2D(50.0, 0.0)};
    ab.sourceRef = "way/100";
    map.roads.push_back(ab);

    MapRoadDesc ba;
    ba.id = 11;
    ba.fromIntersectionId = 2;
    ba.toIntersectionId = 1;
    ba.length = 100.0;
    ba.speedLimit = 13.9;
    ba.laneCount = 2;
    map.roads.push_back(ba);

    MapCrossingDesc crossing;
    crossing.id = 1;
    crossing.roadSegmentId = 10;
    crossing.positionOnRoad = 40.0;
    map.crossings.push_back(crossing);

    return map;
}

void testValidSyntheticMapBuilds() {
    MapDescription map = makeValidTwoWayMap();
    City city;
    string error;

    expect(CityBuilder::build(map, city, error), "map_valid_builds");
    expect(error.empty(), "map_valid_no_error");
    expect(city.getRoadSegments().size() == 2, "map_valid_two_roads");
    expect(city.getIntersections().size() == 2, "map_valid_two_nodes");

    const RoadSegment* ab = city.findRoadSegmentById(10);
    const RoadSegment* ba = city.findRoadSegmentById(11);
    expect(ab != nullptr && ba != nullptr, "map_valid_roads_found");
    expect(ab != nullptr && ab->getLaneCount() == 1, "map_valid_default_lane_count");
    expect(ba != nullptr && ba->getLaneCount() == 2, "map_valid_explicit_lane_count");
    expect(ab != nullptr && ab->getSourceRef() == "way/100", "map_valid_source_ref_set");
    expect(ba != nullptr && ba->getSourceRef().empty(), "map_valid_empty_source_is_synthetic");
    expect(ab != nullptr && ab->getIntermediatePoints().size() == 1, "map_valid_polyline_stored");

    const Intersection* b = city.findIntersectionById(2);
    expect(b != nullptr && b->getPriorityMode() == PriorityMode::PriorityRoad, "map_valid_priority_mode_stored");
    expect(b != nullptr && b->getPriorityRoadIds().size() == 2, "map_valid_priority_roads_stored");
}

void testRejectsMissingIntersection() {
    MapDescription map = makeValidTwoWayMap();
    map.roads[0].toIntersectionId = 99;
    City city;
    string error;
    expect(!CityBuilder::build(map, city, error), "map_reject_missing_intersection");
    expect(!error.empty(), "map_reject_missing_intersection_message");
}

void testRejectsBadLength() {
    MapDescription map = makeValidTwoWayMap();
    map.roads[0].length = 0.0;
    City city;
    string error;
    expect(!CityBuilder::build(map, city, error), "map_reject_bad_length");
}

void testRejectsBadLaneCount() {
    MapDescription map = makeValidTwoWayMap();
    map.roads[0].laneCount = 0;
    City city;
    string error;
    expect(!CityBuilder::build(map, city, error), "map_reject_bad_lane_count");
}

void testRejectsBadCrossingPos() {
    MapDescription map = makeValidTwoWayMap();
    map.crossings[0].positionOnRoad = 150.0;
    City city;
    string error;
    expect(!CityBuilder::build(map, city, error), "map_reject_bad_crossing_pos");
}

void testRejectsInvalidConnection() {
    MapDescription map = makeValidTwoWayMap();
    MapConnectionDesc bad;
    bad.fromRoadId = 10;
    bad.toRoadId = 10;
    bad.allowed = false;
    map.connections.push_back(bad);

    City city;
    string error;
    expect(!CityBuilder::build(map, city, error), "map_reject_invalid_connection");
}

void testRejectsMissingConnectionRoad() {
    MapDescription map = makeValidTwoWayMap();
    MapConnectionDesc bad;
    bad.fromRoadId = 10;
    bad.toRoadId = 999;
    bad.allowed = false;
    map.connections.push_back(bad);

    City city;
    string error;
    expect(!CityBuilder::build(map, city, error), "map_reject_missing_connection_road");
}

void testDirectedRoadsAreSeparate() {
    MapDescription map = makeValidTwoWayMap();
    City city;
    string error;
    expect(CityBuilder::build(map, city, error), "map_directed_builds");

    const RoadSegment* ab = city.findRoadSegmentById(10);
    const RoadSegment* ba = city.findRoadSegmentById(11);
    expect(ab != nullptr && ba != nullptr, "map_directed_both_exist");
    expect(ab->getId() != ba->getId(), "map_directed_separate_ids");
    expect(ab->getFromIntersectionId() == ba->getToIntersectionId(), "map_directed_opposite_geometry");
    expect(ab->getToIntersectionId() == ba->getFromIntersectionId(), "map_directed_opposite_geometry_to");
    // Opposite direction is a separate RoadSegment — lane counts are per direction, not shared.
    expect(ab->getLaneCount() == 1 && ba->getLaneCount() == 2, "map_directed_lanes_per_direction");
    expect(!city.areRoadsConnected(10, 10), "map_directed_no_u_turn_same_segment");
}

void testPolylineStoredLengthAuthoritative() {
    MapDescription map;
    map.name = "Polyline";

    MapIntersectionDesc a;
    a.id = 1;
    a.position = Vector2D(0.0, 0.0);
    map.intersections.push_back(a);

    MapIntersectionDesc b;
    b.id = 2;
    b.position = Vector2D(60.0, 0.0);
    map.intersections.push_back(b);

    MapRoadDesc road;
    road.id = 1;
    road.fromIntersectionId = 1;
    road.toIntersectionId = 2;
    road.length = 80.0; // authoritative (meters); longer than straight chord
    road.speedLimit = 10.0;
    road.intermediatePoints = {Vector2D(30.0, 20.0)}; // polyline ≈ 72.11, within default tolerance? 80-72=8 > 1
    map.roads.push_back(road);

    City city;
    string error;
    expect(!CityBuilder::build(map, city, error), "map_polyline_mismatch_rejected");

    // Explicit length matching polyline geometry is accepted and stored.
    const double polyLen = Vector2D(0.0, 0.0).distanceTo(Vector2D(30.0, 20.0))
        + Vector2D(30.0, 20.0).distanceTo(Vector2D(60.0, 0.0));
    map.roads[0].length = polyLen;
    expect(CityBuilder::build(map, city, error), "map_polyline_matching_length_ok");
    const RoadSegment* built = city.findRoadSegmentById(1);
    expect(built != nullptr, "map_polyline_road_present");
    expect(built != nullptr && built->getIntermediatePoints().size() == 1, "map_polyline_geometry_kept");
    expect(built != nullptr && fabs(built->getLength() - polyLen) < 1e-9, "map_polyline_length_from_data");
}

void testDisallowedTurnStored() {
    MapDescription map;
    map.name = "Turn Ban";

    MapIntersectionDesc a;
    a.id = 1;
    a.position = Vector2D(0.0, 0.0);
    map.intersections.push_back(a);
    MapIntersectionDesc b;
    b.id = 2;
    b.position = Vector2D(50.0, 0.0);
    map.intersections.push_back(b);
    MapIntersectionDesc c;
    c.id = 3;
    c.position = Vector2D(50.0, 50.0);
    map.intersections.push_back(c);

    MapRoadDesc r1;
    r1.id = 1;
    r1.fromIntersectionId = 1;
    r1.toIntersectionId = 2;
    r1.length = 50.0;
    r1.speedLimit = 5.0;
    map.roads.push_back(r1);

    MapRoadDesc r2;
    r2.id = 2;
    r2.fromIntersectionId = 2;
    r2.toIntersectionId = 3;
    r2.length = 50.0;
    r2.speedLimit = 5.0;
    map.roads.push_back(r2);

    MapConnectionDesc ban;
    ban.fromRoadId = 1;
    ban.toRoadId = 2;
    ban.allowed = false;
    map.connections.push_back(ban);

    City city;
    string error;
    expect(CityBuilder::build(map, city, error), "map_turn_ban_builds");
    expect(city.areRoadsGeometricallyConnected(1, 2), "map_turn_ban_still_geometric");
    expect(!city.areRoadsConnected(1, 2), "map_turn_ban_not_allowed");
    expect(city.getOutgoingRoadIds(1).empty(), "map_turn_ban_no_outgoing");
}

} // namespace

int runMapModelTests() {
    cout << "\n--- Map model tests ---\n";
    testValidSyntheticMapBuilds();
    testRejectsMissingIntersection();
    testRejectsBadLength();
    testRejectsBadLaneCount();
    testRejectsBadCrossingPos();
    testRejectsInvalidConnection();
    testRejectsMissingConnectionRoad();
    testDirectedRoadsAreSeparate();
    testPolylineStoredLengthAuthoritative();
    testDisallowedTurnStored();
    return g_failures;
}

int mapModelPassedCount() {
    return g_passed;
}
