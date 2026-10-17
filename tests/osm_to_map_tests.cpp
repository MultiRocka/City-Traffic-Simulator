#include "core/City.h"
#include "core/CityBuilder.h"
#include "core/MapDescription.h"
#include "core/PathFinder.h"
#include "osm/OsmJsonParser.h"
#include "osm/OsmToMapDescription.h"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>
#include <unordered_set>

using namespace std;
namespace fs = std::filesystem;

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

string fixturePath(const string& name) {
    return string("data/osm/fixtures/") + name;
}

bool reportHas(const vector<string>& list, const string& needle) {
    for (const string& item : list) {
        if (item.find(needle) != string::npos) {
            return true;
        }
    }
    return false;
}

const MapRoadDesc* findRoadByDir(const MapDescription& map, const string& dirToken) {
    for (const MapRoadDesc& road : map.roads) {
        if (road.sourceRef.find(dirToken) != string::npos) {
            return &road;
        }
    }
    return nullptr;
}

void testProjectionEquirectangular() {
    const OsmLocalProjection projection = OsmLocalProjection::fromLatLon(52.225, 21.016);
    const Vector2D origin = projection.project(52.225, 21.016);
    expect(fabs(origin.x) < 1e-9 && fabs(origin.y) < 1e-9, "osm3_proj_origin_zero");

    const Vector2D east = projection.project(52.225, 21.017);
    const Vector2D north = projection.project(52.226, 21.016);
    expect(east.x > 50.0 && fabs(east.y) < 1.0, "osm3_proj_east_positive_x");
    expect(north.y > 50.0 && fabs(north.x) < 1.0, "osm3_proj_north_positive_y");
}

void testOnewayForward() {
    const OsmToMapResult result = OsmToMapDescription::convertFile(fixturePath("oneway.overpass.json"));
    expect(result.ok, "osm3_oneway_ok");
    expect(result.map.roads.size() == 1, "osm3_oneway_one_directed_road");
    if (!result.map.roads.empty()) {
        expect(result.map.roads.front().sourceRef.find("dir=fwd") != string::npos,
               "osm3_oneway_fwd_ref");
        expect(fabs(result.map.roads.front().speedLimit - (50.0 * 1000.0 / 3600.0)) < 1e-6,
               "osm3_oneway_speed_50kmh");
    } else {
        expect(false, "osm3_oneway_fwd_ref");
        expect(false, "osm3_oneway_speed_50kmh");
    }
    City city;
    string error;
    expect(CityBuilder::build(result.map, city, error), "osm3_oneway_citybuilder");
}

void testOnewayReverse() {
    const OsmToMapResult result =
        OsmToMapDescription::convertFile(fixturePath("oneway_reverse.overpass.json"));
    expect(result.ok, "osm3_oneway_rev_ok");
    expect(result.map.roads.size() == 1, "osm3_oneway_rev_one_road");
    if (!result.map.roads.empty()) {
        expect(result.map.roads.front().sourceRef.find("dir=rev") != string::npos,
               "osm3_oneway_rev_ref");
    } else {
        expect(false, "osm3_oneway_rev_ref");
    }
}

void testDirectionalLanes() {
    const OsmToMapResult result =
        OsmToMapDescription::convertFile(fixturePath("directional_lanes.overpass.json"));
    expect(result.ok, "osm3_dir_lanes_ok");
    expect(result.map.roads.size() == 2, "osm3_dir_lanes_two_roads");
    const MapRoadDesc* fwd = findRoadByDir(result.map, "dir=fwd");
    const MapRoadDesc* rev = findRoadByDir(result.map, "dir=rev");
    expect(fwd != nullptr && fwd->laneCount == 2, "osm3_dir_lanes_fwd_2");
    expect(rev != nullptr && rev->laneCount == 1, "osm3_dir_lanes_rev_1");
    expect(reportHas(result.report.readFromTags, "lanes:forward"), "osm3_dir_lanes_read_fwd");
    City city;
    string error;
    expect(CityBuilder::build(result.map, city, error), "osm3_dir_lanes_citybuilder");
}

void testSharedJunctionConnected() {
    const OsmToMapResult result =
        OsmToMapDescription::convertFile(fixturePath("shared_junction.overpass.json"));
    expect(result.ok, "osm3_shared_ok");
    expect(result.report.graphComponentCount == 1, "osm3_shared_one_component");
    // East-West splits at shared node → 2 segments × 2 dirs + spur × 2 dirs = 6 roads
    expect(result.map.roads.size() == 6, "osm3_shared_six_directed_roads");
    expect(result.map.intersections.size() == 4, "osm3_shared_four_intersections");

    unordered_set<int> touched;
    for (const MapRoadDesc& road : result.map.roads) {
        touched.insert(road.fromIntersectionId);
        touched.insert(road.toIntersectionId);
    }
    expect(touched.size() == result.map.intersections.size(), "osm3_shared_all_nodes_linked");

    City city;
    string error;
    expect(CityBuilder::build(result.map, city, error), "osm3_shared_citybuilder");
}

void testCrossingGeometriesNotConnected() {
    const OsmToMapResult result =
        OsmToMapDescription::convertFile(fixturePath("crossing_no_shared_node.overpass.json"));
    expect(result.ok, "osm3_cross_geom_ok");
    expect(result.report.graphComponentCount == 2, "osm3_cross_geom_two_components");
    expect(result.map.intersections.size() == 4, "osm3_cross_geom_four_nodes");
    // No shared intersection between the two ways.
    unordered_set<int> componentSeeds;
    // Verify no road connects an endpoint of way A to way B by checking adjacency count.
    expect(result.map.roads.size() == 4, "osm3_cross_geom_four_roads");
}

void testCurvedPolylineLength() {
    const OsmToMapResult result =
        OsmToMapDescription::convertFile(fixturePath("curved_polyline.overpass.json"));
    expect(result.ok, "osm3_curve_ok");
    expect(result.map.roads.size() == 1, "osm3_curve_one_road");
    expect(result.map.intersections.size() == 2, "osm3_curve_two_intersections_only");
    if (!result.map.roads.empty()) {
        const MapRoadDesc& road = result.map.roads.front();
        expect(road.intermediatePoints.size() == 2, "osm3_curve_two_intermediates");

        const MapIntersectionDesc* from = nullptr;
        const MapIntersectionDesc* to = nullptr;
        for (const MapIntersectionDesc& node : result.map.intersections) {
            if (node.id == road.fromIntersectionId) {
                from = &node;
            }
            if (node.id == road.toIntersectionId) {
                to = &node;
            }
        }
        expect(from != nullptr && to != nullptr, "osm3_curve_endpoints_found");
        if (from != nullptr && to != nullptr) {
            vector<Vector2D> poly;
            poly.push_back(from->position);
            poly.insert(poly.end(), road.intermediatePoints.begin(), road.intermediatePoints.end());
            poly.push_back(to->position);
            const double summed = OsmToMapDescription::polylineLengthMeters(poly);
            expect(fabs(summed - road.length) < 1e-6, "osm3_curve_length_matches_sum");
        } else {
            expect(false, "osm3_curve_length_matches_sum");
        }
    } else {
        expect(false, "osm3_curve_two_intermediates");
        expect(false, "osm3_curve_endpoints_found");
        expect(false, "osm3_curve_length_matches_sum");
    }
    City city;
    string error;
    expect(CityBuilder::build(result.map, city, error), "osm3_curve_citybuilder");
}

void testMaxspeedVariants() {
    const OsmToMapResult result =
        OsmToMapDescription::convertFile(fixturePath("maxspeed_variants.overpass.json"));
    expect(result.ok, "osm3_maxspeed_ok");
    expect(result.map.roads.size() == 3, "osm3_maxspeed_three_roads");

    bool found50 = false;
    bool foundMph = false;
    bool foundDefault = false;
    for (const MapRoadDesc& road : result.map.roads) {
        if (fabs(road.speedLimit - (50.0 * 1000.0 / 3600.0)) < 1e-6) {
            found50 = true;
        }
        if (fabs(road.speedLimit - (30.0 * 1609.344 / 3600.0)) < 1e-6) {
            foundMph = true;
        }
        if (fabs(road.speedLimit - (50.0 * 1000.0 / 3600.0)) < 1e-6 &&
            road.sourceRef.find("850003") != string::npos) {
            foundDefault = true;
        }
    }
    // walk way uses default 50 km/h
    for (const MapRoadDesc& road : result.map.roads) {
        if (road.sourceRef.find("850003") != string::npos) {
            foundDefault = fabs(road.speedLimit - (50.0 * 1000.0 / 3600.0)) < 1e-6;
        }
    }
    expect(found50, "osm3_maxspeed_bare_kmh");
    expect(foundMph, "osm3_maxspeed_mph");
    expect(foundDefault, "osm3_maxspeed_walk_default");
    expect(reportHas(result.report.unsupported, "walk"), "osm3_maxspeed_walk_unsupported");
}

void testCrossingAndSignals() {
    const OsmToMapResult result =
        OsmToMapDescription::convertFile(fixturePath("crossing_and_signals.overpass.json"));
    expect(result.ok, "osm3_xing_ok");
    expect(result.map.crossings.size() >= 1, "osm3_xing_has_crossing");
    expect(result.report.signalizedIntersectionCount >= 1, "osm3_xing_has_signal");

    bool signalized = false;
    for (const MapIntersectionDesc& node : result.map.intersections) {
        if (node.priorityMode == PriorityMode::Signalized) {
            signalized = true;
        }
    }
    expect(signalized, "osm3_xing_signal_mode");

    if (!result.map.crossings.empty() && !result.map.roads.empty()) {
        const MapCrossingDesc& crossing = result.map.crossings.front();
        const MapRoadDesc* road = nullptr;
        for (const MapRoadDesc& candidate : result.map.roads) {
            if (candidate.id == crossing.roadSegmentId) {
                road = &candidate;
                break;
            }
        }
        expect(road != nullptr, "osm3_xing_road_found");
        if (road != nullptr) {
            expect(crossing.positionOnRoad > 0.0 && crossing.positionOnRoad < road->length,
                   "osm3_xing_mid_segment");
        } else {
            expect(false, "osm3_xing_mid_segment");
        }
    } else {
        expect(false, "osm3_xing_road_found");
        expect(false, "osm3_xing_mid_segment");
    }

    City city;
    string error;
    expect(CityBuilder::build(result.map, city, error), "osm3_xing_citybuilder");
}

void testMissingNodes() {
    const OsmToMapResult result =
        OsmToMapDescription::convertFile(fixturePath("missing_node_ref.overpass.json"));
    expect(result.ok, "osm3_missing_ok");
    expect(!result.report.missingNodeRefs.empty(), "osm3_missing_refs_reported");
    expect(reportHas(result.report.skipped, "missing node ref"), "osm3_missing_skipped_note");
    // Way has only one present node → no road segment.
    expect(result.map.roads.empty(), "osm3_missing_no_roads");
}

void testDisconnectedComponents() {
    const OsmToMapResult result =
        OsmToMapDescription::convertFile(fixturePath("disconnected_components.overpass.json"));
    expect(result.ok, "osm3_disco_ok");
    expect(result.report.graphComponentCount == 2, "osm3_disco_two_components");
    expect(result.report.graphComponentsNote.find("2") != string::npos, "osm3_disco_note");
    expect(result.report.incompletenessNote.find("disconnected") != string::npos ||
               result.report.incompletenessNote.find("Multiple") != string::npos,
           "osm3_disco_incompleteness");
}

void testIncompleteRestrictionWarned() {
    const OsmToMapResult result =
        OsmToMapDescription::convertFile(fixturePath("with_relation.overpass.json"));
    expect(result.ok, "osm3_rel_ok");
    expect(!result.report.unsupportedRelationIds.empty(), "osm3_rel_ids");
    expect(reportHas(result.report.unsupported, "relation"), "osm3_rel_unsupported");
    expect(result.map.connections.empty(), "osm3_rel_no_connections");
}

void testPrivateAccessExcluded() {
    const OsmToMapResult result =
        OsmToMapDescription::convertFile(fixturePath("unsupported_access.overpass.json"));
    expect(result.ok, "osm3_access_ok");
    expect(result.map.roads.empty(), "osm3_access_private_excluded");
    expect(reportHas(result.report.skipped, "private") ||
               reportHas(result.report.skipped, "access"),
           "osm3_access_skipped_note");
    expect(reportHas(result.report.unsupported, "turn:lanes"), "osm3_turnlanes_unsupported");
}

int findRoadIdByWay(const MapDescription& map, int64_t wayId, const string& dir = "fwd") {
    const string needle = "way/" + to_string(wayId);
    for (const MapRoadDesc& road : map.roads) {
        if (road.sourceRef.find(needle) != string::npos &&
            road.sourceRef.find(string(";dir=") + dir) != string::npos) {
            return road.id;
        }
    }
    return -1;
}

bool hasDisallow(const MapDescription& map, int fromId, int toId) {
    for (const MapConnectionDesc& c : map.connections) {
        if (c.fromRoadId == fromId && c.toRoadId == toId && !c.allowed) {
            return true;
        }
    }
    return false;
}

void testNoLeftTurnEnforced() {
    const OsmToMapResult result =
        OsmToMapDescription::convertFile(fixturePath("restriction_no_left.overpass.json"));
    expect(result.ok, "osm3_noleft_ok");
    expect(reportHas(result.report.readFromTags, "no_left_turn"), "osm3_noleft_read");
    expect(!result.map.connections.empty(), "osm3_noleft_has_disallow");

    const int fromId = findRoadIdByWay(result.map, 810001);
    const int toEast = findRoadIdByWay(result.map, 810003);
    const int toNorth = findRoadIdByWay(result.map, 810002);
    expect(fromId >= 0 && toEast >= 0 && toNorth >= 0, "osm3_noleft_roads_found");
    expect(hasDisallow(result.map, fromId, toEast), "osm3_noleft_disallows_east");
    expect(!hasDisallow(result.map, fromId, toNorth), "osm3_noleft_allows_north");

    City city;
    string error;
    expect(CityBuilder::build(result.map, city, error), "osm3_noleft_city");
    expect(!city.areRoadsConnected(fromId, toEast), "osm3_noleft_city_blocks");
    expect(city.areRoadsConnected(fromId, toNorth), "osm3_noleft_city_allows_straight");

    PathFinder pf;
    const vector<int> blocked = pf.findShortestRoute(city, fromId, toEast);
    expect(blocked.empty(), "osm3_noleft_pathfinder_no_route");
    const vector<int> ok = pf.findShortestRoute(city, fromId, toNorth);
    expect(!ok.empty(), "osm3_noleft_pathfinder_straight_ok");
}

void testOnlyStraightEnforced() {
    const OsmToMapResult result =
        OsmToMapDescription::convertFile(fixturePath("restriction_only_straight.overpass.json"));
    expect(result.ok, "osm3_only_ok");
    expect(reportHas(result.report.readFromTags, "only_straight_on"), "osm3_only_read");

    const int fromId = findRoadIdByWay(result.map, 811001);
    const int toNorth = findRoadIdByWay(result.map, 811002);
    const int toEast = findRoadIdByWay(result.map, 811003);
    const int toWest = findRoadIdByWay(result.map, 811004);
    expect(fromId >= 0 && toNorth >= 0 && toEast >= 0 && toWest >= 0, "osm3_only_roads");
    expect(!hasDisallow(result.map, fromId, toNorth), "osm3_only_allows_north");
    expect(hasDisallow(result.map, fromId, toEast), "osm3_only_blocks_east");
    expect(hasDisallow(result.map, fromId, toWest), "osm3_only_blocks_west");

    City city;
    string error;
    expect(CityBuilder::build(result.map, city, error), "osm3_only_city");
    expect(city.areRoadsConnected(fromId, toNorth), "osm3_only_city_straight");
    expect(!city.areRoadsConnected(fromId, toEast), "osm3_only_city_no_east");
}

void testRestrictionSurvivesWaySplit() {
    const OsmToMapResult result =
        OsmToMapDescription::convertFile(fixturePath("restriction_split_way.overpass.json"));
    expect(result.ok, "osm3_split_ok");
    expect(reportHas(result.report.readFromTags, "no_right_turn"), "osm3_split_read");

    // Long from-way 812001 splits; arriving segment at via still carries the ban.
    int arriving = -1;
    for (const MapRoadDesc& road : result.map.roads) {
        if (road.sourceRef.find("way/812001") != string::npos &&
            road.sourceRef.find("dir=fwd") != string::npos) {
            // Prefer the segment whose to-intersection is the via (node/912003).
            for (const MapIntersectionDesc& node : result.map.intersections) {
                if (node.sourceRef == "node/912003" && road.toIntersectionId == node.id) {
                    arriving = road.id;
                }
            }
        }
    }
    const int forbiddenTo = findRoadIdByWay(result.map, 812003);
    const int allowedTo = findRoadIdByWay(result.map, 812004);
    expect(arriving >= 0 && forbiddenTo >= 0 && allowedTo >= 0, "osm3_split_roads");
    expect(hasDisallow(result.map, arriving, forbiddenTo), "osm3_split_disallows");
    expect(!hasDisallow(result.map, arriving, allowedTo), "osm3_split_allows_other");
}

void testViaWayRestrictionUnsupported() {
    const OsmToMapResult result =
        OsmToMapDescription::convertFile(fixturePath("restriction_via_way_unsupported.overpass.json"));
    expect(result.ok, "osm3_viaway_ok");
    expect(result.map.connections.empty(), "osm3_viaway_no_enforce");
    expect(reportHas(result.report.unsupported, "via-way") ||
               reportHas(result.report.unsupported, "713001"),
           "osm3_viaway_unsupported");
}

void testDestinationAccessFlags() {
    const OsmToMapResult result =
        OsmToMapDescription::convertFile(fixturePath("access_destination.overpass.json"));
    expect(result.ok, "osm3_dest_ok");
    expect(result.map.roads.size() >= 3, "osm3_dest_roads_built");
    bool foundSpur = false;
    bool spurNoDemand = false;
    for (const MapRoadDesc& road : result.map.roads) {
        if (road.sourceRef.find("way/814003") != string::npos) {
            foundSpur = true;
            spurNoDemand = !road.allowAutoDemand;
        }
    }
    expect(foundSpur, "osm3_dest_spur_present");
    expect(spurNoDemand, "osm3_dest_spur_no_autodemand");
    expect(reportHas(result.report.readFromTags, "destination"), "osm3_dest_read");
}

void testStopDirectionBindsApproach() {
    const OsmToMapResult result =
        OsmToMapDescription::convertFile(fixturePath("stop_direction_approach.overpass.json"));
    expect(result.ok, "osm3_stopdir_ok");

    const MapIntersectionDesc* via = nullptr;
    for (const MapIntersectionDesc& node : result.map.intersections) {
        if (node.sourceRef == "node/915002") {
            via = &node;
            break;
        }
    }
    expect(via != nullptr, "osm3_stopdir_via");
    if (via == nullptr) {
        return;
    }
    expect(via->priorityMode == PriorityMode::PriorityRoad, "osm3_stopdir_priority_mode");
    expect(!via->priorityRoadIds.empty(), "osm3_stopdir_has_priority_ids");
    expect(reportHas(result.report.readFromTags, "direction=forward"), "osm3_stopdir_read");

    // Stop approach (north→via) must NOT be listed as priority; west/east should be.
    int stopApproach = -1;
    for (const MapRoadDesc& road : result.map.roads) {
        if (road.sourceRef.find("way/815001") != string::npos &&
            road.toIntersectionId == via->id) {
            stopApproach = road.id;
        }
    }
    expect(stopApproach >= 0, "osm3_stopdir_stop_approach");
    bool stopInPriority = false;
    for (int id : via->priorityRoadIds) {
        if (id == stopApproach) {
            stopInPriority = true;
        }
    }
    expect(!stopInPriority, "osm3_stopdir_stop_not_priority");
}

void testRoundaboutInferredOneway() {
    const OsmToMapResult result =
        OsmToMapDescription::convertFile(fixturePath("roundabout.overpass.json"));
    expect(result.ok, "osm3_round_ok");
    expect(result.map.roads.size() == 1, "osm3_round_one_road");
    expect(reportHas(result.report.inferred, "roundabout"), "osm3_round_inferred");
}

void testBadFilesFailAtParseLayer() {
    const OsmToMapResult empty =
        OsmToMapDescription::convertFile(fixturePath("empty.overpass.json"));
    expect(!empty.ok, "osm3_empty_fails_parse");
    expect(!empty.error.empty(), "osm3_empty_has_error");
    expect(empty.map.roads.empty(), "osm3_empty_no_map");

    const OsmToMapResult bad =
        OsmToMapDescription::convertFile(fixturePath("invalid_not_json.txt"));
    expect(!bad.ok, "osm3_badjson_fails_parse");
    expect(!bad.error.empty(), "osm3_badjson_has_error");
}

void testTwowayDefaultLaneSplit() {
    const OsmToMapResult result =
        OsmToMapDescription::convertFile(fixturePath("twoway_stub.overpass.json"));
    expect(result.ok, "osm3_twoway_ok");
    expect(result.map.roads.size() == 2, "osm3_twoway_two_roads");
    // lanes=2 bidirectional → 1 each (inferred split)
    bool bothOne = true;
    for (const MapRoadDesc& road : result.map.roads) {
        if (road.laneCount != 1) {
            bothOne = false;
        }
    }
    expect(bothOne, "osm3_twoway_lanes_split_1_1");
}

void testWarsawSmokeIfPresent() {
    const string path = "data/osm/warsaw_sample.overpass.json";
    if (!fs::exists(path)) {
        cout << "SKIP  osm3_warsaw_map_smoke (sample file missing)\n";
        return;
    }
    const OsmToMapResult result = OsmToMapDescription::convertFile(path);
    expect(result.ok, "osm3_warsaw_ok");
    expect(result.map.roads.size() > 0, "osm3_warsaw_roads_gt0");
    expect(result.map.intersections.size() > 0, "osm3_warsaw_intersections_gt0");
    expect(result.report.builtRoadCount == result.map.roads.size(), "osm3_warsaw_report_roads");
    expect(result.report.graphComponentCount >= 1, "osm3_warsaw_components");

    City city;
    string error;
    const bool built = CityBuilder::build(result.map, city, error);
    expect(built, "osm3_warsaw_citybuilder");
    if (!built) {
        cout << "      CityBuilder error: " << error << "\n";
    } else {
        cout << "      Warsaw smoke: roads=" << result.map.roads.size()
             << " intersections=" << result.map.intersections.size()
             << " crossings=" << result.map.crossings.size()
             << " components=" << result.report.graphComponentCount
             << " lanes=" << result.report.builtLaneCount << "\n";
    }
}

} // namespace

int runOsmToMapTests() {
    cout << "\n--- OSM → MapDescription tests (Stage 3) ---\n";
    testProjectionEquirectangular();
    testOnewayForward();
    testOnewayReverse();
    testDirectionalLanes();
    testSharedJunctionConnected();
    testCrossingGeometriesNotConnected();
    testCurvedPolylineLength();
    testMaxspeedVariants();
    testCrossingAndSignals();
    testMissingNodes();
    testDisconnectedComponents();
    testIncompleteRestrictionWarned();
    testPrivateAccessExcluded();
    testNoLeftTurnEnforced();
    testOnlyStraightEnforced();
    testRestrictionSurvivesWaySplit();
    testViaWayRestrictionUnsupported();
    testDestinationAccessFlags();
    testStopDirectionBindsApproach();
    testRoundaboutInferredOneway();
    testBadFilesFailAtParseLayer();
    testTwowayDefaultLaneSplit();
    testWarsawSmokeIfPresent();
    return g_failures;
}

int osmToMapPassedCount() {
    return g_passed;
}
