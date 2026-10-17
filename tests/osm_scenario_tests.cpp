#include "core/City.h"
#include "core/MapDescription.h"
#include "core/PathFinder.h"
#include "core/Scenario.h"
#include "core/Simulation.h"
#include "core/VehicleGenerator.h"
#include "osm/ImportReport.h"
#include "osm/OsmScenarioBuilder.h"
#include "osm/OsmToMapDescription.h"
#include "CredibleFreeze.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

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

void ensureOutputDir() {
    filesystem::create_directories("tests/output");
}

void redirectSimOutputs(Simulation& sim, const string& tag) {
    ensureOutputDir();
    const string base = "tests/output/" + tag;
    sim.setEventLogOutputFile(base + "_events.json");
    sim.setStatisticsOutputFiles(base + "_timeline.csv", base + "_summary.csv");
}

string fixturePath(const string& name) {
    return string("data/osm/fixtures/") + name;
}

int countGenerated(const Simulation& sim) {
    int total = 0;
    for (const auto& generator : sim.getVehicleGenerators()) {
        total += generator.getGeneratedCount();
    }
    return total;
}

int countInvalidRouteEvents(const Simulation& sim) {
    int count = 0;
    set<int> seen;
    for (const auto& event : sim.getEventLog().getEvents()) {
        if (event.reason == "route_invalid" && seen.insert(event.vehicleId).second) {
            ++count;
        }
    }
    return count;
}

void scanGapAndSpeed(const City& city, const Simulation& sim, int& speedViolations, int& gapViolations) {
    for (const auto& road : city.getRoadSegments()) {
        vector<const Vehicle*> onRoad;
        for (const auto& vehicle : sim.getVehicles()) {
            if (!vehicle.isFinished() && vehicle.getCurrentRoadSegmentId() == road.getId()) {
                onRoad.push_back(&vehicle);
                if (vehicle.getSpeed() > road.getSpeedLimit() + 0.15) {
                    ++speedViolations;
                }
            }
        }

        for (int lane = 0; lane < road.getLaneCount(); ++lane) {
            vector<const Vehicle*> onLane;
            for (const Vehicle* vehicle : onRoad) {
                if (vehicle->getLaneIndex() == lane) {
                    onLane.push_back(vehicle);
                }
            }
            sort(onLane.begin(), onLane.end(), [](const Vehicle* a, const Vehicle* b) {
                return a->getPositionOnRoad() < b->getPositionOnRoad();
            });
            for (size_t i = 0; i + 1 < onLane.size(); ++i) {
                const double gap =
                    onLane[i + 1]->getPositionOnRoad() - onLane[i]->getPositionOnRoad();
                const double minBodyGap = onLane[i + 1]->getLength() + 4.0 - 0.35;
                if (gap < minBodyGap) {
                    ++gapViolations;
                }
            }
        }
    }
}

int countUnexplainedFreezes(
    const City& city,
    Simulation& sim,
    const map<int, int>& stuckTicks,
    int stuckThresholdTicks
) {
    // Stronger check: claimed stop reason must match a live obstacle.
    return credible_freeze::countUnexplainedFreezes(city, sim, stuckTicks, stuckThresholdTicks);
}

void testStopGiveWayYieldToRight() {
    const OsmToMapResult converted =
        OsmToMapDescription::convertFile(fixturePath("stop_give_way.overpass.json"));
    expect(converted.ok, "osm4_stop_give_way_convert_ok");

    int yieldCount = 0;
    bool emptyPriority = true;
    for (const auto& node : converted.map.intersections) {
        if (node.priorityMode == PriorityMode::YieldToRight) {
            ++yieldCount;
            if (!node.priorityRoadIds.empty()) {
                emptyPriority = false;
            }
        }
    }
    expect(yieldCount >= 2, "osm4_stop_give_way_yield_mode");
    expect(emptyPriority, "osm4_stop_give_way_no_invented_priority_roads");
}

void testOnewayNoReverseRoute() {
    const OsmToMapResult converted =
        OsmToMapDescription::convertFile(fixturePath("oneway.overpass.json"));
    expect(converted.ok, "osm4_oneway_convert_ok");

    City city;
    Simulation sim;
    OsmScenarioConfig config;
    config.maxGenerators = 2;
    config.vehiclesPerGenerator = 1;
    const OsmScenarioResult attached =
        OsmScenarioBuilder::attach(converted.map, converted.report, city, sim, config);
    expect(attached.ok, "osm4_oneway_attach_ok");
    expect(city.getRoadSegments().size() == 1, "osm4_oneway_single_directed_road");

    if (city.getRoadSegments().empty()) {
        expect(false, "osm4_oneway_no_reverse_route");
        return;
    }

    const RoadSegment& road = city.getRoadSegments().front();
    PathFinder pathFinder;
    const vector<int> reverseAttempt =
        pathFinder.findRouteThroughIntersections(city, road.getId(), road.getFromIntersectionId());
    expect(reverseAttempt.empty(), "osm4_oneway_no_reverse_route");

    expect(attached.attachedGeneratorCount == attached.routeCount, "osm4_oneway_gen_eq_routes");
}

void testTransactionalFailedLoadLeavesDemo() {
    City city;
    Simulation sim;
    redirectSimOutputs(sim, "osm4_txn_demo");
    Scenario::setupDemoCity(city, sim);

    const string demoName = city.getName();
    const size_t demoRoads = city.getRoadSegments().size();
    const size_t demoGens = sim.getVehicleGenerators().size();
    const size_t demoLights = sim.getTrafficLights().size();

    expect(demoName.find("Demo") != string::npos, "osm4_txn_demo_loaded");
    expect(demoRoads > 0, "osm4_txn_demo_has_roads");

    const OsmScenarioResult failed = OsmScenarioBuilder::loadOsmFile(
        fixturePath("invalid_not_json.txt"),
        city,
        sim
    );
    expect(!failed.ok, "osm4_txn_fail_ok_false");
    expect(!failed.error.empty(), "osm4_txn_fail_has_error");
    expect(city.getName() == demoName, "osm4_txn_city_name_unchanged");
    expect(city.getRoadSegments().size() == demoRoads, "osm4_txn_roads_unchanged");
    expect(sim.getVehicleGenerators().size() == demoGens, "osm4_txn_gens_unchanged");
    expect(sim.getTrafficLights().size() == demoLights, "osm4_txn_lights_unchanged");
}

void testPathFinderFindsRoutesOnWarsaw() {
    const string path = "data/osm/warsaw_sample.overpass.json";
    if (!fs::exists(path)) {
        cout << "SKIP  osm4_warsaw_routes (sample file missing)\n";
        return;
    }

    City city;
    Simulation sim;
    OsmScenarioConfig config;
    config.seed = VehicleGenerator::defaultSeed;
    config.maxGenerators = 8;
    config.vehiclesPerGenerator = 5;

    const OsmScenarioResult loaded = OsmScenarioBuilder::loadOsmFile(path, city, sim, config);
    expect(loaded.ok, "osm4_warsaw_load_ok");
    expect(city.getRoadSegments().size() > 0, "osm4_warsaw_city_roads");
    expect(loaded.attachedGeneratorCount > 0, "osm4_warsaw_has_generators");
    expect(loaded.routeCount > 0, "osm4_warsaw_has_routes");
    expect(sim.getPedestrianCrossings().size() == loaded.attachedCrossingCount,
           "osm4_warsaw_crossings_attached");

    // Generated signals only for Signalized intersections with approaches.
    size_t signalized = 0;
    for (const auto& node : city.getIntersections()) {
        if (node.getPriorityMode() == PriorityMode::Signalized) {
            ++signalized;
        }
    }
    if (signalized > 0) {
        expect(loaded.generatedPhaseControllerCount > 0, "osm4_warsaw_generated_phases");
        expect(!sim.getPhaseControllers().empty(), "osm4_warsaw_phase_controllers_present");
    } else {
        expect(loaded.generatedPhaseControllerCount == 0, "osm4_warsaw_no_phases_needed");
    }

    PathFinder pathFinder;
    int found = 0;
    const auto& roads = city.getRoadSegments();
    for (size_t i = 0; i < roads.size() && found < 3; ++i) {
        for (size_t j = 0; j < roads.size() && found < 3; ++j) {
            if (i == j) {
                continue;
            }
            vector<int> route =
                pathFinder.findShortestRoute(city, roads[i].getId(), roads[j].getId());
            if (!route.empty() && city.isValidRoute(route)) {
                ++found;
            }
        }
    }
    expect(found > 0, "osm4_warsaw_pathfinder_some_routes");
}

void testWarsaw300sDeterministic() {
    const string path = "data/osm/warsaw_sample.overpass.json";
    if (!fs::exists(path)) {
        cout << "SKIP  osm4_warsaw_300s (sample file missing)\n";
        return;
    }

    OsmScenarioConfig config;
    config.seed = VehicleGenerator::defaultSeed;
    config.maxGenerators = 8;
    config.vehiclesPerGenerator = 5;
    config.spawnIntervalSeconds = 2.5;
    config.phaseGreenSeconds = 8.0;
    config.minVehicleGap = 4.0;
    config.yieldRefusalProbability = 0.05;

    City cityA;
    Simulation simA;
    redirectSimOutputs(simA, "osm4_warsaw_300s_a");
    const OsmScenarioResult loadA = OsmScenarioBuilder::loadOsmFile(path, cityA, simA, config);
    expect(loadA.ok, "osm4_warsaw300_load_a");

    City cityB;
    Simulation simB;
    redirectSimOutputs(simB, "osm4_warsaw_300s_b");
    const OsmScenarioResult loadB = OsmScenarioBuilder::loadOsmFile(path, cityB, simB, config);
    expect(loadB.ok, "osm4_warsaw300_load_b");

    expect(loadA.attachedGeneratorCount == loadB.attachedGeneratorCount,
           "osm4_warsaw300_same_generator_count");

    const double dt = 0.1;
    const double targetTime = 300.0;
    const int stuckThreshold = 250; // 25s at 10 Hz

    map<int, int> stuckTicksA;
    int speedViolations = 0;
    int gapViolations = 0;

    simA.start();
    simB.start();
    bool deterministic = true;

    while (simA.getTime() < targetTime - 1e-9) {
        simA.update(dt, cityA);
        simB.update(dt, cityB);

        scanGapAndSpeed(cityA, simA, speedViolations, gapViolations);

        if (simA.getVehicles().size() != simB.getVehicles().size() ||
            countGenerated(simA) != countGenerated(simB)) {
            deterministic = false;
        }

        for (const auto& vehicle : simA.getVehicles()) {
            if (vehicle.isFinished()) {
                stuckTicksA.erase(vehicle.getId());
                continue;
            }
            if (vehicle.getSpeed() < 0.05) {
                stuckTicksA[vehicle.getId()] += 1;
            } else {
                stuckTicksA[vehicle.getId()] = 0;
            }
        }
    }
    expect(deterministic, "osm4_warsaw300_deterministic");

    const auto& snap = simA.getStatisticsCollector().getLastSnapshot();
    const int generated = countGenerated(simA);
    const int completed = snap.finishedVehicles;
    const int active = snap.activeVehicles;
    const int invalidRoutes = countInvalidRouteEvents(simA);
    const int freezes = countUnexplainedFreezes(cityA, simA, stuckTicksA, stuckThreshold);

    const int targetDemand = loadA.attachedGeneratorCount * config.vehiclesPerGenerator;
    const bool generatorsExhausted = generated >= targetDemand;
    bool queuesCleared = (active == 0);
    // After demand exhausted, allow residual queues explained by lights/yields;
    // progress = completed covers a useful share OR active clearing.
    const bool progressOk =
        completed >= max(1, generated / 5) ||
        (generatorsExhausted && completed + active >= generated);

    expect(simA.getTime() >= 299.0, "osm4_warsaw300_reached_300s");
    expect(generated > 0, "osm4_warsaw300_generated_gt0");
    expect(progressOk, "osm4_warsaw300_progress");
    expect(speedViolations == 0, "osm4_warsaw300_no_speed_violations");
    expect(gapViolations == 0, "osm4_warsaw300_no_gap_violations");
    expect(invalidRoutes == 0, "osm4_warsaw300_no_invalid_routes");
    expect(freezes == 0, "osm4_warsaw300_no_unexplained_freezes");

    // Distinguish queues vs freezes in the report (queues may remain at red/yield).
    ensureOutputDir();
    ofstream report("tests/output/osm4_warsaw_300s_report.txt");
    report << "osm4 warsaw 300s report\n";
    report << "seed=" << config.seed << "\n";
    report << "generators=" << loadA.attachedGeneratorCount << "\n";
    report << "routes=" << loadA.routeCount << "\n";
    report << "phase_controllers_generated=" << loadA.generatedPhaseControllerCount << "\n";
    report << "crossings=" << loadA.attachedCrossingCount << "\n";
    report << "largest_component_roads=" << loadA.largestComponentRoadCount << "\n";
    report << "generated=" << generated << "\n";
    report << "active=" << active << "\n";
    report << "completed=" << completed << "\n";
    report << "waiting_stopped=" << snap.stoppedVehicles << "\n";
    report << "invalid_routes=" << invalidRoutes << "\n";
    report << "gap_violations=" << gapViolations << "\n";
    report << "speed_violations=" << speedViolations << "\n";
    report << "unexplained_freezes=" << freezes << "\n";
    report << "queues_cleared=" << (queuesCleared ? 1 : 0) << "\n";
    report << "generators_exhausted=" << (generatorsExhausted ? 1 : 0) << "\n";
    report << "sim_time=" << simA.getTime() << "\n";
    report << "notes:\n";
    for (const string& note : loadA.notes) {
        report << "  - " << note << "\n";
    }
    report.close();

    cout << "osm4 warsaw300 metrics: generated=" << generated
         << " active=" << active
         << " completed=" << completed
         << " stopped=" << snap.stoppedVehicles
         << " invalid_routes=" << invalidRoutes
         << " gap_viol=" << gapViolations
         << " speed_viol=" << speedViolations
         << " freezes=" << freezes
         << " cleared=" << (queuesCleared ? 1 : 0)
         << " t=" << simA.getTime() << "\n";
}

void testRestrictionBlocksTransitionAndRoutes() {
    City city;
    Simulation sim;
    const OsmScenarioResult loaded =
        OsmScenarioBuilder::loadOsmFile(fixturePath("restriction_no_left.overpass.json"), city, sim);
    expect(loaded.ok, "osm4_restrict_load");
    expect(city.getDisallowedConnections().size() >= 1, "osm4_restrict_city_disallows");

    int fromId = -1;
    int toEast = -1;
    int toNorth = -1;
    for (const auto& road : city.getRoadSegments()) {
        if (road.getSourceRef().find("way/810001") != string::npos) {
            fromId = road.getId();
        }
        if (road.getSourceRef().find("way/810003") != string::npos) {
            toEast = road.getId();
        }
        if (road.getSourceRef().find("way/810002") != string::npos) {
            toNorth = road.getId();
        }
    }
    expect(fromId >= 0 && toEast >= 0 && toNorth >= 0, "osm4_restrict_roads");
    expect(!city.areRoadsConnected(fromId, toEast), "osm4_restrict_no_transition");
    expect(city.areRoadsConnected(fromId, toNorth), "osm4_restrict_straight_ok");

    PathFinder pf;
    expect(pf.findShortestRoute(city, fromId, toEast).empty(), "osm4_restrict_pf_blocked");
    expect(!pf.findShortestRoute(city, fromId, toNorth).empty(), "osm4_restrict_pf_ok");
}

void testDestinationExcludedFromAutoDemand() {
    City city;
    Simulation sim;
    OsmScenarioConfig config;
    config.maxGenerators = 8;
    config.vehiclesPerGenerator = 1;
    const OsmScenarioResult loaded = OsmScenarioBuilder::loadOsmFile(
        fixturePath("access_destination.overpass.json"),
        city,
        sim,
        config
    );
    expect(loaded.ok, "osm4_dest_load");

    int spurId = -1;
    for (const auto& road : city.getRoadSegments()) {
        if (road.getSourceRef().find("way/814003") != string::npos) {
            spurId = road.getId();
            expect(!road.getAllowAutoDemand(), "osm4_dest_flag");
        }
    }
    expect(spurId >= 0, "osm4_dest_spur_exists");

    for (const auto& gen : sim.getVehicleGenerators()) {
        const vector<int>& route = gen.getRoute();
        bool usesSpur = false;
        for (int roadId : route) {
            if (roadId == spurId) {
                usesSpur = true;
            }
        }
        expect(!usesSpur, "osm4_dest_not_in_auto_route");
    }
}

void testNoEmptyRoutesSpawned() {
    const string path = "data/osm/warsaw_sample.overpass.json";
    if (!fs::exists(path)) {
        // Use shared junction fixture for a tiny graph.
        City city;
        Simulation sim;
        const OsmScenarioResult loaded =
            OsmScenarioBuilder::loadOsmFile(fixturePath("shared_junction.overpass.json"), city, sim);
        expect(loaded.ok, "osm4_no_empty_load_fixture");
        expect(loaded.attachedGeneratorCount == loaded.routeCount, "osm4_no_empty_gen_eq_routes");
        return;
    }

    City city;
    Simulation sim;
    const OsmScenarioResult loaded = OsmScenarioBuilder::loadOsmFile(path, city, sim);
    expect(loaded.ok, "osm4_no_empty_load");
    expect(loaded.attachedGeneratorCount == loaded.routeCount, "osm4_no_empty_gen_eq_routes");
}

// Offline: simulation policy caps are attachment/route limits, not map truncation.
void testScenarioBuilderPolicyCaps() {
    MapDescription map;
    map.name = "policy_caps_synthetic";
    for (int i = 0; i < 6; ++i) {
        MapIntersectionDesc node;
        node.id = i;
        node.position = Vector2D{static_cast<double>(i) * 100.0, 0.0};
        if (i >= 1 && i <= 4) {
            node.priorityMode = PriorityMode::Signalized;
        }
        map.intersections.push_back(node);
    }
    for (int i = 0; i < 5; ++i) {
        MapRoadDesc road;
        road.id = 100 + i;
        road.fromIntersectionId = i;
        road.toIntersectionId = i + 1;
        road.length = 100.0;
        road.speedLimit = 13.89;
        road.laneCount = 1;
        map.roads.push_back(road);
    }
    for (int i = 0; i < 4; ++i) {
        MapCrossingDesc crossing;
        crossing.id = 200 + i;
        crossing.roadSegmentId = 100 + i;
        crossing.positionOnRoad = 50.0;
        map.crossings.push_back(crossing);
    }

    OsmScenarioConfig config;
    config.maxAttachedCrossings = 2;
    config.maxGeneratedPhaseControllers = 1;
    config.maxRouteHops = 3;
    config.maxGenerators = 4;
    config.vehiclesPerGenerator = 1;
    config.seed = 42;

    City city;
    Simulation sim;
    ImportReport report;
    const OsmScenarioResult attached =
        OsmScenarioBuilder::attach(map, report, city, sim, config);

    expect(attached.ok, "osm4_caps_attach_ok");
    expect(city.getRoadSegments().size() == map.roads.size(), "osm4_caps_map_not_truncated");
    expect(attached.attachedCrossingCount == 2, "osm4_caps_crossings");
    expect(sim.getPedestrianCrossings().size() == 2, "osm4_caps_crossings_live");
    expect(attached.generatedPhaseControllerCount == 1, "osm4_caps_phases");
    expect(sim.getPhaseControllers().size() == 1, "osm4_caps_phases_live");

    bool routesWithinHopCap = true;
    for (const auto& generator : sim.getVehicleGenerators()) {
        if (static_cast<int>(generator.getRoute().size()) > config.maxRouteHops) {
            routesWithinHopCap = false;
        }
    }
    expect(routesWithinHopCap, "osm4_caps_route_hops");
    expect(attached.attachedGeneratorCount == attached.routeCount, "osm4_caps_gen_eq_routes");
}

} // namespace

int runOsmScenarioTests() {
    cout << "\n--- OSM scenario / sim attachment tests (Stage 4) ---\n";
    testStopGiveWayYieldToRight();
    testOnewayNoReverseRoute();
    testTransactionalFailedLoadLeavesDemo();
    testRestrictionBlocksTransitionAndRoutes();
    testDestinationExcludedFromAutoDemand();
    testPathFinderFindsRoutesOnWarsaw();
    testNoEmptyRoutesSpawned();
    testScenarioBuilderPolicyCaps();
    testWarsaw300sDeterministic();
    return g_failures;
}

int osmScenarioPassedCount() {
    return g_passed;
}
