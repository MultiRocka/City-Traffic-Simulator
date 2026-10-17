#include "core/City.h"
#include "core/CityBuilder.h"
#include "core/CollisionDetector.h"
#include "core/Intersection.h"
#include "core/MapDescription.h"
#include "core/PathFinder.h"
#include "core/Scenario.h"
#include "core/Simulation.h"
#include "core/VehicleGenerator.h"
#include "ui/SimulationSession.h"
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

bool approx(double a, double b, double eps = 1e-9) {
    return fabs(a - b) <= eps;
}

struct DemoMetrics {
    int generated = 0;
    int active = 0;
    int completed = 0;
    int waitingStopped = 0;
    int invalidRoutes = 0;
    int eventsCount = 0;
    int gapViolations = 0;
    int speedViolations = 0;
    int unexplainedFreezes = 0;
    double simTime = 0.0;
    bool queuesCleared = false;
    double queueClearTime = -1.0;
};

int countGenerated(const Simulation& sim) {
    int total = 0;
    for (const auto& generator : sim.getVehicleGenerators()) {
        total += generator.getGeneratedCount();
    }
    return total;
}

// Demo setup targets sum to 40 (14+10+8+8).
bool generatorsExhausted(const Simulation& sim) {
    return countGenerated(sim) >= 40;
}

bool anyActiveLaneBlock(const Simulation& sim) {
    for (const auto& block : sim.getLaneBlockages()) {
        if (block.isActive(sim.getTime())) {
            return true;
        }
    }
    return false;
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

void testDemoMapBuildValidation() {
    MapDescription map = Scenario::buildDemoMapDescription();
    expect(!map.name.empty(), "demo_map_named");
    expect(map.intersections.size() >= 8, "demo_map_intersections");
    expect(map.roads.size() >= 10, "demo_map_roads");
    expect(map.crossings.size() >= 2, "demo_map_crossings");

    bool hasOneLane = false;
    bool hasMultiLane = false;
    bool hasPolyline = false;
    set<double> speeds;
    int pairs = 0;

    for (const auto& road : map.roads) {
        const int lanes = road.laneCount < 0 ? 1 : road.laneCount;
        if (lanes == 1) {
            hasOneLane = true;
        }
        if (lanes >= 2) {
            hasMultiLane = true;
        }
        if (!road.intermediatePoints.empty()) {
            hasPolyline = true;
        }
        speeds.insert(road.speedLimit);

        for (const auto& other : map.roads) {
            if (road.id == other.id) {
                continue;
            }
            if (road.fromIntersectionId == other.toIntersectionId &&
                road.toIntersectionId == other.fromIntersectionId) {
                ++pairs;
            }
        }
    }

    expect(hasOneLane, "demo_has_1lane");
    expect(hasMultiLane, "demo_has_multilane");
    expect(hasPolyline, "demo_has_polyline");
    expect(speeds.size() >= 3, "demo_varied_speed_limits");
    expect(pairs >= 2, "demo_has_twoway_pairs"); // counted both directions

    bool hasSignalized = false;
    bool hasPriorityRoad = false;
    bool hasUnsignalized = false;
    for (const auto& node : map.intersections) {
        if (node.priorityMode == PriorityMode::Signalized) {
            hasSignalized = true;
        } else if (node.priorityMode == PriorityMode::PriorityRoad) {
            hasPriorityRoad = true;
            expect(!node.priorityRoadIds.empty(), "demo_priority_road_ids");
        } else if (node.priorityMode == PriorityMode::YieldToRight ||
                   node.priorityMode == PriorityMode::None) {
            hasUnsignalized = true;
        }
    }
    expect(hasSignalized, "demo_has_signalized");
    expect(hasPriorityRoad, "demo_has_priority_road");
    expect(hasUnsignalized, "demo_has_unsignalized");

    City city;
    string error;
    expect(CityBuilder::build(map, city, error), "demo_citybuilder_ok");
    expect(error.empty(), "demo_citybuilder_no_error");
    expect(static_cast<int>(city.getRoadSegments().size()) == static_cast<int>(map.roads.size()),
           "demo_road_count_matches");

    // Multi-lane approach exists for zipper/block demos (topology, not magic id).
    bool multiLaneApproach = false;
    for (const auto& road : city.getRoadSegments()) {
        if (road.getLaneCount() >= 2 && road.getLength() >= 40.0) {
            multiLaneApproach = true;
        }
    }
    expect(multiLaneApproach, "demo_multilane_approach_present");
}

void testDemoGeneratorsRespectDirectionality() {
    City city;
    Simulation sim;
    redirectSimOutputs(sim, "demo_routes");
    Scenario::setupDemoCity(city, sim);

    PathFinder pathFinder;
    expect(!sim.getVehicleGenerators().empty(), "demo_has_generators");

    sim.start();
    for (int i = 0; i < 80; ++i) {
        sim.update(0.1, city);
    }

    int checked = 0;
    int validNext = 0;
    for (const auto& vehicle : sim.getVehicles()) {
        if (vehicle.isFinished()) {
            continue;
        }
        const int currentId = vehicle.getCurrentRoadSegmentId();
        const int nextId = vehicle.getNextRoadSegmentId();
        if (currentId < 0) {
            continue;
        }
        ++checked;
        if (nextId < 0 || city.areRoadsConnected(currentId, nextId)) {
            ++validNext;
        }
    }

    expect(checked >= 1, "demo_spawned_for_route_check");
    expect(validNext == checked, "demo_routes_respect_connections");

    // PathFinder from west multi-lane approach reaches EastOut.
    int westMulti = -1;
    for (const auto& road : city.getRoadSegments()) {
        if (road.getLaneCount() >= 2 && road.getFromIntersectionId() == 1) {
            westMulti = road.getId();
            break;
        }
    }
    expect(westMulti >= 0, "demo_found_west_approach");
    if (westMulti >= 0) {
        auto route = pathFinder.findRouteThroughIntersections(city, westMulti, 4);
        expect(!route.empty(), "demo_path_west_to_exit");
        bool hopsOk = !route.empty();
        for (size_t i = 0; i + 1 < route.size(); ++i) {
            if (!city.areRoadsConnected(route[i], route[i + 1])) {
                hopsOk = false;
            }
        }
        expect(hopsOk, "demo_path_hops_allowed");
    }
}

void testDemoIntegration300s() {
    City city;
    Simulation sim;
    redirectSimOutputs(sim, "demo_integration_300s");
    Scenario::setupDemoCity(city, sim);
    sim.setPedestrianSeed(VehicleGenerator::defaultSeed + 50u);
    sim.setLaneBlockSeed(VehicleGenerator::defaultSeed + 60u);
    sim.start();

    // Temporary lane block on multi-lane approach (zipper/block demo), found by topology.
    int blockRoadId = -1;
    double blockRoadLength = 0.0;
    for (const auto& road : city.getRoadSegments()) {
        if (road.getLaneCount() >= 2 && road.getLength() >= 50.0) {
            blockRoadId = road.getId();
            blockRoadLength = road.getLength();
            break;
        }
    }
    expect(blockRoadId >= 0, "demo300_found_block_road");
    if (blockRoadId >= 0) {
        const int blockId = sim.addLaneBlockage(
            blockRoadId, 0, blockRoadLength * 0.4, 6.0, 15.0, 35.0, city
        );
        expect(blockId > 0, "demo300_temp_block_added");
    }

    const double dt = 0.1;
    const int steps = 3000; // 300 s
    const int stuckThreshold = 250; // 25 s

    map<int, int> stuckTicks;
    map<int, pair<int, double>> lastPose; // roadId, position
    DemoMetrics metrics;

    bool spawnDoneSeen = false;
    bool blocksClearSeen = false;
    double clearWatchStart = -1.0;
    const double queueClearLimit = 120.0; // after spawn+blocks done

    for (int stepIndex = 0; stepIndex < steps; ++stepIndex) {
        sim.update(dt, city);
        scanGapAndSpeed(city, sim, metrics.speedViolations, metrics.gapViolations);

        for (const auto& vehicle : sim.getVehicles()) {
            if (vehicle.isFinished()) {
                continue;
            }
            auto it = lastPose.find(vehicle.getId());
            if (it != lastPose.end()) {
                const bool samePlace =
                    it->second.first == vehicle.getCurrentRoadSegmentId() &&
                    fabs(it->second.second - vehicle.getPositionOnRoad()) < 0.01 &&
                    vehicle.isStopped();
                if (samePlace) {
                    stuckTicks[vehicle.getId()]++;
                } else {
                    stuckTicks[vehicle.getId()] = 0;
                }
            }
            lastPose[vehicle.getId()] = {
                vehicle.getCurrentRoadSegmentId(),
                vehicle.getPositionOnRoad()
            };
        }

        const bool exhausted = generatorsExhausted(sim);
        const bool blocksClear = !anyActiveLaneBlock(sim);
        if (exhausted) {
            spawnDoneSeen = true;
        }
        if (blocksClear && sim.getTime() > 50.0) {
            blocksClearSeen = true;
        }
        if (spawnDoneSeen && blocksClearSeen && clearWatchStart < 0.0) {
            clearWatchStart = sim.getTime();
        }

        if (clearWatchStart >= 0.0 && !metrics.queuesCleared) {
            const auto& snap = sim.getStatisticsCollector().getLastSnapshot();
            if (snap.activeVehicles == 0) {
                metrics.queuesCleared = true;
                metrics.queueClearTime = sim.getTime();
            } else if (sim.getTime() - clearWatchStart > queueClearLimit) {
                // Still may have transient red-light stops; require no unexplained freezes
                // and that completed >= 80% of generated.
                break;
            }
        }
    }

    const auto& snap = sim.getStatisticsCollector().getLastSnapshot();
    metrics.generated = countGenerated(sim);
    metrics.active = snap.activeVehicles;
    metrics.completed = snap.finishedVehicles;
    metrics.waitingStopped = snap.stoppedVehicles;
    metrics.invalidRoutes = countInvalidRouteEvents(sim);
    metrics.eventsCount = static_cast<int>(sim.getEventLog().getEvents().size());
    metrics.simTime = sim.getTime();
    metrics.unexplainedFreezes = countUnexplainedFreezes(city, sim, stuckTicks, stuckThreshold);

    if (!metrics.queuesCleared && metrics.active == 0) {
        metrics.queuesCleared = true;
        metrics.queueClearTime = metrics.simTime;
    }

    // Queues that can clear: after generators stop and temp blocks end, either active==0
    // within queueClearLimit, or completed covers nearly all generated with no unexplained freeze.
    const bool clearMetricOk =
        metrics.queuesCleared ||
        (spawnDoneSeen && blocksClearSeen &&
         metrics.completed >= static_cast<int>(metrics.generated * 0.85) &&
         metrics.unexplainedFreezes == 0);

    expect(metrics.simTime >= 299.0, "demo300_reached_300s");
    expect(metrics.generated >= 30, "demo300_generated_enough");
    expect(metrics.completed >= 10, "demo300_completed_some");
    expect(metrics.speedViolations == 0, "demo300_no_speed_violations");
    expect(metrics.gapViolations == 0, "demo300_no_gap_violations");
    expect(metrics.invalidRoutes == 0, "demo300_no_invalid_routes");
    expect(metrics.unexplainedFreezes == 0, "demo300_no_unexplained_freezes");
    expect(clearMetricOk, "demo300_queues_that_can_clear_do");

    ofstream report("tests/output/demo_integration_300s_report.txt");
    report << "metric_definitions:\n";
    report << "  generated = sum of VehicleGenerator::getGeneratedCount\n";
    report << "  active = StatisticsSnapshot.activeVehicles (not finished)\n";
    report << "  completed = StatisticsSnapshot.finishedVehicles\n";
    report << "  waiting_stopped = StatisticsSnapshot.stoppedVehicles\n";
    report << "  invalid_routes = unique vehicles with route_invalid events\n";
    report << "  events_count = EventLog size\n";
    report << "  gap_violations = same-lane front spacing < leader.length + minGap - slack\n";
    report << "  speed_violations = speed > road.speedLimit + 0.15\n";
    report << "  unexplained_freeze = stopped >=25s without explained stop reason\n";
    report << "  queue_clear = after generators exhausted AND temp blocks ended,\n";
    report << "                active==0 within 120s OR completed>=85% generated with no freezes\n";
    report << "generated=" << metrics.generated << "\n";
    report << "active=" << metrics.active << "\n";
    report << "completed=" << metrics.completed << "\n";
    report << "waiting_stopped=" << metrics.waitingStopped << "\n";
    report << "invalid_routes=" << metrics.invalidRoutes << "\n";
    report << "events_count=" << metrics.eventsCount << "\n";
    report << "gap_violations=" << metrics.gapViolations << "\n";
    report << "speed_violations=" << metrics.speedViolations << "\n";
    report << "unexplained_freezes=" << metrics.unexplainedFreezes << "\n";
    report << "queues_cleared=" << (metrics.queuesCleared ? 1 : 0) << "\n";
    report << "queue_clear_time=" << metrics.queueClearTime << "\n";
    report << "sim_time=" << metrics.simTime << "\n";

    cout << "demo300 metrics: generated=" << metrics.generated
         << " active=" << metrics.active
         << " completed=" << metrics.completed
         << " stopped=" << metrics.waitingStopped
         << " invalid_routes=" << metrics.invalidRoutes
         << " events=" << metrics.eventsCount
         << " gap_viol=" << metrics.gapViolations
         << " speed_viol=" << metrics.speedViolations
         << " freezes=" << metrics.unexplainedFreezes
         << " cleared=" << (metrics.queuesCleared ? 1 : 0)
         << " t=" << metrics.simTime << "\n";
}

void testDemoResetRestart() {
    SimulationSession session;
    redirectSimOutputs(session.getSimulation(), "demo_reset");
    session.loadDemoScenario();
    expect(session.getLoadedScenario() == SessionScenario::Demo, "demo_reset_loaded_demo");

    session.start();
    for (int i = 0; i < 30; ++i) {
        session.update(SimulationSession::renderDt);
    }
    expect(session.getSimulation().getTime() > 0.2, "demo_reset_time_advanced");

    session.reset();
    expect(!session.isRunning(), "demo_reset_stopped");
    expect(approx(session.getSimulation().getTime(), 0.0), "demo_reset_time_zero");
    expect(session.getSimulation().getVehicles().empty(), "demo_reset_no_vehicles");
    expect(session.getLoadedScenario() == SessionScenario::Demo, "demo_reset_keeps_demo");
    expect(session.getCity().getName().find("Demo") != string::npos, "demo_reset_city_name");

    session.start();
    // Need > first spawn interval (west generator ≈ 2.2 s) after reset.
    for (int i = 0; i < 240; ++i) {
        session.update(SimulationSession::renderDt);
    }
    expect(session.getSimulation().getTime() > 3.0, "demo_restart_works");
    expect(
        !session.getSimulation().getVehicles().empty() ||
        session.getSimulation().getStatisticsCollector().getLastSnapshot().totalVehiclesSeen > 0,
        "demo_restart_generators"
    );
}

void testDemoTimeMultiplierSamePhysics() {
    // Equal sim-time → identical fixed-dt physics. Multiplier only changes wall-clock rate.
    City cityA;
    Simulation simA;
    redirectSimOutputs(simA, "demo_physics_a");
    Scenario::setupDemoCity(cityA, simA);
    simA.setPedestrianSeed(VehicleGenerator::defaultSeed + 50u);
    simA.setLaneBlockSeed(VehicleGenerator::defaultSeed + 60u);

    City cityB;
    Simulation simB;
    redirectSimOutputs(simB, "demo_physics_b");
    Scenario::setupDemoCity(cityB, simB);
    simB.setPedestrianSeed(VehicleGenerator::defaultSeed + 50u);
    simB.setLaneBlockSeed(VehicleGenerator::defaultSeed + 60u);

    simA.start();
    simB.start();

    const double dt = SimulationSession::simulationDt;
    const int steps = 200; // 20 s sim
    for (int i = 0; i < steps; ++i) {
        simA.update(dt, cityA);
        simB.update(dt, cityB);
    }

    expect(approx(simA.getTime(), simB.getTime()), "demo_physics_same_sim_time");
    expect(
        simA.getStatisticsCollector().getLastSnapshot().totalVehiclesSeen ==
        simB.getStatisticsCollector().getLastSnapshot().totalVehiclesSeen,
        "demo_physics_same_generated_seen"
    );
    expect(
        simA.getStatisticsCollector().getLastSnapshot().finishedVehicles ==
        simB.getStatisticsCollector().getLastSnapshot().finishedVehicles,
        "demo_physics_same_finished"
    );

    const auto& va = simA.getVehicles();
    const auto& vb = simB.getVehicles();
    expect(va.size() == vb.size(), "demo_physics_same_vehicle_count");
    bool posesMatch = va.size() == vb.size();
    for (size_t i = 0; i < va.size() && posesMatch; ++i) {
        if (va[i].getId() != vb[i].getId() ||
            va[i].getCurrentRoadSegmentId() != vb[i].getCurrentRoadSegmentId() ||
            !approx(va[i].getPositionOnRoad(), vb[i].getPositionOnRoad(), 1e-9) ||
            !approx(va[i].getSpeed(), vb[i].getSpeed(), 1e-9)) {
            posesMatch = false;
        }
    }
    expect(posesMatch, "demo_physics_identical_poses");

    // Session multipliers: wall-clock differs; equal sim duration still uses fixed dt steps.
    SimulationSession sessionSlow;
    redirectSimOutputs(sessionSlow.getSimulation(), "demo_mul_slow");
    sessionSlow.loadDemoScenario();
    sessionSlow.setTimeMultiplier(1.0);
    sessionSlow.start();

    SimulationSession sessionFast;
    redirectSimOutputs(sessionFast.getSimulation(), "demo_mul_fast");
    sessionFast.loadDemoScenario();
    sessionFast.setTimeMultiplier(2.0);
    sessionFast.start();

    auto pumpToSimTime = [](SimulationSession& session, double targetSim) {
        const double frame = SimulationSession::renderDt;
        int guard = 0;
        while (session.getSimulation().getTime() + 1e-9 < targetSim && guard < 100000) {
            session.update(frame);
            ++guard;
        }
    };

    const double target = 5.0;
    pumpToSimTime(sessionSlow, target);
    pumpToSimTime(sessionFast, target);

    expect(approx(sessionSlow.getSimulation().getTime(), target, 0.15), "demo_mul_1x_sim_time");
    expect(approx(sessionFast.getSimulation().getTime(), target, 0.15), "demo_mul_2x_sim_time");
    expect(
        sessionSlow.getSimulation().getStatisticsCollector().getLastSnapshot().totalVehiclesSeen ==
        sessionFast.getSimulation().getStatisticsCollector().getLastSnapshot().totalVehiclesSeen,
        "demo_mul_equal_sim_same_seen"
    );
}

} // namespace

int runDemoIntegrationTests() {
    cout << "\n--- Demo integration tests ---\n";
    testDemoMapBuildValidation();
    testDemoGeneratorsRespectDirectionality();
    testDemoIntegration300s();
    testDemoResetRestart();
    testDemoTimeMultiplierSamePhysics();
    return g_failures;
}

int demoIntegrationPassedCount() {
    return g_passed;
}
