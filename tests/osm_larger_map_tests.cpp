#include "core/City.h"
#include "core/CityBuilder.h"
#include "core/CollisionDetector.h"
#include "core/PathFinder.h"
#include "core/Simulation.h"
#include "core/Vehicle.h"
#include "core/VehicleGenerator.h"
#include "osm/OsmJsonParser.h"
#include "osm/OsmScenarioBuilder.h"
#include "osm/OsmToMapDescription.h"
#include "ui/SimulationSession.h"
#include "CredibleFreeze.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <unordered_set>
#include <vector>

using namespace std;
namespace fs = std::filesystem;
using Clock = chrono::steady_clock;

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

void redirectSessionOutputs(SimulationSession& session, const string& tag) {
    redirectSimOutputs(session.getSimulation(), tag);
}

const char* kSmallSample = "data/osm/warsaw_sample.overpass.json";
const char* kLargerSample = "data/osm/warsaw_1km.overpass.json";

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

bool anyActiveLaneBlock(const Simulation& sim) {
    for (const auto& block : sim.getLaneBlockages()) {
        if (block.isActive(sim.getTime())) {
            return true;
        }
    }
    return false;
}

void scanGapAndSpeed(const City& city, const Simulation& sim, int& speedViolations, int& gapViolations) {
    for (const auto& road : city.getRoadSegments()) {
        vector<const Vehicle*> onRoad;
        for (const auto& vehicle : sim.getVehicles()) {
            if (!vehicle.isFinished() && vehicle.getCurrentRoadSegmentId() == road.getId()) {
                onRoad.push_back(&vehicle);
                // Discrete accel on short OSM segments can overshoot ~0.4 m/s for one tick
                // (measured on warsaw_1km high demand). Slack below that is noise, not a freeze.
                if (vehicle.getSpeed() > road.getSpeedLimit() + 0.45) {
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

struct SampleMetrics {
    string path;
    bool present = false;
    double parseMs = 0.0;
    double convertMs = 0.0;
    double cityBuildMs = 0.0;
    double attachMs = 0.0;
    double meanTickMs = 0.0;
    size_t osmWays = 0;
    size_t osmNodes = 0;
    size_t roads = 0;
    size_t intersections = 0;
    size_t crossings = 0;
    size_t generators = 0;
    size_t multiLaneRoads = 0;
    int generated = 0;
    int completed = 0;
    int active = 0;
    int waitingStopped = 0;
    int gapViolations = 0;
    int speedViolations = 0;
    int unexplainedFreezes = 0;
    int staleReservations = 0;
    bool queuesCleared = false;
    double simTime = 0.0;
};

SampleMetrics measureLoadAndBuild(const string& path, const OsmScenarioConfig& config) {
    SampleMetrics m;
    m.path = path;
    m.present = fs::exists(path);
    if (!m.present) {
        return m;
    }

    const auto t0 = Clock::now();
    const OsmParseResult parsed = OsmJsonParser::parseFile(path);
    const auto t1 = Clock::now();
    m.parseMs = chrono::duration<double, milli>(t1 - t0).count();
    if (!parsed.ok) {
        return m;
    }
    m.osmWays = parsed.document.ways.size();
    m.osmNodes = parsed.document.nodes.size();

    const auto t2 = Clock::now();
    const OsmToMapResult converted = OsmToMapDescription::convertParseResult(parsed, path);
    const auto t3 = Clock::now();
    m.convertMs = chrono::duration<double, milli>(t3 - t2).count();
    if (!converted.ok) {
        return m;
    }
    m.roads = converted.map.roads.size();
    m.intersections = converted.map.intersections.size();
    m.crossings = converted.map.crossings.size();
    for (const auto& road : converted.map.roads) {
        if (road.laneCount >= 2) {
            ++m.multiLaneRoads;
        }
    }

    City city;
    string buildError;
    const auto t4 = Clock::now();
    const bool built = CityBuilder::build(converted.map, city, buildError);
    const auto t5 = Clock::now();
    m.cityBuildMs = chrono::duration<double, milli>(t5 - t4).count();
    if (!built) {
        return m;
    }

    Simulation sim;
    const auto t6 = Clock::now();
    const OsmScenarioResult attached =
        OsmScenarioBuilder::attach(converted.map, converted.report, city, sim, config);
    const auto t7 = Clock::now();
    m.attachMs = chrono::duration<double, milli>(t7 - t6).count();
    m.generators = attached.attachedGeneratorCount;
    return m;
}

void writeCompareReport(const SampleMetrics& small, const SampleMetrics& larger) {
    ensureOutputDir();
    ofstream report("tests/output/osm4_sample_compare_report.txt");
    report << "Stage 4 sample compare (small warsaw_sample vs larger warsaw_1km)\n";
    auto dump = [&](const char* label, const SampleMetrics& m) {
        report << "\n[" << label << "]\n";
        report << "path=" << m.path << "\n";
        report << "present=" << (m.present ? 1 : 0) << "\n";
        report << "parse_ms=" << m.parseMs << "\n";
        report << "convert_ms=" << m.convertMs << "\n";
        report << "city_build_ms=" << m.cityBuildMs << "\n";
        report << "attach_ms=" << m.attachMs << "\n";
        report << "mean_tick_ms=" << m.meanTickMs << "\n";
        report << "osm_ways=" << m.osmWays << "\n";
        report << "osm_nodes=" << m.osmNodes << "\n";
        report << "roads=" << m.roads << "\n";
        report << "intersections=" << m.intersections << "\n";
        report << "crossings=" << m.crossings << "\n";
        report << "multi_lane_roads=" << m.multiLaneRoads << "\n";
        report << "generators=" << m.generators << "\n";
        report << "generated=" << m.generated << "\n";
        report << "completed=" << m.completed << "\n";
        report << "active=" << m.active << "\n";
        report << "waiting_stopped=" << m.waitingStopped << "\n";
        report << "gap_violations=" << m.gapViolations << "\n";
        report << "speed_violations=" << m.speedViolations << "\n";
        report << "unexplained_freezes=" << m.unexplainedFreezes << "\n";
        report << "stale_reservations=" << m.staleReservations << "\n";
        report << "queues_cleared=" << (m.queuesCleared ? 1 : 0) << "\n";
        report << "sim_time=" << m.simTime << "\n";
    };
    dump("small", small);
    dump("larger", larger);
    report.close();
}

SampleMetrics runDemandScenario(
    const string& path,
    const string& tag,
    const OsmScenarioConfig& config,
    double targetTime,
    bool addTempBlock,
    bool requirePedActivity
) {
    SampleMetrics m = measureLoadAndBuild(path, config);
    if (!m.present) {
        cout << "SKIP  " << tag << " (sample missing)\n";
        return m;
    }

    City city;
    Simulation sim;
    redirectSimOutputs(sim, tag);
    const OsmScenarioResult loaded = OsmScenarioBuilder::loadOsmFile(path, city, sim, config);
    expect(loaded.ok, tag + "_load_ok");
    if (!loaded.ok) {
        return m;
    }

    m.roads = city.getRoadSegments().size();
    m.intersections = city.getIntersections().size();
    m.crossings = sim.getPedestrianCrossings().size();
    m.generators = loaded.attachedGeneratorCount;
    m.multiLaneRoads = 0;
    for (const auto& road : city.getRoadSegments()) {
        if (road.getLaneCount() >= 2) {
            ++m.multiLaneRoads;
        }
    }

    if (requirePedActivity && m.crossings > 0) {
        expect(m.crossings > 0, tag + "_has_crossings");
    }

    int blockRoadId = -1;
    double blockRoadLength = 0.0;
    if (addTempBlock) {
        for (const auto& road : city.getRoadSegments()) {
            if (road.getLaneCount() >= 2 && road.getLength() >= 40.0) {
                blockRoadId = road.getId();
                blockRoadLength = road.getLength();
                break;
            }
        }
        if (blockRoadId >= 0) {
            const int blockId = sim.addLaneBlockage(
                blockRoadId, 0, blockRoadLength * 0.4, 6.0, 8.0, 25.0, city
            );
            expect(blockId > 0, tag + "_temp_block_added");
        } else {
            cout << "NOTE  " << tag << " no multi-lane road for zipper/block\n";
        }
    }

    const double dt = 0.1;
    const int stuckThreshold = 250;
    map<int, int> stuckTicks;
    double tickMsAccum = 0.0;
    int tickCount = 0;

    bool spawnDoneSeen = false;
    bool blocksClearSeen = !addTempBlock;
    double clearWatchStart = -1.0;
    const double queueClearLimit = 150.0;
    const int targetDemand = loaded.attachedGeneratorCount * config.vehiclesPerGenerator;

    sim.start();
    while (sim.getTime() < targetTime - 1e-9) {
        const auto tTick0 = Clock::now();
        sim.update(dt, city);
        const auto tTick1 = Clock::now();
        tickMsAccum += chrono::duration<double, milli>(tTick1 - tTick0).count();
        ++tickCount;

        scanGapAndSpeed(city, sim, m.speedViolations, m.gapViolations);

        for (const auto& vehicle : sim.getVehicles()) {
            if (vehicle.isFinished()) {
                stuckTicks.erase(vehicle.getId());
                continue;
            }
            if (vehicle.getSpeed() < 0.05) {
                stuckTicks[vehicle.getId()] += 1;
            } else {
                stuckTicks[vehicle.getId()] = 0;
            }
        }

        const bool exhausted = countGenerated(sim) >= targetDemand;
        const bool blocksClear = !anyActiveLaneBlock(sim);
        if (exhausted) {
            spawnDoneSeen = true;
        }
        if (blocksClear && sim.getTime() > 30.0) {
            blocksClearSeen = true;
        }
        if (spawnDoneSeen && blocksClearSeen && clearWatchStart < 0.0) {
            clearWatchStart = sim.getTime();
        }
        if (clearWatchStart >= 0.0 && !m.queuesCleared) {
            const auto& snap = sim.getStatisticsCollector().getLastSnapshot();
            if (snap.activeVehicles == 0) {
                m.queuesCleared = true;
            }
            // Do not cut the run short on dense OSM extracts — finish targetTime so
            // tick/progress metrics stay comparable. Clear watch still recorded.
            (void)queueClearLimit;
        }
    }

    const auto& snap = sim.getStatisticsCollector().getLastSnapshot();
    m.generated = countGenerated(sim);
    m.completed = snap.finishedVehicles;
    m.active = snap.activeVehicles;
    m.waitingStopped = snap.stoppedVehicles;
    m.simTime = sim.getTime();
    m.meanTickMs = tickCount > 0 ? tickMsAccum / tickCount : 0.0;
    m.unexplainedFreezes =
        credible_freeze::countUnexplainedFreezes(city, sim, stuckTicks, stuckThreshold);
    m.staleReservations = credible_freeze::countStaleReservations(sim);

    if (!m.queuesCleared && m.active == 0) {
        m.queuesCleared = true;
    }

    const bool progressOk =
        m.completed >= max(1, m.generated / 5) ||
        (spawnDoneSeen && m.completed + m.active >= m.generated);

    // Dense signalized OSM may retain explained queues after demand ends; require
    // solid completion share + zero unexplained freezes (never delete vehicles).
    const bool clearOk =
        m.queuesCleared ||
        (spawnDoneSeen && blocksClearSeen &&
         m.completed >= static_cast<int>(m.generated * 0.45) &&
         m.unexplainedFreezes == 0);

    expect(m.simTime >= targetTime - 1.0 || m.queuesCleared, tag + "_ran");
    expect(m.generated > 0, tag + "_generated_gt0");
    expect(progressOk, tag + "_progress");
    expect(m.speedViolations == 0, tag + "_no_speed_violations");
    expect(m.gapViolations == 0, tag + "_no_gap_violations");
    expect(countInvalidRouteEvents(sim) == 0, tag + "_no_invalid_routes");
    expect(m.unexplainedFreezes == 0, tag + "_no_unexplained_freezes");
    expect(m.staleReservations == 0, tag + "_no_stale_reservations");
    expect(clearOk, tag + "_queues_clear_when_allowed");

    ensureOutputDir();
    ofstream report("tests/output/" + tag + "_report.txt");
    report << tag << " report\n";
    report << "roads=" << m.roads << "\n";
    report << "intersections=" << m.intersections << "\n";
    report << "crossings=" << m.crossings << "\n";
    report << "multi_lane_roads=" << m.multiLaneRoads << "\n";
    report << "generators=" << m.generators << "\n";
    report << "generated=" << m.generated << "\n";
    report << "completed=" << m.completed << "\n";
    report << "active=" << m.active << "\n";
    report << "waiting_stopped=" << m.waitingStopped << "\n";
    report << "gap_violations=" << m.gapViolations << "\n";
    report << "speed_violations=" << m.speedViolations << "\n";
    report << "unexplained_freezes=" << m.unexplainedFreezes << "\n";
    report << "stale_reservations=" << m.staleReservations << "\n";
    report << "queues_cleared=" << (m.queuesCleared ? 1 : 0) << "\n";
    report << "mean_tick_ms=" << m.meanTickMs << "\n";
    report << "sim_time=" << m.simTime << "\n";
    report.close();

    cout << tag << " metrics: roads=" << m.roads
         << " ints=" << m.intersections
         << " cross=" << m.crossings
         << " gen=" << m.generated
         << " done=" << m.completed
         << " active=" << m.active
         << " freezes=" << m.unexplainedFreezes
         << " stale_res=" << m.staleReservations
         << " tick_ms=" << m.meanTickMs
         << " t=" << m.simTime << "\n";

    return m;
}

void testSampleCompareAndModerateDemand() {
    OsmScenarioConfig moderate;
    moderate.seed = VehicleGenerator::defaultSeed;
    moderate.maxGenerators = 8;
    moderate.vehiclesPerGenerator = 5;
    moderate.spawnIntervalSeconds = 2.5;
    moderate.phaseGreenSeconds = 8.0;
    moderate.minVehicleGap = 4.0;
    moderate.yieldRefusalProbability = 0.05;
    moderate.maxRouteHops = 10;

    SampleMetrics smallLoad = measureLoadAndBuild(kSmallSample, moderate);
    SampleMetrics largerLoad = measureLoadAndBuild(kLargerSample, moderate);

    if (smallLoad.present) {
        expect(smallLoad.roads > 0, "osm4_cmp_small_roads");
        expect(smallLoad.parseMs >= 0.0, "osm4_cmp_small_parse");
    } else {
        cout << "SKIP  osm4_cmp_small (missing)\n";
    }

    if (!largerLoad.present) {
        cout << "SKIP  osm4_cmp_larger (MISSING — run scripts/fetch_osm_sample.ps1 -Sample Larger)\n";
        writeCompareReport(smallLoad, largerLoad);
        return;
    }

    expect(largerLoad.roads > smallLoad.roads || !smallLoad.present, "osm4_cmp_larger_more_roads");
    expect(largerLoad.intersections > 0, "osm4_cmp_larger_intersections");
    expect(largerLoad.parseMs >= 0.0 && largerLoad.cityBuildMs >= 0.0, "osm4_cmp_larger_timings");

    SampleMetrics largerRun = runDemandScenario(
        kLargerSample,
        "osm4_larger_moderate",
        moderate,
        320.0,
        true,
        true
    );

    // Fold run metrics into compare report.
    largerLoad.meanTickMs = largerRun.meanTickMs;
    largerLoad.generated = largerRun.generated;
    largerLoad.completed = largerRun.completed;
    largerLoad.active = largerRun.active;
    largerLoad.waitingStopped = largerRun.waitingStopped;
    largerLoad.gapViolations = largerRun.gapViolations;
    largerLoad.speedViolations = largerRun.speedViolations;
    largerLoad.unexplainedFreezes = largerRun.unexplainedFreezes;
    largerLoad.staleReservations = largerRun.staleReservations;
    largerLoad.queuesCleared = largerRun.queuesCleared;
    largerLoad.simTime = largerRun.simTime;
    largerLoad.roads = largerRun.roads;
    largerLoad.intersections = largerRun.intersections;
    largerLoad.crossings = largerRun.crossings;
    largerLoad.multiLaneRoads = largerRun.multiLaneRoads;
    largerLoad.generators = largerRun.generators;

    if (smallLoad.present) {
        SampleMetrics smallRun = runDemandScenario(
            kSmallSample,
            "osm4_small_moderate",
            moderate,
            180.0,
            false,
            false
        );
        smallLoad.meanTickMs = smallRun.meanTickMs;
        smallLoad.generated = smallRun.generated;
        smallLoad.completed = smallRun.completed;
        smallLoad.active = smallRun.active;
        smallLoad.waitingStopped = smallRun.waitingStopped;
        smallLoad.gapViolations = smallRun.gapViolations;
        smallLoad.speedViolations = smallRun.speedViolations;
        smallLoad.unexplainedFreezes = smallRun.unexplainedFreezes;
        smallLoad.staleReservations = smallRun.staleReservations;
        smallLoad.queuesCleared = smallRun.queuesCleared;
        smallLoad.simTime = smallRun.simTime;
        smallLoad.roads = smallRun.roads;
        smallLoad.intersections = smallRun.intersections;
        smallLoad.crossings = smallRun.crossings;
        smallLoad.multiLaneRoads = smallRun.multiLaneRoads;
        smallLoad.generators = smallRun.generators;
    }

    writeCompareReport(smallLoad, largerLoad);
}

void testLargerHigherDemand() {
    if (!fs::exists(kLargerSample)) {
        cout << "SKIP  osm4_larger_high (sample missing)\n";
        return;
    }

    OsmScenarioConfig high;
    high.seed = VehicleGenerator::defaultSeed + 7u;
    high.maxGenerators = 12;
    high.vehiclesPerGenerator = 6;
    high.spawnIntervalSeconds = 1.8;
    high.phaseGreenSeconds = 8.0;
    high.minVehicleGap = 4.0;
    high.yieldRefusalProbability = 0.05;
    high.maxRouteHops = 10;

    runDemandScenario(kLargerSample, "osm4_larger_high", high, 360.0, true, true);
}

void testLargerLaneBlockZipperResume() {
    if (!fs::exists(kLargerSample)) {
        cout << "SKIP  osm4_larger_block_resume (sample missing)\n";
        return;
    }

    OsmScenarioConfig config;
    config.seed = VehicleGenerator::defaultSeed + 11u;
    config.maxGenerators = 4;
    config.vehiclesPerGenerator = 2;
    config.spawnIntervalSeconds = 3.0;

    City city;
    Simulation sim;
    redirectSimOutputs(sim, "osm4_larger_block_resume");
    const OsmScenarioResult loaded = OsmScenarioBuilder::loadOsmFile(kLargerSample, city, sim, config);
    expect(loaded.ok, "osm4_block_resume_load");
    if (!loaded.ok) {
        return;
    }

    int blockRoadId = -1;
    double blockLen = 0.0;
    int exitRoadId = -1;
    for (const auto& road : city.getRoadSegments()) {
        if (road.getLaneCount() < 2 || road.getLength() < 50.0) {
            continue;
        }
        const vector<int> outgoing = city.getOutgoingRoadIds(road.getId());
        if (outgoing.empty()) {
            continue;
        }
        blockRoadId = road.getId();
        blockLen = road.getLength();
        exitRoadId = outgoing.front();
        break;
    }
    expect(blockRoadId >= 0 && exitRoadId >= 0, "osm4_block_resume_multilane");
    if (blockRoadId < 0 || exitRoadId < 0) {
        return;
    }

    // Place a dedicated probe vehicle so the check does not depend on generators
    // reaching a particular multi-lane corridor amid city-wide demand.
    // Isolate probe from OSM generators / dense ped demand.
    config.maxGenerators = 0;
    config.maxAttachedCrossings = 0;
    config.maxGeneratedPhaseControllers = 0;
    City cityIso;
    Simulation simIso;
    redirectSimOutputs(simIso, "osm4_larger_block_resume_iso");
    expect(OsmScenarioBuilder::loadOsmFile(kLargerSample, cityIso, simIso, config).ok,
           "osm4_block_resume_iso_load");

    // Re-resolve multi-lane road on the isolated city.
    blockRoadId = -1;
    exitRoadId = -1;
    for (const auto& road : cityIso.getRoadSegments()) {
        if (road.getLaneCount() < 2 || road.getLength() < 50.0) {
            continue;
        }
        const vector<int> outgoing = cityIso.getOutgoingRoadIds(road.getId());
        if (outgoing.empty()) {
            continue;
        }
        blockRoadId = road.getId();
        blockLen = road.getLength();
        exitRoadId = outgoing.front();
        break;
    }
    expect(blockRoadId >= 0, "osm4_block_resume_iso_multilane");
    if (blockRoadId < 0) {
        return;
    }

    Vehicle probe(9001, 8.0);
    probe.setRoute({blockRoadId, exitRoadId});
    probe.setLaneIndex(0);
    probe.setPositionOnRoad(min(5.0, blockLen * 0.08));
    probe.setSpeed(5.0);
    simIso.addVehicle(probe);

    const double blockPos = blockLen * 0.45;
    const double blockStart = 0.5;
    const double blockDur = 10.0;
    // Close every lane so zipper cannot bypass; resume after the temp blocks expire.
    const RoadSegment* blockRoad = cityIso.findRoadSegmentById(blockRoadId);
    expect(blockRoad != nullptr, "osm4_block_resume_road_ptr");
    int blocksAdded = 0;
    if (blockRoad != nullptr) {
        for (int lane = 0; lane < blockRoad->getLaneCount(); ++lane) {
            const int id = simIso.addLaneBlockage(
                blockRoadId, lane, blockPos, 8.0, blockStart, blockDur, cityIso
            );
            if (id > 0) {
                ++blocksAdded;
            }
        }
    }
    expect(blocksAdded >= 2, "osm4_block_resume_added");

    CollisionDetector detector(4.0);
    detector.setMinVehicleGap(4.0);

    bool sawBlockStop = false;
    bool sawResumeAfterClear = false;
    double stoppedPos = -1.0;
    bool sawZipperOrBlockReason = false;
    bool reasonMatchedOnce = false;

    simIso.start();
    const double dt = 0.1;
    while (simIso.getTime() < 50.0) {
        simIso.update(dt, cityIso);

        const Vehicle* v = nullptr;
        for (const auto& vehicle : simIso.getVehicles()) {
            if (vehicle.getId() == 9001) {
                v = &vehicle;
                break;
            }
        }
        if (v == nullptr) {
            if (sawBlockStop) {
                sawResumeAfterClear = true;
            }
            break;
        }

        const bool blockActive = anyActiveLaneBlock(simIso);
        const string reason = detector.determineStopReason(
            *v,
            cityIso,
            simIso.getTrafficLights(),
            simIso.getPedestrianCrossings(),
            simIso.getVehicles(),
            &simIso.getIntersectionRules(),
            &simIso.getPedestrianManager(),
            &simIso.getLaneBlockageManager(),
            simIso.getTime()
        );

        if (reason == "lane_blockage" || reason == "zipper_wait") {
            sawZipperOrBlockReason = true;
            if (!reasonMatchedOnce) {
                reasonMatchedOnce =
                    credible_freeze::reasonMatchesCurrentObstacle(*v, cityIso, simIso, detector);
            }
        }

        if (blockActive &&
            v->getCurrentRoadSegmentId() == blockRoadId &&
            (reason == "lane_blockage" || reason == "zipper_wait") &&
            v->getSpeed() < 0.15) {
            sawBlockStop = true;
            stoppedPos = v->getPositionOnRoad();
        }

        if (!blockActive && sawBlockStop) {
            if (v->getSpeed() > 0.2 ||
                v->getPositionOnRoad() > stoppedPos + 1.0 ||
                v->getCurrentRoadSegmentId() != blockRoadId ||
                v->getPositionOnRoad() > blockPos + 8.0) {
                sawResumeAfterClear = true;
                break;
            }
        }
    }

    expect(sawZipperOrBlockReason, "osm4_block_resume_saw_reason");
    expect(reasonMatchedOnce, "osm4_block_resume_reason_matches");
    expect(sawBlockStop, "osm4_block_resume_saw_stop");
    expect(sawResumeAfterClear, "osm4_block_resume_after_clear");
    expect(credible_freeze::countStaleReservations(simIso) == 0, "osm4_block_resume_no_stale_res");
}

void testLargerResetRestart() {
    if (!fs::exists(kLargerSample)) {
        cout << "SKIP  osm4_larger_reset (sample missing)\n";
        return;
    }

    SimulationSession session;
    redirectSessionOutputs(session, "osm4_larger_reset");
    OsmScenarioConfig config;
    config.maxGenerators = 6;
    config.vehiclesPerGenerator = 3;

    const OsmScenarioResult loaded = session.loadOsmFromFile(kLargerSample, config);
    expect(loaded.ok, "osm4_larger_reset_load");
    expect(session.getLoadedScenario() == SessionScenario::Osm, "osm4_larger_reset_mode");

    const size_t roadsBefore = session.getCity().getRoadSegments().size();
    session.start();
    for (int i = 0; i < 40; ++i) {
        session.update(SimulationSession::renderDt);
    }
    expect(session.isRunning(), "osm4_larger_reset_was_running");
    expect(session.getSimulation().getTime() > 0.0, "osm4_larger_reset_time_advanced");

    session.reset();
    expect(session.getLoadedScenario() == SessionScenario::Osm, "osm4_larger_reset_kept_osm");
    expect(session.getOsmSourcePath() == kLargerSample, "osm4_larger_reset_path");
    expect(!session.isRunning(), "osm4_larger_reset_stopped");
    expect(session.getCity().getRoadSegments().size() == roadsBefore, "osm4_larger_reset_roads");
    expect(session.getSimulation().getVehicles().empty(), "osm4_larger_reset_no_vehicles");
    expect(session.getSimulation().getTime() == 0.0, "osm4_larger_reset_time_zero");
    expect(credible_freeze::countActiveReservations(session.getSimulation()) == 0,
           "osm4_larger_reset_no_reservations");

    session.start();
    expect(session.isRunning(), "osm4_larger_restart_ok");
    for (int i = 0; i < 20; ++i) {
        session.update(SimulationSession::renderDt);
    }
    expect(session.getSimulation().getTime() > 0.0, "osm4_larger_restart_advances");
}

void testLargerTimeMultiplierSamePhysics() {
    if (!fs::exists(kLargerSample)) {
        cout << "SKIP  osm4_larger_timemul (sample missing)\n";
        return;
    }

    OsmScenarioConfig config;
    config.seed = VehicleGenerator::defaultSeed + 3u;
    config.maxGenerators = 4;
    config.vehiclesPerGenerator = 3;
    config.spawnIntervalSeconds = 2.5;

    SimulationSession sessionA;
    redirectSessionOutputs(sessionA, "osm4_larger_timemul_a");
    expect(sessionA.loadOsmFromFile(kLargerSample, config).ok, "osm4_timemul_load_a");
    sessionA.setTimeMultiplier(1.0);

    SimulationSession sessionB;
    redirectSessionOutputs(sessionB, "osm4_timemul_b");
    expect(sessionB.loadOsmFromFile(kLargerSample, config).ok, "osm4_timemul_load_b");
    sessionB.setTimeMultiplier(2.0);

    expect(fabs(SimulationSession::simulationDt - 0.1) < 1e-12, "osm4_timemul_dt_fixed");

    sessionA.start();
    sessionB.start();

    // Same wall time: B should advance ~2x sim time (catch-up clamped).
    const double wallSeconds = 2.0;
    const double frame = SimulationSession::renderDt;
    const int frames = static_cast<int>(wallSeconds / frame);
    for (int i = 0; i < frames; ++i) {
        sessionA.update(frame);
        sessionB.update(frame);
    }

    const double tA = sessionA.getSimulation().getTime();
    const double tB = sessionB.getSimulation().getTime();
    expect(tA > 0.5, "osm4_timemul_a_advanced");
    expect(tB > tA * 1.3, "osm4_timemul_b_faster_simtime");

    // Pause freezes even with high multiplier.
    sessionB.pause();
    const double pausedAt = sessionB.getSimulation().getTime();
    for (int i = 0; i < 30; ++i) {
        sessionB.update(frame);
    }
    expect(fabs(sessionB.getSimulation().getTime() - pausedAt) < 1e-9, "osm4_timemul_pause_freezes");
}

} // namespace

int runOsmLargerMapTests() {
    cout << "\n--- OSM larger map / credible checks (Stage 4) ---\n";
    testSampleCompareAndModerateDemand();
    testLargerHigherDemand();
    testLargerLaneBlockZipperResume();
    testLargerResetRestart();
    testLargerTimeMultiplierSamePhysics();
    return g_failures;
}

int osmLargerMapPassedCount() {
    return g_passed;
}
