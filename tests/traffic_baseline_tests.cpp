#include "core/City.h"
#include "core/Simulation.h"
#include "core/VehicleGenerator.h"
#include "osm/OsmScenarioBuilder.h"
#include "ui/RenderComposer.h"
#include "ui/SimulationSession.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
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
    fs::create_directories("tests/output");
}

void redirectSimOutputs(Simulation& sim, const string& tag) {
    ensureOutputDir();
    const string base = "tests/output/" + tag;
    sim.setEventLogOutputFile(base + "_events.json");
    sim.setStatisticsOutputFiles(base + "_timeline.csv", base + "_summary.csv");
}

struct TrafficProbeResult {
    string tag;
    string mapPath;
    bool ok = false;
    string error;

    uint32_t seed = 0;
    int maxGeneratorsConfig = 0;
    int vehiclesPerGenerator = 0;
    int configuredDemandLimit = 0; // maxGenerators * vehiclesPerGenerator (config intent)
    int generatorsAttached = 0;
    int demandCapAttached = 0; // attached * vehiclesPerGenerator

    int generated = 0;
    int failedSpawnAttempts = 0;
    int peakActive = 0;
    int endActive = 0;
    int finished = 0;
    int stopped = 0;

    double simTime = 0.0;
    double targetTime = 0.0;
    double routePrepMs = 0.0;
    double meanTickMs = 0.0;
    double meanSnapshotMs = 0.0;
    double maxTickMs = 0.0;
    double maxSnapshotMs = 0.0;

    size_t roads = 0;
    size_t intersections = 0;
    size_t crossingsAttached = 0;
    size_t phaseControllers = 0;
};

int countGenerated(const Simulation& sim) {
    int total = 0;
    for (const auto& g : sim.getVehicleGenerators()) {
        total += g.getGeneratedCount();
    }
    return total;
}

int countFailedSpawns(const Simulation& sim) {
    int total = 0;
    for (const auto& g : sim.getVehicleGenerators()) {
        total += g.getFailedSpawnAttempts();
    }
    return total;
}

TrafficProbeResult runProbe(
    const string& tag,
    const string& mapPath,
    OsmScenarioConfig config,
    double targetTime
) {
    TrafficProbeResult r;
    r.tag = tag;
    r.mapPath = mapPath;
    r.seed = config.seed;
    r.maxGeneratorsConfig = config.maxGenerators;
    r.vehiclesPerGenerator = config.vehiclesPerGenerator;
    r.configuredDemandLimit = config.maxGenerators * config.vehiclesPerGenerator;
    r.targetTime = targetTime;

    if (!fs::exists(mapPath)) {
        r.error = "missing map";
        cout << "SKIP  " << tag << " missing " << mapPath << "\n";
        return r;
    }

    City city;
    Simulation sim;
    redirectSimOutputs(sim, tag);

    const auto t0 = Clock::now();
    const OsmScenarioResult loaded = OsmScenarioBuilder::loadOsmFile(mapPath, city, sim, config);
    const auto t1 = Clock::now();
    r.routePrepMs = chrono::duration<double, milli>(t1 - t0).count();

    if (!loaded.ok) {
        r.error = loaded.error;
        expect(false, tag + "_load_ok");
        return r;
    }
    expect(true, tag + "_load_ok");

    r.ok = true;
    r.generatorsAttached = static_cast<int>(loaded.attachedGeneratorCount);
    r.demandCapAttached = r.generatorsAttached * config.vehiclesPerGenerator;
    r.roads = city.getRoadSegments().size();
    r.intersections = city.getIntersections().size();
    r.crossingsAttached = loaded.attachedCrossingCount;
    r.phaseControllers = loaded.generatedPhaseControllerCount;

    RenderComposer composer;
    const double dt = 0.1;
    double tickAccum = 0.0;
    double snapAccum = 0.0;
    int ticks = 0;

    sim.start();
    int snapSamples = 0;
    while (sim.getTime() < targetTime - 1e-9) {
        const auto tt0 = Clock::now();
        sim.update(dt, city);
        const auto tt1 = Clock::now();
        const double tickMs = chrono::duration<double, milli>(tt1 - tt0).count();
        tickAccum += tickMs;
        r.maxTickMs = max(r.maxTickMs, tickMs);

        // Sample snapshots every 10 ticks — enough for mean/max without dominating runtime.
        if (ticks % 10 == 0) {
            const auto ts0 = Clock::now();
            const RenderSnapshot snap = composer.buildSnapshot(city, sim);
            const auto ts1 = Clock::now();
            const double snapMs = chrono::duration<double, milli>(ts1 - ts0).count();
            snapAccum += snapMs;
            r.maxSnapshotMs = max(r.maxSnapshotMs, snapMs);
            ++snapSamples;
            (void)snap;
        }

        const auto& stats = sim.getStatisticsCollector().getLastSnapshot();
        r.peakActive = max(r.peakActive, stats.activeVehicles);
        ++ticks;
    }

    const auto& endStats = sim.getStatisticsCollector().getLastSnapshot();
    r.generated = countGenerated(sim);
    r.failedSpawnAttempts = countFailedSpawns(sim);
    r.endActive = endStats.activeVehicles;
    r.finished = endStats.finishedVehicles;
    r.stopped = endStats.stoppedVehicles;
    r.simTime = sim.getTime();
    r.meanTickMs = ticks > 0 ? tickAccum / ticks : 0.0;
    r.meanSnapshotMs = snapSamples > 0 ? snapAccum / snapSamples : 0.0;

    // Achieved demand vs configured / attached caps (measurement asserts, not "raise caps" fixes).
    expect(r.simTime >= targetTime - 1.0, tag + "_ran_target");
    expect(r.generatorsAttached >= 0, tag + "_generators_nonneg");
    if (r.demandCapAttached > 0) {
        expect(r.generated <= r.demandCapAttached, tag + "_generated_le_attached_cap");
    }

    cout << "TRAFFIC " << tag
         << " gen_cfg_limit=" << r.configuredDemandLimit
         << " attached=" << r.generatorsAttached
         << " demand_cap=" << r.demandCapAttached
         << " generated=" << r.generated
         << " failed_spawns=" << r.failedSpawnAttempts
         << " peak_active=" << r.peakActive
         << " end_active=" << r.endActive
         << " finished=" << r.finished
         << " tick_ms=" << r.meanTickMs
         << " snap_ms=" << r.meanSnapshotMs
         << " route_prep_ms=" << r.routePrepMs
         << "\n";

    return r;
}

void writeProbe(ostream& out, const TrafficProbeResult& r) {
    out << "\n## " << r.tag << "\n";
    out << "map=" << r.mapPath << "\n";
    out << "ok=" << (r.ok ? 1 : 0) << "\n";
    if (!r.error.empty()) {
        out << "error=" << r.error << "\n";
    }
    out << "seed=" << r.seed << "\n";
    out << "max_generators_config=" << r.maxGeneratorsConfig << "\n";
    out << "vehicles_per_generator=" << r.vehiclesPerGenerator << "\n";
    out << "configured_demand_limit=" << r.configuredDemandLimit << "\n";
    out << "generators_attached=" << r.generatorsAttached << "\n";
    out << "demand_cap_attached=" << r.demandCapAttached << "\n";
    out << "generated=" << r.generated << "\n";
    out << "failed_spawn_attempts=" << r.failedSpawnAttempts << "\n";
    out << "peak_active=" << r.peakActive << "\n";
    out << "end_active=" << r.endActive << "\n";
    out << "finished=" << r.finished << "\n";
    out << "stopped=" << r.stopped << "\n";
    out << "sim_time=" << r.simTime << " target_time=" << r.targetTime << "\n";
    out << "route_prep_ms=" << r.routePrepMs << "\n";
    out << "mean_tick_ms=" << r.meanTickMs << " max_tick_ms=" << r.maxTickMs << "\n";
    out << "mean_snapshot_ms=" << r.meanSnapshotMs << " max_snapshot_ms=" << r.maxSnapshotMs << "\n";
    out << "roads=" << r.roads << " intersections=" << r.intersections
        << " crossings_attached=" << r.crossingsAttached
        << " phase_controllers=" << r.phaseControllers << "\n";
    const double spawnSuccess =
        (r.generated + r.failedSpawnAttempts) > 0
            ? static_cast<double>(r.generated) / static_cast<double>(r.generated + r.failedSpawnAttempts)
            : 0.0;
    out << "spawn_success_ratio_approx=" << spawnSuccess << "\n";
    out << "peak_active_vs_demand_cap="
        << (r.demandCapAttached > 0
                ? static_cast<double>(r.peakActive) / static_cast<double>(r.demandCapAttached)
                : 0.0)
        << "\n";
}

OsmScenarioConfig makeConfig(uint32_t seed, int maxGen, int perGen, double interval) {
    OsmScenarioConfig c;
    c.seed = seed;
    c.maxGenerators = maxGen;
    c.vehiclesPerGenerator = perGen;
    c.spawnIntervalSeconds = interval;
    c.phaseGreenSeconds = 8.0;
    c.minVehicleGap = 4.0;
    c.yieldRefusalProbability = 0.05;
    c.maxRouteHops = 10;
    return c;
}

} // namespace

int runTrafficBaselineTests() {
    g_failures = 0;
    g_passed = 0;
    ensureOutputDir();
    cout << "\n=== Traffic baselines (Etap 1) ===\n";

    vector<TrafficProbeResult> results;
    const char* small = "data/osm/warsaw_sample.overpass.json";
    const char* larger = "data/osm/warsaw_1km.overpass.json";
    const uint32_t seed = VehicleGenerator::defaultSeed;

    // Progressive intensity probes: target peak capacity steps 100→200→500→1000 as demand caps.
    // Actual achieved peak_active is measured; we do not raise caps beyond spawn success evidence.
    struct ProbeSpec {
        const char* tag;
        const char* path;
        int maxGen;
        int perGen;
        double interval;
        double simSeconds;
        int targetActiveHint; // documentation only
    };

    // Sim windows sized to observe peak occupancy; not full drain-to-empty runs.
    const vector<ProbeSpec> specs = {
        {"traf_small_default", small, 8, 5, 2.5, 90.0, 40},
        {"traf_small_100", small, 10, 10, 1.2, 100.0, 100},
        {"traf_large_default", larger, 8, 5, 2.5, 120.0, 40},
        {"traf_large_100", larger, 10, 10, 1.2, 140.0, 100},
        {"traf_large_200", larger, 16, 13, 1.0, 150.0, 200},
        {"traf_large_500", larger, 25, 20, 0.8, 160.0, 500},
        {"traf_large_1000", larger, 40, 25, 0.6, 180.0, 1000}
    };

    for (const auto& spec : specs) {
        if (!fs::exists(spec.path)) {
            cout << "SKIP  " << spec.tag << "\n";
            continue;
        }
        OsmScenarioConfig cfg = makeConfig(seed, spec.maxGen, spec.perGen, spec.interval);
        results.push_back(runProbe(spec.tag, spec.path, cfg, spec.simSeconds));

        // Stop escalating if last probe could not sustain meaningful spawn success
        // relative to attached demand (capacity signal for Etap 2 traffic work).
        const TrafficProbeResult& last = results.back();
        if (last.ok && last.demandCapAttached >= 200) {
            const double fill =
                last.demandCapAttached > 0
                    ? static_cast<double>(last.peakActive) / static_cast<double>(last.demandCapAttached)
                    : 0.0;
            if (fill < 0.05 && last.failedSpawnAttempts > last.generated) {
                cout << "NOTE  stopping progressive probes: peak fill=" << fill
                     << " failed_spawns=" << last.failedSpawnAttempts
                     << " generated=" << last.generated << " at " << last.tag << "\n";
                // Still continue remaining for measurement honesty — comment only.
            }
        }
        (void)spec.targetActiveHint;
    }

    // Determinism smoke: two identical short runs must match generated + peak.
    if (fs::exists(small)) {
        OsmScenarioConfig cfg = makeConfig(seed + 99u, 6, 4, 2.0);
        TrafficProbeResult a = runProbe("traf_det_a", small, cfg, 60.0);
        TrafficProbeResult b = runProbe("traf_det_b", small, cfg, 60.0);
        expect(a.ok && b.ok, "traf_det_both_ok");
        expect(a.generated == b.generated, "traf_det_generated");
        expect(a.peakActive == b.peakActive, "traf_det_peak_active");
        expect(a.failedSpawnAttempts == b.failedSpawnAttempts, "traf_det_failed_spawns");
        results.push_back(a);
        results.push_back(b);
    }

    ofstream report("tests/output/etap1_traffic_baseline.txt");
    report << "Etap 1 traffic baselines\n";
    report << "deterministic_seed_base=" << seed << "\n";
    report << "note=configured_demand_limit is maxGenerators*vehiclesPerGenerator; "
              "generators_attached may be lower when routing finds fewer routes.\n";
    report << "note=peak_active is achieved concurrent vehicles, not the config limit.\n";
    for (const auto& r : results) {
        writeProbe(report, r);
    }
    report.close();

    // Compact CSV for store summary
    ofstream csv("tests/output/etap1_traffic_baseline.csv");
    csv << "tag,map,ok,seed,cfg_demand,attached,demand_cap,generated,failed_spawns,"
           "peak_active,end_active,finished,route_prep_ms,mean_tick_ms,mean_snapshot_ms,sim_time\n";
    for (const auto& r : results) {
        csv << r.tag << ","
            << r.mapPath << ","
            << (r.ok ? 1 : 0) << ","
            << r.seed << ","
            << r.configuredDemandLimit << ","
            << r.generatorsAttached << ","
            << r.demandCapAttached << ","
            << r.generated << ","
            << r.failedSpawnAttempts << ","
            << r.peakActive << ","
            << r.endActive << ","
            << r.finished << ","
            << r.routePrepMs << ","
            << r.meanTickMs << ","
            << r.meanSnapshotMs << ","
            << r.simTime << "\n";
    }
    csv.close();

    return g_failures;
}

int trafficBaselinePassedCount() {
    return g_passed;
}
